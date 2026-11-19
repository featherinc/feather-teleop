#include "src/robot/actuators/sim/virtual_robstride.hpp"

#include "system.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace feather::robot::sim {
namespace {

constexpr double kKi = 4;
constexpr double kDAlpha = 0.3;
constexpr double kInertia = 0.2;
constexpr int kIntegralWindow = 20;

// Fault-word bit positions. Feedback frames use the low 5 error bits directly.
constexpr struct { uint8_t wire, bit; } kFaultBits[] = {
    {0, ROB_MOTOR_ERROR_OVER_TEMPERATURE},
    {1, ROB_MOTOR_ERROR_DRIVER_CHIP_FAULT},
    {2, ROB_MOTOR_ERROR_UNDER_VOLTAGE},
    {3, ROB_MOTOR_ERROR_OVER_VOLTAGE},
    {4, ROB_MOTOR_ERROR_B_PHASE_OVERCURRENT},
    {5, ROB_MOTOR_ERROR_C_PHASE_OVERCURRENT},
    {7, ROB_MOTOR_ERROR_ENCODER_NOT_CALIBRATED},
    {8, ROB_MOTOR_ERROR_HARDWARE_IDENTIFICATION_FAULT},
    {9, ROB_MOTOR_ERROR_POSITION_INITIALIZATION_FAULT},
    {14, ROB_MOTOR_ERROR_GRIDLOCK_OVERLOAD},
    {16, ROB_MOTOR_ERROR_A_PHASE_OVERCURRENT},
};

uint32_t reply_id(int motor_id, RobOperation op, uint8_t host_id, uint8_t e_byte) {
    return ((uint32_t)op << 24) | ((uint32_t)e_byte << 16) | ((uint32_t)motor_id << 8) | host_id;
}

uint16_t read_be(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

void write_be(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

// Big-endian uint16 across [min, max], truncated the same way the firmware packs a sample.
uint16_t pack_u16(double value, double min, double max) {
    double mapped = (value - min) / (max - min) * 65535.0;
    if (mapped < 0) mapped = 0;
    if (mapped > 65535) mapped = 65535;
    return (uint16_t)mapped;
}

double unpack_u16(uint16_t raw, double min, double max) {
    return (double)raw / 65535.0 * (max - min) + min;
}

const RobParam* find_param(uint16_t param_id) {
    if (param_id == 0) return nullptr;
    for (const RobParam& p : rob_all_params) {
        if (p.param_id == param_id) return &p;
    }
    return nullptr;
}

bool same_bytes(const uint8_t* data, const uint8_t* expect, int n) {
    return std::memcmp(data, expect, (size_t)n) == 0;
}

// Commands that carry the 1..6 prefix and a small code in byte 6.
void require_prefixed(const uint8_t* data, int min_code, int max_code, const char* what) {
    static const uint8_t kPrefix[6] = {1, 2, 3, 4, 5, 6};
    if (!same_bytes(data, kPrefix, 6) || data[6] < min_code || data[6] > max_code) {
        throw std::invalid_argument(what);
    }
}

}  // namespace

VirtualRobstride::VirtualRobstride(int motor_id, RobVersion version, double hard_stop_min, double hard_stop_max)
    : motor_id_(motor_id), version_(version), hard_stop_min_(hard_stop_min), hard_stop_max_(hard_stop_max) {
    if (motor_id < 0 || motor_id > 0xFF) throw std::invalid_argument("Robstride motor id must be in [0, 255]");
    if (version >= NUM_RobVersion) throw std::invalid_argument("Unknown Robstride version");
    if (!(hard_stop_min < hard_stop_max)) throw std::invalid_argument("Hard-stop range is empty");

    for (const RobParam& p : rob_all_params) {
        const bool is_float = p.dtype == DATA_TYPE_F32 || p.dtype == DATA_TYPE_F64;
        params_[p.pcode] = is_float ? p.default_float[version] : (double)p.default_int[version];
    }
}

void VirtualRobstride::set_error(RobMotorError error) {
    errors_ = (uint16_t)(errors_ | (uint16_t)(1u << error));
    sent_fault_feedback_ = false;
}

void VirtualRobstride::enqueue(uint32_t id, const uint8_t* data) {
    SimCanFrame frame;
    frame.id = id;
    std::memcpy(frame.data, data, 8);
    out_.push_back(frame);
}

bool VirtualRobstride::pop_frame(SimCanFrame& out) {
    if (out_.empty()) return false;
    out = out_.front();
    out_.pop_front();
    return true;
}

void VirtualRobstride::send_broadcast() {
    uint8_t data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, (uint8_t)version_, 0x00, (uint8_t)motor_id_};
    enqueue(reply_id(motor_id_, ROB_OPERATION_GET_DEVICE_ID, 0xFE, 0), data);
}

void VirtualRobstride::send_feedback(RobOperation op, bool firmware_version) {
    uint8_t data[8]{};
    uint8_t e_byte = 0;
    if (firmware_version) {
        // "0.<version>.0.0", the layout the firmware accepts as a version reply.
        data[1] = 0xC4;
        data[2] = 'V';
        data[4] = (uint8_t)version_;
        data[7] = 7;
    } else {
        const int v = version_;
        write_be(data + 0, pack_u16(params_[ROB_PARAM_CODE_FB_POSITION], ROB_ANGLE_MIN, ROB_ANGLE_MAX));
        write_be(data + 2, pack_u16(params_[ROB_PARAM_CODE_FB_VELOCITY], rob_n_spd_lim[v], rob_spd_lim[v]));
        write_be(data + 4, pack_u16(params_[ROB_PARAM_CODE_FB_TORQUE], rob_n_torque_lim[v], rob_torque_lim[v]));
        write_be(data + 6, pack_u16(params_[ROB_PARAM_CODE_FB_TEMP] * 10.0, 0, 65535));
        const uint8_t mode = (uint8_t)params_[ROB_PARAM_CODE_FB_MODE];
        e_byte = (uint8_t)(((mode & 3) << 6) | (errors_ & 0x1F));
    }
    enqueue(reply_id(motor_id_, op, last_host_id_, e_byte), data);
}

void VirtualRobstride::read_param(uint8_t host_id, const uint8_t* data) {
    uint16_t param_id;
    std::memcpy(&param_id, data, 2);
    const RobParam* p = find_param(param_id);
    if (p == nullptr || !p->readable) throw std::invalid_argument("Robstride param is not readable");

    uint8_t out[8]{};
    std::memcpy(out, &param_id, 2);
    const double value = params_[p->pcode];
    if (p->dtype == DATA_TYPE_F32 || p->dtype == DATA_TYPE_F64) {
        float f = (float)value;
        std::memcpy(out + 4, &f, 4);
    } else if (p->dtype == DATA_TYPE_U16 || p->dtype == DATA_TYPE_I16) {
        uint16_t v = (uint16_t)value;
        std::memcpy(out + 4, &v, 2);
    } else if (p->dtype == DATA_TYPE_U32 || p->dtype == DATA_TYPE_I32) {
        uint32_t v = (uint32_t)value;
        std::memcpy(out + 4, &v, 4);
    } else {
        out[4] = (uint8_t)value;
    }
    enqueue(reply_id(motor_id_, ROB_OPERATION_READ_PARAM, host_id, 0), out);
}

void VirtualRobstride::write_param(const uint8_t* data) {
    uint16_t param_id;
    std::memcpy(&param_id, data, 2);
    const RobParam* p = find_param(param_id);
    if (p == nullptr || !p->writable) throw std::invalid_argument("Robstride param is not writable");

    double value;
    const int v = version_;
    if (p->dtype == DATA_TYPE_F32 || p->dtype == DATA_TYPE_F64) {
        float raw;
        std::memcpy(&raw, data + 4, 4);
        value = std::clamp((double)raw, (double)p->min_float[v], (double)p->max_float[v]);
    } else {
        uint32_t raw = data[4];
        if (p->dtype == DATA_TYPE_U16 || p->dtype == DATA_TYPE_I16) std::memcpy(&raw, data + 4, 2);
        else if (p->dtype == DATA_TYPE_U32 || p->dtype == DATA_TYPE_I32) std::memcpy(&raw, data + 4, 4);
        if (raw < p->min_int[v]) raw = (uint32_t)p->min_int[v];
        if (raw > p->max_int[v]) raw = (uint32_t)p->max_int[v];
        value = raw;
    }

    params_[p->pcode] = value;
    if (p->pcode == ROB_PARAM_CODE_LOC_REF && params_[ROB_PARAM_CODE_RUN_MODE] == ROB_RUN_MODE_POSITION_PP) {
        integrals_.clear();
        have_derivative_ = false;
    }
    send_feedback(ROB_OPERATION_MOTOR_FEEDBACK, false);
}

void VirtualRobstride::handle_frame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (data == nullptr || len != 8 || (int)(can_id & 0xFF) != motor_id_) return;

    const auto op = (RobOperation)((can_id >> 24) & 0x1F);
    const uint8_t e_byte = (uint8_t)((can_id >> 16) & 0xFF);
    last_host_id_ = (uint8_t)((can_id >> 8) & 0xFF);

    switch (op) {
        case ROB_OPERATION_GET_DEVICE_ID:
            send_broadcast();
            break;
        case ROB_OPERATION_OPERATION_CONTROL:
            // Position mode uses this kd in the derivative term. Operation mode is not simulated.
            op_kd_ = unpack_u16(read_be(data + 6), 0, rob_kd_lim[version_]);
            break;
        case ROB_OPERATION_ENABLE:
            params_[ROB_PARAM_CODE_FB_MODE] = ROB_MOTOR_MODE_RUN;
            send_feedback(ROB_OPERATION_MOTOR_FEEDBACK, false);
            break;
        case ROB_OPERATION_DISABLE: {
            params_[ROB_PARAM_CODE_FB_MODE] = ROB_MOTOR_MODE_RESET;
            static const uint8_t kClear[8] = {1, 0, 0, 0, 0, 0, 0, 0};
            if (same_bytes(data, kClear, 8)) {
                errors_ = 0;
                sent_fault_feedback_ = false;
            }
            send_feedback(ROB_OPERATION_MOTOR_FEEDBACK, data[0] == 0x00 && data[1] == 0xC4);
            break;
        }
        case ROB_OPERATION_ZERO_POS: {
            static const uint8_t kZero[8] = {1, 0, 0, 0, 0, 0, 0, 0};
            if (!same_bytes(data, kZero, 8)) throw std::invalid_argument("Invalid zero-position command");
            params_[ROB_PARAM_CODE_FB_POSITION] = 0;
            send_feedback(ROB_OPERATION_MOTOR_FEEDBACK, false);
            break;
        }
        case ROB_OPERATION_SET_ID:
            motor_id_ = e_byte;
            send_broadcast();
            break;
        case ROB_OPERATION_READ_PARAM:
            read_param(last_host_id_, data);
            break;
        case ROB_OPERATION_WRITE_PARAM:
            write_param(data);
            break;
        case ROB_OPERATION_SAVE_MOTOR_DATA: {
            static const uint8_t kSave[8] = {1, 2, 3, 4, 5, 6, 7, 8};
            if (!same_bytes(data, kSave, 8)) throw std::invalid_argument("Invalid save-motor-data command");
            send_feedback(ROB_OPERATION_MOTOR_FEEDBACK, false);
            break;
        }
        case ROB_OPERATION_SET_MOTOR_BAUD_RATE:
            require_prefixed(data, 1, 4, "Invalid baud-rate command");
            INFO("Ignoring set-baud-rate on virtual motor 0x%X", motor_id_);
            send_broadcast();
            break;
        case ROB_OPERATION_SET_ACTIVE_REPORTING:
            require_prefixed(data, 0, 1, "Invalid active-reporting command");
            active_reporting_ = data[6] == 1;
            send_feedback(ROB_OPERATION_SET_ACTIVE_REPORTING, false);
            last_active_reporting_ = feather::time::now();
            break;
        case ROB_OPERATION_SET_MOTOR_PROTOCOL:
            require_prefixed(data, 0, 2, "Invalid protocol command");
            INFO("Ignoring set-protocol on virtual motor 0x%X", motor_id_);
            send_broadcast();
            break;
        default:
            break;
    }
}

void VirtualRobstride::step(double dt) {
    if (!sent_first_broadcast_) {
        send_broadcast();
        sent_first_broadcast_ = true;
    }

    if (params_[ROB_PARAM_CODE_FB_MODE] == ROB_MOTOR_MODE_RUN) {
        const int run_mode = (int)params_[ROB_PARAM_CODE_RUN_MODE];
        double torque = 0;
        if (run_mode == ROB_RUN_MODE_POSITION_PP) torque = position_torque(dt);
        else if (run_mode == ROB_RUN_MODE_VELOCITY) {
            if (std::fabs(params_[ROB_PARAM_CODE_SPD_REF]) > 1e-6) {
                throw std::runtime_error("Virtual Robstride velocity mode is not implemented");
            }
        } else {
            throw std::runtime_error("Virtual Robstride run mode is not implemented");
        }
        apply_torque(torque, dt);
    }

    const double scan = params_[ROB_PARAM_CODE_EPSCAN_TIME];
    const double interval = scan == 0 ? 0 : (scan + 1) * 0.005;
    if (active_reporting_ && feather::time::elapsed(last_active_reporting_) > feather::time::from_seconds(interval)) {
        send_feedback(ROB_OPERATION_MOTOR_FEEDBACK, false);
        last_active_reporting_ = feather::time::now();
    }

    if (errors_ == 0 || sent_fault_feedback_) return;
    uint32_t word = 0;
    for (auto bit : kFaultBits) {
        if (errors_ & (uint16_t)(1u << bit.bit)) word |= 1u << bit.wire;
    }
    uint8_t data[8]{};
    std::memcpy(data, &word, 4);
    enqueue(reply_id(motor_id_, ROB_OPERATION_FAULT_FEEDBACK, last_host_id_, 0), data);
    sent_fault_feedback_ = true;
}

double VirtualRobstride::position_torque(double dt) {
    const double error = params_[ROB_PARAM_CODE_LOC_REF] - params_[ROB_PARAM_CODE_FB_POSITION];
    if (!have_derivative_) last_error_ = error;

    integrals_.push_back(error * dt);
    if ((int)integrals_.size() > kIntegralWindow) integrals_.pop_front();
    double integral = 0;
    for (double sample : integrals_) integral += sample;

    const double d_raw = dt == 0 ? 0 : (error - last_error_) / dt;
    if (!have_derivative_) d_filt_ = d_raw;
    d_filt_ = d_raw * kDAlpha + d_filt_ * (1 - kDAlpha);
    last_error_ = error;
    have_derivative_ = true;

    const double total = params_[ROB_PARAM_CODE_LOC_KP] * error + kKi * integral + op_kd_ * d_filt_;
    const double cap = std::min(std::fabs(params_[ROB_PARAM_CODE_LIMIT_TORQUE]),
        std::fabs(params_[ROB_PARAM_CODE_ACC_SET]) * kInertia);
    return std::clamp(total, -cap, cap);
}

void VirtualRobstride::apply_torque(double torque, double dt) {
    const double physical = std::fabs(rob_torque_lim[version_]);
    torque = std::clamp(torque, -physical, physical);

    const double pos = params_[ROB_PARAM_CODE_FB_POSITION];
    double external = external_torque_;
    if (pos >= hard_stop_max_ && torque > 0) external = std::min(external, -torque);
    else if (pos <= hard_stop_min_ && torque < 0) external = std::max(external, -torque);

    double velocity = params_[ROB_PARAM_CODE_FB_VELOCITY] + (torque + external) / kInertia * dt;
    const int run_mode = (int)params_[ROB_PARAM_CODE_RUN_MODE];
    double max_velocity = run_mode == ROB_RUN_MODE_VELOCITY
        ? params_[ROB_PARAM_CODE_SPD_REF] : params_[ROB_PARAM_CODE_VEL_MAX];
    // The version speed limit is a floor on the clamp, matching the Python simulator.
    max_velocity = std::max(max_velocity, (double)rob_spd_lim[version_]);
    velocity = std::clamp(velocity, -max_velocity, max_velocity);

    double new_pos = pos + velocity * dt;
    double reported = torque;
    if (!(hard_stop_min_ < new_pos && new_pos < hard_stop_max_)) {
        new_pos = std::clamp(new_pos, hard_stop_min_, hard_stop_max_);
        velocity = 0;
        reported = torque > 0 ? physical : -physical;
    }

    params_[ROB_PARAM_CODE_FB_TORQUE] = reported;
    params_[ROB_PARAM_CODE_FB_VELOCITY] = velocity;
    params_[ROB_PARAM_CODE_FB_POSITION] = new_pos;
}

}  // namespace feather::robot::sim
