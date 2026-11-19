#include "robstride.hpp"

#include "logging.hpp"
#include "system.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

namespace feather::robot {
namespace {

constexpr double kTolerance = 0.01;
constexpr int kSpamMs = 2;

const char* kErrorName[NUM_RobMotorError] = {
    "UNDER_VOLTAGE", "OVER_CURRENT", "OVER_TEMPERATURE", "MAGNETIC_ENCODING", "GRIDLOCK_OVERLOAD",
    "DRIVER_CHIP_FAULT", "OVER_VOLTAGE", "ENCODER_NOT_CALIBRATED", "A_PHASE_OVERCURRENT", "B_PHASE_OVERCURRENT",
    "C_PHASE_OVERCURRENT", "HARDWARE_IDENTIFICATION_FAULT", "POSITION_INITIALIZATION_FAULT",
};

void pause_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

double as_double(DataType dtype, const std::uint8_t* buf) {
    switch (dtype) {
        case DATA_TYPE_F32: { float v; std::memcpy(&v, buf, 4); return v; }
        case DATA_TYPE_F64: { double v; std::memcpy(&v, buf, 8); return v; }
        case DATA_TYPE_U8: return buf[0];
        case DATA_TYPE_U16: { std::uint16_t v; std::memcpy(&v, buf, 2); return v; }
        case DATA_TYPE_U32: { std::uint32_t v; std::memcpy(&v, buf, 4); return v; }
        case DATA_TYPE_U64: { std::uint64_t v; std::memcpy(&v, buf, 8); return static_cast<double>(v); }
        case DATA_TYPE_I8: return static_cast<std::int8_t>(buf[0]);
        case DATA_TYPE_I16: { std::int16_t v; std::memcpy(&v, buf, 2); return v; }
        case DATA_TYPE_I32: { std::int32_t v; std::memcpy(&v, buf, 4); return v; }
        case DATA_TYPE_I64: { std::int64_t v; std::memcpy(&v, buf, 8); return static_cast<double>(v); }
        case DATA_TYPE_BOOL: return buf[0] != 0;
        default: return 0;
    }
}

// Stored little-endian as <0x00><0x00><'V'><major><version><minor><micro><0x07>.
std::string format_firmware(std::uint64_t version) {
    std::uint8_t b[8];
    std::memcpy(b, &version, sizeof b);
    return std::to_string(b[3]) + "." + std::to_string(b[4]) + "." + std::to_string(b[5]) + "." + std::to_string(b[6]);
}

std::string error_names(unsigned bits) {
    std::string out;
    for (int i = 0; i < NUM_RobMotorError; ++i) {
        if ((bits & (1u << i)) == 0) continue;
        if (!out.empty()) out += ", ";
        out += kErrorName[i];
    }
    return out;
}

template <typename F>
void wait_until(F done, const std::string& err) {
    const feather_time_t deadline = feather::time::now() + feather::time::from_seconds(2.0);
    while (feather::time::now() < deadline) {
        if (done()) return;
        pause_ms(5);
    }
    throw std::runtime_error(err);
}

double param_default(const RobParam& param, int version) {
    if (param.dtype == DATA_TYPE_F32 || param.dtype == DATA_TYPE_F64) return param.default_float[version];
    return static_cast<double>(param.default_int[version]);
}

double param_max(const RobParam& param, int version) {
    if (param.dtype == DATA_TYPE_F32 || param.dtype == DATA_TYPE_F64) return param.max_float[version];
    return static_cast<double>(param.max_int[version]);
}

void validate_config(const RobstrideMotorConfig& cfg) {
    if (cfg.name.empty()) throw std::invalid_argument("Robstride motor name is empty");
    if (cfg.id < 0 || cfg.id > 0xFD) throw std::invalid_argument(cfg.name + ": motor id out of range");
    if (cfg.direction != 1 && cfg.direction != -1) throw std::invalid_argument(cfg.name + ": direction must be 1 or -1");
    if (cfg.version >= NUM_RobVersion) throw std::invalid_argument(cfg.name + ": unknown motor version");
    if (cfg.teensy_telemetry_max_size <= 0 || cfg.teensy_telemetry_max_size > 0xFFFF) {
        throw std::invalid_argument(cfg.name + ": teensy telemetry size out of range");
    }
    if (!(cfg.max_jerk > 0.0 && cfg.max_jerk < 1000.0)) {
        throw std::invalid_argument(cfg.name + ": max_jerk out of range");
    }
    if (cfg.feedback_delay_threshold < 0.0) {
        throw std::invalid_argument(cfg.name + ": feedback_delay_threshold is negative");
    }
    if (!(cfg.feedback_delay_warning_interval > 0.0)) {
        throw std::invalid_argument(cfg.name + ": feedback_delay_warning_interval must be positive");
    }
}

}  // namespace

RobstrideMotor::RobstrideMotor(
    const RobstrideMotorConfig& cfg, feather::comms::CommsManager& comms, Telemetry& telemetry
) : config(cfg), comms_(comms), telemetry_(telemetry) {
    validate_config(cfg);
    if (telemetry.initialized()) throw std::runtime_error(cfg.name + ": telemetry is already initialized");

    for (int i = 0; i < NUM_RobParamCode; ++i) {
        const auto cap = rob_all_params[i].max_telemetry_size == 0 ? 1 : rob_all_params[i].max_telemetry_size;
        const int idx = telemetry.add_telemetry(cap, rob_all_params[i].dtype);
        if (idx < 0 || idx > 0xFFFF) throw std::runtime_error(config.name + ": failed to add telemetry");
        telem_[i] = static_cast<std::uint16_t>(idx);
    }
}

std::uint8_t RobstrideMotor::motor_id() const {
    return static_cast<std::uint8_t>(config.id);
}

std::uint8_t RobstrideMotor::can_bus() const {
    if (config.endpoint < 0 || config.endpoint > 254) throw std::runtime_error(config.name + ": no CAN endpoint");
    return static_cast<std::uint8_t>(config.endpoint);
}

bool RobstrideMotor::try_latest(RobParamCode param, double* out) const {
    if (out == nullptr) return false;
    std::uint8_t buf[8]{};
    const DataType dtype = rob_all_params[param].dtype;
    const std::size_t n = dtype_size(dtype);
    if (n == 0 || n > sizeof buf || !telemetry_.latest(telem_[param], buf, n)) return false;
    *out = as_double(dtype, buf);
    return true;
}

double RobstrideMotor::value(RobParamCode param) const {
    double out = 0;
    try_latest(param, &out);
    return out;
}

std::uint64_t RobstrideMotor::bits(RobParamCode param) const {
    std::uint64_t out = 0;
    const std::size_t n = dtype_size(rob_all_params[param].dtype);
    if (n == 0 || n > sizeof out || !telemetry_.latest(telem_[param], &out, n)) return 0;
    return out;
}

bool RobstrideMotor::try_stamp(RobParamCode param, double* out) const {
    const RobParamCode ts = rob_all_params[param].timestamp_pcode;
    return try_latest(ts == NUM_RobParamCode ? param : ts, out);
}

double RobstrideMotor::stamp(RobParamCode param) const {
    double out = 0;
    try_stamp(param, &out);
    return out;
}

void RobstrideMotor::fail(const std::string& msg) const {
    throw std::runtime_error("[" + config.name + "] " + msg);
}

bool RobstrideMotor::wait_for(RobParamCode watch, double start, const double* expect, double timeout_s) const {
    const feather_time_t deadline = feather::time::now() + feather::time::from_seconds(timeout_s);
    while (feather::time::now() < deadline) {
        if (stamp(watch) > start && (expect == nullptr || std::fabs(value(watch) - *expect) < kTolerance)) return true;
        pause_ms(1);
    }
    return false;
}

bool RobstrideMotor::send_ok(
    TeensyCommandOpcode opcode, const void* body, std::size_t size, bool block, RobParamCode watch,
    const double* expect, double timeout_s, int attempts
) {
    if (timeout_s < 0) timeout_s = config.message_block_timeout;
    if (attempts < 1) attempts = config.message_send_attempts;
    const std::string_view payload(static_cast<const char*>(body), size);
    if (!block) return comms_.send_teensy_message(opcode, payload).msg_id != 0;

    const double start = stamp(watch);
    for (int i = 0; i < attempts; ++i) {
        if (comms_.send_teensy_message(opcode, payload).msg_id == 0) continue;
        if (wait_for(watch, start, expect, timeout_s)) return true;
    }
    return false;
}

bool RobstrideMotor::command_read(RobParamCode param, bool block, const double* expect, double timeout_s, int attempts) {
    // Feedback params are not readable via READ_PARAM; request a feedback frame instead.
    if (rob_all_params[param].is_feedback_param) {
        const RobWriteParamPayload body{motor_id(), can_bus(), ROB_PARAM_CODE_CAN_TIMEOUT, 0, false};
        return send_ok(
            TEENSY_COMMAND_OPCODE_CMD_ROB_WRITE_PARAM, &body, sizeof body, block, ROB_PARAM_CODE_FB_MODE, nullptr,
            timeout_s, attempts
        );
    }
    if (param == ROB_PARAM_CODE_DEVICE_ID) {
        const RobGetDeviceIdPayload body{motor_id(), can_bus()};
        return send_ok(
            TEENSY_COMMAND_OPCODE_CMD_ROB_GET_DEVICE_ID, &body, sizeof body, block, param, expect, timeout_s, attempts
        );
    }
    const RobReadParamPayload body{motor_id(), can_bus(), static_cast<std::uint8_t>(param)};
    return send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_READ_PARAM, &body, sizeof body, block, param, expect, timeout_s, attempts);
}

void RobstrideMotor::init(bool allow_reinit) {
    if (!telemetry_.initialized()) fail("telemetry is not initialized");

    RobMotorInitPayload body{};
    body.actuator_id = motor_id();
    body.allow_reinit = allow_reinit;
    body.config.version = config.version;
    body.config.max_velocity = static_cast<float>(config.max_velocity);
    body.config.max_torque = static_cast<float>(config.max_torque);
    body.config.max_acceleration = static_cast<float>(config.max_acceleration);
    body.config.cal_range_min = static_cast<float>(config.angle_min);
    body.config.cal_range_max = static_cast<float>(config.angle_max);
    body.config.direction = static_cast<std::int8_t>(config.direction);
    std::memcpy(body.telem_inds, telem_.data(), sizeof body.telem_inds);
    body.max_telemetry_size = static_cast<std::uint16_t>(config.teensy_telemetry_max_size);

    char err[512]{};
    const auto resp = comms_.send_await(
        TEENSY_COMMAND_OPCODE_CMD_ROB_INITIALIZE_MOTOR,
        {reinterpret_cast<const char*>(&body), sizeof body},
        feather::time::from_seconds(0.4),
        {.data = reinterpret_cast<std::uint8_t*>(err), .size = sizeof err}
    );
    if (!resp) fail("timed out initializing motor");
    if (resp->opcode == TEENSY_COMMAND_OPCODE_RESP_ERROR) {
        fail(std::string("initialize failed: ") + std::string(err, resp->payload_size));
    }
    if (resp->opcode != TEENSY_COMMAND_OPCODE_RESP_SUCCESS) fail("initialize failed");
}

void RobstrideMotor::write_defaults() {
    // Disable, then put the motor in operation mode with zero speed and the configured torque and acceleration.
    // Gains and limits that are not listed here are reset to the version's default.
    disable(true, true);

    const int ver = config.version;
    const auto def = [&](RobParamCode code) { return param_default(rob_all_params[code], ver); };
    const double current_limit = param_max(rob_all_params[ROB_PARAM_CODE_LIMIT_CUR], ver);
    const std::pair<RobParamCode, double> sets[] = {
        {ROB_PARAM_CODE_CUR_KP, def(ROB_PARAM_CODE_CUR_KP)},
        {ROB_PARAM_CODE_CUR_KI, def(ROB_PARAM_CODE_CUR_KI)},
        {ROB_PARAM_CODE_CUR_FILT_GAIN, def(ROB_PARAM_CODE_CUR_FILT_GAIN)},
        {ROB_PARAM_CODE_SPD_KI, def(ROB_PARAM_CODE_SPD_KI)},
        {ROB_PARAM_CODE_SPD_FILT_GAIN, def(ROB_PARAM_CODE_SPD_FILT_GAIN)},
        {ROB_PARAM_CODE_ADD_OFFSET, def(ROB_PARAM_CODE_ADD_OFFSET)},
        {ROB_PARAM_CODE_EPSCAN_TIME, def(ROB_PARAM_CODE_EPSCAN_TIME)},
        {ROB_PARAM_CODE_CAN_TIMEOUT, def(ROB_PARAM_CODE_CAN_TIMEOUT)},
        {ROB_PARAM_CODE_ZERO_STA, def(ROB_PARAM_CODE_ZERO_STA)},
        {ROB_PARAM_CODE_DAMPER, def(ROB_PARAM_CODE_DAMPER)},
        {ROB_PARAM_CODE_RUN_MODE, ROB_RUN_MODE_OPERATION},
        {ROB_PARAM_CODE_VEL_MAX, 0.0},
        {ROB_PARAM_CODE_ACC_RAD, config.max_acceleration},
        {ROB_PARAM_CODE_ACC_SET, config.max_acceleration},
        {ROB_PARAM_CODE_LIMIT_TORQUE, config.max_torque},
        {ROB_PARAM_CODE_IQ_REF, current_limit},
        {ROB_PARAM_CODE_LIMIT_CUR, current_limit},
        {ROB_PARAM_CODE_LOC_KP, config.loc_kp},
        {ROB_PARAM_CODE_SPD_KP, config.spd_kp},
    };

    const double start = stamp(ROB_PARAM_CODE_FB_MODE);
    for (const auto& set : sets) {
        write_param(set.first, set.second, false);
        pause_ms(kSpamMs);
    }
    pause_ms(15);
    for (int i = 0; i < NUM_RobParamCode; ++i) {
        if (!rob_all_params[i].readable) continue;
        read_param(static_cast<RobParamCode>(i), false);
        pause_ms(kSpamMs);
    }

    wait_until([&] {
        for (const auto& set : sets) {
            if (stamp(set.first) < start || std::fabs(value(set.first) - set.second) > kTolerance) return false;
        }
        return true;
    }, "[" + config.name + "] failed to write default parameters");
}

void RobstrideMotor::on_endpoint_found() {
    const double start = stamp(ROB_PARAM_CODE_FB_MODE);
    read_firmware_version(false);
    pause_ms(kSpamMs);
    read_device_id(false);
    pause_ms(kSpamMs);
    for (int i = 0; i < NUM_RobParamCode; ++i) {
        if (!rob_all_params[i].readable || rob_all_params[i].writable) continue;
        read_param(static_cast<RobParamCode>(i), false);
        pause_ms(kSpamMs);
    }
    trigger_feedback_frame();

    wait_until([&] {
        for (int i = 0; i < NUM_RobParamCode; ++i) {
            if (!rob_all_params[i].readable) continue;
            if (stamp(static_cast<RobParamCode>(i)) < start) return false;
        }
        return stamp(ROB_PARAM_CODE_FB_MODE) >= start;
    }, "[" + config.name + "] failed to read parameters");

    // Device id and firmware version are not readable parameters, so the wait above does not cover them.
    if (bits(ROB_PARAM_CODE_FIRMWARE_VERSION) == 0) read_firmware_version(true);
    const std::uint64_t version = bits(ROB_PARAM_CODE_FIRMWARE_VERSION);
    std::uint8_t raw[8];
    std::memcpy(raw, &version, sizeof raw);
    if (raw[4] != static_cast<std::uint8_t>(config.version)) {
        fail("firmware version " + std::to_string(raw[4]) + " does not match config version " + std::to_string(config.version));
    }

    write_defaults();
    initialized = true;
}

void RobstrideMotor::enable(bool block) {
    const RobEnablePayload body{motor_id(), can_bus()};
    const double mode = ROB_MOTOR_MODE_RUN;
    if (!send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_ENABLE, &body, sizeof body, block, ROB_PARAM_CODE_FB_MODE, block ? &mode : nullptr))
        fail("enable failed");
}

void RobstrideMotor::disable(bool clear_faults, bool block) {
    const RobDisablePayload body{motor_id(), can_bus(), clear_faults};
    const double mode = ROB_MOTOR_MODE_RESET;
    if (!send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_DISABLE, &body, sizeof body, block, ROB_PARAM_CODE_FB_MODE, block ? &mode : nullptr))
        fail("disable failed");
}

void RobstrideMotor::zero_position(bool block) {
    const RobZeroPositionPayload body{motor_id(), can_bus()};
    if (!send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_ZERO_POSITION, &body, sizeof body, block, ROB_PARAM_CODE_FB_MODE))
        fail("zero position failed");
}

void RobstrideMotor::set_active_reporting(bool active, int scan_time_ms, bool block) {
    if (scan_time_ms < 10 || scan_time_ms > 327680) fail("scan time must be between 10ms and 327680ms");
    // For the motor, 1 = 10ms, and each +1 adds 5ms.
    write_param(ROB_PARAM_CODE_EPSCAN_TIME, scan_time_ms / 5 - 1, block, false, block);
    const RobSetActiveReportingPayload body{motor_id(), can_bus(), active};
    if (!send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_SET_ACTIVE_REPORTING, &body, sizeof body, block, ROB_PARAM_CODE_FB_MODE))
        fail("set active reporting failed");
}

std::optional<std::uint64_t> RobstrideMotor::read_device_id(bool block) {
    if (!command_read(ROB_PARAM_CODE_DEVICE_ID, block)) fail("read device id failed");
    if (!block) return std::nullopt;
    return bits(ROB_PARAM_CODE_DEVICE_ID);
}

std::uint64_t RobstrideMotor::device_id() {
    const std::uint64_t id = bits(ROB_PARAM_CODE_DEVICE_ID);
    if (id != 0) return id;
    return *read_device_id(true);
}

std::optional<std::string> RobstrideMotor::read_firmware_version(bool block) {
    const RobGetFirmwareVersionPayload body{motor_id(), can_bus()};
    if (!send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_GET_FIRMWARE_VERSION, &body, sizeof body, block, ROB_PARAM_CODE_FIRMWARE_VERSION))
        fail("read firmware version failed");
    if (!block) return std::nullopt;
    return format_firmware(bits(ROB_PARAM_CODE_FIRMWARE_VERSION));
}

std::string RobstrideMotor::firmware_version() {
    const std::uint64_t version = bits(ROB_PARAM_CODE_FIRMWARE_VERSION);
    if (version != 0) return format_firmware(version);
    return *read_firmware_version(true);
}

std::optional<double> RobstrideMotor::read_param(RobParamCode param, bool block) {
    if (!command_read(param, block)) {
        fail(std::string("read ") + rob_all_params[param].param_name + " failed");
    }
    if (!block) return std::nullopt;
    return value(param);
}

double RobstrideMotor::read_param_await(RobParamCode param) {
    return *read_param(param, true);
}

void RobstrideMotor::write_param(RobParamCode param, double value, bool block, bool clamp, bool check) {
    const RobWriteParamPayload body{
        motor_id(), can_bus(), static_cast<std::uint8_t>(param), static_cast<float>(value), clamp
    };
    if (!send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_WRITE_PARAM, &body, sizeof body, block, ROB_PARAM_CODE_FB_MODE))
        fail(std::string("write ") + rob_all_params[param].param_name + " failed");
    if (check && !command_read(param, true, &value))
        fail(std::string("write check ") + rob_all_params[param].param_name + " failed");
}

void RobstrideMotor::trigger_feedback_frame(std::optional<double> update_hz, bool block) {
    if (update_hz.has_value() && *update_hz > 0) {
        const double now_s = static_cast<double>(feather::time::now()) * 1e-9;
        if (value(ROB_PARAM_CODE_FEEDBACK_TS) + 1.0 / *update_hz > now_s) return;
    }
    write_param(ROB_PARAM_CODE_CAN_TIMEOUT, 0, block);
}

void RobstrideMotor::set_run_mode(RobRunMode mode, bool block, bool check) {
    write_param(ROB_PARAM_CODE_RUN_MODE, mode, block, false, check);
}

void RobstrideMotor::set_limit_torque(double limit_torque, bool block, bool check) {
    write_param(ROB_PARAM_CODE_LIMIT_TORQUE, limit_torque, block, false, check);
}

void RobstrideMotor::set_target_position(
    double target_position, bool block, bool await_position, double await_timeout, bool check
) {
    write_param(ROB_PARAM_CODE_LOC_REF, target_position, block, false, check);
    if (await_position) await_param(ROB_PARAM_CODE_FB_POSITION, target_position, kTolerance, true, std::nullopt, await_timeout);
}

void RobstrideMotor::set_target_velocity(double target_velocity, bool block, bool check) {
    const double age_s = static_cast<double>(feather::time::now()) * 1e-9 - stamp(ROB_PARAM_CODE_RUN_MODE);
    if (age_s > 0.5) read_param_await(ROB_PARAM_CODE_RUN_MODE);

    const auto mode = static_cast<RobRunMode>(value(ROB_PARAM_CODE_RUN_MODE));
    if (mode == ROB_RUN_MODE_OPERATION) fail("cannot set target velocity in operation mode");
    if (mode == ROB_RUN_MODE_CURRENT) fail("cannot set target velocity in current mode");
    if (mode == ROB_RUN_MODE_POSITION_PP) write_param(ROB_PARAM_CODE_VEL_MAX, std::fabs(target_velocity), block, false, check);
    else if (mode == ROB_RUN_MODE_VELOCITY) write_param(ROB_PARAM_CODE_SPD_REF, target_velocity, block, false, check);
    else fail("invalid run mode");
}

void RobstrideMotor::set_target_acceleration(double target_acceleration, bool block, bool check) {
    write_param(ROB_PARAM_CODE_ACC_RAD, target_acceleration, block, false, check);
    write_param(ROB_PARAM_CODE_ACC_SET, target_acceleration, block, false, check);
}

void RobstrideMotor::set_target_current(double target_current, bool block, bool check) {
    write_param(ROB_PARAM_CODE_IQ_REF, target_current, block, false, check);
    write_param(ROB_PARAM_CODE_LIMIT_CUR, target_current, block, false, check);
}

void RobstrideMotor::operation_command(double torque, double angle, double velocity, double kp, double kd, bool clamp) {
    const RobOperationControlPayload body{
        motor_id(), can_bus(), static_cast<float>(torque), static_cast<float>(angle), static_cast<float>(velocity),
        static_cast<float>(kp), static_cast<float>(kd), clamp
    };
    if (!send_ok(TEENSY_COMMAND_OPCODE_CMD_ROB_OPERATION_CONTROL, &body, sizeof body, false, ROB_PARAM_CODE_FB_MODE))
        fail("operation command failed");
}

double RobstrideMotor::await_param(
    RobParamCode param, double expect, double abs_tol, bool check_errors, std::optional<double> torque_limit, double timeout
) {
    const feather_time_t deadline = feather::time::now() + feather::time::from_seconds(timeout);
    double last = value(param);
    const bool feedback = rob_all_params[param].is_feedback_param;
    while (feather::time::now() < deadline) {
        // A feedback read already requests one frame. Requesting another would consume it before the wait starts.
        if ((check_errors || torque_limit.has_value()) && !feedback) trigger_feedback_frame();
        if (!command_read(param, true)) {
            pause_ms(10);
            continue;
        }
        if (check_errors) {
            const unsigned errs = static_cast<unsigned>(value(ROB_PARAM_CODE_FB_ERRORS));
            if (errs != 0) fail("errors during wait for " + std::string(rob_all_params[param].param_name) + ": " + error_names(errs));
        }
        if (torque_limit.has_value() && std::fabs(value(ROB_PARAM_CODE_FB_TORQUE)) > *torque_limit) {
            fail("torque limit exceeded during wait for " + std::string(rob_all_params[param].param_name));
        }
        last = value(param);
        if (std::fabs(last - expect) < abs_tol) return last;
        pause_ms(10);
    }
    fail(
        "failed to reach " + std::string(rob_all_params[param].param_name) + " = " + std::to_string(expect) +
        " within " + std::to_string(abs_tol) + ". Last value: " + std::to_string(last)
    );
}

std::pair<double, double> RobstrideMotor::command_angle_range() const {
    if (calibrated_range_) return *calibrated_range_;
    return {config.angle_min, config.angle_max};
}

void RobstrideMotor::set_calibrated_angle_range(double minimum, double maximum) {
    if (!(minimum < maximum)) throw std::invalid_argument(config.name + ": calibrated range is empty");
    calibrated_range_ = std::make_pair(minimum, maximum);
}

void RobstrideMotor::set_feedback_warnings(bool enabled) {
    feedback_warnings_ = enabled;
}

void RobstrideMotor::check_warn_on_feedback_delay(RobParamCode param) {
    if (!feedback_warnings_) return;

    const double now_s = static_cast<double>(feather::time::now()) * 1e-9;
    const double threshold = config.feedback_delay_threshold;
    const double interval = config.feedback_delay_warning_interval;
    double stamp_s = 0;
    const bool have = try_stamp(param, &stamp_s);
    if (have && now_s - stamp_s <= threshold) return;
    if (last_feedback_warning_s_ != 0.0 && now_s - last_feedback_warning_s_ < interval) return;

    if (!have) {
        WARNING(
            "Actuator %s has feedback delay for parameter %s (threshold: %gs): Parameter is None",
            config.name.c_str(), rob_all_params[param].param_name, threshold
        );
    } else {
        WARNING(
            "Actuator %s has feedback delay for parameter %s (threshold: %gs): Delay: %gs",
            config.name.c_str(), rob_all_params[param].param_name, threshold, now_s - stamp_s
        );
    }
    last_feedback_warning_s_ = now_s;
}

bool RobstrideMotor::probe_endpoint(int endpoint, double timeout) {
    config.endpoint = endpoint;
    const RobReadParamPayload body{motor_id(), can_bus(), ROB_PARAM_CODE_RUN_MODE};
    const bool found = send_ok(
        TEENSY_COMMAND_OPCODE_CMD_ROB_READ_PARAM, &body, sizeof body, true, ROB_PARAM_CODE_RUN_MODE, nullptr, timeout, 2
    );
    if (!found) config.endpoint = 255;
    return found;
}

}  // namespace feather::robot
