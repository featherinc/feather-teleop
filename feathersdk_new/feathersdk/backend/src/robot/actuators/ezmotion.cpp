#include "ezmotion.hpp"

#include "logging.hpp"
#include "system.hpp"

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace feather::robot {
namespace {

struct NamedError {
    EZMotionError bit;
    const char* name;
};

constexpr NamedError kErrors[] = {
    {EZMotionError::OCP, "OCP"},
    {EZMotionError::ULVO, "ULVO"},
    {EZMotionError::LOCK, "LOCK"},
    {EZMotionError::VIN_LIMIT, "VIN_LIMIT"},
    {EZMotionError::OVERLOAD, "OVERLOAD"},
    {EZMotionError::POS_LIMIT, "POS_LIMIT"},
    {EZMotionError::SENSOR_ERR, "SENSOR_ERR"},
    {EZMotionError::OVER_VOLTAGE, "OVER_VOLTAGE"},
};

std::int64_t as_int(DataType dtype, const std::uint8_t* buf) {
    switch (dtype) {
        case DATA_TYPE_U8:
        case DATA_TYPE_BOOL: return buf[0];
        case DATA_TYPE_I8: return static_cast<std::int8_t>(buf[0]);
        case DATA_TYPE_U16: { std::uint16_t v; std::memcpy(&v, buf, 2); return v; }
        case DATA_TYPE_I16: { std::int16_t v; std::memcpy(&v, buf, 2); return v; }
        case DATA_TYPE_U32: { std::uint32_t v; std::memcpy(&v, buf, 4); return v; }
        case DATA_TYPE_I32: { std::int32_t v; std::memcpy(&v, buf, 4); return v; }
        case DATA_TYPE_U64: { std::uint64_t v; std::memcpy(&v, buf, 8); return static_cast<std::int64_t>(v); }
        case DATA_TYPE_I64: { std::int64_t v; std::memcpy(&v, buf, 8); return v; }
        default: return 0;
    }
}

void validate_config(const EZMotionMotorConfig& cfg) {
    if (cfg.name.empty()) throw std::invalid_argument("EZMotion motor name is empty");
    if (cfg.id < 1 || cfg.id > 0x7F) throw std::invalid_argument(cfg.name + ": motor id out of range");
    if (cfg.teensy_telemetry_max_size <= 0 || cfg.teensy_telemetry_max_size > 0xFFFF) {
        throw std::invalid_argument(cfg.name + ": teensy telemetry size out of range");
    }
    if (cfg.target_velocity > cfg.max_velocity) {
        throw std::invalid_argument(cfg.name + ": target_velocity must be <= max_velocity");
    }
}

std::string error_names(std::int64_t bits) {
    std::string out;
    for (const auto& err : kErrors) {
        if ((bits & (1LL << static_cast<int>(err.bit))) == 0) continue;
        if (!out.empty()) out += ", ";
        out += err.name;
    }
    return out;
}

}  // namespace

EZMotionMotor::EZMotionMotor(
    const EZMotionMotorConfig& cfg, feather::comms::CommsManager& comms, Telemetry& telemetry
) : config(cfg), comms_(comms), telemetry_(telemetry) {
    validate_config(cfg);
    if (telemetry.initialized()) throw std::runtime_error(cfg.name + ": telemetry is already initialized");

    for (int i = 0; i < NUM_EZMotionParamCode; ++i) {
        const auto cap = ezmotion_all_params[i].max_telemetry_size == 0 ? 1 : ezmotion_all_params[i].max_telemetry_size;
        const int idx = telemetry.add_telemetry(cap, ezmotion_all_params[i].dtype);
        if (idx < 0 || idx > 0xFFFF) throw std::runtime_error(config.name + ": failed to add telemetry");
        telem_[i] = static_cast<std::uint16_t>(idx);
    }
}

std::uint8_t EZMotionMotor::motor_id() const {
    return static_cast<std::uint8_t>(config.id);
}

std::uint8_t EZMotionMotor::can_bus() const {
    if (config.endpoint < 0 || config.endpoint > 254) throw std::runtime_error(config.name + ": no CAN endpoint");
    return static_cast<std::uint8_t>(config.endpoint);
}

std::int64_t EZMotionMotor::ivalue(EZMotionParamCode param) const {
    std::uint8_t buf[8]{};
    const DataType dtype = ezmotion_all_params[param].dtype;
    const std::size_t n = dtype_size(dtype);
    if (n == 0 || n > sizeof buf || !telemetry_.latest(telem_[param], buf, n)) return 0;
    return as_int(dtype, buf);
}

double EZMotionMotor::stamp(EZMotionParamCode param) const {
    const EZMotionParamCode ts = ezmotion_all_params[param].timestamp_pcode;
    const EZMotionParamCode code = ts == NUM_EZMotionParamCode ? param : ts;
    double out = 0;
    if (!telemetry_.latest(telem_[code], &out, sizeof out)) return 0;
    return out;
}

void EZMotionMotor::fail(const std::string& msg) const {
    throw std::runtime_error("[" + config.name + "] " + msg);
}

void EZMotionMotor::command(TeensyCommandOpcode opcode, const void* body, std::size_t size, const char* what) {
    char err[512]{};
    const auto resp = comms_.send_await(
        opcode, {static_cast<const char*>(body), size}, feather::time::from_seconds(config.teensy_message_timeout),
        {.data = reinterpret_cast<std::uint8_t*>(err), .size = sizeof err}
    );
    if (!resp) fail(std::string(what) + " timed out");
    if (resp->opcode == TEENSY_COMMAND_OPCODE_RESP_ERROR) {
        const std::size_t n = resp->payload_size < sizeof err ? resp->payload_size : sizeof err;
        fail(std::string(what) + " failed: " + std::string(err, n));
    }
    if (resp->opcode != TEENSY_COMMAND_OPCODE_RESP_SUCCESS) fail(std::string(what) + " failed");
}

bool EZMotionMotor::wait_stamp(EZMotionParamCode param, double start, double timeout_s) const {
    const feather_time_t deadline = feather::time::now() + feather::time::from_seconds(timeout_s);
    while (feather::time::now() < deadline) {
        if (stamp(param) > start) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

void EZMotionMotor::control(Word w) {
    EZMotionSetControlWordPayload body{};
    body.actuator_id = motor_id();
    body.can_bus = can_bus();
    body.enable_operation = w.enable;
    body.switch_on = w.on;
    body.quick_stop = true;
    body.halt = w.halt;
    body.transition_position = w.transition;
    body.fault_reset = w.fault_reset;
    body.disable_power = w.power_off;
    command(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_SET_CONTROL_WORD, &body, sizeof body, "set control word");
}

void EZMotionMotor::init(bool allow_reinit) {
    if (!telemetry_.initialized()) fail("telemetry is not initialized");

    EZMotionMotorInitPayload body{};
    body.actuator_id = motor_id();
    body.allow_reinit = allow_reinit;
    std::memcpy(body.telem_inds, telem_, sizeof body.telem_inds);
    body.max_telemetry_size = static_cast<std::uint16_t>(config.teensy_telemetry_max_size);
    command(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_INITIALIZE_MOTOR, &body, sizeof body, "initialize");
}

void EZMotionMotor::write_param(EZMotionParamCode param, std::int64_t value, bool verify) {
    if (param == EZ_MOTION_PARAM_CODE_CONTROL_WORD) fail("Cannot write CONTROL_WORD; use the control word command instead");
    const char* name = ezmotion_all_params[param].param_name;
    const EZMotionWriteParamPayload body{motor_id(), can_bus(), static_cast<std::uint8_t>(param), value};
    command(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_WRITE_PARAM, &body, sizeof body, name);
    if (!verify) return;
    const std::int64_t readback = read_param_await(param);
    if (readback != value) fail(std::string("Wrote ") + name + "=" + std::to_string(value) + " but read back " + std::to_string(readback));
}

std::optional<std::int64_t> EZMotionMotor::read_param(EZMotionParamCode param, bool block) {
    const EZMotionReadParamPayload body{motor_id(), can_bus(), static_cast<std::uint8_t>(param)};
    const double start = stamp(param);
    for (int attempt = 0; attempt < config.message_send_attempts; ++attempt) {
        command(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_READ_PARAM, &body, sizeof body, ezmotion_all_params[param].param_name);
        if (!block) return std::nullopt;
        if (wait_stamp(param, start, config.message_block_timeout)) return ivalue(param);
    }
    fail(
        std::string("Failed to read ") + ezmotion_all_params[param].param_name + " after " +
        std::to_string(config.message_send_attempts) + " attempts"
    );
}

std::int64_t EZMotionMotor::read_param_await(EZMotionParamCode param) {
    return *read_param(param, true);
}

void EZMotionMotor::set_admin_params(int node_id, int voltage_upper, int voltage_lower) {
    const EZMotionSetAdminParamsPayload body{
        motor_id(), can_bus(), static_cast<std::uint8_t>(node_id), static_cast<std::uint16_t>(voltage_upper),
        static_cast<std::uint16_t>(voltage_lower)
    };
    command(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_SET_ADMIN_PARAMS, &body, sizeof body, "set admin params");
}

void EZMotionMotor::enable() { control({.enable = true, .on = true}); }
void EZMotionMotor::disable() { control({}); }
void EZMotionMotor::halt() { control({.enable = true, .on = true, .halt = true}); }

void EZMotionMotor::hold_position() {
    set_target_position(read_param_await(EZ_MOTION_PARAM_CODE_POSITION), false);
    control({.enable = true, .on = true, .halt = true, .transition = true});
    control({.enable = true, .on = true, .transition = true});
}

void EZMotionMotor::reset_faults() { control({.fault_reset = true}); }
void EZMotionMotor::transition_to_new_position() { control({.enable = true, .on = true, .transition = true}); }

void EZMotionMotor::set_operation_mode(EZMotionControlMode mode) {
    write_param(EZ_MOTION_PARAM_CODE_OPERATION_MODE, mode);
}

void EZMotionMotor::set_target_position(std::int64_t target_position, bool transition_now) {
    write_param(EZ_MOTION_PARAM_CODE_TARGET_POSITION, target_position);
    if (transition_now) transition_to_new_position();
}

void EZMotionMotor::set_target_velocity(std::int64_t target_velocity) {
    write_param(EZ_MOTION_PARAM_CODE_PROFILE_VELOCITY, target_velocity);
}

void EZMotionMotor::set_target_acceleration(std::int64_t target_acceleration) {
    write_param(EZ_MOTION_PARAM_CODE_PROFILE_ACCELERATION, target_acceleration);
}

void EZMotionMotor::set_target_deceleration(std::int64_t target_deceleration) {
    write_param(EZ_MOTION_PARAM_CODE_PROFILE_DECELERATION, target_deceleration);
}

void EZMotionMotor::read_state() {
    const EZMotionParamCode params[] = {
        EZ_MOTION_PARAM_CODE_POSITION, EZ_MOTION_PARAM_CODE_ERROR_STATUS, EZ_MOTION_PARAM_CODE_VELOCITY,
        EZ_MOTION_PARAM_CODE_TORQUE, EZ_MOTION_PARAM_CODE_CURRENT, EZ_MOTION_PARAM_CODE_TEMPERATURE,
    };
    for (const auto param : params) read_param(param);
}

bool EZMotionMotor::has_error() {
    return (read_param_await(EZ_MOTION_PARAM_CODE_STATUS_WORD) & (1 << 3)) != 0;
}

std::vector<EZMotionError> EZMotionMotor::get_errors() {
    const std::int64_t bits = read_param_await(EZ_MOTION_PARAM_CODE_ERROR_STATUS);
    std::vector<EZMotionError> out;
    for (const auto& err : kErrors) {
        if ((bits & (1LL << static_cast<int>(err.bit))) != 0) out.push_back(err.bit);
    }
    return out;
}

void EZMotionMotor::on_endpoint_found() {
    if (has_error()) fail("Motor has errors, cannot initialize: " + error_names(read_param_await(EZ_MOTION_PARAM_CODE_ERROR_STATUS)));
    read_state();
    read_param(EZ_MOTION_PARAM_CODE_SERIAL_NUMBER);
    read_param(EZ_MOTION_PARAM_CODE_FIRMWARE_VERSION);
    read_param(EZ_MOTION_PARAM_CODE_ENCODER_PART_NUMBER);
    initialized = true;
}

void EZMotionMotor::calibrate(EZMotionHomingMethod homing_method) {
    if (homing_method != EZ_MOTION_HOMING_METHOD_TORQUE_UP && homing_method != EZ_MOTION_HOMING_METHOD_TORQUE_DOWN) {
        fail("Unsupported homing method");
    }
    if (has_error()) fail("Motor has errors, cannot calibrate: " + error_names(read_param_await(EZ_MOTION_PARAM_CODE_ERROR_STATUS)));

    const int sign = homing_method == EZ_MOTION_HOMING_METHOD_TORQUE_UP ? 1 : -1;
    in_calibration = true;
    calibrated = false;
    control({.power_off = true});
    disable();
    set_operation_mode(EZ_MOTION_CONTROL_MODE_HOMING);
    enable();
    write_param(EZ_MOTION_PARAM_CODE_HOMING_METHOD, homing_method);
    write_param(EZ_MOTION_PARAM_CODE_HOMING_TORQUE, config.homing_torque);
    write_param(EZ_MOTION_PARAM_CODE_HOMING_SPEED_OTHER, config.homing_speed);
    write_param(EZ_MOTION_PARAM_CODE_HOMING_SPEED_ZERO, config.homing_speed);
    write_param(EZ_MOTION_PARAM_CODE_HOMING_ACCELERATION, config.homing_acceleration);
    write_param(EZ_MOTION_PARAM_CODE_HOMING_OFFSET, static_cast<std::int64_t>(sign) * config.homing_offset);
    transition_to_new_position();

    const feather_time_t deadline = feather::time::now() + feather::time::from_seconds(config.homing_timeout);
    while (feather::time::now() < deadline) {
        if ((read_param_await(EZ_MOTION_PARAM_CODE_STATUS_WORD) & (1 << 12)) != 0) {
            INFO(
                "Calibration complete for %s. Zeroed position: %lld", config.name.c_str(),
                static_cast<long long>(read_param_await(EZ_MOTION_PARAM_CODE_POSITION))
            );
            calibrated = true;
            in_calibration = false;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    in_calibration = false;
    fail("Failed to reach homing position after " + std::to_string(config.homing_timeout) + " seconds");
}

void EZMotionMotor::go_to(
    std::int64_t position, std::optional<std::int64_t> velocity, std::optional<std::int64_t> acceleration,
    std::optional<std::int64_t> deceleration, bool halted
) {
    set_operation_mode(EZ_MOTION_CONTROL_MODE_POSITION);
    set_target_velocity(velocity.value_or(config.target_velocity));
    set_target_acceleration(acceleration.value_or(config.target_acceleration));
    set_target_deceleration(deceleration.value_or(config.target_deceleration));
    set_target_position(position, false);
    if (halted) {
        control({.enable = true, .on = true, .halt = true, .transition = true});
        control({.enable = true, .on = true, .transition = true});
        return;
    }
    // Enabling clears a halt, and a cleared halt resumes the move the halt interrupted until the new target latches.
    enable();
    transition_to_new_position();
}

bool EZMotionMotor::probe_endpoint(int endpoint, double timeout) {
    config.endpoint = endpoint;
    const EZMotionReadParamPayload body{motor_id(), can_bus(), EZ_MOTION_PARAM_CODE_STATUS_WORD};
    const double start = stamp(EZ_MOTION_PARAM_CODE_STATUS_WORD);
    command(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_READ_PARAM, &body, sizeof body, "probe");
    if (wait_stamp(EZ_MOTION_PARAM_CODE_STATUS_WORD, start, timeout)) return true;
    config.endpoint = 255;
    return false;
}

}  // namespace feather::robot
