#pragma once

// Host-side EZMotion motor. Registers one telemetry channel per parameter in the constructor, so construct every
// motor before Telemetry::initialize(). init() talks to the Teensy and requires telemetry to already be initialized.
// Blocking reads wait for a newer telemetry sample, so a telemetry poll has to be running.

#include "qdefs/ezmotion_qdef.hpp"
#include "src/robot/comms/comms_manager.hpp"
#include "src/robot/telemetry.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace feather::robot {

// Same bit positions as EZMotionError in ezmotion.py.
enum class EZMotionError : int {
    OCP = 0,
    ULVO = 1,
    LOCK = 2,
    VIN_LIMIT = 3,
    OVERLOAD = 4,
    POS_LIMIT = 5,
    SENSOR_ERR = 6,
    OVER_VOLTAGE = 9,
};

struct EZMotionMotorConfig {
    std::string name;
    int id = 0;
    int endpoint = 255;  // CAN bus. 255 means it has not been found yet.
    double calibration_timeout = 60.0;
    int teensy_telemetry_max_size = 300;
    int target_velocity = EZ_MOTION_ENCODER_COUNTS_PER_ROTATION * 2;
    int max_velocity = EZ_MOTION_ENCODER_COUNTS_PER_ROTATION * 50;
    int target_acceleration = EZ_MOTION_ENCODER_COUNTS_PER_ROTATION * 50;
    int target_deceleration = EZ_MOTION_ENCODER_COUNTS_PER_ROTATION * 50;
    int homing_torque = 1000;
    int homing_offset = EZ_MOTION_ENCODER_COUNTS_PER_ROTATION * 2;
    int homing_speed = EZ_MOTION_ENCODER_COUNTS_PER_ROTATION * 5;
    int homing_acceleration = EZ_MOTION_ENCODER_COUNTS_PER_ROTATION * 50;
    double homing_timeout = 60.0;
    double teensy_message_timeout = 0.1;
    double message_block_timeout = 0.05;
    int message_send_attempts = 5;
    bool load_calibration = false;
    bool brake_enable = true;
    int brake_stop_delay_ms = 500;
    int brake_off_speed_rpm = 50;
    int brake_off_delay_ms = 100;
};

class EZMotionMotor {
public:
    EZMotionMotor(const EZMotionMotorConfig& config, feather::comms::CommsManager& comms, Telemetry& telemetry);

    void init(bool allow_reinit = false);
    void on_endpoint_found();

    // Homes against a torque stop. The offset is positive for TORQUE_UP and negative for TORQUE_DOWN.
    void calibrate(EZMotionHomingMethod homing_method = EZ_MOTION_HOMING_METHOD_TORQUE_UP);

    // Move to `position` encoder counts. A halted move latches the new target before the halt is cleared.
    void go_to(
        std::int64_t position, std::optional<std::int64_t> velocity = std::nullopt,
        std::optional<std::int64_t> acceleration = std::nullopt, std::optional<std::int64_t> deceleration = std::nullopt,
        bool halted = false
    );

    void enable();
    void disable();
    void halt();
    void hold_position();
    void reset_faults();

    void set_operation_mode(EZMotionControlMode mode);
    void set_target_position(std::int64_t target_position, bool transition_now = true);
    void set_target_velocity(std::int64_t target_velocity);
    void set_target_acceleration(std::int64_t target_acceleration);
    void set_target_deceleration(std::int64_t target_deceleration);

    // CONTROL_WORD is rejected. `verify` reads the parameter back and requires an exact match.
    void write_param(EZMotionParamCode param, std::int64_t value, bool verify = false);
    std::optional<std::int64_t> read_param(EZMotionParamCode param, bool block = true);
    std::int64_t read_param_await(EZMotionParamCode param);
    void read_state();

    // Stores the node id and voltage limits in the drive. Do not use unless you know what you are doing.
    void set_admin_params(int node_id, int voltage_upper, int voltage_lower);

    bool has_error();
    std::vector<EZMotionError> get_errors();

    // Point this motor at `endpoint` and try to read its status word. Clears the endpoint if that fails.
    bool probe_endpoint(int endpoint, double timeout = 0.05);

    EZMotionMotorConfig config;
    bool initialized = false;
    bool calibrated = false;
    bool in_calibration = false;

private:
    // quick_stop stays set. The other bits start clear.
    struct Word {
        bool enable = false;
        bool on = false;
        bool halt = false;
        bool transition = false;
        bool fault_reset = false;
        bool power_off = false;
    };

    feather::comms::CommsManager& comms_;
    Telemetry& telemetry_;
    std::uint16_t telem_[NUM_EZMotionParamCode]{};

    std::uint8_t motor_id() const;
    std::uint8_t can_bus() const;
    std::int64_t ivalue(EZMotionParamCode param) const;
    double stamp(EZMotionParamCode param) const;
    void command(TeensyCommandOpcode opcode, const void* body, std::size_t size, const char* what);
    bool wait_stamp(EZMotionParamCode param, double start, double timeout_s) const;
    void control(Word w);
    void transition_to_new_position();
    [[noreturn]] void fail(const std::string& msg) const;
};

}  // namespace feather::robot
