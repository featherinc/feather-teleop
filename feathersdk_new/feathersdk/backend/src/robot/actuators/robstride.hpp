#pragma once

// Host-side Robstride motor. Registers one telemetry channel per parameter in the constructor, so construct every
// motor before Telemetry::initialize(). init() talks to the Teensy and requires telemetry to already be initialized.

#include "qdefs/robstride_qdef.hpp"
#include "src/robot/comms/comms_manager.hpp"
#include "src/robot/telemetry.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace feather::robot {

struct RobstrideMotorConfig {
    std::string name;
    int id = 0;
    int endpoint = 255;  // CAN bus. 255 means it has not been found yet.
    RobVersion version = ROB_VERSION_RS01;
    int direction = 1;  // 1 clockwise, -1 counterclockwise, shaft pointing toward you.
    double max_velocity = 1.1;
    double max_torque = 1.1;
    double max_acceleration = 50.0;
    double angle_min = -6.283185307179586;
    double angle_max = 6.283185307179586;
    int teensy_telemetry_max_size = 300;
    double message_block_timeout = 0.3;
    int message_send_attempts = 2;
    double loc_kp = 4.0;
    double spd_kp = 2.0;
    double max_jerk = 100.0;
    double feedback_delay_threshold = 0.1;
    double feedback_delay_warning_interval = 10.0;
};

class RobstrideMotor {
public:
    RobstrideMotor(const RobstrideMotorConfig& config, feather::comms::CommsManager& comms, Telemetry& telemetry);

    // Tell the Teensy about this motor. Uses the configured angle range and a zero homing offset.
    void init(bool allow_reinit = false);

    // Read the motor, check its version, and write the default parameters.
    void on_endpoint_found();

    void enable(bool block = false);
    void disable(bool clear_faults = false, bool block = false);
    void zero_position(bool block = false);
    void set_active_reporting(bool active, int scan_time_ms = 10, bool block = false);

    std::uint64_t device_id();
    std::optional<std::uint64_t> read_device_id(bool block = false);
    std::string firmware_version();
    std::optional<std::string> read_firmware_version(bool block = false);

    std::optional<double> read_param(RobParamCode param, bool block = false);
    double read_param_await(RobParamCode param);
    void write_param(RobParamCode param, double value, bool block = false, bool clamp = false, bool check = false);

    // update_hz skips the request when the last feedback is newer than 1/update_hz seconds.
    void trigger_feedback_frame(std::optional<double> update_hz = std::nullopt, bool block = false);
    void set_run_mode(RobRunMode mode, bool block = false, bool check = false);
    void set_limit_torque(double limit_torque, bool block = false, bool check = false);
    void set_target_position(
        double target_position, bool block = false, bool await_position = false, double await_timeout = 1.0,
        bool check = false
    );
    void set_target_velocity(double target_velocity, bool block = false, bool check = false);
    void set_target_acceleration(double target_acceleration, bool block = false, bool check = false);
    void set_target_current(double target_current, bool block = false, bool check = false);
    void operation_command(double torque, double angle, double velocity, double kp, double kd, bool clamp = false);

    // torque_limit is checked when set. Returns the value once it is within abs_tol.
    double await_param(
        RobParamCode param, double value, double abs_tol, bool check_errors = false,
        std::optional<double> torque_limit = std::nullopt, double timeout = 1.0
    );

    // Point this motor at `endpoint` and try to read its run mode. Clears the endpoint if that fails.
    bool probe_endpoint(int endpoint, double timeout = 0.05);

    // Newest sample. False when that channel has never been written.
    bool try_latest(RobParamCode param, double* out) const;
    // Timestamp channel for `param` (FEEDBACK_TS for a feedback parameter). False when it has never been written.
    bool try_stamp(RobParamCode param, double* out) const;

    // Calibrated hard-stop range once set_calibrated_angle_range() has been called, otherwise the configured range.
    std::pair<double, double> command_angle_range() const;
    void set_calibrated_angle_range(double minimum, double maximum);

    bool has_endpoint() const { return config.endpoint != 255; }

    // Warn at most once per feedback_delay_warning_interval. No-op until set_feedback_warnings(true), so startup
    // does not warn while the rest of the robot is still coming up.
    void set_feedback_warnings(bool enabled);
    void check_warn_on_feedback_delay(RobParamCode param);

    RobstrideMotorConfig config;
    bool initialized = false;

private:
    feather::comms::CommsManager& comms_;
    Telemetry& telemetry_;
    std::array<std::uint16_t, NUM_RobParamCode> telem_{};
    std::optional<std::pair<double, double>> calibrated_range_;
    bool feedback_warnings_ = false;
    double last_feedback_warning_s_ = 0;

    std::uint8_t motor_id() const;
    std::uint8_t can_bus() const;
    double value(RobParamCode param) const;
    std::uint64_t bits(RobParamCode param) const;
    double stamp(RobParamCode param) const;

    bool send_ok(
        TeensyCommandOpcode opcode, const void* body, std::size_t size, bool block, RobParamCode watch,
        const double* expect = nullptr, double timeout_s = -1, int attempts = -1
    );
    bool wait_for(RobParamCode watch, double start, const double* expect, double timeout_s) const;
    // Feedback params request a feedback frame. Device id uses its own command. Everything else is READ_PARAM.
    bool command_read(
        RobParamCode param, bool block, const double* expect = nullptr, double timeout_s = -1, int attempts = -1
    );
    [[noreturn]] void fail(const std::string& msg) const;
    void write_defaults();
};

}  // namespace feather::robot
