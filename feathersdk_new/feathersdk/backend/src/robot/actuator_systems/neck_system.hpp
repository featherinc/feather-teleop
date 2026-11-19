#pragma once

// Two-motor neck. Pitch and yaw are Robstride motors in position mode. Construct the neck before
// Telemetry::initialize(): each motor registers its telemetry channels in its constructor.
//
// Hard-stop calibration is still Python-only. recalibrate() does not move the motors, and initialize()
// enables position mode on the configured angle range. The control thread then repeats the commanded
// yaw and pitch and requests feedback. Home the motors before calling initialize().

#include "src/robot/actuators/robstride.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace feather::robot {

struct NeckSystemConfig {
    std::string module_name = "neck_system";
    double operating_frequency = 30.0;
};

struct TwoDOFRobstrideNeckSystemConfig : NeckSystemConfig {
    RobstrideMotorConfig pitch_motor;
    RobstrideMotorConfig yaw_motor;
};

class TwoDOFRobstrideNeckSystem {
public:
    TwoDOFRobstrideNeckSystem(
        const TwoDOFRobstrideNeckSystemConfig& cfg,
        feather::comms::CommsManager& comms,
        Telemetry& telemetry
    );
    ~TwoDOFRobstrideNeckSystem();

    TwoDOFRobstrideNeckSystem(const TwoDOFRobstrideNeckSystem&) = delete;
    TwoDOFRobstrideNeckSystem& operator=(const TwoDOFRobstrideNeckSystem&) = delete;

    // Enable both motors in position mode, command the current yaw and pitch, and start the control thread.
    // Motors must already have an endpoint (probe and on_endpoint_found). `load_calibration` is logged and ignored.
    void initialize(std::optional<bool> load_calibration = std::nullopt);

    // Does not calibrate. Logs that hard-stop calibration is not in this backend.
    void recalibrate(std::optional<bool> load_calibration = std::nullopt);

    // Angles the control thread sends, in radians. Safe to call from another thread.
    void set_position(double pitch, double yaw);

    // Lower the pitch to 0 when it is in position mode, then disable both motors and stop the control thread.
    void abort();

    // The control thread warns about stale frames only after this is set, matching the Python robot flag.
    void set_robot_initialized(bool ready);

    TwoDOFRobstrideNeckSystemConfig config;
    // Yaw is declared first so its telemetry channels are registered first, matching the Python constructor.
    RobstrideMotor yaw_motor;
    RobstrideMotor pitch_motor;

private:
    double pitch_ = 0;
    double yaw_ = 0;
    std::mutex command_mu_;

    std::atomic<bool> stop_{true};
    std::mutex sleep_mu_;
    std::condition_variable sleep_cv_;
    std::thread thread_;

    void start_thread();
    void stop_thread();
    void thread_main();
    void thread_step();
    void disable_motors();
};

}  // namespace feather::robot
