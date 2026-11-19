#include "neck_system.hpp"

#include "logging.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>

namespace feather::robot {

TwoDOFRobstrideNeckSystem::TwoDOFRobstrideNeckSystem(
    const TwoDOFRobstrideNeckSystemConfig& cfg, feather::comms::CommsManager& comms, Telemetry& telemetry
) : config(cfg), yaw_motor(cfg.yaw_motor, comms, telemetry), pitch_motor(cfg.pitch_motor, comms, telemetry) {
    if (!(config.operating_frequency > 0.0 && config.operating_frequency < 1000.0)) {
        throw std::invalid_argument(config.module_name + ": operating_frequency must be in (0, 1000)");
    }
    if (pitch_motor.config.name == yaw_motor.config.name) {
        throw std::invalid_argument(config.module_name + ": pitch and yaw motors have the same name");
    }
}

TwoDOFRobstrideNeckSystem::~TwoDOFRobstrideNeckSystem() {
    stop_thread();
}

void TwoDOFRobstrideNeckSystem::recalibrate(std::optional<bool> load_calibration) {
    const char* load = "default";
    if (load_calibration.has_value()) load = *load_calibration ? "true" : "false";
    WARNING(
        "%s: skipping calibration (load_calibration=%s). "
        "Robstride hard-stop calibration is not in the C++ backend; command ranges stay at the configured angle limits",
        config.module_name.c_str(), load
    );
}

void TwoDOFRobstrideNeckSystem::disable_motors() {
    RobstrideMotor* motors[] = {&yaw_motor, &pitch_motor};
    for (RobstrideMotor* motor : motors) {
        if (!motor->has_endpoint()) continue;
        try {
            motor->disable();
        } catch (const std::exception& ex) {
            ERROR("%s: disable failed: %s", motor->config.name.c_str(), ex.what());
        }
    }
}

void TwoDOFRobstrideNeckSystem::initialize(std::optional<bool> load_calibration) {
    try {
        recalibrate(load_calibration);
        double pitch = 0;
        double yaw = 0;
        {
            std::lock_guard<std::mutex> lock(command_mu_);
            pitch = pitch_;
            yaw = yaw_;
        }
        RobstrideMotor* motors[] = {&yaw_motor, &pitch_motor};
        const double positions[] = {yaw, pitch};
        for (int i = 0; i < 2; ++i) {
            motors[i]->set_run_mode(ROB_RUN_MODE_POSITION_PP, true, true);
            motors[i]->enable(true);
            motors[i]->set_target_velocity(motors[i]->config.max_velocity, true, true);
            motors[i]->set_target_acceleration(motors[i]->config.max_acceleration, true, true);
            motors[i]->set_target_position(positions[i], true, false, 1.0, true);
        }
        start_thread();
    } catch (...) {
        disable_motors();
        throw;
    }
}

void TwoDOFRobstrideNeckSystem::set_position(double pitch, double yaw) {
    std::lock_guard<std::mutex> lock(command_mu_);
    pitch_ = pitch;
    yaw_ = yaw;
}

void TwoDOFRobstrideNeckSystem::start_thread() {
    if (thread_.joinable()) {
        throw std::runtime_error("Thread already started for actuator system " + config.module_name);
    }
    stop_.store(false);
    thread_ = std::thread([this] { thread_main(); });
}

void TwoDOFRobstrideNeckSystem::stop_thread() {
    {
        std::lock_guard<std::mutex> lock(sleep_mu_);
        stop_.store(true);
    }
    sleep_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void TwoDOFRobstrideNeckSystem::thread_main() {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / config.operating_frequency));
    while (!stop_.load()) {
        const auto start = std::chrono::steady_clock::now();
        try {
            thread_step();
        } catch (const std::exception& ex) {
            ERROR("Error in thread \"%s_thread\": %s", config.module_name.c_str(), ex.what());
            return;
        }
        std::unique_lock<std::mutex> lock(sleep_mu_);
        sleep_cv_.wait_until(lock, start + period, [this] { return stop_.load(); });
    }
}

void TwoDOFRobstrideNeckSystem::thread_step() {
    double pitch = 0;
    double yaw = 0;
    {
        std::lock_guard<std::mutex> lock(command_mu_);
        pitch = pitch_;
        yaw = yaw_;
    }

    RobstrideMotor* motors[] = {&pitch_motor, &yaw_motor};
    const double positions[] = {pitch, yaw};
    for (int i = 0; i < 2; ++i) {
        if (!motors[i]->initialized) continue;
        motors[i]->set_target_position(positions[i], false);
        motors[i]->trigger_feedback_frame(config.operating_frequency, false);
        motors[i]->check_warn_on_feedback_delay(ROB_PARAM_CODE_FB_MODE);
    }
}

void TwoDOFRobstrideNeckSystem::abort() {
    // Stop repeating the last command before the pitch is lowered.
    stop_thread();
    try {
        double mode = 0;
        const bool have_mode = pitch_motor.try_latest(ROB_PARAM_CODE_RUN_MODE, &mode);
        if (pitch_motor.has_endpoint() && have_mode && std::lround(mode) == ROB_RUN_MODE_POSITION_PP) {
            pitch_motor.set_target_velocity(2.0, true, true);
            pitch_motor.set_target_position(0, true, false, 1.0, true);
            pitch_motor.await_param(ROB_PARAM_CODE_FB_POSITION, 0, 0.01, false, std::nullopt, 2.0);
        }
    } catch (...) {
        disable_motors();
        throw;
    }
    disable_motors();
}

void TwoDOFRobstrideNeckSystem::set_robot_initialized(bool ready) {
    pitch_motor.set_feedback_warnings(ready);
    yaw_motor.set_feedback_warnings(ready);
}

}  // namespace feather::robot
