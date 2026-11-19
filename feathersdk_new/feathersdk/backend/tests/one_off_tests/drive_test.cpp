// Drive the three base wheels, then stop.
//
// kForward picks the sign of the 0.5 rad/s host-frame wheel rate. kDryRun does
// the setup and the feedback poll, and skips that movement command.
//
// Motor ids, direction, version, and limits come from
// /feathersys/configs/systems/navigation_system_conf.py. Comms rates come from
// /feathersys/configs/infra/comms_manager_conf.py. SPD_REF is a direction
// parameter, so the Teensy multiplies the command by each motor's configured
// direction before the frame goes on the bus.
//
// Candump and actuator telemetry are recorded under one_off_tests/drive_test/.
// Build and run from backend/: make drive_test

#include "src/robot/actuators/robstride.hpp"
#include "src/robot/comms/comms_manager.hpp"
#include "src/robot/services/data_monitor.hpp"
#include "src/robot/telemetry.hpp"
#include "logging.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <thread>

#include <errno.h>
#include <sys/stat.h>

namespace {

constexpr bool kForward = true;
constexpr bool kDryRun = true;

constexpr double kDriveRadPerSec = 0.5;
constexpr double kDriveSeconds = 3.0;
constexpr int kFeedbackHz = 50;

// 256 KiB holds the whole run, so the writers do not seal or wrap while driving.
constexpr std::uint64_t kChunkBytes = 256 * 1024;
constexpr std::uint64_t kMaxFileBytes = sizeof(FthrFileHeader) + kChunkBytes;

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

std::string output_dir() {
    const std::string file = __FILE__;
    const auto slash = file.rfind('/');
    const std::string dir = (slash == std::string::npos ? std::string(".") : file.substr(0, slash)) + "/drive_test";
    if (mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) throw std::runtime_error("failed to create " + dir);
    return dir;
}

feather::robot::DataMonitorFileWriterConfig monitor_file(const std::string& path, const char* name) {
    feather::robot::DataMonitorFileWriterConfig cfg;
    cfg.path = path;
    cfg.data_name = name;
    cfg.format = FTHR_FILE_FORMAT_VARIABLE;
    cfg.chunk_size = kChunkBytes;
    cfg.max_file_size = kMaxFileBytes;
    return cfg;
}

// Drive motors from navigation_system_conf.py. Turning motors are not used.
feather::robot::RobstrideMotorConfig drive_motor(const char* name, int id, int direction) {
    feather::robot::RobstrideMotorConfig cfg;
    cfg.name = name;
    cfg.id = id;
    cfg.direction = direction;
    cfg.max_velocity = 10.47;
    cfg.max_torque = 5.5;
    cfg.max_acceleration = 30.0;
    return cfg;
}

// Fields left unset already match comms_manager_conf.py.
feather::comms::CommsManagerConfig comms_config() {
    feather::comms::CommsManagerConfig cfg;
    cfg.teensy.use_emulated_teensy = false;
    cfg.teensy.teensy_correct_clock_drift = false;
    cfg.teensy.max_can_rate = 3500.0;
    cfg.teensy.server_run_time_nanos = 500000;
    cfg.teensy.can_read_run_time_nanos = 500000;
    return cfg;
}

void stop_motors(feather::robot::RobstrideMotor* motors[], int n, bool& enabled) {
    if (!enabled) return;
    enabled = false;
    for (int i = 0; i < n; ++i) {
        try {
            motors[i]->set_target_velocity(0.0, true);
        } catch (const std::exception& ex) {
            std::fprintf(stderr, "%s\n", ex.what());
        }
        try {
            motors[i]->disable(true, true);
        } catch (const std::exception& ex) {
            std::fprintf(stderr, "%s\n", ex.what());
        }
    }
}

}  // namespace

int main() {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    feather::log::Config log_cfg;
    log_cfg.console_enabled = true;
    log_cfg.file_enabled = false;
    feather::log::init(log_cfg);

    try {
        const std::string dir = output_dir();
        feather::robot::DataMonitorServiceConfig mon_cfg;
        mon_cfg.candump_config = monitor_file(dir + "/candump.fthr", "candump");
        mon_cfg.telemetry_config = monitor_file(dir + "/telemetry.fthr", "telemetry");
        feather::robot::DataMonitorService monitor(mon_cfg);

        feather::robot::Telemetry telemetry;
        const auto comms_cfg = comms_config();
        feather::comms::CommsManager comms(comms_cfg, &telemetry, &monitor);

        feather::robot::RobstrideMotor center(drive_motor("drive_center", 0x73, -1), comms, telemetry);
        feather::robot::RobstrideMotor right(drive_motor("drive_right", 0x74, 1), comms, telemetry);
        feather::robot::RobstrideMotor left(drive_motor("drive_left", 0x75, -1), comms, telemetry);
        feather::robot::RobstrideMotor* motors[] = {&center, &right, &left};
        constexpr int kMotors = 3;
        bool enabled = false;

        if (!telemetry.initialize()) {
            std::fprintf(stderr, "telemetry initialize failed\n");
            return 1;
        }
        for (auto* motor : motors) motor->init();

        int rc = 0;
        try {
            const int buses = comms.num_can_lines();
            if (buses <= 0) throw std::runtime_error("Teensy reported no CAN buses");

            for (auto* motor : motors) {
                int bus = -1;
                for (int i = 0; i < buses; ++i) {
                    std::printf("probing %s (0x%02x) on CAN bus %d\n", motor->config.name.c_str(), motor->config.id, i);
                    if (motor->probe_endpoint(i)) {
                        bus = i;
                        break;
                    }
                }
                if (bus < 0) throw std::runtime_error(motor->config.name + " did not answer on any CAN bus");
                std::printf("%s on CAN bus %d, direction %+d\n", motor->config.name.c_str(), bus, motor->config.direction);
                motor->on_endpoint_found();
                motor->set_run_mode(ROB_RUN_MODE_VELOCITY, true, true);
                motor->set_target_acceleration(motor->config.max_acceleration, true, true);
                motor->set_limit_torque(motor->config.max_torque, true, true);
                motor->set_target_velocity(0.0, true, true);
            }
            if (g_stop != 0) throw std::runtime_error("stopped by signal");

            for (auto* motor : motors) motor->enable(true);
            enabled = true;
            const double speed = kForward ? kDriveRadPerSec : -kDriveRadPerSec;
            if (kDryRun) {
                std::printf("dry run, holding 0 rad/s for %.0f s\n", kDriveSeconds);
            } else {
                for (auto* motor : motors) motor->set_target_velocity(speed, true, true);
                std::printf("driving %s at %.1f rad/s for %.0f s\n", kForward ? "forward" : "backward", kDriveRadPerSec, kDriveSeconds);
            }

            const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(kDriveSeconds);
            auto next_print = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() < end && g_stop == 0) {
                for (auto* motor : motors) motor->trigger_feedback_frame();
                if (std::chrono::steady_clock::now() >= next_print) {
                    for (auto* motor : motors) {
                        const double vel = motor->read_param_await(ROB_PARAM_CODE_FB_VELOCITY);
                        const double torque = motor->read_param_await(ROB_PARAM_CODE_FB_TORQUE);
                        std::printf(
                            "  %s 0x%02x dir %+d  vel %7.3f rad/s  torque %6.2f Nm\n", motor->config.name.c_str(),
                            motor->config.id, motor->config.direction, vel, torque
                        );
                    }
                    std::fflush(stdout);
                    next_print += std::chrono::milliseconds(500);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kFeedbackHz));
            }
        } catch (const std::exception& ex) {
            std::fprintf(stderr, "%s\n", ex.what());
            rc = 1;
        }
        stop_motors(motors, kMotors, enabled);

        comms.abort();
        monitor.abort();
        if (g_stop != 0) {
            std::printf("stopped by signal\n");
            rc = 1;
        }
        std::printf("data monitor files in %s\n", dir.c_str());
        return rc;
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        return 1;
    }
}
