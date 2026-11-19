// Feedback round-trip rate for every Robstride motor on the base and both arms.
//
// One update sends one feedback request to every motor. A feedback request is the
// same CAN_TIMEOUT write RobstrideMotor::trigger_feedback_frame sends. Latency is
// the host clock at send to the host clock when the Teensy reply is recorded.
//
// Barrier: send one update, then wait for every reply, and repeat. A missing reply
// or a Teensy response other than success fails the run. The next round waits so the
// busiest bus stays under the 3500 msg/s overload cap sent at Teensy init. That gap
// also drains the 16-deep FlexCAN transmit queue.
// Paced: send the next update as soon as the previous send finished, but no faster
// than 350 updates/s. A Teensy response other than success fails the run. Replies
// that never arrive are counted as dropped.
//
// The motor list matches /feathersys/configs/systems/navigation_system_conf.py and
// arms_system_conf.py. Comms rates match comms_manager_conf.py. Keep the constants
// below in sync with tests/speed_tests/full_motor_update.py.
//
// Build and run from backend/: make full_motor_update

#include "src/robot/actuators/robstride.hpp"
#include "src/robot/comms/comms_manager.hpp"
#include "logging.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kMeasureSeconds = 2.0;
constexpr double kWarmupSeconds = 0.25;
constexpr double kMaxUpdatesPerSec = 350.0;
constexpr double kReplyTimeoutSeconds = 0.5;
// Sent on Teensy initialize. The overload check adds kCanLeniency per frame and decays
// with that same constant, so a burst of N frames then a gap peaks near
// N * leniency / (1 - exp(-leniency * gap)). The barrier gap targets 90% of the cap.
constexpr double kMaxCanRate = 3500.0;
constexpr double kCanLeniency = 100.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTau = 2.0 * kPi;

constexpr double deg(double degrees) { return degrees * kPi / 180.0; }

struct Spec {
    const char* name;
    int id;
    int direction;
    RobVersion version;
    double max_velocity;
    double max_torque;
    double max_acceleration;
    double angle_min;
    double angle_max;
};

// Base, then left arm, then right arm. Same order as the Python test.
constexpr Spec kMotors[] = {
    {"turning_center", 0x70, -1, ROB_VERSION_RS02, 10.47, 5.5, 100.0, -1.920, 1.920},
    {"turning_right", 0x71, -1, ROB_VERSION_RS02, 10.47, 5.5, 100.0, -1.920, 1.920},
    {"turning_left", 0x72, -1, ROB_VERSION_RS02, 10.47, 5.5, 100.0, -1.920, 1.920},
    {"drive_center", 0x73, -1, ROB_VERSION_RS01, 10.47, 5.5, 30.0, -kTau, kTau},
    {"drive_right", 0x74, 1, ROB_VERSION_RS01, 10.47, 5.5, 30.0, -kTau, kTau},
    {"drive_left", 0x75, -1, ROB_VERSION_RS01, 10.47, 5.5, 30.0, -kTau, kTau},

    {"shoulder_pitch_left", 0x20, 1, ROB_VERSION_RS04, 1.5, 40.0, 50.0, deg(-147), deg(157)},
    {"shoulder_roll_left", 0x21, 1, ROB_VERSION_RS04, 1.5, 40.0, 50.0, deg(-19), deg(197)},
    {"shoulder_yaw_left", 0x22, 1, ROB_VERSION_RS03, 2.0, 20.0, 50.0, deg(-128), deg(128)},
    {"elbow_left", 0x23, 1, ROB_VERSION_RS03, 2.0, 20.0, 50.0, deg(-155), deg(55)},
    {"wrist_yaw_left", 0x24, 1, ROB_VERSION_RS02, 4.0, 6.0, 50.0, deg(-161), deg(161)},
    {"wrist_pitch_left", 0x25, 1, ROB_VERSION_RS02, 4.0, 6.0, 50.0, deg(-106), deg(106)},
    {"wrist_roll_left", 0x26, 1, ROB_VERSION_RS02, 4.0, 6.0, 50.0, deg(-111), deg(111)},

    {"shoulder_pitch_right", 0x10, 1, ROB_VERSION_RS04, 1.5, 40.0, 50.0, deg(-153), deg(153)},
    {"shoulder_roll_right", 0x11, 1, ROB_VERSION_RS04, 1.5, 40.0, 50.0, deg(-199), deg(17)},
    {"shoulder_yaw_right", 0x12, 1, ROB_VERSION_RS03, 2.0, 20.0, 50.0, deg(-127), deg(127)},
    {"elbow_right", 0x13, 1, ROB_VERSION_RS03, 2.0, 20.0, 50.0, deg(-55), deg(155)},
    {"wrist_yaw_right", 0x14, 1, ROB_VERSION_RS02, 4.0, 6.0, 50.0, deg(-161), deg(161)},
    {"wrist_pitch_right", 0x15, 1, ROB_VERSION_RS02, 4.0, 6.0, 50.0, deg(-105), deg(105)},
    {"wrist_roll_right", 0x16, 1, ROB_VERSION_RS02, 4.0, 6.0, 50.0, deg(-107), deg(107)},
};

using Clock = std::chrono::steady_clock;

feather::comms::CommsManagerConfig comms_config() {
    feather::comms::CommsManagerConfig cfg;
    cfg.teensy.use_emulated_teensy = false;
    cfg.teensy.teensy_correct_clock_drift = false;
    cfg.teensy.max_can_rate = kMaxCanRate;
    cfg.teensy.can_leniency = kCanLeniency;
    cfg.teensy.server_run_time_nanos = 500000;
    cfg.teensy.can_read_run_time_nanos = 500000;
    return cfg;
}

feather::robot::RobstrideMotorConfig motor_config(const Spec& spec) {
    feather::robot::RobstrideMotorConfig cfg;
    cfg.name = spec.name;
    cfg.id = spec.id;
    cfg.direction = spec.direction;
    cfg.version = spec.version;
    cfg.max_velocity = spec.max_velocity;
    cfg.max_torque = spec.max_torque;
    cfg.max_acceleration = spec.max_acceleration;
    cfg.angle_min = spec.angle_min;
    cfg.angle_max = spec.angle_max;
    return cfg;
}

feather::comms::SentTeensyMessage request_feedback(
    feather::comms::CommsManager& comms, const feather::robot::RobstrideMotor& motor
) {
    RobWriteParamPayload body{};
    body.motor_id = static_cast<std::uint8_t>(motor.config.id);
    body.can_bus = static_cast<std::uint8_t>(motor.config.endpoint);
    body.pcode = static_cast<std::uint8_t>(ROB_PARAM_CODE_CAN_TIMEOUT);
    body.value = 0.f;
    body.clamp = false;
    return comms.send_teensy_message(
        TEENSY_COMMAND_OPCODE_CMD_ROB_WRITE_PARAM, {reinterpret_cast<const char*>(&body), sizeof body}
    );
}

bool take_reply(
    feather::comms::CommsManager& comms, const feather::comms::SentTeensyMessage& msg, feather_time_t timeout,
    std::uint64_t& latency_ns, const char* run
) {
    char err[256]{};
    const auto resp = comms.get_response(
        msg, timeout, {.data = reinterpret_cast<std::uint8_t*>(err), .size = sizeof err}
    );
    if (!resp) return false;
    if (resp->opcode != TEENSY_COMMAND_OPCODE_RESP_SUCCESS) {
        const auto n = std::min<std::uint32_t>(resp->payload_size, sizeof err);
        const std::string detail(err, err + n);
        throw std::runtime_error(
            std::string(run) + " got Teensy opcode " + std::to_string(static_cast<unsigned>(resp->opcode)) +
            (detail.empty() ? "" : ": " + detail)
        );
    }
    if (resp->received_ts < resp->inserted_ts) throw std::runtime_error(std::string(run) + " reply timestamp went backwards");
    latency_ns = resp->received_ts - resp->inserted_ts;
    return true;
}

struct Run {
    int updates = 0;
    int dropped = 0;
    double seconds = 0;
    std::vector<std::uint64_t> latency_ns;
};

Run run_barrier(
    feather::comms::CommsManager& comms, std::vector<feather::robot::RobstrideMotor>& motors, double seconds, bool record
) {
    Run run;
    const auto timeout = feather::time::from_seconds(kReplyTimeoutSeconds);
    const auto t0 = Clock::now();
    const auto end = t0 + std::chrono::duration<double>(seconds);
    std::vector<feather::comms::SentTeensyMessage> sent;
    sent.reserve(motors.size());
    int per_bus[8] = {};
    for (const auto& motor : motors) {
        if (motor.config.endpoint >= 0 && motor.config.endpoint < 8) ++per_bus[motor.config.endpoint];
    }
    const int busiest = std::max(1, *std::max_element(per_bus, per_bus + 8));
    const double ratio = static_cast<double>(busiest) * kCanLeniency / (kMaxCanRate * 0.9);
    const auto drain = std::chrono::duration<double>(-std::log(1.0 - ratio) / kCanLeniency);

    while (Clock::now() < end) {
        const auto round_start = Clock::now();
        sent.clear();
        for (const auto& motor : motors) {
            const auto msg = request_feedback(comms, motor);
            if (msg.msg_id == 0) throw std::runtime_error("barrier failed to send a feedback request");
            sent.push_back(msg);
        }
        for (const auto& msg : sent) {
            std::uint64_t latency = 0;
            if (!take_reply(comms, msg, timeout, latency, "barrier")) {
                throw std::runtime_error("dropped a feedback reply during the barrier run");
            }
            if (record) run.latency_ns.push_back(latency);
        }
        const auto ready = round_start + drain;
        if (Clock::now() < ready) std::this_thread::sleep_until(ready);
        ++run.updates;
    }
    run.seconds = std::chrono::duration<double>(Clock::now() - t0).count();
    return run;
}

Run run_paced(feather::comms::CommsManager& comms, std::vector<feather::robot::RobstrideMotor>& motors, double seconds) {
    Run run;
    const auto period = std::chrono::duration<double>(1.0 / kMaxUpdatesPerSec);
    std::vector<feather::comms::SentTeensyMessage> inflight;
    const auto drain_ready = [&] {
        std::size_t kept = 0;
        for (std::size_t i = 0; i < inflight.size(); ++i) {
            std::uint64_t latency = 0;
            if (!take_reply(comms, inflight[i], 0, latency, "paced")) {
                inflight[kept++] = inflight[i];
                continue;
            }
            run.latency_ns.push_back(latency);
        }
        inflight.resize(kept);
    };

    const auto t0 = Clock::now();
    const auto end = t0 + std::chrono::duration<double>(seconds);
    auto next = t0;
    while (Clock::now() < end) {
        const auto now = Clock::now();
        if (now < next) std::this_thread::sleep_until(next);
        if (Clock::now() >= end) break;
        for (const auto& motor : motors) {
            const auto msg = request_feedback(comms, motor);
            if (msg.msg_id == 0) throw std::runtime_error("paced failed to send a feedback request");
            inflight.push_back(msg);
        }
        ++run.updates;
        next += std::chrono::duration_cast<Clock::duration>(period);
        if (next < Clock::now()) next = Clock::now();
        drain_ready();
    }
    run.seconds = std::chrono::duration<double>(Clock::now() - t0).count();

    const auto drain_end = Clock::now() + std::chrono::duration<double>(kReplyTimeoutSeconds);
    while (!inflight.empty() && Clock::now() < drain_end) {
        drain_ready();
        if (!inflight.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    run.dropped = static_cast<int>(inflight.size());
    return run;
}

std::uint64_t percentile(const std::vector<std::uint64_t>& sorted, double p) {
    const std::size_t idx = static_cast<std::size_t>(std::ceil(p * static_cast<double>(sorted.size()))) - 1;
    return sorted[idx];
}

void report(const char* mode, int motors, const Run& run, bool show_dropped) {
    if (run.latency_ns.empty() || run.seconds <= 0) throw std::runtime_error(std::string(mode) + " recorded no replies");
    auto lat = run.latency_ns;
    const double sum = static_cast<double>(std::accumulate(lat.begin(), lat.end(), static_cast<std::uint64_t>(0)));
    std::sort(lat.begin(), lat.end());
    const double rate = static_cast<double>(run.updates) / run.seconds;

    std::printf("full motor update, %s, %d motors, %d updates in %.3f s", mode, motors, run.updates, run.seconds);
    if (show_dropped) std::printf(", dropped %d", run.dropped);
    std::printf("\n");
    std::printf("  updates/s   %8.1f\n", rate);
    std::printf(
        "  latency us  min %8.1f  mean %8.1f  p95 %8.1f  p99 %8.1f  max %8.1f\n",
        static_cast<double>(lat.front()) / 1000.0, sum / static_cast<double>(lat.size()) / 1000.0,
        static_cast<double>(percentile(lat, 0.95)) / 1000.0, static_cast<double>(percentile(lat, 0.99)) / 1000.0,
        static_cast<double>(lat.back()) / 1000.0
    );
}

}  // namespace

int main() {
    feather::log::Config log_cfg;
    log_cfg.console_enabled = true;
    log_cfg.file_enabled = false;
    feather::log::init(log_cfg);

    try {
        const auto cfg = comms_config();
        feather::robot::Telemetry telemetry;
        feather::comms::CommsManager comms(cfg, &telemetry, nullptr);
        std::vector<feather::robot::RobstrideMotor> motors;
        motors.reserve(std::size(kMotors));
        for (const Spec& spec : kMotors) motors.emplace_back(motor_config(spec), comms, telemetry);
        if (!telemetry.initialize()) throw std::runtime_error("telemetry initialize failed");
        for (auto& motor : motors) motor.init();

        const int buses = comms.num_can_lines();
        if (buses <= 0) throw std::runtime_error("Teensy reported no CAN buses");
        for (auto& motor : motors) {
            int bus = -1;
            for (int i = 0; i < buses; ++i) {
                std::printf("probing %s (0x%02x) on CAN bus %d\n", motor.config.name.c_str(), motor.config.id, i);
                if (motor.probe_endpoint(i)) {
                    bus = i;
                    break;
                }
            }
            if (bus < 0) throw std::runtime_error(motor.config.name + " did not answer on any CAN bus");
            std::printf("%s on CAN bus %d\n", motor.config.name.c_str(), bus);
        }

        run_barrier(comms, motors, kWarmupSeconds, false);
        report("barrier", static_cast<int>(motors.size()), run_barrier(comms, motors, kMeasureSeconds, true), false);
        report("paced", static_cast<int>(motors.size()), run_paced(comms, motors, kMeasureSeconds), true);
        comms.abort();
        return 0;
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        return 1;
    }
}
