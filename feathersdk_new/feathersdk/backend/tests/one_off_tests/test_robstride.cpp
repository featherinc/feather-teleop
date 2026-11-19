// Init one Robstride motor on the connected Teensy, find its CAN bus, and print temperature.
//
// Parameter replies arrive as actuator telemetry. The Teensy poll thread requests those samples.
// Build and run from backend/: make test_robstride
// Optional motor id, decimal or 0x: make test_robstride MOTOR_ID=0x30

#include "src/robot/actuators/robstride.hpp"
#include "src/robot/comms/comms_manager.hpp"
#include "src/robot/telemetry.hpp"
#include "logging.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>

namespace {

constexpr int kDefaultMotorId = 1;

int parse_motor_id(int argc, char** argv) {
    if (argc > 2) {
        std::fprintf(stderr, "usage: test_robstride [motor_id]\n");
        return -1;
    }
    if (argc < 2) return kDefaultMotorId;

    char* end = nullptr;
    const long parsed = std::strtol(argv[1], &end, 0);
    if (argv[1][0] == '\0' || end == argv[1] || *end != '\0' || parsed < 0 || parsed > 0xFD) {
        std::fprintf(stderr, "usage: test_robstride [motor_id]\n");
        return -1;
    }
    return static_cast<int>(parsed);
}

}  // namespace

int main(int argc, char** argv) {
    const int motor_id = parse_motor_id(argc, argv);
    if (motor_id < 0) return 1;

    feather::log::Config log_cfg;
    log_cfg.console_enabled = true;
    log_cfg.file_enabled = false;
    feather::log::init(log_cfg);

    try {
        feather::robot::Telemetry telemetry;

        feather::comms::CommsManagerConfig cfg;
        cfg.teensy.use_emulated_teensy = false;
        feather::comms::CommsManager comms(cfg, &telemetry);

        feather::robot::RobstrideMotorConfig motor_cfg;
        motor_cfg.name = "robstride";
        motor_cfg.id = motor_id;
        feather::robot::RobstrideMotor motor(motor_cfg, comms, telemetry);
        if (!telemetry.initialize()) {
            std::fprintf(stderr, "telemetry initialize failed\n");
            return 1;
        }

        motor.init();

        const int buses = comms.num_can_lines();
        if (buses <= 0) {
            std::fprintf(stderr, "Teensy reported %d CAN buses\n", buses);
            return 1;
        }

        int found = -1;
        for (int bus = 0; bus < buses; ++bus) {
            std::printf("probing CAN bus %d\n", bus);
            std::fflush(stdout);
            if (motor.probe_endpoint(bus)) {
                found = bus;
                break;
            }
        }
        if (found < 0) {
            std::fprintf(stderr, "motor %d did not answer on any CAN bus\n", motor_id);
            return 1;
        }

        const double temp = motor.read_param_await(ROB_PARAM_CODE_FB_TEMP);
        std::printf("motor %d on CAN bus %d\n", motor_id, found);
        std::printf("temperature %.1f C\n", temp);
        return 0;
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        return 1;
    }
}
