#include "backend.hpp"

#include "logging.hpp"

#include <stdexcept>
#include <string>

namespace feather {

Backend::Backend(const BackendConfig& cfg)
    : telemetry(),
      data_monitor(cfg.data_monitor),
      comms(cfg.comms, &telemetry, &data_monitor),
      neck(cfg.neck, comms, telemetry),
      name_(cfg.name) {
    try {
        if (!telemetry.initialize()) throw std::runtime_error(name_ + ": telemetry initialize failed");

        const int buses = comms.num_can_lines();
        if (buses <= 0) throw std::runtime_error(name_ + ": Teensy reported no CAN buses");

        robot::RobstrideMotor* motors[] = {&neck.yaw_motor, &neck.pitch_motor};
        for (robot::RobstrideMotor* motor : motors) {
            motor->init();
            find_motor(*motor, buses);
            motor->on_endpoint_found();
        }

        neck.initialize();
        neck.set_robot_initialized(true);
        INFO("%s initialized", name_.c_str());
    } catch (...) {
        abort();
        throw;
    }
}

Backend::~Backend() {
    abort();
}

void Backend::find_motor(robot::RobstrideMotor& motor, int buses) {
    if (motor.has_endpoint()) {
        const int endpoint = motor.config.endpoint;
        if (!motor.probe_endpoint(endpoint)) {
            throw std::runtime_error(motor.config.name + ": not on CAN bus " + std::to_string(endpoint));
        }
        return;
    }
    for (int bus = 0; bus < buses; ++bus) {
        if (motor.probe_endpoint(bus)) return;
    }
    throw std::runtime_error(motor.config.name + ": did not answer on any CAN bus");
}

void Backend::abort() {
    try {
        neck.abort();
    } catch (const std::exception& ex) {
        ERROR("%s: neck abort failed: %s", name_.c_str(), ex.what());
    }
    comms.abort();
    data_monitor.abort();
}

}  // namespace feather
