#pragma once

// Library entry for the C++ backend. The feathersdk constructs one Backend and uses the members it exposes.
// For now that is the neck: telemetry, the data monitor, the Teensy link, and the two neck motors.

#include "src/robot/actuator_systems/neck_system.hpp"
#include "src/robot/comms/comms_manager.hpp"
#include "src/robot/services/data_monitor.hpp"
#include "src/robot/telemetry.hpp"

#include <string>

namespace feather {

struct BackendConfig {
    std::string name = "backend";
    comms::CommsManagerConfig comms;
    robot::DataMonitorServiceConfig data_monitor;
    robot::TwoDOFRobstrideNeckSystemConfig neck;
};

class Backend {
public:
    // Connects to the Teensy, finds both neck motors, and enables the neck. Throws if any step fails.
    explicit Backend(const BackendConfig& config);
    ~Backend();

    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;

    // Disable the neck, stop the Teensy link, and close the monitor files.
    void abort();

    // Constructed in this order. The neck registers telemetry channels, and Telemetry::initialize()
    // runs after that, in the constructor body.
    robot::Telemetry telemetry;
    robot::DataMonitorService data_monitor;
    comms::CommsManager comms;
    robot::TwoDOFRobstrideNeckSystem neck;

private:
    std::string name_;

    void find_motor(robot::RobstrideMotor& motor, int buses);
};

}  // namespace feather
