#pragma once

// Emulated Robstride motor. Feed it the extended CAN frames the Teensy sends
// (host id in bits 8-15, motor id in the low byte), call step(dt), then drain
// the replies. Reply ids swap those two bytes, which is what the firmware parser expects.

#include "qdefs/robstride_qdef.hpp"

#include <cstdint>
#include <deque>
#include <limits>

namespace feather::robot::sim {

struct SimCanFrame {
    uint32_t id = 0;
    uint8_t data[8]{};
    uint8_t len = 8;
};

class VirtualRobstride {
public:
    VirtualRobstride(int motor_id, RobVersion version,
        double hard_stop_min = -std::numeric_limits<double>::infinity(),
        double hard_stop_max = std::numeric_limits<double>::infinity());

    // Host-to-motor frame. Frames for a different motor id are ignored.
    void handle_frame(uint32_t can_id, const uint8_t* data, uint8_t len);

    // Advance the shaft by dt seconds. May queue a broadcast, feedback, or a fault frame.
    void step(double dt);

    // Oldest queued reply. Returns false when the queue is empty.
    bool pop_frame(SimCanFrame& out);

    void set_external_torque(double torque) { external_torque_ = torque; }
    void set_error(RobMotorError error);

    double param(RobParamCode code) const { return params_[code]; }
    int motor_id() const { return motor_id_; }

private:
    int motor_id_;
    RobVersion version_;
    double hard_stop_min_;
    double hard_stop_max_;
    double params_[NUM_RobParamCode]{};
    uint16_t errors_ = 0;

    bool active_reporting_ = false;
    std::uint64_t last_active_reporting_ = 0;
    bool sent_first_broadcast_ = false;
    bool sent_fault_feedback_ = false;
    uint8_t last_host_id_ = 0xAA;

    double external_torque_ = 0;
    double op_kd_ = 20;
    std::deque<double> integrals_;
    bool have_derivative_ = false;
    double last_error_ = 0;
    double d_filt_ = 0;

    std::deque<SimCanFrame> out_;

    void enqueue(uint32_t id, const uint8_t* data);
    void send_feedback(RobOperation op, bool firmware_version);
    void send_broadcast();
    void read_param(uint8_t host_id, const uint8_t* data);
    void write_param(const uint8_t* data);
    double position_torque(double dt);
    void apply_torque(double torque, double dt);
};

}  // namespace feather::robot::sim
