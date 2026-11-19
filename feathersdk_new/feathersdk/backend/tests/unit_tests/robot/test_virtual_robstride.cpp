#include "../../framework.hpp"
#include "../../../src/robot/actuators/sim/virtual_robstride.hpp"

#include <cstring>

using feather::robot::sim::SimCanFrame;
using feather::robot::sim::VirtualRobstride;

static uint32_t host_can_id(RobOperation op, uint8_t e_byte, uint8_t host, uint8_t motor) {
    return ((uint32_t)op << 24) | ((uint32_t)e_byte << 16) | ((uint32_t)host << 8) | motor;
}

static void drain(VirtualRobstride& motor) {
    SimCanFrame frame;
    while (motor.pop_frame(frame)) {}
}

static void write_param(VirtualRobstride& motor, RobParamCode code, const void* value, int size) {
    uint8_t data[8]{};
    const uint16_t param_id = rob_all_params[code].param_id;
    std::memcpy(data, &param_id, 2);
    std::memcpy(data + 4, value, (size_t)size);
    motor.handle_frame(host_can_id(ROB_OPERATION_WRITE_PARAM, 0, 0xAA, (uint8_t)motor.motor_id()), data, 8);
}

static void enable_position(VirtualRobstride& motor, float target) {
    const uint8_t mode = ROB_RUN_MODE_POSITION_PP;
    write_param(motor, ROB_PARAM_CODE_RUN_MODE, &mode, 1);
    write_param(motor, ROB_PARAM_CODE_LOC_REF, &target, 4);
    uint8_t data[8]{};
    motor.handle_frame(host_can_id(ROB_OPERATION_ENABLE, 0, 0xAA, (uint8_t)motor.motor_id()), data, 8);
    drain(motor);
}

DECLARE_TEST(first_step_broadcasts_device_id) {
    VirtualRobstride motor(0x31, ROB_VERSION_RS05);
    motor.step(0.01);

    SimCanFrame frame;
    TEST_ASSERT(motor.pop_frame(frame), "broadcast frame");
    TEST_ASSERT(((frame.id >> 24) & 0x1F) == ROB_OPERATION_GET_DEVICE_ID, "opcode");
    TEST_ASSERT(((frame.id >> 8) & 0xFF) == 0x31, "motor id sits above the host id");
    TEST_ASSERT((frame.id & 0xFF) == 0xFE, "broadcast host id");
    TEST_ASSERT(frame.data[0] == 0xDE && frame.data[1] == 0xAD && frame.data[2] == 0xBE && frame.data[3] == 0xEF,
        "device id prefix");
    TEST_ASSERT(frame.data[5] == ROB_VERSION_RS05 && frame.data[7] == 0x31, "version and motor id");
    TEST_ASSERT(!motor.pop_frame(frame), "a single broadcast");
    return TEST_SUCCESS;
}

DECLARE_TEST(ignores_frames_for_other_motors) {
    VirtualRobstride motor(0x31, ROB_VERSION_RS05);
    uint8_t data[8]{};
    motor.handle_frame(host_can_id(ROB_OPERATION_ENABLE, 0, 0xAA, 0x30), data, 8);
    SimCanFrame frame;
    TEST_ASSERT(!motor.pop_frame(frame), "no reply");
    TEST_ASSERT(motor.param(ROB_PARAM_CODE_FB_MODE) == ROB_MOTOR_MODE_RESET, "still disabled");
    return TEST_SUCCESS;
}

DECLARE_TEST(position_mode_moves_toward_the_command) {
    VirtualRobstride motor(0x31, ROB_VERSION_RS05);
    enable_position(motor, 1.0f);

    for (int i = 0; i < 80; ++i) motor.step(0.01);
    drain(motor);

    const double pos = motor.param(ROB_PARAM_CODE_FB_POSITION);
    TEST_ASSERT(pos > 0.1 && pos < 1.0, "shaft moved toward 1 rad, got %f", pos);
    return TEST_SUCCESS;
}

DECLARE_TEST(hard_stop_holds_the_shaft) {
    VirtualRobstride motor(0x31, ROB_VERSION_RS05, -0.05, 0.05);
    enable_position(motor, 1.0f);
    for (int i = 0; i < 40; ++i) motor.step(0.01);

    const double pos = motor.param(ROB_PARAM_CODE_FB_POSITION);
    TEST_ASSERT(pos <= 0.05 && pos >= -0.05, "held inside the stop, got %f", pos);
    TEST_ASSERT(motor.param(ROB_PARAM_CODE_FB_VELOCITY) == 0, "velocity cleared at the stop");
    return TEST_SUCCESS;
}

DECLARE_TEST(fault_frame_is_sent_once) {
    VirtualRobstride motor(0x31, ROB_VERSION_RS05);
    motor.set_error(ROB_MOTOR_ERROR_OVER_TEMPERATURE);
    motor.step(0.01);

    bool saw_fault = false;
    SimCanFrame frame;
    while (motor.pop_frame(frame)) {
        if (((frame.id >> 24) & 0x1F) == ROB_OPERATION_FAULT_FEEDBACK) saw_fault = true;
    }
    TEST_ASSERT(saw_fault, "fault feedback");

    motor.step(0.01);
    while (motor.pop_frame(frame)) {
        TEST_ASSERT(((frame.id >> 24) & 0x1F) != ROB_OPERATION_FAULT_FEEDBACK, "fault is sent once");
    }
    return TEST_SUCCESS;
}

RUN_TESTS()
