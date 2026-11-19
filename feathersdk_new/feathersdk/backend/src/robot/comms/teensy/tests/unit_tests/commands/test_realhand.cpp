#include "../../../src/logging.cpp"
#include "../../../src/system.cpp"
#include "../../../src/comms/comms.cpp"
#include "../../../src/comms/can.cpp"
#include "../../../src/comms/ethernet.cpp"
#include "../../../src/actuator.cpp"
#include "../../../src/commands/robstride.cpp"
#include "../../../src/commands/realhand.cpp"
#include "../../../src/commands/ezmotion.cpp"
#include "../../../src/commands/soc.cpp"
#include "../../framework.hpp"
#include "../../../src/commands/realhand.hpp"
#include <linux/can.h>

extern uint8_t num_can_lines;
extern uint8_t _testing_can_buf[MAX_NUM_CAN_LINES][sizeof(struct can_frame)];
extern bool _testing_can_buf_used;
extern float can_last_loads[];
extern uint64_t can_last_times_ns[];

static const uint32_t kMsgId = 1u;
static const uint8_t kActId = 0x11;
static const uint8_t kCanBus = 0u;

static void can_setup(void) {
    num_can_lines = 3;
    _testing_can_buf_used = true;
    memset(can_last_loads, 0, sizeof(float) * MAX_NUM_CAN_LINES);
    memset(can_last_times_ns, 0, sizeof(uint64_t) * MAX_NUM_CAN_LINES);
    SET_DEFAULT_COMMS_CONFIG();
    memset(_testing_can_buf, 0, sizeof(_testing_can_buf));
}

static test_result_t init_hand(uint8_t act_id, bool allow_reinit) {
    RealHandMotorInitPayload payload = {0};
    payload.actuator_id = act_id;
    payload.allow_reinit = allow_reinit;
    payload.max_telemetry_size = UINT16_MAX;
    for (uint8_t i = 0; i < NUM_RealHandParamCode; i++) payload.telem_inds[i] = (uint16_t)(100u + i);
    TEST_ASSERT(!realhand_handle_initialize_motor(kMsgId, &payload), "init returns false for the default success");
    TEST_ASSERT(is_realhand_slot(act_id), "slot marked");
    return TEST_SUCCESS;
}

static test_result_t assert_frame(uint8_t dlc, const uint8_t* expect) {
    struct can_frame got = {0};
    memcpy(&got, _testing_can_buf[kCanBus], sizeof(got));
    TEST_ASSERT(got.can_id == kActId, "can id 0x%X", got.can_id);
    TEST_ASSERT(got.can_dlc == dlc, "dlc %u", got.can_dlc);
    TEST_ASSERT(memcmp(got.data, expect, dlc) == 0, "payload mismatch");
    return TEST_SUCCESS;
}

static uint64_t stored_u64(uint16_t pcode) {
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(kActId, pcode, ERROR_SCOPE_LOCAL);
    uint64_t value = 0;
    if (telem != NULL) memcpy(&value, telem->values, sizeof(value));
    return value;
}

static CAN_message_t make_msg(uint32_t id, uint8_t len, const uint8_t* data) {
    CAN_message_t msg = {0};
    msg.id = id;
    msg.len = len;
    if (data != NULL && len > 0) memcpy(msg.buf, data, len);
    return msg;
}


DECLARE_TEST(test_realhand_init) {
    reset_actuators();

    RealHandMotorInitPayload bad = {.actuator_id = MAX_NUM_ACTUATORS};
    TEST_ASSERT(!realhand_handle_initialize_motor(kMsgId, &bad), "id past the table");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid actuator ID");

    TEST_ASSERT_STACKED(init_hand(kActId, false), "init");
    for (uint8_t i = 0; i < NUM_RealHandParamCode; i++) {
        RealHandParam param = realhand_all_params[i];
        TEST_ASSERT(get_actuator_param_telemetry(kActId, i, ERROR_SCOPE_LOCAL) != NULL, "ring %u", i);
        if (!param.is_timestamp_param) {
            TEST_ASSERT(param.timestamp_pcode == (uint8_t)(i + 1u), "timestamp for %u is not the next slot", i);
        }
    }

    bad.actuator_id = kActId;
    TEST_ASSERT(!realhand_handle_initialize_motor(kMsgId, &bad), "second init");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("already initialized");
    TEST_ASSERT_STACKED(init_hand(kActId, true), "reinit");

    reset_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_realhand_read_write) {
    reset_actuators();
    can_setup();

    RealHandReadParamPayload read = {.actuator_id = kActId, .can_bus = kCanBus, .pcode = REAL_HAND_PARAM_CODE_POSITION};
    TEST_ASSERT(!realhand_handle_read_param(kMsgId, &read), "uninitialized");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT_STACKED(init_hand(kActId, true), "init");

    read.pcode = NUM_RealHandParamCode;
    TEST_ASSERT(!realhand_handle_read_param(kMsgId, &read), "pcode past the end");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid RealHand parameter code");

    read.pcode = REAL_HAND_PARAM_CODE_POSITION_TS;
    TEST_ASSERT(!realhand_handle_read_param(kMsgId, &read), "timestamp pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid RealHand parameter code");

    read.pcode = REAL_HAND_PARAM_CODE_POSITION;
    memset(_testing_can_buf, 0, sizeof(_testing_can_buf));
    TEST_ASSERT(!realhand_handle_read_param(kMsgId, &read), "read ok");
    uint8_t query[1] = {0x01};
    TEST_ASSERT_STACKED(assert_frame(1, query), "position query is property only");

    RealHandWriteParamPayload write = {
        .actuator_id = kActId, .can_bus = kCanBus, .pcode = REAL_HAND_PARAM_CODE_NORMAL_FORCE, .value = 1
    };
    TEST_ASSERT(!realhand_handle_write_param(kMsgId, &write), "read-only");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not writable");

    write.pcode = REAL_HAND_PARAM_CODE_POSITION;
    write.value = REALHAND_VALUE_MAX + 1ull;
    TEST_ASSERT(!realhand_handle_write_param(kMsgId, &write), "above 6 bytes");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("out of range");

    write.value = 0x010203040506ull;
    memset(_testing_can_buf, 0, sizeof(_testing_can_buf));
    TEST_ASSERT(!realhand_handle_write_param(kMsgId, &write), "write ok");
    uint8_t fingers[7] = {0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    TEST_ASSERT_STACKED(assert_frame(7, fingers), "position write");

    write.value = 0;
    memset(_testing_can_buf, 0, sizeof(_testing_can_buf));
    TEST_ASSERT(!realhand_handle_write_param(kMsgId, &write), "zero write");
    uint8_t zeros[7] = {0x01, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_STACKED(assert_frame(7, zeros), "zero fingers");

    write.value = REALHAND_VALUE_MAX;
    memset(_testing_can_buf, 0, sizeof(_testing_can_buf));
    TEST_ASSERT(!realhand_handle_write_param(kMsgId, &write), "max write");
    uint8_t full[7] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    TEST_ASSERT_STACKED(assert_frame(7, full), "max fingers");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_realhand_replies) {
    reset_actuators();
    can_setup();
    TEST_ASSERT_STACKED(init_hand(kActId, true), "init");

    uint8_t position[7] = {0x01, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
    CAN_message_t msg = make_msg(kActId, 7, position);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT(stored_u64(REAL_HAND_PARAM_CODE_POSITION) == 0x0A0B0C0D0E0Full, "position stored");
    TEST_ASSERT(get_actuator_param_telemetry(kActId, REAL_HAND_PARAM_CODE_POSITION_TS, ERROR_SCOPE_LOCAL)->end_ind == 1,
        "position timestamp written");

    uint8_t property_only[1] = {0x01};
    msg = make_msg(kActId, 1, property_only);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT(stored_u64(REAL_HAND_PARAM_CODE_POSITION) == 0x0A0B0C0D0E0Full, "property-only frame ignored");
    TEST_ASSERT(get_actuator_param_telemetry(kActId, REAL_HAND_PARAM_CODE_POSITION_TS, ERROR_SCOPE_LOCAL)->end_ind == 1,
        "property-only frame writes no timestamp");

    uint8_t version[3] = {0xC2, 0x01, 0x02};
    msg = make_msg(kActId, 3, version);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT(stored_u64(REAL_HAND_PARAM_CODE_VERSION) == 0x0102ull, "version property");

    uint8_t error[8] = {'E', 'R', 'R', 'O', 'R', 0x00, 0x00, 0xAB};
    msg = make_msg(kActId, 8, error);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("RealHand error on actuator");

    uint8_t unknown[2] = {0x99, 0x00};
    msg = make_msg(kActId, 2, unknown);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("unknown frame property");

    msg = make_msg(kActId, 0, NULL);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("invalid frame length");

    // A freed slot must fall through to the other standard-frame parser, not stay claimed as a hand.
    free_actuator(kActId);
    msg = make_msg(kActId, 7, position);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("invalid/unhandled canopen");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

RUN_TESTS();
