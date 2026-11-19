#include "../../../src/logging.cpp"
#include "../../../src/system.cpp"
#include "../../../src/comms/comms.cpp"
#include "../../../src/comms/can.cpp"
#include "../../../src/comms/ethernet.cpp"
#include "../../../src/actuator.cpp"
#include "../../../src/commands/ezmotion.cpp"
#include "../../framework.hpp"
#include "../../../src/commands/ezmotion.hpp"
#include <linux/can.h>

extern uint8_t num_can_lines;
extern uint8_t _testing_can_buf[MAX_NUM_CAN_LINES][sizeof(struct can_frame)];
extern bool _testing_can_buf_used;

extern bool ez_write(uint8_t can_bus, uint8_t node_id, EZMotionParam param, int64_t value, int16_t pcode);
extern void handle_ezmotion_sdo_response(uint8_t act_id, const CAN_message_t* msg);
extern float can_last_loads[];
extern uint64_t can_last_times_ns[];

static void reset_can_throttle(void) {
    memset(can_last_loads, 0, sizeof(float) * MAX_NUM_CAN_LINES);
    memset(can_last_times_ns, 0, sizeof(uint64_t) * MAX_NUM_CAN_LINES);
}

static const uint32_t kMsgId = 1u;
static const uint8_t kActId = 1u;
static const uint8_t kCanBus = 0u;

static CAN_message_t make_msg(uint32_t id, uint8_t len, const uint8_t* data) {
    CAN_message_t msg = {0};
    msg.id = id;
    msg.len = len;
    if (data != NULL && len > 0) memcpy(msg.buf, data, MIN(len, 8));
    return msg;
}

static void clear_testing_can_bufs(void) {
    memset(_testing_can_buf, 0xA5, sizeof(_testing_can_buf));
}

static test_result_t assert_testing_bufs_untouched(void) {
    uint8_t untouched[sizeof(struct can_frame)];
    memset(untouched, 0xA5, sizeof(untouched));
    for (uint8_t b = 0; b < MAX_NUM_CAN_LINES; b++) {
        TEST_ASSERT(memcmp(_testing_can_buf[b], untouched, sizeof(untouched)) == 0, "unexpected write bus %u", b);
    }
    return TEST_SUCCESS;
}

static void can_test_setup(void) {
    num_can_lines = 3;
    _testing_can_buf_used = true;
    reset_can_throttle();
    SET_DEFAULT_COMMS_CONFIG();
    clear_testing_can_bufs();
}

static test_result_t assert_sdo_frame(uint8_t bus, uint8_t node_id, const uint8_t expect[8]) {
    struct can_frame got = {0};
    memcpy(&got, _testing_can_buf[bus], sizeof(got));
    TEST_ASSERT(got.can_id == (0x600u | node_id), "can_id got 0x%X", got.can_id);
    TEST_ASSERT(got.can_dlc == 8, "dlc");
    TEST_ASSERT(memcmp(got.data, expect, 8) == 0, "sdo payload mismatch");
    return TEST_SUCCESS;
}

static test_result_t init_ez_motor(uint8_t act_id, bool allow_reinit) {
    EZMotionMotorInitPayload payload = {0};
    payload.actuator_id = act_id;
    payload.allow_reinit = allow_reinit;
    payload.max_telemetry_size = UINT16_MAX;
    for (uint8_t i = 0; i < NUM_EZMotionParamCode; i++) payload.telem_inds[i] = (uint16_t)(100u + i);
    TEST_ASSERT(!ezmotion_handle_initialize_motor(kMsgId, &payload), "init returns false for default success");
    return TEST_SUCCESS;
}


DECLARE_TEST(test_ezmotion_handle_initialize_motor) {
    reset_actuators();
    can_test_setup();

    EZMotionMotorInitPayload bad = {.actuator_id = MAX_NUM_ACTUATORS, .allow_reinit = false};
    TEST_ASSERT(!ezmotion_handle_initialize_motor(kMsgId, &bad), "invalid actuator");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid actuator ID");

    TEST_ASSERT_STACKED(init_ez_motor(0, false), "id 0 init");
    TEST_ASSERT(get_actuator_config(0, ERROR_SCOPE_LOCAL) != NULL, "id 0 allocated");
    free_actuator(0);

    TEST_ASSERT_STACKED(init_ez_motor(kActId, false), "first init");
    TEST_ASSERT(get_actuator_config(kActId, ERROR_SCOPE_LOCAL) != NULL, "config allocated");
    for (uint8_t i = 0; i < NUM_EZMotionParamCode; i++) {
        TEST_ASSERT(get_actuator_param_telemetry(kActId, i, ERROR_SCOPE_LOCAL) != NULL, "telem %u added", i);
    }

    bad.actuator_id = kActId;
    TEST_ASSERT(!ezmotion_handle_initialize_motor(kMsgId, &bad), "duplicate init");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("already initialized");

    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "reinit");
    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_ez_write) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "init");

    EZMotionParam ro = ezmotion_all_params[EZ_MOTION_PARAM_CODE_POSITION];
    TEST_ASSERT(!ez_write(kCanBus, kActId, ro, 0, EZ_MOTION_PARAM_CODE_POSITION), "not writable");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not writable");

    EZMotionParam wr = ezmotion_all_params[EZ_MOTION_PARAM_CODE_TARGET_POSITION];
    TEST_ASSERT(!ez_write(kCanBus, kActId, wr, (int64_t)INT32_MAX + 1, EZ_MOTION_PARAM_CODE_TARGET_POSITION), "out of range");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("out of range");

    clear_testing_can_bufs();
    int64_t value = 12345;
    TEST_ASSERT(ez_write(kCanBus, kActId, wr, value, EZ_MOTION_PARAM_CODE_TARGET_POSITION), "write ok");
    uint8_t expect[8] = {0x23, 0x7A, 0x60, 0x00, 0x39, 0x30, 0x00, 0x00};  // 0x607A write, i32 12345
    TEST_ASSERT_STACKED(assert_sdo_frame(kCanBus, kActId, expect), "target position sdo");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_ezmotion_handle_read_param) {
    reset_actuators();
    can_test_setup();

    EZMotionReadParamPayload payload = {.actuator_id = kActId, .can_bus = kCanBus, .pcode = EZ_MOTION_PARAM_CODE_POSITION};
    TEST_ASSERT(!ezmotion_handle_read_param(kMsgId, &payload), "uninitialized actuator");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "init");

    payload.pcode = NUM_EZMotionParamCode;
    TEST_ASSERT(!ezmotion_handle_read_param(kMsgId, &payload), "invalid pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid EZMotion parameter code");

    payload.pcode = EZ_MOTION_PARAM_CODE_CONTROL_WORD_TS;
    TEST_ASSERT(!ezmotion_handle_read_param(kMsgId, &payload), "timestamp pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid EZMotion parameter code");

    payload.pcode = EZ_MOTION_PARAM_CODE_POSITION;
    clear_testing_can_bufs();
    TEST_ASSERT(!ezmotion_handle_read_param(kMsgId, &payload), "read ok");
    uint8_t expect[8] = {0x40, 0x64, 0x60, 0x00, 0, 0, 0, 0};
    TEST_ASSERT_STACKED(assert_sdo_frame(kCanBus, kActId, expect), "position read sdo");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_ezmotion_handle_write_param) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "init");

    EZMotionWriteParamPayload payload = {
        .actuator_id = kActId, .can_bus = kCanBus, .pcode = EZ_MOTION_PARAM_CODE_CONTROL_WORD, .value = 1
    };
    TEST_ASSERT(!ezmotion_handle_write_param(kMsgId, &payload), "control word blocked");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("control word parameter");

    payload.pcode = EZ_MOTION_PARAM_CODE_TARGET_POSITION;
    payload.value = 500;
    clear_testing_can_bufs();
    TEST_ASSERT(!ezmotion_handle_write_param(kMsgId, &payload), "write ok");
    uint8_t expect[8] = {0x23, 0x7A, 0x60, 0x00, 0xF4, 0x01, 0x00, 0x00};
    TEST_ASSERT_STACKED(assert_sdo_frame(kCanBus, kActId, expect), "write param sdo");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}


DECLARE_TEST(test_ezmotion_handle_set_control_word) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "init");

    EZMotionSetControlWordPayload payload = {
        .actuator_id = kActId,
        .can_bus = kCanBus,
        .enable_operation = true,
        .quick_stop = true,
        .fault_reset = true,
        .switch_on = true,
        .disable_power = false,
        .transition_position = true,
        .halt = false,
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!ezmotion_handle_set_control_word(kMsgId, &payload), "set control word");
    uint8_t expect[8] = {0x2B, 0x40, 0x60, 0x00, 0x9F, 0x00, 0x00, 0x00};  // u16 control word 0x009F
    TEST_ASSERT_STACKED(assert_sdo_frame(kCanBus, kActId, expect), "control word sdo");

    EZMotionSetControlWordPayload halt_payload = {
        .actuator_id = kActId,
        .can_bus = kCanBus,
        .enable_operation = true,
        .quick_stop = true,
        .fault_reset = false,
        .switch_on = true,
        .disable_power = false,
        .transition_position = false,
        .halt = true,
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!ezmotion_handle_set_control_word(kMsgId, &halt_payload), "set halt control word");
    uint8_t expect_halt[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x01, 0x00, 0x00};  // u16 control word 0x010F
    TEST_ASSERT_STACKED(assert_sdo_frame(kCanBus, kActId, expect_halt), "halt control word sdo");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_ezmotion_handle_set_admin_params) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "init");

    EZMotionSetAdminParamsPayload bad = {
        .actuator_id = kActId, .can_bus = kCanBus, .node_id = 2, .voltage_upper_limit = 48, .voltage_lower_limit = 10
    };
    TEST_ASSERT(!ezmotion_handle_set_admin_params(kMsgId, &bad), "voltage out of range");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("out of range");
    TEST_ASSERT_STACKED(assert_testing_bufs_untouched(), "no writes on failure");

    EZMotionSetAdminParamsPayload ok = {
        .actuator_id = kActId, .can_bus = kCanBus, .node_id = 2, .voltage_upper_limit = 48, .voltage_lower_limit = 20
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!ezmotion_handle_set_admin_params(kMsgId, &ok), "admin params ok");
    uint8_t expect[8] = {0x23, 0x0D, 0x20, 0x00, 0x73, 0x61, 0x76, 0x65};  // store parameters
    TEST_ASSERT_STACKED(assert_sdo_frame(kCanBus, kActId, expect), "final admin sdo");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_ezmotion_sdo_response) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "init");

    uint8_t empty_frame[8] = {0};
    CAN_message_t bad_len = make_msg(0x581, 7, empty_frame);
    handle_ezmotion_sdo_response(kActId, &bad_len);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("invalid SDO response length");
    clear_global_error_msg();

    uint8_t err_data[8] = {0x80, 0x64, 0x60, 0x00, 0, 0, 0, 0};
    CAN_message_t msg = make_msg(0x581, 8, err_data);
    handle_ezmotion_sdo_response(kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("SDO ERROR response");
    clear_global_error_msg();

    uint8_t unknown_idx[8] = {0x43, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0};
    msg = make_msg(0x581, 8, unknown_idx);
    handle_ezmotion_sdo_response(kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("unknown SDO pcode");
    clear_global_error_msg();

    uint8_t write_ok[8] = {0x60, 0x7A, 0x60, 0x00, 0, 0, 0, 0};
    msg = make_msg(0x581, 8, write_ok);
    handle_ezmotion_sdo_response(kActId, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "write ack ignored");

    uint8_t bad_size[8] = {0x4F, 0x41, 0x60, 0x00, 0x12, 0, 0, 0};
    msg = make_msg(0x581, 8, bad_size);
    handle_ezmotion_sdo_response(kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("data size mismatch");
    clear_global_error_msg();

    uint8_t bad_status[8] = {0x50, 0x64, 0x60, 0x00, 0, 0, 0, 0};
    msg = make_msg(0x581, 8, bad_status);
    handle_ezmotion_sdo_response(kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("status code unknown");
    clear_global_error_msg();

    int32_t pos = 0x01020304;
    uint8_t read_ok[8] = {0x43, 0x64, 0x60, 0x00, 0x04, 0x03, 0x02, 0x01};
    msg = make_msg(0x581, 8, read_ok);
    handle_ezmotion_sdo_response(kActId, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "read response ok");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(kActId, EZ_MOTION_PARAM_CODE_POSITION, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* ts = get_actuator_param_telemetry(kActId, EZ_MOTION_PARAM_CODE_POSITION_TS, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->end_ind == 1 && ts->end_ind == 1, "value + timestamp inserted");
    TEST_ASSERT(memcmp(telem->values, &pos, sizeof(pos)) == 0, "position value stored");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_ezmotion_can_message) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_ez_motor(kActId, true), "init");

    uint8_t read_ok[8] = {0x43, 0x64, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
    CAN_message_t msg = make_msg(0x581, 8, read_ok);
    handle_ezmotion_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "sdo response path ok");

    msg = make_msg(0x701, 0, NULL);
    handle_ezmotion_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "heartbeat path ok");

    msg = make_msg(0x200, 0, NULL);
    handle_ezmotion_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("invalid/unhandled canopen cmd");
    clear_global_error_msg();

    free_actuator(kActId);
    msg = make_msg(0x581, 8, read_ok);
    handle_ezmotion_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("motor 1 not found");
    clear_global_error_msg();

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

RUN_TESTS();
