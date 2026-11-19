#include "../../../src/logging.cpp"
#include "../../../src/system.cpp"
#include "../../../src/comms/comms.cpp"
#include "../../../src/comms/can.cpp"
#include "../../../src/comms/ethernet.cpp"
#include "../../../src/actuator.cpp"
#include "../../../src/commands/robstride.cpp"
#include "../../framework.hpp"
#include "../../../src/commands/robstride.hpp"
#include <linux/can.h>

extern uint8_t num_can_lines;
extern uint8_t _testing_can_buf[MAX_NUM_CAN_LINES][sizeof(struct can_frame)];
extern bool _testing_can_buf_used;

extern RobMotorConfig* get_rob_motor_config(uint8_t motor_id, ErrorScope scope);
extern void rob_cansend(
    uint8_t bus, RobOperation op, uint8_t e_byte, uint8_t host_id, uint8_t motor_id, const uint8_t* data
);
extern void opcom_map_insert(float value, float min, float max, uint8_t* data);
extern float opcom_map_extract(const uint8_t* data, float min, float max);
extern uint8_t _get_robstride_repsonse_motor_id(const CAN_message_t* msg);
extern uint8_t _get_robstride_response_host_id(const CAN_message_t* msg);
extern uint8_t _get_robstride_opcode(const CAN_message_t* msg);
extern uint8_t _get_robstride_e_byte(const CAN_message_t* msg);
extern void _handle_read_param_response(uint8_t bus, uint8_t motor_id, const CAN_message_t* msg);
extern void _handle_motor_feedback(uint8_t bus, uint8_t motor_id, const CAN_message_t* msg);

static const uint32_t kMsgId = 1u;
static const uint8_t kActId = 1u;
static const uint8_t kCanBus = 0u;
static const uint8_t kHostId = 0xAAu;

static CAN_message_t make_msg(uint32_t id, uint8_t len, const uint8_t* data) {
    CAN_message_t msg = {0};
    msg.id = id;
    msg.len = len;
    if (data != NULL && len > 0) memcpy(msg.buf, data, MIN(len, 8));
    return msg;
}

static RobMotorConfig default_rob_config(void) {
    return (RobMotorConfig){
        .skip_write_param_checks = false,
        .version = ROB_VERSION_RS00,
        .max_velocity = rob_spd_lim[ROB_VERSION_RS00],
        .max_torque = rob_torque_lim[ROB_VERSION_RS00],
        .max_acceleration = ROB_MAX_ACCELERATION,
        .cal_range_min = -1.0f,
        .cal_range_max = 1.0f,
        .homing_offset = 0.0f,
        .direction = 1,
    };
}

static void clear_testing_can_bufs(void) {
    memset(_testing_can_buf, 0xA5, sizeof(_testing_can_buf));
}

static void can_test_setup(void) {
    num_can_lines = 3;
    _testing_can_buf_used = true;
    memset(can_last_loads, 0, sizeof(float) * MAX_NUM_CAN_LINES);
    memset(can_last_times_ns, 0, sizeof(uint64_t) * MAX_NUM_CAN_LINES);
    SET_DEFAULT_COMMS_CONFIG();
    clear_testing_can_bufs();
}

static test_result_t assert_testing_bufs_untouched(void) {
    uint8_t untouched[sizeof(struct can_frame)];
    memset(untouched, 0xA5, sizeof(untouched));
    for (uint8_t b = 0; b < MAX_NUM_CAN_LINES; b++) {
        TEST_ASSERT(memcmp(_testing_can_buf[b], untouched, sizeof(untouched)) == 0, "unexpected write bus %u", b);
    }
    return TEST_SUCCESS;
}

static test_result_t assert_rob_can_frame(
    uint8_t bus, RobOperation op, uint8_t e_byte, uint8_t host_id, uint8_t motor_id, const uint8_t* data
) {
    struct can_frame expect = {0};
    uint32_t can_id = ((uint32_t)op << 24) | ((uint32_t)e_byte << 16) | ((uint32_t)host_id << 8) | motor_id;
    expect.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    expect.can_dlc = 8;
    memcpy(expect.data, data, 8);

    struct can_frame got = {0};
    memcpy(&got, _testing_can_buf[bus], sizeof(got));
    TEST_ASSERT(got.can_id == expect.can_id, "can_id got 0x%X expect 0x%X", got.can_id, expect.can_id);
    TEST_ASSERT(got.can_dlc == 8, "dlc");
    TEST_ASSERT(memcmp(got.data, expect.data, 8) == 0, "payload mismatch");
    return TEST_SUCCESS;
}

static test_result_t init_rob_motor(uint8_t act_id, bool allow_reinit, const RobMotorConfig* cfg) {
    RobMotorInitPayload payload = {0};
    payload.actuator_id = act_id;
    payload.allow_reinit = allow_reinit;
    payload.config = cfg ? *cfg : default_rob_config();
    payload.max_telemetry_size = UINT16_MAX;
    for (uint8_t i = 0; i < NUM_RobParamCode; i++) payload.telem_inds[i] = (uint16_t)(100u + i);
    TEST_ASSERT(!rob_handle_initialize_motor(kMsgId, &payload), "init returns false on success");
    return TEST_SUCCESS;
}

static uint32_t rob_response_id(uint8_t opcode, uint8_t e_byte, uint8_t motor_id) {
    return ((uint32_t)opcode << 24) | ((uint32_t)e_byte << 16) | ((uint32_t)motor_id << 8) | kHostId;
}


DECLARE_TEST(test_opcom_map) {
    struct { float value, min, max; } cases[] = {
        {ROB_ANGLE_MIN, ROB_ANGLE_MIN, ROB_ANGLE_MAX},
        {ROB_ANGLE_MAX, ROB_ANGLE_MIN, ROB_ANGLE_MAX},
        {0.0f, -10.0f, 10.0f},
        {7.5f, 0.0f, 15.0f},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        uint8_t data[2];
        opcom_map_insert(cases[i].value, cases[i].min, cases[i].max, data);
        float got = opcom_map_extract(data, cases[i].min, cases[i].max);
        TEST_ASSERT(fabsf(got - cases[i].value) < 0.001f, "roundtrip %zu: got %f", i, got);
        TEST_ASSERT(data[0] == (uint8_t)((((uint16_t)((cases[i].value - cases[i].min) / (cases[i].max - cases[i].min) * 65535.0f)) >> 8) & 0xFF),
            "BE high byte case %zu", i);
    }
    return TEST_SUCCESS;
}

DECLARE_TEST(test_robstride_can_id_helpers) {
    CAN_message_t msg = {.id = 0x12AB3400u | kHostId};
    TEST_ASSERT(_get_robstride_opcode(&msg) == 0x12, "opcode");
    TEST_ASSERT(_get_robstride_e_byte(&msg) == 0xAB, "e_byte");
    TEST_ASSERT(_get_robstride_repsonse_motor_id(&msg) == 0x34, "motor_id");
    TEST_ASSERT(_get_robstride_response_host_id(&msg) == kHostId, "host_id");

    msg.id = 0x1F000000;
    TEST_ASSERT(_get_robstride_opcode(&msg) == 0x1F, "opcode masked to 5 bits");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_get_rob_motor_config) {
    reset_actuators();

    TEST_ASSERT(get_rob_motor_config(0, ERROR_SCOPE_LOCAL) == NULL, "id 0 uninitialized");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT(get_rob_motor_config(kActId, ERROR_SCOPE_LOCAL) == NULL, "uninitialized");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(0, false, NULL), "init id 0");
    RobMotorConfig* cfg0 = get_rob_motor_config(0, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(cfg0 != NULL && cfg0->version == ROB_VERSION_RS00, "id 0 config");

    TEST_ASSERT_STACKED(init_rob_motor(kActId, false, NULL), "init");
    RobMotorConfig* cfg = get_rob_motor_config(kActId, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(cfg != NULL && cfg->version == ROB_VERSION_RS00, "initialized config");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_cansend_errors) {
    can_test_setup();
    rob_cansend(num_can_lines, ROB_OPERATION_ENABLE, 0, kHostId, kActId, NULL);
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid CAN bus");
    rob_cansend(kCanBus, ROB_OPERATION_ENABLE, 0, kHostId, MAX_NUM_MOTORS, NULL);
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Motor ID out of bounds");
    TEST_ASSERT_STACKED(assert_testing_bufs_untouched(), "no writes on error");
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_handle_initialize_motor) {
    reset_actuators();
    can_test_setup();

    RobMotorInitPayload bad = {.actuator_id = MAX_NUM_ACTUATORS, .allow_reinit = false, .config = default_rob_config()};
    TEST_ASSERT(!rob_handle_initialize_motor(kMsgId, &bad), "invalid actuator");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid actuator ID");

    TEST_ASSERT_STACKED(init_rob_motor(0, false, NULL), "id 0 init");
    TEST_ASSERT(get_actuator_config(0, ERROR_SCOPE_LOCAL) != NULL, "id 0 allocated");
    free_actuator(0);

    struct { const char* err; RobMotorConfig cfg; } invalid[] = {
        {"version", {.version = NUM_RobVersion, .max_velocity = 1, .max_torque = 1, .max_acceleration = 1,
            .cal_range_min = 0, .cal_range_max = 1, .direction = 1}},
        {"max_velocity", {.version = ROB_VERSION_RS00, .max_velocity = rob_spd_lim[ROB_VERSION_RS00] + 1.0f,
            .max_torque = 1, .max_acceleration = 1, .cal_range_min = 0, .cal_range_max = 1, .direction = 1}},
        {"max_torque", {.version = ROB_VERSION_RS00, .max_velocity = 1, .max_torque = -1.0f, .max_acceleration = 1,
            .cal_range_min = 0, .cal_range_max = 1, .direction = 1}},
        {"max_acceleration", {.version = ROB_VERSION_RS00, .max_velocity = 1, .max_torque = 1,
            .max_acceleration = ROB_MAX_ACCELERATION + 1.0f, .cal_range_min = 0, .cal_range_max = 1, .direction = 1}},
        {"cal_range_min", {.version = ROB_VERSION_RS00, .max_velocity = 1, .max_torque = 1, .max_acceleration = 1,
            .cal_range_min = ROB_ANGLE_MIN - 1.0f, .cal_range_max = 1, .direction = 1}},
        {"cal_range_max", {.version = ROB_VERSION_RS00, .max_velocity = 1, .max_torque = 1, .max_acceleration = 1,
            .cal_range_min = 0, .cal_range_max = ROB_ANGLE_MAX + 1.0f, .direction = 1}},
        {"homing_offset", {.version = ROB_VERSION_RS00, .max_velocity = 1, .max_torque = 1, .max_acceleration = 1,
            .cal_range_min = 0, .cal_range_max = 1, .homing_offset = ROB_ANGLE_MAX + 1.0f, .direction = 1}},
        {"direction", {.version = ROB_VERSION_RS00, .max_velocity = 1, .max_torque = 1, .max_acceleration = 1,
            .cal_range_min = 0, .cal_range_max = 1, .direction = 0}},
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        bad = (RobMotorInitPayload){.actuator_id = kActId, .allow_reinit = false, .config = invalid[i].cfg};
        TEST_ASSERT(!rob_handle_initialize_motor(kMsgId, &bad), "bad config %zu", i);
        TEST_ASSERT(strstr(get_local_error_msg(), invalid[i].err) != NULL, "expected err '%s' got '%s'",
            invalid[i].err, get_local_error_msg());
        clear_local_error_msg();
    }

    TEST_ASSERT_STACKED(init_rob_motor(kActId, false, NULL), "first init");
    TEST_ASSERT(get_actuator_config(kActId, ERROR_SCOPE_LOCAL) != NULL, "config allocated");
    for (uint8_t i = 0; i < NUM_RobParamCode; i++) {
        TEST_ASSERT(get_actuator_param_telemetry(kActId, i, ERROR_SCOPE_LOCAL) != NULL, "telem %u", i);
    }

    bad.actuator_id = kActId;
    bad.config = default_rob_config();
    TEST_ASSERT(!rob_handle_initialize_motor(kMsgId, &bad), "duplicate init");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("already initialized");

    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "reinit");
    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_simple_handlers) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    struct { bool (*fn)(uint32_t, void*); RobOperation op; uint8_t data[8]; } cases[] = {
        {(bool (*)(uint32_t, void*))rob_handle_get_device_id, ROB_OPERATION_GET_DEVICE_ID, {0}},
        {(bool (*)(uint32_t, void*))rob_handle_enable, ROB_OPERATION_ENABLE, {0}},
        {(bool (*)(uint32_t, void*))rob_handle_zero_position, ROB_OPERATION_ZERO_POS, {1, 0, 0, 0, 0, 0, 0, 0}},
        {(bool (*)(uint32_t, void*))rob_handle_save_motor_data, ROB_OPERATION_SAVE_MOTOR_DATA, {1, 2, 3, 4, 5, 6, 7, 8}},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        clear_testing_can_bufs();
        uint8_t payload_buf[16] = {kActId, kCanBus};
        TEST_ASSERT(!cases[i].fn(kMsgId, payload_buf), "handler %zu", i);
        TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, cases[i].op, 0, kHostId, kActId, cases[i].data), "can frame");
    }

    RobDisablePayload disable = {.motor_id = kActId, .can_bus = kCanBus, .clear_faults = true};
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_disable(kMsgId, &disable), "disable");
    uint8_t disable_data[8] = {1, 0};
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_DISABLE, 0, kHostId, kActId, disable_data), "disable frame");

    RobSetActiveReportingPayload reporting = {.motor_id = kActId, .can_bus = kCanBus, .active = true};
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_set_active_reporting(kMsgId, &reporting), "active reporting");
    uint8_t reporting_data[8] = {1, 2, 3, 4, 5, 6, 1, 8};
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_SET_ACTIVE_REPORTING, 0, kHostId, kActId,
        reporting_data), "reporting frame");

    RobSetSkipWriteParamChecksPayload skip = {.motor_id = kActId, .can_bus = kCanBus, .skip_write_param_checks = true};
    TEST_ASSERT(!rob_handle_set_skip_write_param_checks(kMsgId, &skip), "skip checks");
    RobMotorConfig* cfg = get_rob_motor_config(kActId, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(cfg->skip_write_param_checks, "flag stored");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_handle_read_param) {
    reset_actuators();
    can_test_setup();

    RobReadParamPayload payload = {.motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_VBUS};
    TEST_ASSERT(!rob_handle_read_param(kMsgId, &payload), "uninitialized");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    payload.pcode = NUM_RobParamCode;
    TEST_ASSERT(!rob_handle_read_param(kMsgId, &payload), "invalid pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid param code");

    payload.pcode = ROB_PARAM_CODE_RUN_MODE_TS;
    TEST_ASSERT(!rob_handle_read_param(kMsgId, &payload), "timestamp pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid param code");

    payload.pcode = ROB_PARAM_CODE_FB_POSITION;
    TEST_ASSERT(!rob_handle_read_param(kMsgId, &payload), "feedback pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid param code");

    const RobParam param = rob_all_params[ROB_PARAM_CODE_VBUS];
    payload.pcode = ROB_PARAM_CODE_VBUS;
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_read_param(kMsgId, &payload), "read ok");
    uint8_t expect[8] = {(uint8_t)(param.param_id & 0xFF), (uint8_t)(param.param_id >> 8), 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_READ_PARAM, 0, kHostId, kActId, expect), "read frame");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_handle_write_param) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    RobWriteParamPayload payload = {.motor_id = kActId, .can_bus = kCanBus, .pcode = NUM_RobParamCode, .value = 0};
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "invalid pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid param code");

    payload.pcode = ROB_PARAM_CODE_VBUS;
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "not writable");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not writable");

    payload = (RobWriteParamPayload){.motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_IQ_REF, .value = NAN};
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "nan");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Cannot send NaN");

    payload = (RobWriteParamPayload){
        .motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_SPD_REF, .value = 999.0f, .clamp = false
    };
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "out of range no clamp");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("out of motor config range");

    RobMotorConfig* cfg = get_rob_motor_config(kActId, ERROR_SCOPE_LOCAL);
    cfg->direction = -1;
    cfg->homing_offset = 0.25f;
    cfg->cal_range_min = -2.0f;
    cfg->cal_range_max = 2.0f;
    payload = (RobWriteParamPayload){
        .motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_LOC_REF, .value = 1.0f, .clamp = true
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "loc ref write");
    float wire = -1.0f * (1.0f + 0.25f);
    uint8_t expect[8];
    memset(expect, 0, sizeof(expect));
    const RobParam loc = rob_all_params[ROB_PARAM_CODE_LOC_REF];
    expect[0] = (uint8_t)(loc.param_id & 0xFF);
    expect[1] = (uint8_t)(loc.param_id >> 8);
    memcpy(expect + 4, &wire, 4);
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_WRITE_PARAM, 0, kHostId, kActId, expect), "loc ref frame");

    payload = (RobWriteParamPayload){
        .motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_VEL_MAX, .value = 0.0f, .clamp = true
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "vel max zero");
    float tiny = 1e-30f;
    memset(expect, 0, sizeof(expect));
    expect[0] = (uint8_t)(rob_all_params[ROB_PARAM_CODE_VEL_MAX].param_id & 0xFF);
    expect[1] = (uint8_t)(rob_all_params[ROB_PARAM_CODE_VEL_MAX].param_id >> 8);
    memcpy(expect + 4, &tiny, 4);
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_WRITE_PARAM, 0, kHostId, kActId, expect), "vel max tiny");

    payload = (RobWriteParamPayload){
        .motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_RUN_MODE, .value = 2.0f, .clamp = true
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "run mode u8");
    memset(expect, 0, sizeof(expect));
    expect[0] = (uint8_t)(rob_all_params[ROB_PARAM_CODE_RUN_MODE].param_id & 0xFF);
    expect[1] = (uint8_t)(rob_all_params[ROB_PARAM_CODE_RUN_MODE].param_id >> 8);
    expect[4] = 2;
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_WRITE_PARAM, 0, kHostId, kActId, expect), "run mode frame");

    payload = (RobWriteParamPayload){
        .motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_RUN_MODE, .value = 99.0f, .clamp = false
    };
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "integer out of range");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("out of parameter range");

    cfg->skip_write_param_checks = true;
    payload = (RobWriteParamPayload){
        .motor_id = kActId, .can_bus = kCanBus, .pcode = ROB_PARAM_CODE_RUN_MODE, .value = 99.0f, .clamp = false
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_write_param(kMsgId, &payload), "skip checks");
    memset(expect, 0, sizeof(expect));
    expect[0] = (uint8_t)(rob_all_params[ROB_PARAM_CODE_RUN_MODE].param_id & 0xFF);
    expect[1] = (uint8_t)(rob_all_params[ROB_PARAM_CODE_RUN_MODE].param_id >> 8);
    expect[4] = 99;
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_WRITE_PARAM, 0, kHostId, kActId, expect), "skipped range check");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_handle_operation_control) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    RobOperationControlPayload payload = {
        .motor_id = kActId, .can_bus = kCanBus, .torque = 0, .angle = 0, .velocity = 0, .kp = 1, .kd = 1, .clamp = true
    };
    payload.torque = NAN;
    TEST_ASSERT(!rob_handle_operation_control(kMsgId, &payload), "nan torque");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Cannot send NaN");

    payload.torque = 0;
    payload.kp = rob_kp_lim[ROB_VERSION_RS00] + 1.0f;
    payload.clamp = false;
    TEST_ASSERT(!rob_handle_operation_control(kMsgId, &payload), "kp out of range");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("out of range");

    RobMotorConfig* cfg = get_rob_motor_config(kActId, ERROR_SCOPE_LOCAL);
    cfg->direction = -1;
    cfg->homing_offset = 0.5f;
    cfg->cal_range_min = -1.0f;
    cfg->cal_range_max = 1.0f;
    payload = (RobOperationControlPayload){
        .motor_id = kActId, .can_bus = kCanBus,
        .torque = 1.0f, .angle = 0.25f, .velocity = -2.0f, .kp = 10.0f, .kd = 1.0f, .clamp = true
    };
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_operation_control(kMsgId, &payload), "op control ok");

    float angle = -1.0f * (0.25f + 0.5f);
    float velocity = -1.0f * -2.0f;
    float torque = -1.0f * 1.0f;
    uint8_t data[10] = {0};
    opcom_map_insert(angle, ROB_ANGLE_MIN, ROB_ANGLE_MAX, data + 0);
    opcom_map_insert(velocity, rob_n_spd_lim[cfg->version], rob_spd_lim[cfg->version], data + 2);
    opcom_map_insert(10.0f, 0, rob_kp_lim[cfg->version], data + 4);
    opcom_map_insert(1.0f, 0, rob_kd_lim[cfg->version], data + 6);
    opcom_map_insert(torque, rob_n_torque_lim[cfg->version], rob_torque_lim[cfg->version], data + 8);
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_OPERATION_CONTROL, data[8], data[9], kActId, data), "op frame");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_admin_handlers) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    RobSetIdPayload set_id = {.motor_id = kActId, .can_bus = kCanBus, .new_id = 0xFE};
    TEST_ASSERT(!rob_handle_set_id(kMsgId, &set_id), "bad new id");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid new ID");

    set_id.new_id = MAX_NUM_MOTORS;
    TEST_ASSERT(!rob_handle_set_id(kMsgId, &set_id), "new id outside teensy table");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid new ID");

    set_id.new_id = 7;
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_set_id(kMsgId, &set_id), "set id ok");
    uint8_t set_id_data[8] = {0};
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_SET_ID, 7, kHostId, kActId, set_id_data), "set id");

    RobBeginFlashPayload begin = {.motor_id = kActId, .can_bus = kCanBus, .device_id = {1, 2, 3, 4, 5, 6, 7, 8}};
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_begin_flash(kMsgId, &begin), "begin flash");
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_BEGIN_FLASH, 0, kHostId, kActId, begin.device_id), "begin flash frame");

    RobFlashFileInfoPayload info = {.motor_id = kActId, .can_bus = kCanBus, .file_size = 0x01020304, .num_packets = 0x05060708};
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_flash_file_info(kMsgId, &info), "flash info");
    uint8_t info_data[8];
    memcpy(info_data, &info.file_size, 4);
    memcpy(info_data + 4, &info.num_packets, 4);
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_FLASH_FILE_INFO, 0, kHostId, kActId, info_data), "info frame");

    RobFlashDataPayload data = {.motor_id = kActId, .can_bus = kCanBus, .packet_index = 0x0102, .data = {9, 8, 7, 6, 5, 4, 3, 2}};
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_flash_data(kMsgId, &data), "flash data");
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_FLASH_DATA, 0x01, 0x02, kActId, data.data), "data frame");

    RobEndFlashPayload end = {.motor_id = kActId, .can_bus = kCanBus, .num_packets = 0x0A0B0C0D};
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_end_flash(kMsgId, &end), "end flash");
    uint8_t end_data[8] = {0};
    memcpy(end_data, &end.num_packets, 4);
    TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_END_FLASH, 0, 0, kActId, end_data), "end frame");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_read_param_response) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    CAN_message_t msg = {0};
    _handle_read_param_response(kCanBus, kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Unknown RobStride param ID");
    clear_global_error_msg();

    uint16_t bad_id = 0x7999;
    memcpy(msg.buf, &bad_id, 2);
    _handle_read_param_response(kCanBus, kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Unknown RobStride param ID");
    clear_global_error_msg();

    bad_id = rob_all_params[ROB_PARAM_CODE_RUN_MODE_TS].param_id;
    memcpy(msg.buf, &bad_id, 2);
    _handle_read_param_response(kCanBus, kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Unknown RobStride param ID");
    clear_global_error_msg();

    uint16_t param_id = rob_all_params[ROB_PARAM_CODE_IQ_REF].param_id;
    float nan_val = NAN;
    memcpy(msg.buf, &param_id, 2);
    memcpy(msg.buf + 4, &nan_val, 4);
    _handle_read_param_response(kCanBus, kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("RobStride NaN value");
    clear_global_error_msg();

    RobMotorConfig* cfg = get_rob_motor_config(kActId, ERROR_SCOPE_LOCAL);
    cfg->direction = -1;
    cfg->homing_offset = 0.5f;
    float wire = 2.0f;
    param_id = rob_all_params[ROB_PARAM_CODE_LOC_REF].param_id;
    memcpy(msg.buf, &param_id, 2);
    memcpy(msg.buf + 4, &wire, 4);
    _handle_read_param_response(kCanBus, kActId, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "loc ref response ok");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_LOC_REF, ERROR_SCOPE_LOCAL);
    float expect_host = -1.0f * wire - 0.5f;
    TEST_ASSERT(telem->end_ind == 1, "value inserted");
    TEST_ASSERT(fabsf(((float*)telem->values)[0] - expect_host) < 1e-5f, "host loc ref %f", ((float*)telem->values)[0]);

    uint32_t mode = 2;
    param_id = rob_all_params[ROB_PARAM_CODE_RUN_MODE].param_id;
    memcpy(msg.buf, &param_id, 2);
    msg.buf[4] = (uint8_t)mode;
    _handle_read_param_response(kCanBus, kActId, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "u8 response ok");
    telem = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_RUN_MODE, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(((uint8_t*)telem->values)[0] == mode, "run mode stored");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_motor_feedback) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    RobMotorConfig* cfg = get_rob_motor_config(kActId, ERROR_SCOPE_LOCAL);
    cfg->direction = -1;
    cfg->homing_offset = 0.5f;

    float wire_angle = 1.0f, wire_vel = -3.0f, wire_torq = 2.0f;
    uint8_t buf[8] = {0};
    opcom_map_insert(wire_angle, ROB_ANGLE_MIN, ROB_ANGLE_MAX, buf + 0);
    opcom_map_insert(wire_vel, rob_n_spd_lim[cfg->version], rob_spd_lim[cfg->version], buf + 2);
    opcom_map_insert(wire_torq, rob_n_torque_lim[cfg->version], rob_torque_lim[cfg->version], buf + 4);
    buf[6] = 0x01;
    buf[7] = 0xF4;  // 500 -> 50.0 C

    uint8_t e_byte = (uint8_t)((2u << 6) | 0x15u);  // mode=2, errors=0x15
    CAN_message_t msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, e_byte, kActId), 8, buf);
    _handle_motor_feedback(kCanBus, kActId, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "feedback ok");

    float expect_angle = -1.0f * wire_angle - 0.5f;
    float expect_vel = -1.0f * wire_vel;
    float expect_torq = -1.0f * wire_torq;
    ActuatorParamTelemetry* pos = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_POSITION, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* vel = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_VELOCITY, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* torq = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_TORQUE, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* temp = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_TEMP, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* errors = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_ERRORS, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* mode = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_MODE, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(fabsf(((float*)pos->values)[0] - expect_angle) < 1e-3f, "fb angle");
    TEST_ASSERT(fabsf(((float*)vel->values)[0] - expect_vel) < 1e-3f, "fb vel");
    TEST_ASSERT(fabsf(((float*)torq->values)[0] - expect_torq) < 1e-3f, "fb torque");
    TEST_ASSERT(fabsf(((float*)temp->values)[0] - 50.0f) < 1e-3f, "fb temp");
    TEST_ASSERT(((uint16_t*)errors->values)[0] == 0x15u, "fb errors");
    TEST_ASSERT(((uint8_t*)mode->values)[0] == 2u, "fb mode");

    // Over-current only. Earlier bits stay set until disable(clear_faults).
    e_byte = (uint8_t)((2u << 6) | 0x02u);
    msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, e_byte, kActId), 8, buf);
    _handle_motor_feedback(kCanBus, kActId, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "second feedback ok");
    TEST_ASSERT(errors->end_ind == 2, "second error sample");
    TEST_ASSERT(((uint16_t*)errors->values)[1] == 0x17u, "errors or'd");

    e_byte = (uint8_t)(2u << 6);
    msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, e_byte, kActId), 8, buf);
    _handle_motor_feedback(kCanBus, kActId, &msg);
    TEST_ASSERT(((uint16_t*)errors->values)[2] == 0x17u, "zero frame keeps tally");

    RobDisablePayload keep = {.motor_id = kActId, .can_bus = kCanBus, .clear_faults = false};
    TEST_ASSERT(!rob_handle_disable(kMsgId, &keep), "disable keeps errors");
    TEST_ASSERT(errors->end_ind == 3, "disable without clear_faults adds no sample");
    TEST_ASSERT(((uint16_t*)errors->values)[2] == 0x17u, "tally kept");

    RobDisablePayload clear = {.motor_id = kActId, .can_bus = kCanBus, .clear_faults = true};
    TEST_ASSERT(!rob_handle_disable(kMsgId, &clear), "disable clears errors");
    TEST_ASSERT(errors->end_ind == 4, "clear stored a sample");
    TEST_ASSERT(((uint16_t*)errors->values)[3] == 0u, "errors cleared");

    cfg->version = NUM_RobVersion;
    _handle_motor_feedback(kCanBus, kActId, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Invalid RobStride version");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_robstride_can_message) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    free_actuator(kActId);
    CAN_message_t msg = make_msg(rob_response_id(ROB_OPERATION_READ_PARAM, 0, kActId), 8, NULL);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("not initialized");
    clear_global_error_msg();

    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "reinit");

    msg = make_msg(rob_response_id(ROB_OPERATION_READ_PARAM, 0, kActId), 7, NULL);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Invalid RobStride msg len");
    clear_global_error_msg();

    msg = make_msg(rob_response_id(0x1F, 0, kActId), 8, NULL);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Unknown RobStride opcode");
    clear_global_error_msg();

    msg = make_msg(rob_response_id(ROB_OPERATION_FAULT_FEEDBACK, 0, kActId), 8, NULL);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "empty fault feedback");
    ActuatorParamTelemetry* fault_errors = get_actuator_param_telemetry(
        kActId, ROB_PARAM_CODE_FB_ERRORS, ERROR_SCOPE_LOCAL
    );
    TEST_ASSERT(fault_errors->end_ind == 1, "fault stored errors");
    TEST_ASSERT(((uint16_t*)fault_errors->values)[0] == 0u, "empty fault is zero");

    uint16_t param_id = rob_all_params[ROB_PARAM_CODE_VBUS].param_id;
    float vbus = 24.5f;
    uint8_t read_buf[8] = {0};
    memcpy(read_buf, &param_id, 2);
    memcpy(read_buf + 4, &vbus, 4);
    msg = make_msg(rob_response_id(ROB_OPERATION_READ_PARAM, 0, kActId), 8, read_buf);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "read param routed");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_VBUS, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->end_ind == 1, "vbus inserted");
    TEST_ASSERT(fabsf(((float*)telem->values)[0] - vbus) < 1e-5f, "vbus value");

    uint8_t fb_buf[8] = {0};
    opcom_map_insert(0.0f, ROB_ANGLE_MIN, ROB_ANGLE_MAX, fb_buf);
    opcom_map_insert(0.0f, rob_n_spd_lim[ROB_VERSION_RS00], rob_spd_lim[ROB_VERSION_RS00], fb_buf + 2);
    opcom_map_insert(0.0f, rob_n_torque_lim[ROB_VERSION_RS00], rob_torque_lim[ROB_VERSION_RS00], fb_buf + 4);
    msg = make_msg(rob_response_id(ROB_OPERATION_SET_ACTIVE_REPORTING, 0x40, kActId), 8, fb_buf);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "active reporting routed");
    telem = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_MODE, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->end_ind == 1, "feedback telem inserted");

    // Device-id replies use host 0xFE. The 8 data bytes are a big-endian uint64.
    uint8_t device_bytes[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x04, 0x00, 0x10};
    msg = make_msg(((uint32_t)ROB_OPERATION_GET_DEVICE_ID << 24) | ((uint32_t)kActId << 8) | 0xFEu, 8, device_bytes);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "device id frame");
    telem = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_DEVICE_ID, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* device_ts = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_DEVICE_ID_TS, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->end_ind == 1, "device id stored");
    TEST_ASSERT(((uint64_t*)telem->values)[0] == 0xDEADBEEF00040010ULL, "device id big-endian");
    TEST_ASSERT(device_ts->end_ind == 1, "device id timestamp stored");

    // Flash replies store only the operation code in FLASH_INFO.
    RobOperation flash_ops[] = {
        ROB_OPERATION_BEGIN_FLASH,
        ROB_OPERATION_FLASH_FILE_INFO,
        ROB_OPERATION_FLASH_DATA,
        ROB_OPERATION_END_FLASH,
    };
    telem = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FLASH_INFO, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* flash_ts = get_actuator_param_telemetry(
        kActId, ROB_PARAM_CODE_FLASH_INFO_TS, ERROR_SCOPE_LOCAL
    );
    TEST_ASSERT(telem->end_ind == 0, "flash info empty");
    uint8_t flash_payload[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    for (size_t i = 0; i < sizeof(flash_ops) / sizeof(flash_ops[0]); i++) {
        msg = make_msg(rob_response_id(flash_ops[i], 0, kActId), 8, flash_payload);
        handle_robstride_can_message(kCanBus, &msg);
        TEST_ASSERT(get_global_error_msg() == NULL, "flash op 0x%X", flash_ops[i]);
        TEST_ASSERT(telem->end_ind == (uint16_t)(i + 1), "flash info count %zu", i);
        TEST_ASSERT(((uint8_t*)telem->values)[i] == flash_ops[i], "stored flash opcode 0x%X", flash_ops[i]);
        TEST_ASSERT(flash_ts->end_ind == (uint16_t)(i + 1), "flash timestamp count %zu", i);
    }

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_get_firmware_version) {
    reset_actuators();
    can_test_setup();

    RobGetFirmwareVersionPayload payload = {.motor_id = kActId, .can_bus = kCanBus};
    TEST_ASSERT(!rob_handle_get_firmware_version(kMsgId, &payload), "uninitialized");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    payload.can_bus = num_can_lines;
    clear_testing_can_bufs();
    TEST_ASSERT(!rob_handle_get_firmware_version(kMsgId, &payload), "bad bus");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid CAN bus");
    TEST_ASSERT_STACKED(assert_testing_bufs_untouched(), "no frame on send failure");

    payload.can_bus = kCanBus;
    uint8_t request[8] = {0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < 2; i++) {
        clear_testing_can_bufs();
        TEST_ASSERT(!rob_handle_get_firmware_version(kMsgId, &payload), "send %d", i);
        TEST_ASSERT_STACKED(assert_rob_can_frame(kCanBus, ROB_OPERATION_DISABLE, 0, kHostId, kActId, request),
            "firmware request frame %d", i);
    }

    uint8_t fb_buf[8] = {0};
    opcom_map_insert(0.0f, ROB_ANGLE_MIN, ROB_ANGLE_MAX, fb_buf);
    opcom_map_insert(0.0f, rob_n_spd_lim[ROB_VERSION_RS00], rob_spd_lim[ROB_VERSION_RS00], fb_buf + 2);
    opcom_map_insert(0.0f, rob_n_torque_lim[ROB_VERSION_RS00], rob_torque_lim[ROB_VERSION_RS00], fb_buf + 4);
    CAN_message_t msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, 0x40, kActId), 8, fb_buf);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "normal feedback");
    ActuatorParamTelemetry* mode = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_MODE, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* fw = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FIRMWARE_VERSION, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(mode->end_ind == 1, "normal feedback stored");
    TEST_ASSERT(fw->end_ind == 0, "firmware telem empty");

    uint8_t version_bytes[8] = {0x00, 0xC4, 'V', '1', '.', '2', '.', '3'};
    uint16_t fb_end = mode->end_ind;
    msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, 0, kActId), 8, version_bytes);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "version frame ok");
    TEST_ASSERT(mode->end_ind == fb_end, "feedback not updated");
    TEST_ASSERT(fw->end_ind == 1, "version stored");
    uint64_t expect = 0;
    memcpy(&expect, version_bytes, sizeof(expect));
    TEST_ASSERT(((uint64_t*)fw->values)[0] == expect, "version bytes");
    ActuatorParamTelemetry* fw_ts = get_actuator_param_telemetry(
        kActId, ROB_PARAM_CODE_FIRMWARE_VERSION_TS, ERROR_SCOPE_LOCAL
    );
    TEST_ASSERT(fw_ts->end_ind == 1, "version timestamp stored");

    msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, 0, kActId), 8, version_bytes);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(fw->end_ind == 2, "second version stored");
    TEST_ASSERT(mode->end_ind == fb_end, "feedback still not updated");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_rob_fault_feedback_errors) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_rob_motor(kActId, true, NULL), "init");

    // wire bit 0 -> over-temperature (enum bit 2); wire bit 16 -> A-phase overcurrent (enum bit 8)
    // The last four bytes are a warning value and are ignored.
    uint32_t fault = (1u << 0) | (1u << 16);
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44};
    memcpy(buf, &fault, sizeof(fault));
    CAN_message_t msg = make_msg(rob_response_id(ROB_OPERATION_FAULT_FEEDBACK, 0, kActId), 8, buf);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "fault ok");
    ActuatorParamTelemetry* errors = get_actuator_param_telemetry(kActId, ROB_PARAM_CODE_FB_ERRORS, ERROR_SCOPE_LOCAL);
    uint16_t expect = (uint16_t)((1u << ROB_MOTOR_ERROR_OVER_TEMPERATURE) | (1u << ROB_MOTOR_ERROR_A_PHASE_OVERCURRENT));
    TEST_ASSERT(((uint16_t*)errors->values)[0] == expect, "fault bits remapped");

    uint8_t fb[8] = {0};
    opcom_map_insert(0.0f, ROB_ANGLE_MIN, ROB_ANGLE_MAX, fb);
    opcom_map_insert(0.0f, rob_n_spd_lim[ROB_VERSION_RS00], rob_spd_lim[ROB_VERSION_RS00], fb + 2);
    opcom_map_insert(0.0f, rob_n_torque_lim[ROB_VERSION_RS00], rob_torque_lim[ROB_VERSION_RS00], fb + 4);
    msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, 0x01, kActId), 8, fb);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "feedback after fault");
    expect = (uint16_t)(expect | (uint16_t)(1u << ROB_MOTOR_ERROR_UNDER_VOLTAGE));
    TEST_ASSERT(((uint16_t*)errors->values)[1] == expect, "feedback or'd onto fault tally");

    // Undefined feedback bit 5 is ignored and left out of the tally.
    msg = make_msg(rob_response_id(ROB_OPERATION_MOTOR_FEEDBACK, 0x20, kActId), 8, fb);
    handle_robstride_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "unknown feedback bit ignored");
    TEST_ASSERT(((uint16_t*)errors->values)[2] == expect, "unknown bit dropped");

    RobDisablePayload clear = {.motor_id = kActId, .can_bus = kCanBus, .clear_faults = true};
    TEST_ASSERT(!rob_handle_disable(kMsgId, &clear), "disable clears errors");
    TEST_ASSERT(((uint16_t*)errors->values)[3] == 0u, "cleared");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

RUN_TESTS();
