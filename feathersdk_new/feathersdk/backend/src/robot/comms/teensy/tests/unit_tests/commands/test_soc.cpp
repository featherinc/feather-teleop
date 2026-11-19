#include "../../../src/logging.cpp"
#include "../../../src/system.cpp"
#include "../../../src/comms/comms.cpp"
#include "../../../src/comms/can.cpp"
#include "../../../src/comms/ethernet.cpp"
#include "../../../src/actuator.cpp"
#include "../../../src/commands/soc.cpp"
#include "../../../src/commands/ezmotion.cpp"
#include "../../../src/commands/robstride.cpp"
#include "../../framework.hpp"
#include "../../../src/commands/soc.hpp"
#include <linux/can.h>

extern uint8_t num_can_lines;
extern uint8_t _testing_can_buf[MAX_NUM_CAN_LINES][sizeof(struct can_frame)];
extern bool _testing_can_buf_used;
extern float can_last_loads[];
extern uint64_t can_last_times_ns[];
extern uint8_t soc_actuator_id;
extern bool soc_initialized;

static const uint32_t kMsgId = 1u;
static const uint8_t kActId = 0x7Du;
static const uint8_t kCanBus = 0u;

static void reset_can_throttle(void) {
    memset(can_last_loads, 0, sizeof(float) * MAX_NUM_CAN_LINES);
    memset(can_last_times_ns, 0, sizeof(uint64_t) * MAX_NUM_CAN_LINES);
}

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

static void can_test_setup(void) {
    num_can_lines = 3;
    _testing_can_buf_used = true;
    reset_can_throttle();
    SET_DEFAULT_COMMS_CONFIG();
    clear_testing_can_bufs();
    soc_initialized = false;
}

static test_result_t init_soc(void) {
    SOCInitPayload payload = {0};
    payload.actuator_id = kActId;
    payload.allow_reinit = true;
    for (uint8_t i = 0; i < NUM_SOCParamCode; i++) payload.telem_inds[i] = i;
    TEST_ASSERT(!soc_handle_initialize_soc(kMsgId, &payload), "init");
    TEST_ASSERT(soc_initialized && soc_actuator_id == kActId, "soc id recorded");
    return TEST_SUCCESS;
}

static test_result_t assert_bridge_frame(uint8_t dlc, const uint8_t* expect) {
    struct can_frame got = {0};
    memcpy(&got, _testing_can_buf[kCanBus], sizeof(got));
    TEST_ASSERT(got.can_id == SOCBRIDGE_CAN_ID, "can_id got 0x%X", got.can_id);
    TEST_ASSERT(got.can_dlc == dlc, "dlc got %u", got.can_dlc);
    TEST_ASSERT(memcmp(got.data, expect, dlc) == 0, "bridge payload mismatch");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_soc_handle_read_param) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_soc(), "init");

    SOCReadParamPayload bad = {.bus = kCanBus, .node_id = 1, .pcode = NUM_SOCParamCode};
    TEST_ASSERT(!soc_handle_read_param(kMsgId, &bad), "bad pcode");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Unknown SOC pcode");

    bad.pcode = SOC_PARAM_CODE_INA_CONFIG_TS;
    TEST_ASSERT(!soc_handle_read_param(kMsgId, &bad), "timestamp not readable");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not readable");

    SOCReadParamPayload ok = {.bus = kCanBus, .node_id = 1, .pcode = SOC_PARAM_CODE_INA_VBUS};
    clear_testing_can_bufs();
    TEST_ASSERT(!soc_handle_read_param(kMsgId, &ok), "read vbus");
    uint8_t expect[4] = {SOC_BRIDGE_CMD_READ_REGISTER, SOC_BRIDGE_STATUS_CMD_SENT, INA_I2C_ADDR, 0x05};
    TEST_ASSERT_STACKED(assert_bridge_frame(4, expect), "vbus read frame");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_soc_handle_write_param) {
    reset_actuators();
    can_test_setup();
    TEST_ASSERT_STACKED(init_soc(), "init");

    SOCWriteParamPayload ro = {.bus = kCanBus, .pcode = SOC_PARAM_CODE_INA_VBUS, .value = 1};
    TEST_ASSERT(!soc_handle_write_param(kMsgId, &ro), "vbus not writable");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not writable");

    SOCWriteParamPayload ina = {.bus = kCanBus, .pcode = SOC_PARAM_CODE_INA_SHUNT_CAL, .value = 0x052D};
    clear_testing_can_bufs();
    TEST_ASSERT(!soc_handle_write_param(kMsgId, &ina), "write shunt cal");
    uint8_t ina_expect[6] = {
        SOC_BRIDGE_CMD_WRITE_REGISTER, SOC_BRIDGE_STATUS_CMD_SENT, INA_I2C_ADDR, 0x02, 0x05, 0x2D
    };
    TEST_ASSERT_STACKED(assert_bridge_frame(6, ina_expect), "ina big-endian write");

    SOCWriteParamPayload bq = {.bus = kCanBus, .pcode = SOC_PARAM_CODE_BQ_CONTROL, .value = 0x0001};
    clear_testing_can_bufs();
    TEST_ASSERT(!soc_handle_write_param(kMsgId, &bq), "write bq control");
    uint8_t bq_expect[6] = {
        SOC_BRIDGE_CMD_WRITE_REGISTER, SOC_BRIDGE_STATUS_CMD_SENT, BQ_I2C_ADDR, 0x00, 0x01, 0x00
    };
    TEST_ASSERT_STACKED(assert_bridge_frame(6, bq_expect), "bq little-endian write");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_soc_can_message) {
    reset_actuators();
    can_test_setup();

    uint8_t vbus_ok[8] = {SOC_BRIDGE_CMD_READ_REGISTER, SOC_BRIDGE_STATUS_OK, INA_I2C_ADDR, 0x05, 0x12, 0x34, 0, 0};
    CAN_message_t msg = make_msg(SOCBRIDGE_CAN_ID, 8, vbus_ok);
    handle_soc_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("no initialized SOC");
    clear_global_error_msg();

    TEST_ASSERT_STACKED(init_soc(), "init");

    uint8_t loopback[4] = {SOC_BRIDGE_CMD_READ_REGISTER, SOC_BRIDGE_STATUS_CMD_SENT, INA_I2C_ADDR, 0x05};
    msg = make_msg(SOCBRIDGE_CAN_ID, 4, loopback);
    handle_soc_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "loopback ignored");

    uint8_t bad_op[8] = {0x40, SOC_BRIDGE_STATUS_BAD_OPERATION, 0, 0, 0, 0, 0, 0};
    msg = make_msg(SOCBRIDGE_CAN_ID, 8, bad_op);
    handle_soc_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("error status");
    clear_global_error_msg();

    uint8_t unknown[8] = {SOC_BRIDGE_CMD_READ_REGISTER, SOC_BRIDGE_STATUS_OK, 0x11, 0x22, 0, 0, 0, 0};
    msg = make_msg(SOCBRIDGE_CAN_ID, 8, unknown);
    handle_soc_can_message(kCanBus, &msg);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("unknown register");
    clear_global_error_msg();

    msg = make_msg(SOCBRIDGE_CAN_ID, 8, vbus_ok);
    handle_can_message(kCanBus, &msg);
    TEST_ASSERT(get_global_error_msg() == NULL, "dispatched read ok");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(kActId, SOC_PARAM_CODE_INA_VBUS, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* ts = get_actuator_param_telemetry(kActId, SOC_PARAM_CODE_INA_VBUS_TS, ERROR_SCOPE_LOCAL);
    uint16_t vbus = 0x1234;
    TEST_ASSERT(telem->end_ind == 1 && ts->end_ind == 1, "vbus + timestamp");
    TEST_ASSERT(memcmp(telem->values, &vbus, sizeof(vbus)) == 0, "ina big-endian value");

    uint8_t soc_ok[8] = {SOC_BRIDGE_CMD_READ_REGISTER, SOC_BRIDGE_STATUS_OK, BQ_I2C_ADDR, 0x02, 0x2A, 0x00, 0, 0};
    msg = make_msg(SOCBRIDGE_CAN_ID, 8, soc_ok);
    handle_soc_can_message(kCanBus, &msg);
    telem = get_actuator_param_telemetry(kActId, SOC_PARAM_CODE_BQ_STATE_OF_CHARGE, ERROR_SCOPE_LOCAL);
    uint16_t soc = 42;
    TEST_ASSERT(memcmp(telem->values, &soc, sizeof(soc)) == 0, "bq little-endian value");

    uint8_t write_ack[8] = {
        SOC_BRIDGE_CMD_WRITE_REGISTER, SOC_BRIDGE_STATUS_OK, INA_I2C_ADDR, 0x02, 0x05, 0x2D, 0, 0
    };
    msg = make_msg(SOCBRIDGE_CAN_ID, 8, write_ack);
    handle_soc_can_message(kCanBus, &msg);
    telem = get_actuator_param_telemetry(kActId, SOC_PARAM_CODE_INA_SHUNT_CAL, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->end_ind == 0, "write ack does not insert");
    TEST_ASSERT(get_global_error_msg() == NULL, "write ack ok");

    reset_actuators();
    _testing_can_buf_used = false;
    return TEST_SUCCESS;
}

RUN_TESTS();
