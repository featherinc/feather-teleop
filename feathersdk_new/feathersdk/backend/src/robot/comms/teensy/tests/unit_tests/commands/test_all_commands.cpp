#include "../../../src/logging.cpp"
#include "../../../src/system.cpp"
#include "../../../src/comms/comms.cpp"
#include "../../../src/comms/can.cpp"
#include "../../../src/comms/ethernet.cpp"
#include "../../../src/actuator.cpp"
#include "../../../src/commands/robstride.cpp"
#include "../../../src/commands/ezmotion.cpp"
#include "../../../src/commands/realhand.cpp"
#include "../../../src/commands/soc.cpp"
#include "../../../src/commands/all_commands.cpp"
#include "../../../src/ota_flash/FlashTxx.cpp"
#include "../../../src/ota_flash/FXUtil.cpp"
#include "../../framework.hpp"
#include "../../../src/commands/all_commands.hpp"
#include <linux/can.h>

extern Actuator actuators[MAX_NUM_ACTUATORS];
extern CommsConfig comms_config;
extern int64_t timestamp_offset_ns;
extern uint32_t candump_telem_start;
extern uint32_t candump_telem_end;
extern uint8_t num_can_lines;
extern bool logging_enabled;
extern void _insert_candump_telemetry(uint8_t bus, bool sent, const CAN_message_t* msg);
extern uint8_t _testing_can_buf[MAX_NUM_CAN_LINES][sizeof(struct can_frame)];
extern bool _testing_can_buf_used;
extern float can_last_loads[MAX_NUM_CAN_LINES];
extern uint64_t can_last_times_ns[MAX_NUM_CAN_LINES];

typedef struct { uint32_t tag; } TestConfig;

static const uint32_t kMsgId = 55u;

static InitializeConfig valid_init_config(void) {
    InitializeConfig init = {
        .comms_config = {
            .server_run_time_nanos = 5000,
            .can_read_run_time_nanos = 6000,
            .max_can_rate = 50.0f,
            .can_leniency = 10.0f,
        },
    };
    return init;
}


DECLARE_TEST(test_handle_echo) {
    uint8_t payload[ECHO_PAYLOAD_SIZE];
    for (uint32_t i = 0; i < ECHO_PAYLOAD_SIZE; i++) payload[i] = (uint8_t)(0xC0u + i);

    eth_write_buf_clear();
    TEST_ASSERT(handle_echo(kMsgId, payload), "echo returns true");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(kMsgId, TEENSY_COMMAND_OPCODE_RESP_ECHO, payload, ECHO_PAYLOAD_SIZE), "echo payload");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_initialize) {
    reset_actuators();
    clear_candump_telemetry();
    SET_DEFAULT_COMMS_CONFIG();
    timestamp_offset_ns = 12345;

    CommsConfig base = DEFAULT_COMMS_CONFIG;
    struct { CommsConfig cfg; const char* err; } bad[] = {
        {{.server_run_time_nanos = 30000001u, .can_read_run_time_nanos = base.can_read_run_time_nanos,
          .max_can_rate = base.max_can_rate, .can_leniency = base.can_leniency}, "Server run time"},
        {{.server_run_time_nanos = base.server_run_time_nanos, .can_read_run_time_nanos = 30000001u,
          .max_can_rate = base.max_can_rate, .can_leniency = base.can_leniency}, "CAN read run time"},
        {{.server_run_time_nanos = base.server_run_time_nanos, .can_read_run_time_nanos = base.can_read_run_time_nanos,
          .max_can_rate = 9.0f, .can_leniency = base.can_leniency}, "Max CAN rate"},
        {{.server_run_time_nanos = base.server_run_time_nanos, .can_read_run_time_nanos = base.can_read_run_time_nanos,
          .max_can_rate = 3501.0f, .can_leniency = base.can_leniency}, "Max CAN rate"},
        {{.server_run_time_nanos = base.server_run_time_nanos, .can_read_run_time_nanos = base.can_read_run_time_nanos,
          .max_can_rate = 100.0f, .can_leniency = 0.5f}, "CAN leniency"},
        {{.server_run_time_nanos = base.server_run_time_nanos, .can_read_run_time_nanos = base.can_read_run_time_nanos,
          .max_can_rate = 100.0f, .can_leniency = 101.0f}, "CAN leniency"},
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        InitializeConfig init = {.comms_config = bad[i].cfg};
        TEST_ASSERT(!handle_initialize(kMsgId, &init), "bad case %zu", i);
        TEST_ASSERT_LOCAL_ERROR_MESSAGE(bad[i].err);
    }

    // Success: applies config, clears actuators/candump, resets clock offset; caller sends success.
    TestConfig act_cfg = {.tag = 0xBEEFu};
    TEST_ASSERT(init_actuator(1, false, &act_cfg, sizeof(act_cfg), 1), "seed actuator");
    num_can_lines = 1;
    CAN_message_t msg = {0};
    msg.id = 0x123;
    _insert_candump_telemetry(0, true, &msg);
    TEST_ASSERT(candump_telem_start != candump_telem_end, "candump seeded");

    InitializeConfig init = valid_init_config();
    TEST_ASSERT(!handle_initialize(kMsgId, &init), "success returns false for default response");
    TEST_ASSERT(comms_config.server_run_time_nanos == 5000, "server run time applied");
    TEST_ASSERT(comms_config.can_read_run_time_nanos == 6000, "can read run time applied");
    TEST_ASSERT(comms_config.max_can_rate == 50.0f, "max can rate applied");
    TEST_ASSERT(comms_config.can_leniency == 10.0f, "can leniency applied");
    TEST_ASSERT(actuators[1].config == NULL, "actuators reset");
    TEST_ASSERT(candump_telem_start == candump_telem_end, "candump cleared");
    TEST_ASSERT(timestamp_offset_ns == 0, "timestamp offset reset");

    reset_actuators();
    SET_DEFAULT_COMMS_CONFIG();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_get_global_error_msg) {
    eth_write_buf_clear();
    TEST_ASSERT(handle_get_global_error_msg(kMsgId, NULL), "empty error returns true");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(kMsgId, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0), "success when no error");
    TEST_ASSERT(get_global_error_msg() == NULL, "already clear");

    set_global_error_msg("motor fault %u", 3u);
    eth_write_buf_clear();
    TEST_ASSERT(handle_get_global_error_msg(kMsgId, NULL), "error path returns true");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(kMsgId, TEENSY_COMMAND_OPCODE_RESP_ERROR, "motor fault 3", 13), "error payload");
    TEST_ASSERT(get_global_error_msg() == NULL, "error cleared after send");

    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_set_logging) {
    SetLoggingPayload payload = {.enabled = false};
    TEST_ASSERT(!handle_set_logging(kMsgId, &payload), "returns false for default success");
    TEST_ASSERT(!logging_enabled, "logging disabled");

    payload.enabled = true;
    TEST_ASSERT(!handle_set_logging(kMsgId, &payload), "returns false for default success");
    TEST_ASSERT(logging_enabled, "logging enabled");

    return TEST_SUCCESS;
}


DECLARE_TEST(test_handle_get_src_hash) {
    eth_write_buf_clear();
    TEST_ASSERT(handle_get_src_hash(kMsgId, NULL), "returns true");
    TEST_ASSERT_STACKED(
        check_eth_write_buf_eq(kMsgId, TEENSY_COMMAND_OPCODE_RESP_SRC_HASH, teensy_source_hash_str, (uint32_t)strlen(teensy_source_hash_str)),
        "source hash payload");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_get_num_can_lines) {
    num_can_lines = 4;
    GetNumCanLinesResponsePayload expect = {.num_can_lines = 4};

    eth_write_buf_clear();
    TEST_ASSERT(handle_get_num_can_lines(kMsgId, NULL), "returns true");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(kMsgId, TEENSY_COMMAND_OPCODE_RESP_NUM_CAN_LINES, &expect, sizeof(expect)),
        "num can lines payload");

    num_can_lines = 0;
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_send_can) {
    num_can_lines = 3;
    _testing_can_buf_used = true;
    clear_candump_telemetry();
    memset(can_last_loads, 0, sizeof(can_last_loads));
    memset(can_last_times_ns, 0, sizeof(can_last_times_ns));
    SET_DEFAULT_COMMS_CONFIG();
    clear_local_error_msg();

    uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    SendCanPayload payload = {.can_bus = 1, .extended = true, .dlc = 4, .can_id = 0x1ABCDEF, .data = {0}};
    memcpy(payload.data, data, 8);

    memset(_testing_can_buf, 0xA5, sizeof(_testing_can_buf));
    TEST_ASSERT(!handle_send_can(kMsgId, &payload), "returns false for default response");
    TEST_ASSERT(get_local_error_msg() == NULL, "no error on success");

    struct can_frame expect = {0};
    expect.can_id = (payload.can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    expect.can_dlc = payload.dlc;
    memcpy(expect.data, data, payload.dlc);
    TEST_ASSERT(memcmp(_testing_can_buf[1], &expect, sizeof(expect)) == 0, "frame written to bus 1");

    // Invalid bus propagates cansend's local error.
    payload.can_bus = 3;
    memset(_testing_can_buf, 0xA5, sizeof(_testing_can_buf));
    TEST_ASSERT(!handle_send_can(kMsgId, &payload), "returns false on error");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid CAN bus");

    _testing_can_buf_used = false;
    num_can_lines = 0;
    clear_local_error_msg();
    return TEST_SUCCESS;
}

RUN_TESTS();
