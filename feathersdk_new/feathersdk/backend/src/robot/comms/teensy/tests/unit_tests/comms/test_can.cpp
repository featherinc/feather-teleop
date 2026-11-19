#include "../../../src/logging.cpp"
#include "../../../src/system.cpp"
#include "../../../src/comms/comms.cpp"
#include "../../../src/comms/ethernet.cpp"
#include "../../../src/comms/can.cpp"
#include "../../framework.hpp"
#include <linux/can.h>

extern uint8_t num_can_lines;
extern uint8_t _testing_can_buf[MAX_NUM_CAN_LINES][sizeof(struct can_frame)];
extern bool _testing_can_buf_used;


static CAN_message_t make_msg(uint32_t id, bool extended, uint8_t len, const uint8_t* data) {
    CAN_message_t msg = {0};
    msg.id = id;
    msg.flags.extended = extended;
    msg.len = len;
    if (data != NULL && len > 0) memcpy(msg.buf, data, MIN(len, 8));
    return msg;
}

static uint32_t candump_count(void) {
    return (candump_telem_end + CANDUMP_TELEMETRY_SIZE - candump_telem_start) % CANDUMP_TELEMETRY_SIZE;
}

static CandumpTelemetry* last_candump(void) {
    uint32_t idx = (candump_telem_end + CANDUMP_TELEMETRY_SIZE - 1) % CANDUMP_TELEMETRY_SIZE;
    return &candump_telemetry[idx];
}

static void reset_can_throttle(void) {
    memset(can_last_loads, 0, sizeof(can_last_loads));
    memset(can_last_times_ns, 0, sizeof(can_last_times_ns));
}

static void clear_testing_can_bufs(void) {
    memset(_testing_can_buf, 0xA5, sizeof(_testing_can_buf));
}

static void seed_candump(uint32_t n) {
    clear_candump_telemetry();
    for (uint32_t i = 0; i < n; i++) {
        CAN_message_t msg = make_msg(i, false, 0, NULL);
        _insert_candump_telemetry(0, true, &msg);
    }
}

/** Assert only ``bus`` was written, with the expected SocketCAN frame. */
static test_result_t assert_testing_write(uint8_t bus, bool extended, uint32_t can_id, const uint8_t* data, uint8_t len) {
    struct can_frame expect = {0};
    expect.can_id = extended ? ((can_id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (can_id & CAN_SFF_MASK);
    expect.can_dlc = len;
    if (len > 0) memcpy(expect.data, data, len);

    TEST_ASSERT(memcmp(_testing_can_buf[bus], &expect, sizeof(expect)) == 0, "bus %u frame mismatch", bus);

    uint8_t untouched[sizeof(struct can_frame)];
    memset(untouched, 0xA5, sizeof(untouched));
    for (uint8_t b = 0; b < MAX_NUM_CAN_LINES; b++) {
        if (b == bus) continue;
        TEST_ASSERT(memcmp(_testing_can_buf[b], untouched, sizeof(untouched)) == 0, "unexpected write on bus %u", b);
    }
    return TEST_SUCCESS;
}

static test_result_t assert_testing_bufs_untouched(void) {
    uint8_t untouched[sizeof(struct can_frame)];
    memset(untouched, 0xA5, sizeof(untouched));
    for (uint8_t b = 0; b < MAX_NUM_CAN_LINES; b++) {
        TEST_ASSERT(memcmp(_testing_can_buf[b], untouched, sizeof(untouched)) == 0, "unexpected write on bus %u", b);
    }
    return TEST_SUCCESS;
}

/** Assert latest candump entry matches a successful ``cansend`` (sent=1). */
static test_result_t assert_last_candump_sent(uint8_t bus, uint32_t can_id, const uint8_t* data, uint8_t len) {
    CandumpTelemetry* got = last_candump();
    uint8_t expect_info = (uint8_t)((bus << 5) | (1 << 4) | len);
    TEST_ASSERT(got->info == expect_info, "candump info: got 0x%02X expected 0x%02X", got->info, expect_info);
    TEST_ASSERT(got->can_id == can_id, "candump can_id mismatch");
    TEST_ASSERT(memcmp(got->data, data, len) == 0, "candump payload mismatch");
    return TEST_SUCCESS;
}


DECLARE_TEST(test_can_insert_candump_telemetry) {
    // Avoid depending on host SocketCAN; only get_num_can_lines() is needed here.
    num_can_lines = 3;
    clear_candump_telemetry();

    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    CAN_message_t ok = make_msg(0x123, false, 4, data);

    // --- Error paths ---
    _insert_candump_telemetry(3, false, &ok);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Invalid CAN bus");
    TEST_ASSERT(candump_count() == 0, "error path must not insert");

    CAN_message_t bad_ext = make_msg(0x20000000u, true, 0, NULL);
    _insert_candump_telemetry(0, true, &bad_ext);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Extended CAN ID");
    TEST_ASSERT(candump_count() == 0, "error path must not insert");

    CAN_message_t bad_std = make_msg(0x800, false, 0, NULL);
    _insert_candump_telemetry(0, false, &bad_std);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Standard CAN ID");
    TEST_ASSERT(candump_count() == 0, "error path must not insert");

    CAN_message_t bad_len = make_msg(0x1, false, 9, data);
    _insert_candump_telemetry(0, false, &bad_len);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Invalid CAN data length");
    TEST_ASSERT(candump_count() == 0, "error path must not insert");

    // --- Success: standard/extended, sent/recv, info + payload ---
    struct { uint32_t id; bool ext; bool sent; uint8_t bus; uint8_t len; } cases[] = {
        {0x123, false, false, 0, 4},
        {0x7FF, false, true,  1, 0},
        {0x1ABCDEFu, true, false, 2, 8},
        {0x1FFFFFFFu, true, true, 0, 1},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        CAN_message_t msg = make_msg(cases[i].id, cases[i].ext, cases[i].len, data);
        uint64_t before = currtime_ns();
        _insert_candump_telemetry(cases[i].bus, cases[i].sent, &msg);
        uint64_t after = currtime_ns();

        TEST_ASSERT(candump_count() == i + 1, "count after insert %zu", i);
        CandumpTelemetry* got = last_candump();
        uint8_t expect_info = (uint8_t)((cases[i].bus << 5) | ((cases[i].sent ? 1 : 0) << 4) | cases[i].len);
        TEST_ASSERT(got->info == expect_info, "info: got 0x%02X expected 0x%02X", got->info, expect_info);
        TEST_ASSERT(got->can_id == cases[i].id, "can_id mismatch");
        TEST_ASSERT(memcmp(got->data, data, cases[i].len) == 0, "payload mismatch");
        TEST_ASSERT(got->timestamp_ns >= before && got->timestamp_ns <= after, "timestamp out of range");
    }

    // --- Ring overflow drops oldest ---
    clear_candump_telemetry();
    CAN_message_t fill = make_msg(0x1, false, 0, NULL);
    for (uint32_t i = 0; i < CANDUMP_TELEMETRY_SIZE; i++) {
        fill.id = i;
        _insert_candump_telemetry(0, false, &fill);
    }
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Candump telemetry buffer is full");
    TEST_ASSERT(candump_count() == CANDUMP_TELEMETRY_SIZE - 1, "ring should hold SIZE-1 after overflow");
    TEST_ASSERT(candump_telemetry[candump_telem_start].can_id == 1, "oldest (id 0) should be dropped");

    return TEST_SUCCESS;
}

DECLARE_TEST(test_can_clear_candump_telemetry) {
    num_can_lines = 1;
    clear_candump_telemetry();

    CAN_message_t msg = make_msg(0x10, false, 0, NULL);
    _insert_candump_telemetry(0, false, &msg);
    _insert_candump_telemetry(0, true, &msg);
    TEST_ASSERT(candump_count() == 2, "precondition: inserts succeeded");

    clear_candump_telemetry();
    TEST_ASSERT(candump_telem_start == 0 && candump_telem_end == 0, "pointers should reset to 0");
    TEST_ASSERT(candump_count() == 0, "ring should be empty");

    return TEST_SUCCESS;
}

DECLARE_TEST(test_can_would_overload) {
    // Mirror feathersdk/tests/utils/test_math.py::test_event_throttler:
    // at time_diff=0, exactly int(rate/leniency) events succeed, then the next overloads.
    float rates[] = {1.0f, 100.0f, 3000.0f};
    float leniencies[] = {0.5f, 1.0f, 100.0f, 300.0f};
    const uint64_t t0 = 1;  // fixed "now"; reuse so every event has dt=0

    for (size_t ri = 0; ri < sizeof(rates) / sizeof(rates[0]); ri++) {
        for (size_t li = 0; li < sizeof(leniencies) / sizeof(leniencies[0]); li++) {
            float rate = rates[ri];
            float leniency = leniencies[li];
            if (leniency > rate) continue;  // invalid config; validated at initialize, not here

            CommsConfig cfg = DEFAULT_COMMS_CONFIG;
            cfg.max_can_rate = rate;
            cfg.can_leniency = leniency;
            set_comms_config(&cfg);

            int n_ok = (int)(rate / leniency);

            can_last_loads[0] = 0.0f;
            can_last_times_ns[0] = t0;
            for (int i = 0; i < n_ok; i++) {
                TEST_ASSERT(!can_would_overload(0, t0),
                    "event %d/%d should pass (rate=%g leniency=%g)", i + 1, n_ok, (double)rate, (double)leniency);
            }

            float load_before = can_last_loads[0];
            TEST_ASSERT(can_would_overload(0, t0),
                "event %d should overload (rate=%g leniency=%g)", n_ok + 1, (double)rate, (double)leniency);
            TEST_ASSERT(can_last_loads[0] == load_before, "overload must not update load");

            // reset() then burst again
            can_last_loads[0] = 0.0f;
            can_last_times_ns[0] = t0;
            for (int i = 0; i < n_ok; i++) {
                TEST_ASSERT(!can_would_overload(0, t0), "after reset event %d/%d should pass (rate=%g leniency=%g)",
                    i + 1, n_ok, (double)rate, (double)leniency);
            }
        }
    }

    CommsConfig restore = DEFAULT_COMMS_CONFIG;
    set_comms_config(&restore);
    return TEST_SUCCESS;
}

DECLARE_TEST(test_cansend) {
    num_can_lines = 3;
    _testing_can_buf_used = true;
    clear_candump_telemetry();
    reset_can_throttle();
    SET_DEFAULT_COMMS_CONFIG();

    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

    // --- Validation errors: no physical write, no candump ---
    struct { uint8_t bus; bool ext; uint32_t id; uint8_t len; const char* err; } bad[] = {
        {0, false, 0x1, 9, "CAN data length too long"},
        {3, false, 0x1, 0, "Invalid CAN bus"},
        {0, false, 0x800, 0, "CAN ID too large"},
        {0, true, 0x20000000u, 0, "CAN ID too large"},
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        clear_testing_can_bufs();
        TEST_ASSERT(!cansend(bad[i].bus, bad[i].ext, bad[i].id, data, bad[i].len), "expected fail: %s", bad[i].err);
        TEST_ASSERT_LOCAL_ERROR_MESSAGE(bad[i].err);
        TEST_ASSERT(candump_count() == 0, "validation fail must not candump");
        TEST_ASSERT_STACKED(assert_testing_bufs_untouched(), "validation fail must not write");
    }

    // --- Success: bus / standard+extended / len extremes ---
    struct { uint8_t bus; bool ext; uint32_t id; uint8_t len; } ok[] = {
        {0, false, 0x123, 4},
        {1, false, 0x7FF, 0},
        {2, true,  0x1ABCDEFu, 8},
        {0, true,  0x1FFFFFFFu, 1},
    };
    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
        clear_testing_can_bufs();
        TEST_ASSERT(cansend(ok[i].bus, ok[i].ext, ok[i].id, data, ok[i].len), "cansend should succeed");
        TEST_ASSERT(candump_count() == i + 1, "candump count after send %zu", i);
        TEST_ASSERT_STACKED(assert_testing_write(ok[i].bus, ok[i].ext, ok[i].id, data, ok[i].len), "write check");
        TEST_ASSERT_STACKED(assert_last_candump_sent(ok[i].bus, ok[i].id, data, ok[i].len), "candump check");
    }

    // --- Overload: rate=leniency=1 → second immediate send fails, no write/candump ---
    {
        CommsConfig cfg = DEFAULT_COMMS_CONFIG;
        cfg.max_can_rate = 1.0f;
        cfg.can_leniency = 1.0f;
        set_comms_config(&cfg);
        reset_can_throttle();
        clear_candump_telemetry();

        clear_testing_can_bufs();
        TEST_ASSERT(cansend(0, false, 0x10, data, 1), "first send under limit");
        uint32_t after_first = candump_count();

        clear_testing_can_bufs();
        TEST_ASSERT(!cansend(0, false, 0x11, data, 1), "second send should overload");
        TEST_ASSERT_LOCAL_ERROR_MESSAGE("overloaded");
        TEST_ASSERT(candump_count() == after_first, "overload must not candump");
        TEST_ASSERT_STACKED(assert_testing_bufs_untouched(), "overload must not write");
    }

    // --- Physical write failure (testing buf off, no open sockets) ---
    {
        SET_DEFAULT_COMMS_CONFIG();
        reset_can_throttle();
        clear_candump_telemetry();
        _testing_can_buf_used = false;

        TEST_ASSERT(!cansend(0, false, 0x20, data, 1), "write should fail without socket");
        TEST_ASSERT_LOCAL_ERROR_MESSAGE("Failed to send CAN message");
        TEST_ASSERT(candump_count() == 0, "failed write must not candump");
    }

    _testing_can_buf_used = false;
    SET_DEFAULT_COMMS_CONFIG();
    reset_can_throttle();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_opencan_send_sdo) {
    num_can_lines = 1;
    _testing_can_buf_used = true;
    clear_candump_telemetry();
    reset_can_throttle();
    SET_DEFAULT_COMMS_CONFIG();

    uint32_t v32 = 0x04030201u;
    uint16_t v16 = 0x0201u;
    uint8_t v8 = 0xABu;

    // --- Validation errors (SDO-specific; no write) ---
    clear_testing_can_bufs();
    TEST_ASSERT(!opencan_send_sdo(0, 99, 1, 0x2000, 0, DATA_TYPE_U8, NULL), "bad cmd");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid SDO command");

    TEST_ASSERT(!opencan_send_sdo(0, SDO_READ, 128, 0x2000, 0, DATA_TYPE_U8, NULL), "node 128");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid node ID");

    TEST_ASSERT(!opencan_send_sdo(0, SDO_READ, 1, 0x2000, 0, DATA_TYPE_U8, &v8), "read needs NULL value");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Value must be NULL when reading");
    TEST_ASSERT(!opencan_send_sdo(0, SDO_WRITE, 1, 0x2000, 0, DATA_TYPE_U8, NULL), "write needs value");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Value is NULL when writing");

    TEST_ASSERT(!opencan_send_sdo(0, SDO_WRITE, 1, 0x2000, 0, DATA_TYPE_U64, &v32), "8-byte dtype unsupported");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Unhandled sdo cmd/data type");

    TEST_ASSERT_STACKED(assert_testing_bufs_untouched(), "validation must not write");

    // --- Success: read + writes for 1/2/4-byte dtypes; check COB-ID and SDO payload ---
    struct {
        SDOCommand cmd; uint8_t node; uint16_t idx; uint8_t sub; DataType dtype;
        const void* value; uint8_t cs;
    } ok[] = {
        {SDO_READ,  0,   0x2000, 0x05, DATA_TYPE_U8,  NULL, 0x40},
        {SDO_READ,  1,   0x2000, 0x05, DATA_TYPE_U8,  NULL, 0x40},
        {SDO_WRITE, 127, 0xABCD, 0x12, DATA_TYPE_U8,  &v8,  0x2F},
        {SDO_WRITE, 42,  0x1018, 0x00, DATA_TYPE_U16, &v16, 0x2B},
        {SDO_WRITE, 7,   0x6040, 0x01, DATA_TYPE_U32, &v32, 0x23},
        {SDO_WRITE, 7,   0x6040, 0x01, DATA_TYPE_F32, &v32, 0x23},  // size-4 path via float
    };
    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
        uint8_t expect[8] = {
            ok[i].cs, (uint8_t)(ok[i].idx & 0xFF), (uint8_t)(ok[i].idx >> 8), ok[i].sub, 0, 0, 0, 0
        };
        if (ok[i].cmd == SDO_WRITE) memcpy(expect + 4, ok[i].value, dtype_size(ok[i].dtype));

        clear_testing_can_bufs();
        TEST_ASSERT(opencan_send_sdo(0, ok[i].cmd, ok[i].node, ok[i].idx, ok[i].sub, ok[i].dtype, ok[i].value),
            "sdo case %zu should succeed", i);
        TEST_ASSERT_STACKED(assert_testing_write(0, false, 0x600 | ok[i].node, expect, 8), "sdo frame %zu", i);
    }

    _testing_can_buf_used = false;
    reset_can_throttle();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_get_candump) {
    const uint32_t msg_id = 77;
    const uint32_t hdr = (uint32_t)sizeof(ClientCommandHeader);
    const uint32_t telem_sz = (uint32_t)sizeof(CandumpTelemetry);
    num_can_lines = 1;

    // --- Empty ring == header-only response ---
    clear_candump_telemetry();
    eth_write_buf_clear();
    TEST_ASSERT(handle_get_candump(msg_id, NULL), "empty dump should succeed");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_CANDUMP, NULL, 0), "empty payload");
    TEST_ASSERT_NO_ERROR_MESSAGES(__FUNCTION__);

    // --- Empty ring, no room for header ---
    eth_write_buf_leave_space(hdr - 1);
    uint32_t end_before = eth_write_buf_end;
    TEST_ASSERT(!handle_get_candump(msg_id, NULL), "empty + no space should fail");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Not enough space in the buffer to send a candump response");
    TEST_ASSERT(eth_write_buf_end == end_before, "must not write on empty no-space");

    // --- Non-empty, no room for header + one telem ---
    seed_candump(1);
    eth_write_buf_leave_space(hdr + telem_sz - 1);
    end_before = eth_write_buf_end;
    TEST_ASSERT(!handle_get_candump(msg_id, NULL), "nonempty + no space should fail");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Not enough space in the buffer to send a candump telemetry response");
    TEST_ASSERT(candump_count() == 1 && eth_write_buf_end == end_before, "must not drain or write");

    // --- Full drain (contiguous) + partial drain then remainder ---
    seed_candump(5);
    CandumpTelemetry all5[5];
    memcpy(all5, candump_telemetry, sizeof(all5));

    eth_write_buf_clear();
    TEST_ASSERT(handle_get_candump(msg_id, NULL), "full drain should succeed");
    TEST_ASSERT(candump_count() == 0, "ring should be empty after full drain");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_CANDUMP, all5, 5 * telem_sz), "full drain payload");

    seed_candump(5);
    memcpy(all5, candump_telemetry, sizeof(all5));
    eth_write_buf_leave_space(hdr + 2 * telem_sz);
    uint32_t resp_start = eth_write_buf_end;
    TEST_ASSERT(handle_get_candump(msg_id, NULL), "partial drain should succeed");
    TEST_ASSERT(candump_count() == 3, "3 should remain after partial");
    eth_write_buf_start = resp_start;  // isolate just the new response for check
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_CANDUMP, all5, 2 * telem_sz), "partial payload");

    eth_write_buf_clear();
    TEST_ASSERT(handle_get_candump(msg_id, NULL), "remainder drain should succeed");
    TEST_ASSERT(candump_count() == 0, "ring empty after remainder");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_CANDUMP, all5 + 2, 3 * telem_sz), "remainder payload");

    // --- Source ring wraps: drain across end→0 in two inserts ---
    clear_candump_telemetry();
    candump_telem_start = CANDUMP_TELEMETRY_SIZE - 2;
    candump_telem_end = 1;  // pending = 3
    CandumpTelemetry wrapped[3];
    for (uint32_t i = 0; i < 3; i++) {
        uint32_t idx = (candump_telem_start + i) % CANDUMP_TELEMETRY_SIZE;
        candump_telemetry[idx] = (CandumpTelemetry){
            .info = (uint8_t)i, .can_id = 100 + i, .timestamp_ns = 1000 + i
        };
        wrapped[i] = candump_telemetry[idx];
    }
    eth_write_buf_clear();
    TEST_ASSERT(handle_get_candump(msg_id, NULL), "wrapped drain should succeed");
    TEST_ASSERT(candump_count() == 0, "wrapped ring should be empty");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_CANDUMP, wrapped, 3 * telem_sz), "wrapped payload");

    eth_write_buf_clear();
    return TEST_SUCCESS;
}

RUN_TESTS();
