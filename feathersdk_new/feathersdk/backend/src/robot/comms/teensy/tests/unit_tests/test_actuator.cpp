#include "../../src/logging.cpp"
#include "../../src/system.cpp"
#include "../../src/comms/comms.cpp"
#include "../../src/comms/can.cpp"
#include "../../src/comms/ethernet.cpp"
#include "../../src/actuator.cpp"
#include "../framework.hpp"
#include "../../src/commands/all_commands.hpp"

extern Actuator actuators[MAX_NUM_ACTUATORS];
extern bool _check_actuator_id(uint8_t actuator_id, ErrorScope scope, bool allow_uninitialized);
extern bool _check_telem_idx(uint16_t telem_idx, uint16_t num_telems, ErrorScope scope);
extern bool _insert_telemetry_value_locked(
    uint8_t actuator_id, uint16_t telem_idx, const void* value, DataType from_dtype, bool include_timestamp
);
extern void _ethbuf_write_telem_data(ActuatorParamTelemetry* telem);
extern bool _handle_get_actuator_telemetry_locked(uint32_t msg_id, void* payload);

typedef struct { uint32_t tag; } TestConfig;

static const uint32_t kMsgId = 42u;
static const uint32_t kFramingSize = (uint32_t)(sizeof(ClientCommandHeader) + sizeof(GetTelemetryPayloadHeader));

static void clear_actuators(void) {
    reset_actuators();
}

static test_result_t init_act(uint8_t id, uint8_t num_telems, uint32_t tag) {
    TestConfig cfg = {.tag = tag};
    TEST_ASSERT(init_actuator(id, false, &cfg, sizeof(cfg), num_telems), "init actuator %u", id);
    return TEST_SUCCESS;
}

static test_result_t add_telem(uint8_t id, uint16_t idx, DataType dtype, uint16_t max_size, uint16_t host_id) {
    TEST_ASSERT(add_actuator_param_telemetry(id, idx, dtype, max_size, host_id),
        "add telem %u for actuator %u", idx, id);
    return TEST_SUCCESS;
}

static test_result_t assert_eth_ring_eq(const uint8_t* expected, uint32_t len) {
    uint32_t sz = (eth_write_buf_end - eth_write_buf_start + ETH_WRITE_BUF_SIZE) % ETH_WRITE_BUF_SIZE;
    TEST_ASSERT(sz == len, "eth buf size %u expected %u", sz, len);
    uint8_t got[ETH_WRITE_BUF_SIZE];
    if (eth_write_buf_end >= eth_write_buf_start) {
        memcpy(got, eth_write_buf + eth_write_buf_start, eth_write_buf_end - eth_write_buf_start);
    } else {
        uint32_t first = ETH_WRITE_BUF_SIZE - eth_write_buf_start;
        memcpy(got, eth_write_buf + eth_write_buf_start, first);
        memcpy(got + first, eth_write_buf, eth_write_buf_end);
    }
    TEST_ASSERT(memcmp(got, expected, len) == 0, "eth buf payload mismatch");
    return TEST_SUCCESS;
}


DECLARE_TEST(test_check_actuator_id) {
    clear_actuators();

    struct { uint8_t id; bool allow_uninit; const char* err; } bad[] = {
        {MAX_NUM_ACTUATORS, true, "Invalid actuator ID"},
        {1, false, "not initialized"},
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        TEST_ASSERT(!_check_actuator_id(bad[i].id, ERROR_SCOPE_LOCAL, bad[i].allow_uninit), "id %u", bad[i].id);
        TEST_ASSERT_LOCAL_ERROR_MESSAGE(bad[i].err);
    }

    TEST_ASSERT(_check_actuator_id(0, ERROR_SCOPE_LOCAL, true), "id 0 allowed when uninitialized");
    TEST_ASSERT(_check_actuator_id(1, ERROR_SCOPE_LOCAL, true), "uninit slot ok when allowed");
    TEST_ASSERT_STACKED(init_act(1, 1, 1), "init for initialized check");
    TEST_ASSERT(_check_actuator_id(1, ERROR_SCOPE_LOCAL, false), "initialized slot ok");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_check_telem_idx) {
    struct { uint16_t idx; uint16_t num; const char* err; } bad[] = {
        {0, 0, "0 num_telems"},
        {2, 2, "Invalid telemetry idx"},
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        TEST_ASSERT(!_check_telem_idx(bad[i].idx, bad[i].num, ERROR_SCOPE_LOCAL), "idx %u num %u", bad[i].idx, bad[i].num);
        TEST_ASSERT_LOCAL_ERROR_MESSAGE(bad[i].err);
    }

    TEST_ASSERT(_check_telem_idx(1, 3, ERROR_SCOPE_GLOBAL), "valid idx");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_init_actuator) {
    clear_actuators();
    TestConfig cfg = {.tag = 0x1111u};

    TEST_ASSERT(init_actuator(0, false, &cfg, sizeof(cfg), 1), "id 0");
    TEST_ASSERT(actuators[0].id == 0 && actuators[0].config != NULL, "id 0 stored");
    free_actuator(0);

    TEST_ASSERT(!init_actuator(1, false, NULL, sizeof(cfg), 1), "null config");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid config");

    TEST_ASSERT(!init_actuator(1, false, &cfg, 0, 1), "zero config size");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid config");

    TEST_ASSERT(!init_actuator(1, false, &cfg, sizeof(cfg), MAX_NUM_ACTUATOR_PARAM_TELEMETRIES + 1),
        "too many telems");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid number of telemetries");

    TEST_ASSERT(init_actuator(1, false, &cfg, sizeof(cfg), 2), "first init");
    TEST_ASSERT(actuators[1].config != NULL && actuators[1].telemetries.num_telems == 2, "state after init");
    TEST_ASSERT(((TestConfig*)actuators[1].config)->tag == 0x1111u, "config copied");

    TestConfig cfg2 = {.tag = 0x2222u};
    TEST_ASSERT(!init_actuator(1, false, &cfg2, sizeof(cfg2), 1), "duplicate init");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("already initialized");

    TEST_ASSERT(init_actuator(1, true, &cfg2, sizeof(cfg2), 1), "reinit");
    TEST_ASSERT(((TestConfig*)actuators[1].config)->tag == 0x2222u, "reinit config");
    TEST_ASSERT(init_actuator(1, true, &cfg, sizeof(cfg), 0), "zero telems init");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_add_actuator_param_telemetry) {
    clear_actuators();
    TEST_ASSERT_STACKED(init_act(1, 2, 1), "init");

    struct { uint8_t id; uint16_t idx; DataType dtype; uint16_t max; uint16_t host; const char* err; } bad[] = {
        {2, 0, DATA_TYPE_F32, 4, 1, "not initialized"},
        {1, 2, DATA_TYPE_F32, 4, 1, "Invalid telemetry idx"},
        {1, 0, DATA_TYPE_F32, 0, 1, "Invalid telemetry max size"},
        {1, 0, DATA_TYPE_F32, MAX_TELEMETRY_SIZE + 1, 1, "Invalid telemetry max size"},
        {1, 0, (DataType)255, 4, 1, "Unknown telemetry data type"},
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        TEST_ASSERT(!add_actuator_param_telemetry(bad[i].id, bad[i].idx, bad[i].dtype, bad[i].max, bad[i].host),
            "bad case %zu", i);
        TEST_ASSERT_LOCAL_ERROR_MESSAGE(bad[i].err);
    }

    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_F32, 4, 10), "first add");
    TEST_ASSERT(!add_actuator_param_telemetry(1, 0, DATA_TYPE_F32, 4, 10), "duplicate add");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("already added");

    TestConfig zero_telem_cfg = {.tag = 2};
    TEST_ASSERT(init_actuator(2, false, &zero_telem_cfg, sizeof(TestConfig), 0), "zero telem actuator");
    TEST_ASSERT(!add_actuator_param_telemetry(2, 0, DATA_TYPE_F32, 4, 1), "zero telem slots");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("0 num_telems");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_free_actuator) {
    clear_actuators();
    TEST_ASSERT_STACKED(init_act(1, 2, 1), "init");
    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_U32, 2, 5), "add telem 0");
    TEST_ASSERT_STACKED(add_telem(1, 1, DATA_TYPE_F64, 2, 6), "add telem 1");

    free_actuator(1);
    TEST_ASSERT(actuators[1].config == NULL, "config cleared");
    TEST_ASSERT(actuators[1].telemetries.telemetry_list == NULL, "telem list cleared");
    TEST_ASSERT(actuators[1].telemetries.num_telems == 0, "num telems cleared");

    free_actuator(MAX_NUM_ACTUATORS);  // out of range no-op
    TEST_ASSERT_STACKED(init_act(1, 1, 9), "re-init after free");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_reset_actuators) {
    clear_actuators();
    TEST_ASSERT_STACKED(init_act(1, 1, 1), "init 1");
    TEST_ASSERT_STACKED(init_act(2, 1, 2), "init 2");
    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_I32, 2, 1), "add 1");
    TEST_ASSERT_STACKED(add_telem(2, 0, DATA_TYPE_I32, 2, 2), "add 2");

    reset_actuators();
    for (uint8_t id = 1; id <= 2; id++) {
        TEST_ASSERT(actuators[id].config == NULL, "actuator %u cleared", id);
    }
    return TEST_SUCCESS;
}

DECLARE_TEST(test_get_actuator_config) {
    clear_actuators();

    TEST_ASSERT(get_actuator_config(0, ERROR_SCOPE_LOCAL) == NULL, "id 0 uninitialized");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT(get_actuator_config(MAX_NUM_ACTUATORS, ERROR_SCOPE_LOCAL) == NULL, "invalid id");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid actuator ID");

    TEST_ASSERT(get_actuator_config(1, ERROR_SCOPE_GLOBAL) == NULL, "uninitialized");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT_STACKED(init_act(0, 1, 0xA0u), "init id 0");
    TestConfig* cfg0 = (TestConfig*)get_actuator_config(0, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(cfg0 != NULL && cfg0->tag == 0xA0u, "got id 0 config");

    TEST_ASSERT_STACKED(init_act(3, 1, 0x33u), "init");
    TestConfig* cfg = (TestConfig*)get_actuator_config(3, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(cfg != NULL && cfg->tag == 0x33u, "got config");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_get_actuator_param_telemetry) {
    clear_actuators();
    TEST_ASSERT_STACKED(init_act(1, 2, 1), "init");

    TEST_ASSERT(get_actuator_param_telemetry(0, 0, ERROR_SCOPE_LOCAL) == NULL, "id 0 not initialized");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not initialized");

    TEST_ASSERT(get_actuator_param_telemetry(MAX_NUM_ACTUATORS, 0, ERROR_SCOPE_LOCAL) == NULL, "bad actuator");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Invalid actuator ID");

    TEST_ASSERT(get_actuator_param_telemetry(1, 2, ERROR_SCOPE_GLOBAL) == NULL, "bad idx");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Invalid telemetry idx");

    TEST_ASSERT(get_actuator_param_telemetry(1, 0, ERROR_SCOPE_LOCAL) == NULL, "not added yet");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("not added");

    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_F32, 3, 7), "add");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(1, 0, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem != NULL && telem->host_telem_id == 7 && telem->max_size == 3, "got telem slot");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_insert_telemetry_value_locked) {
    clear_actuators();
    TEST_ASSERT_STACKED(init_act(1, 2, 1), "init");
    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_F32, 3, 1), "value telem");
    TEST_ASSERT_STACKED(add_telem(1, 1, DATA_TYPE_F64, 3, 2), "timestamp telem");

    float fval = 3.5f;
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &fval, DATA_TYPE_F32, false), "insert");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(1, 0, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->start_ind == 0 && telem->end_ind == 1, "one sample buffered");

    TEST_ASSERT(!_insert_telemetry_value_locked(1, 0, NULL, DATA_TYPE_F32, false), "null value");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Null value pointer");

    TEST_ASSERT(!_insert_telemetry_value_locked(2, 0, &fval, DATA_TYPE_F32, false), "bad actuator");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("not initialized");

    // Fill buffer (max_size=3) then overflow: drops oldest, sets global warning, still succeeds.
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &fval, DATA_TYPE_F32, false), "fill 2");
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &fval, DATA_TYPE_F32, false), "fill 3");
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &fval, DATA_TYPE_F32, false), "overflow");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("overflowed");
    clear_global_error_msg();
    TEST_ASSERT(telem->start_ind == 2 && telem->end_ind == 1, "full ring dropped oldest");

    telem->start_ind = telem->end_ind = 0;
    int32_t ival = 9;
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &ival, DATA_TYPE_I32, false), "dtype cast insert");
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &fval, DATA_TYPE_F32, true), "insert with timestamp");
    ActuatorParamTelemetry* ts = get_actuator_param_telemetry(1, 1, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(ts->end_ind == 1, "timestamp slot filled");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_insert_telemetry_value) {
    clear_actuators();
    TEST_ASSERT_STACKED(init_act(1, 1, 1), "init");
    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_U16, 2, 3), "add");

    uint16_t v = 1234;
    TEST_ASSERT(insert_telemetry_value(1, 0, &v, DATA_TYPE_U16, false), "public insert");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(1, 0, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->end_ind == 1, "sample stored");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_ethbuf_write_telem_data) {
    ActuatorParamTelemetry telem = {};
    telem.dtype = DATA_TYPE_U16;
    telem.max_size = 4;
    telem.values = calloc(4, sizeof(uint16_t));
    TEST_ASSERT(telem.values != NULL, "calloc telem values");

    uint16_t* vals = (uint16_t*)telem.values;
    vals[0] = 10; vals[1] = 11; vals[2] = 12; vals[3] = 13;

    // Contiguous segment [1, 3)
    telem.start_ind = 1;
    telem.end_ind = 3;
    eth_write_buf_clear();
    _ethbuf_write_telem_data(&telem);
    uint8_t expect_contig[] = {11, 0, 12, 0};
    TEST_ASSERT_STACKED(assert_eth_ring_eq(expect_contig, sizeof(expect_contig)), "contiguous");

    // Wrapped segment [2, 4) + [0, 1)
    telem.start_ind = 2;
    telem.end_ind = 1;
    eth_write_buf_clear();
    _ethbuf_write_telem_data(&telem);
    uint8_t expect_wrap[] = {12, 0, 13, 0, 10, 0};
    TEST_ASSERT_STACKED(assert_eth_ring_eq(expect_wrap, sizeof(expect_wrap)), "wrapped");

    free(telem.values);
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_get_actuator_telemetry_locked) {
    clear_actuators();
    const uint32_t msg_id = kMsgId;

    // Empty dump
    eth_write_buf_clear();
    TEST_ASSERT(_handle_get_actuator_telemetry_locked(msg_id, NULL), "empty dump");
    uint16_t empty_updates = 0;
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_ACTUATOR_TELEMETRY, &empty_updates, sizeof(empty_updates)),
        "empty payload");

    // Not enough space for framing headers
    eth_write_buf_leave_space(kFramingSize - 1);
    TEST_ASSERT(!_handle_get_actuator_telemetry_locked(msg_id, NULL), "no header space");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Not enough space for get_actuator_telemetry payload headers");

    // One float sample
    eth_write_buf_clear();
    TEST_ASSERT_STACKED(init_act(1, 1, 1), "init");
    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_F32, 4, 99), "add");
    float sample = 2.25f;
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &sample, DATA_TYPE_F32, false), "insert sample");

    TEST_ASSERT(_handle_get_actuator_telemetry_locked(msg_id, NULL), "single dump");
    GetTelemetrySingleTelemetryHeader telem_hdr = {.telemetry_id = 99, .num_values = 1};
    uint8_t payload[sizeof(uint16_t) + sizeof(GetTelemetrySingleTelemetryHeader) + sizeof(float)];
    uint16_t num_updates = 1;
    memcpy(payload, &num_updates, sizeof(num_updates));
    memcpy(payload + sizeof(num_updates), &telem_hdr, sizeof(telem_hdr));
    memcpy(payload + sizeof(num_updates) + sizeof(telem_hdr), &sample, sizeof(sample));
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_ACTUATOR_TELEMETRY, payload, sizeof(payload)),
        "single telem payload");
    ActuatorParamTelemetry* telem = get_actuator_param_telemetry(1, 0, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(telem->start_ind == telem->end_ind, "drained after send");

    // No space for any non-empty telemetry
    eth_write_buf_clear();
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &sample, DATA_TYPE_F32, false), "re-insert sample");
    uint32_t one_size = (uint32_t)(sizeof(GetTelemetrySingleTelemetryHeader) + sizeof(float));
    eth_write_buf_leave_space(kFramingSize + one_size - 1);
    uint32_t end_before = eth_write_buf_end;
    TEST_ASSERT(!_handle_get_actuator_telemetry_locked(msg_id, NULL), "no space for any telem");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("No space to fit any telemetries with updates");
    TEST_ASSERT(eth_write_buf_end == end_before, "must not write on no-space failure");

    // Partial fit: buffer fits one u8 telem update but not two
    clear_actuators();
    eth_write_buf_clear();
    TEST_ASSERT_STACKED(init_act(1, 1, 1), "init a1");
    TEST_ASSERT_STACKED(init_act(2, 1, 1), "init a2");
    TEST_ASSERT_STACKED(add_telem(1, 0, DATA_TYPE_U8, 4, 11), "add a1");
    TEST_ASSERT_STACKED(add_telem(2, 0, DATA_TYPE_U8, 4, 22), "add a2");
    uint8_t b1 = 1, b2 = 2;
    TEST_ASSERT(_insert_telemetry_value_locked(1, 0, &b1, DATA_TYPE_U8, false), "insert a1");
    TEST_ASSERT(_insert_telemetry_value_locked(2, 0, &b2, DATA_TYPE_U8, false), "insert a2");

    one_size = (uint32_t)(sizeof(GetTelemetrySingleTelemetryHeader) + sizeof(uint8_t));
    eth_write_buf_leave_space(kFramingSize + one_size);
    TEST_ASSERT(_handle_get_actuator_telemetry_locked(msg_id, NULL), "partial dump");
    ActuatorParamTelemetry* a1 = get_actuator_param_telemetry(1, 0, ERROR_SCOPE_LOCAL);
    ActuatorParamTelemetry* a2 = get_actuator_param_telemetry(2, 0, ERROR_SCOPE_LOCAL);
    TEST_ASSERT(a1->start_ind == a1->end_ind, "first actuator drained");
    TEST_ASSERT(a2->start_ind != a2->end_ind, "second actuator still buffered");

    clear_actuators();
    return TEST_SUCCESS;
}

DECLARE_TEST(test_handle_get_actuator_telemetry) {
    clear_actuators();
    eth_write_buf_clear();
    TEST_ASSERT(handle_get_actuator_telemetry(kMsgId, NULL), "wrapper empty dump");
    uint16_t empty_updates = 0;
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(kMsgId, TEENSY_COMMAND_OPCODE_RESP_ACTUATOR_TELEMETRY, &empty_updates, sizeof(empty_updates)),
        "wrapper payload");
    clear_actuators();
    return TEST_SUCCESS;
}

RUN_TESTS();
