#include "../../src/logging.cpp"
#include "../../src/system.cpp"
#include "../../src/comms/ethernet.cpp"
#include "../../src/system.hpp"
#include "../framework.hpp"


DECLARE_TEST(test_system_util_macros) {
    struct { int a, b, min, max; } cases[] = {
        {1, 2, 1, 2},
        {2, 1, 1, 2},
        {5, 5, 5, 5},
        {-3, 1, -3, 1},
        {-1, -5, -5, -1},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        TEST_ASSERT(MIN(cases[i].a, cases[i].b) == cases[i].min, "MIN(%d, %d)", cases[i].a, cases[i].b);
        TEST_ASSERT(MAX(cases[i].a, cases[i].b) == cases[i].max, "MAX(%d, %d)", cases[i].a, cases[i].b);
    }
    return TEST_SUCCESS;
}

DECLARE_TEST(test_system_dtype_size) {
    for (int i = 0; i<256; i++) {
        DataType dtype = (DataType)i;
        switch (dtype) {
            case DATA_TYPE_U8:
                TEST_ASSERT(dtype_size(dtype) == sizeof(uint8_t), "dtype_size(%d) != %zu", dtype, sizeof(uint8_t));
                break;
            case DATA_TYPE_U16:
                TEST_ASSERT(dtype_size(dtype) == sizeof(uint16_t), "dtype_size(%d) != %zu", dtype, sizeof(uint16_t));
                break;
            case DATA_TYPE_U32:
                TEST_ASSERT(dtype_size(dtype) == sizeof(uint32_t), "dtype_size(%d) != %zu", dtype, sizeof(uint32_t));
                break;
            case DATA_TYPE_U64:
                TEST_ASSERT(dtype_size(dtype) == sizeof(uint64_t), "dtype_size(%d) != %zu", dtype, sizeof(uint64_t));
                break;
            case DATA_TYPE_I8:
                TEST_ASSERT(dtype_size(dtype) == sizeof(int8_t), "dtype_size(%d) != %zu", dtype, sizeof(int8_t));
                break;
            case DATA_TYPE_I16:
                TEST_ASSERT(dtype_size(dtype) == sizeof(int16_t), "dtype_size(%d) != %zu", dtype, sizeof(int16_t));
                break;
            case DATA_TYPE_I32:
                TEST_ASSERT(dtype_size(dtype) == sizeof(int32_t), "dtype_size(%d) != %zu", dtype, sizeof(int32_t));
                break;
            case DATA_TYPE_I64:
                TEST_ASSERT(dtype_size(dtype) == sizeof(int64_t), "dtype_size(%d) != %zu", dtype, sizeof(int64_t));
                break;
            case DATA_TYPE_F32:
                TEST_ASSERT(dtype_size(dtype) == sizeof(float), "dtype_size(%d) != %zu", dtype, sizeof(float));
                break;
            case DATA_TYPE_F64:
                TEST_ASSERT(dtype_size(dtype) == sizeof(double), "dtype_size(%d) != %zu", dtype, sizeof(double));
                break;
            case DATA_TYPE_BOOL:
                TEST_ASSERT(dtype_size(dtype) == sizeof(bool), "dtype_size(%d) != %zu", dtype, sizeof(bool));
                break;
            default:
                TEST_ASSERT(dtype_size(dtype) == 0, "dtype_size(%d) != 0", dtype);
                break;
        }
    }
    return TEST_SUCCESS;
}

DECLARE_TEST(test_system_dtype_cast) {
    // Basic widening / same-family casts
    {
        uint8_t src = 42;
        uint16_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_U16, &src, DATA_TYPE_U8, ERROR_SCOPE_LOCAL), "u8->u16 cast failed");
        TEST_ASSERT(dst == 42, "u8->u16: got %u", dst);
    }
    {
        int32_t src = -1000;
        int64_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_I64, &src, DATA_TYPE_I32, ERROR_SCOPE_LOCAL), "i32->i64 cast failed");
        TEST_ASSERT(dst == -1000, "i32->i64: got %lld", (long long)dst);
    }
    {
        float src = 1.5f;
        double dst = 0.0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_F64, &src, DATA_TYPE_F32, ERROR_SCOPE_LOCAL), "f32->f64 cast failed");
        TEST_ASSERT(dst == (double)1.5f, "f32->f64: got %f", dst);
    }

    // Identity cast
    {
        int32_t src = 12345;
        int32_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_I32, &src, DATA_TYPE_I32, ERROR_SCOPE_LOCAL), "i32->i32 cast failed");
        TEST_ASSERT(dst == 12345, "i32->i32: got %d", dst);
    }

    // Float -> int (truncates toward zero)
    {
        float src = 3.9f;
        int32_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_I32, &src, DATA_TYPE_F32, ERROR_SCOPE_LOCAL), "f32->i32 cast failed");
        TEST_ASSERT(dst == 3, "f32->i32 truncate +: got %d", dst);
    }
    {
        float src = -3.9f;
        int32_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_I32, &src, DATA_TYPE_F32, ERROR_SCOPE_LOCAL), "f32->i32 (-) cast failed");
        TEST_ASSERT(dst == -3, "f32->i32 truncate -: got %d", dst);
    }

    // Int -> float
    {
        int32_t src = -42;
        float dst = 0.0f;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_F32, &src, DATA_TYPE_I32, ERROR_SCOPE_LOCAL), "i32->f32 cast failed");
        TEST_ASSERT(dst == -42.0f, "i32->f32: got %f", (double)dst);
    }
    {
        uint64_t src = 1000000ull;
        double dst = 0.0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_F64, &src, DATA_TYPE_U64, ERROR_SCOPE_LOCAL), "u64->f64 cast failed");
        TEST_ASSERT(dst == 1000000.0, "u64->f64: got %f", dst);
    }

    // Unsigned -> signed (bit pattern reinterpret via C cast)
    {
        uint32_t src = 0xFFFFFFFFu;
        int32_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_I32, &src, DATA_TYPE_U32, ERROR_SCOPE_LOCAL), "u32->i32 cast failed");
        TEST_ASSERT(dst == -1, "u32->i32: got %d", dst);
    }
    {
        uint8_t src = 200;
        int8_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_I8, &src, DATA_TYPE_U8, ERROR_SCOPE_LOCAL), "u8->i8 cast failed");
        TEST_ASSERT(dst == (int8_t)200, "u8->i8: got %d", (int)dst);
    }

    // Overflow / narrowing (matches C cast / modular wrap)
    {
        uint16_t src = 300;
        uint8_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_U8, &src, DATA_TYPE_U16, ERROR_SCOPE_LOCAL), "u16->u8 cast failed");
        TEST_ASSERT(dst == (uint8_t)300, "u16->u8 overflow: got %u", dst);
    }
    {
        int32_t src = 1000;
        int8_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_I8, &src, DATA_TYPE_I32, ERROR_SCOPE_LOCAL), "i32->i8 cast failed");
        TEST_ASSERT(dst == (int8_t)1000, "i32->i8 overflow: got %d", (int)dst);
    }
    {
        int32_t src = -1;
        uint32_t dst = 0;
        TEST_ASSERT(dtype_cast(&dst, DATA_TYPE_U32, &src, DATA_TYPE_I32, ERROR_SCOPE_LOCAL), "i32->u32 cast failed");
        TEST_ASSERT(dst == (uint32_t)-1, "i32->u32: got %u", dst);
    }

    // Unaligned src/dst: f64 at aligned(8)+1 -> f32 at another aligned(8)+1
    {
        uint8_t src_mem[16] __attribute__((aligned(8)));
        uint8_t dst_mem[16] __attribute__((aligned(8)));
        void* unaligned_src = src_mem + 1;
        void* unaligned_dst = dst_mem + 1;

        double src = 3.141592653589793;
        float expected = (float)src;
        memcpy(unaligned_src, &src, sizeof(src));

        TEST_ASSERT(
            dtype_cast(unaligned_dst, DATA_TYPE_F32, unaligned_src, DATA_TYPE_F64, ERROR_SCOPE_LOCAL),
            "unaligned f64->f32 cast failed"
        );

        float got;
        memcpy(&got, unaligned_dst, sizeof(got));
        TEST_ASSERT(got == expected, "unaligned f64->f32: got %f, expected %f", (double)got, (double)expected);
    }

    // Invalid dtypes: only failure paths in dtype_cast
    {
        uint8_t src = 1, dst = 0;
        TEST_ASSERT(!dtype_cast(&dst, (DataType)255, &src, DATA_TYPE_U8, ERROR_SCOPE_LOCAL), "bad dst_dtype should fail");
        TEST_ASSERT_LOCAL_ERROR_MESSAGE("Unknown dst_dtype");

        TEST_ASSERT(!dtype_cast(&dst, DATA_TYPE_U8, &src, (DataType)255, ERROR_SCOPE_LOCAL), "bad src_dtype should fail");
        TEST_ASSERT_LOCAL_ERROR_MESSAGE("Unknown src_dtype");
    }

    return TEST_SUCCESS;
}

DECLARE_TEST(test_system_currtime_ns) {
    uint64_t start_raw, currtime, using_time;

    int64_t add_times[] = {0, 100000000, -100000000};
    size_t add_times_len = sizeof(add_times) / sizeof(add_times[0]);

    for (size_t i = 0; i < add_times_len; i++) {
        start_raw = raw_currtime_ns();
        timestamp_offset_ns = add_times[i];
        currtime = currtime_ns();
        using_time = currtime_ns();
        TEST_ASSERT(using_time >= currtime, "%llu < %llu", (unsigned long long)using_time, (unsigned long long)currtime);
        TEST_ASSERT(currtime >= start_raw + add_times[i], "%llu < %llu + %lld", 
            (unsigned long long)currtime, (unsigned long long)start_raw, (long long)add_times[i]);
    }

    return TEST_SUCCESS;
}

DECLARE_TEST(test_system_set_timestamp_offset_ns) {
    timestamp_offset_ns = 0;

    // offset >= 2^60
    TEST_ASSERT(!set_timestamp_offset_ns((int64_t)(1ull << 60)), "2^60 should be rejected");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Timestamp offset out of range");
    TEST_ASSERT(timestamp_offset_ns == 0, "offset changed after rejected set");

    // max valid non-negative offset
    TEST_ASSERT(set_timestamp_offset_ns((int64_t)((1ull << 60) - 1)), "2^60 - 1 should succeed");
    TEST_ASSERT(timestamp_offset_ns == (int64_t)((1ull << 60) - 1), "offset not set");
    timestamp_offset_ns = 0;

    // zero and positive offsets
    TEST_ASSERT(set_timestamp_offset_ns(0), "zero offset should succeed");
    TEST_ASSERT(timestamp_offset_ns == 0, "offset not set");

    TEST_ASSERT(set_timestamp_offset_ns(12345), "positive offset should succeed");
    TEST_ASSERT(timestamp_offset_ns == 12345, "offset not set");
    timestamp_offset_ns = 0;

    // negative offset: magnitude >= raw board time
    uint64_t now = raw_currtime_ns();
    TEST_ASSERT(!set_timestamp_offset_ns(-((int64_t)now + 1000000)), "offset less than now should be rejected");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Negative timestamp offset magnitude");
    TEST_ASSERT(timestamp_offset_ns == 0, "offset changed after rejected set");

    // negative offset: magnitude < raw board time
    TEST_ASSERT(now > 0, "raw board time must be > 0 for negative offset tests");
    TEST_ASSERT(set_timestamp_offset_ns(-1), "small negative offset should succeed");
    TEST_ASSERT(timestamp_offset_ns == -1, "offset not set");
    timestamp_offset_ns = 0;

    TEST_ASSERT(set_timestamp_offset_ns(-((int64_t)now - 1)), "max valid negative offset should succeed");
    TEST_ASSERT(timestamp_offset_ns == -((int64_t)now - 1), "offset not set");

    return TEST_SUCCESS;
}

DECLARE_TEST(test_system_get_board_time) {
    uint32_t msg_id = 42;
    int64_t offsets[] = {0, 100000000, -1000};
    uint8_t dummy_payload = 0xAB;

    for (size_t j = 0; j < sizeof(offsets) / sizeof(offsets[0]); j++) {
        eth_write_buf_start = eth_write_buf_end = 0;
        timestamp_offset_ns = offsets[j];

        uint64_t before = currtime_ns();
        TEST_ASSERT(handle_get_board_time(msg_id, &dummy_payload), "handler should return true");  // payload is ignored
        uint64_t after = currtime_ns();

        // Copy the response so we can check the result board time
        GetBoardTimeResponsePayload resp;
        memcpy(&resp, eth_write_buf + sizeof(ClientCommandHeader), sizeof(resp));
        TEST_ASSERT(resp.board_time_nanos >= before && resp.board_time_nanos <= after,
            "board time %llu not in [%llu, %llu]",
            (unsigned long long)resp.board_time_nanos, (unsigned long long)before, (unsigned long long)after);

        TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_BOARD_TIME, &resp, sizeof(resp)), "buf mismatch");
    }

    return TEST_SUCCESS;
}

DECLARE_TEST(test_system_handle_set_timestamp_offset) {
    uint32_t msg_id = 99;  // ignored by handler
    SetTimestampOffsetPayload payload;
    uint64_t now = raw_currtime_ns();

    struct {
        int64_t offset;
        bool should_succeed;
        const char* err_substr;
    } cases[] = {
        {0, true, NULL},
        {12345, true, NULL},
        {-1, true, NULL},
        {(int64_t)((1ull << 60) - 1), true, NULL},
        {(int64_t)(1ull << 60), false, "Timestamp offset out of range"},
        {-((int64_t)now + 1000000), false, "Negative timestamp offset magnitude"},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        timestamp_offset_ns = 0;
        eth_write_buf_start = eth_write_buf_end = 0;
        payload.offset = cases[i].offset;

        TEST_ASSERT(!handle_set_timestamp_offset(msg_id, &payload), "handler always returns false");
        TEST_ASSERT(eth_write_buf_start == eth_write_buf_end, "handler should not write to eth buffer");

        if (cases[i].should_succeed) {
            TEST_ASSERT(timestamp_offset_ns == cases[i].offset,
                "offset was %lld, expected %lld",
                (long long)timestamp_offset_ns, (long long)cases[i].offset);
        } else {
            TEST_ASSERT_LOCAL_ERROR_MESSAGE(cases[i].err_substr);
            TEST_ASSERT(timestamp_offset_ns == 0, "offset changed after rejected set");
        }
    }

    return TEST_SUCCESS;
}

RUN_TESTS();
