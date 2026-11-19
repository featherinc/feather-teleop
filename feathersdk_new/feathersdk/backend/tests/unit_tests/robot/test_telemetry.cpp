#include "../../framework.hpp"
#include "../../../src/robot/telemetry.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

using feather::robot::Telemetry;
using feather::robot::TelemetryAccess;

namespace feather::robot {

struct TelemetryAccess {
    template <typename T>
    static std::vector<T> read(const Telemetry& t, std::size_t index) {
        const std::uint64_t count = t.counts[index];
        const std::uint64_t cap = t.capacities[index];
        const std::uint64_t start = t.starts[index];
        const std::uint8_t* ring = t.data.data() + t.offsets[index];
        std::vector<T> out(static_cast<std::size_t>(count));
        for (std::uint64_t i = 0; i < count; ++i) {
            std::memcpy(&out[i], ring + ((start + i) % cap) * sizeof(T), sizeof(T));
        }
        return out;
    }

    static std::uint64_t offset(const Telemetry& t, std::size_t i) { return t.offsets[i]; }
    static std::uint64_t itemsize(const Telemetry& t, std::size_t i) { return t.itemsizes[i]; }
    static DataType dtype(const Telemetry& t, std::size_t i) { return t.dtypes[i]; }
    static std::size_t bytes(const Telemetry& t) { return t.data.size(); }
};

}  // namespace feather::robot

template <typename T>
static std::vector<std::uint8_t> record(unsigned index, std::initializer_list<T> values) {
    std::vector<std::uint8_t> buf;
    const auto put16 = [&](unsigned v) {
        buf.push_back(static_cast<std::uint8_t>(v));
        buf.push_back(static_cast<std::uint8_t>(v >> 8));
    };
    put16(index);
    put16(static_cast<unsigned>(values.size()));
    for (T value : values) {
        const auto* p = reinterpret_cast<const std::uint8_t*>(&value);
        buf.insert(buf.end(), p, p + sizeof(T));
    }
    return buf;
}

template <typename T>
static test_result_t check(const Telemetry& t, std::size_t index, std::initializer_list<T> want) {
    const std::vector<T> got = TelemetryAccess::read<T>(t, index);
    TEST_ASSERT(got.size() == want.size(), "count %zu != %zu", got.size(), want.size());
    std::size_t i = 0;
    for (T value : want) {
        TEST_ASSERT(got[i] == value, "[%zu]", i);
        ++i;
    }
    return TEST_SUCCESS;
}

template <typename T>
static bool push(Telemetry& t, unsigned index, std::initializer_list<T> values) {
    const std::vector<std::uint8_t> buf = record(index, values);
    return t.update_telemetry(1, buf.data(), buf.size());
}

DECLARE_TEST(test_lifecycle) {
    std::uint8_t byte = 0;
    Telemetry t;
    TEST_ASSERT(!t.initialized(), "fresh manager is not initialized");
    TEST_ASSERT(!t.update_telemetry(0, &byte, 0), "update before initialize");
    TEST_ASSERT(!t.initialize(), "initialize with no channels");

    TEST_ASSERT(t.add_telemetry(4, DATA_TYPE_F32) == 0, "first channel index");
    TEST_ASSERT(t.add_telemetry(2, DATA_TYPE_I16) == 1, "second channel index");
    TEST_ASSERT(t.add_telemetry(1, static_cast<DataType>(255)) == -1, "unknown dtype");
    TEST_ASSERT(t.initialize(), "initialize");
    TEST_ASSERT(t.initialized(), "initialized");
    TEST_ASSERT(TelemetryAccess::itemsize(t, 0) == 4 && TelemetryAccess::itemsize(t, 1) == 2, "itemsizes");
    TEST_ASSERT(TelemetryAccess::offset(t, 0) == 0 && TelemetryAccess::offset(t, 1) == 16, "offsets");
    TEST_ASSERT(TelemetryAccess::dtype(t, 0) == DATA_TYPE_F32 && TelemetryAccess::dtype(t, 1) == DATA_TYPE_I16, "dtypes");
    TEST_ASSERT(TelemetryAccess::bytes(t) == 20, "arena is 4*4 + 2*2 bytes");
    TEST_ASSERT(t.add_telemetry(1, DATA_TYPE_U8) == -1, "add after initialize");
    TEST_ASSERT(!t.initialize(), "second initialize");

    Telemetry at_cap;
    TEST_ASSERT(at_cap.add_telemetry(1024, DATA_TYPE_U8) == 0, "capacity 1024");
    TEST_ASSERT(at_cap.initialize(), "capacity 1024 is the maximum");
    Telemetry over_cap;
    TEST_ASSERT(over_cap.add_telemetry(1025, DATA_TYPE_U8) == 0, "capacity 1025");
    TEST_ASSERT(!over_cap.initialize() && !over_cap.initialized(), "capacity 1025");

    Telemetry at_n;
    int last = -1;
    for (int i = 0; i < 1024; ++i) last = at_n.add_telemetry(1, DATA_TYPE_U8);
    TEST_ASSERT(last == 1023 && at_n.initialize(), "1024 channels");
    Telemetry over_n;
    for (int i = 0; i < 1025; ++i) over_n.add_telemetry(1, DATA_TYPE_U8);
    TEST_ASSERT(!over_n.initialize() && !over_n.initialized(), "1025 channels");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_ring) {
    Telemetry t;
    TEST_ASSERT(t.add_telemetry(4, DATA_TYPE_I32) == 0 && t.initialize(), "setup");
    TEST_ASSERT_STACKED(check<std::int32_t>(t, 0, {}), "empty");

    TEST_ASSERT(push(t, 0, {10, 20}), "write 2");
    TEST_ASSERT_STACKED(check<std::int32_t>(t, 0, {10, 20}), "two values");
    TEST_ASSERT(push(t, 0, {30}), "write 1");
    TEST_ASSERT_STACKED(check<std::int32_t>(t, 0, {10, 20, 30}), "three values");

    // Past the physical end: the oldest value is dropped.
    TEST_ASSERT(push(t, 0, {40, 50}), "wrap");
    TEST_ASSERT_STACKED(check<std::int32_t>(t, 0, {20, 30, 40, 50}), "wrapped");

    // A batch bigger than the ring keeps the last `capacity` values and resets it.
    TEST_ASSERT(push(t, 0, {1, 2, 3, 4, 5, 6}), "oversized");
    TEST_ASSERT_STACKED(check<std::int32_t>(t, 0, {3, 4, 5, 6}), "last four");

    Telemetry filled;
    TEST_ASSERT(filled.add_telemetry(4, DATA_TYPE_I32) == 0 && filled.initialize(), "setup");
    TEST_ASSERT(push(filled, 0, {1, 2, 3}) && push(filled, 0, {4}), "fill exactly");
    TEST_ASSERT_STACKED(check<std::int32_t>(filled, 0, {1, 2, 3, 4}), "full");
    TEST_ASSERT(push(filled, 0, {5, 6}), "overwrite");
    TEST_ASSERT_STACKED(check<std::int32_t>(filled, 0, {3, 4, 5, 6}), "overwritten");

    TEST_ASSERT(push<std::int32_t>(filled, 0, {}), "empty update");
    TEST_ASSERT_STACKED(check<std::int32_t>(filled, 0, {3, 4, 5, 6}), "empty update changes nothing");
    return TEST_SUCCESS;
}

static void i32_channel(Telemetry& t) {
    t.add_telemetry(4, DATA_TYPE_I32);
    t.initialize();
}

// A valid [10, 20] record followed by `tail`. The call asks for two records and must fail,
// leaving the first record stored.
static test_result_t reject_after_good(std::vector<std::uint8_t> tail, const char* why) {
    Telemetry t;
    i32_channel(t);
    auto payload = record<std::int32_t>(0, {10, 20});
    payload.insert(payload.end(), tail.begin(), tail.end());
    TEST_ASSERT(!t.update_telemetry(2, payload.data(), payload.size()), "%s", why);
    TEST_ASSERT_STACKED(check<std::int32_t>(t, 0, {10, 20}), "%s", why);
    return TEST_SUCCESS;
}

DECLARE_TEST(test_update) {
    Telemetry t;
    TEST_ASSERT(t.add_telemetry(4, DATA_TYPE_F32) == 0, "f32");
    TEST_ASSERT(t.add_telemetry(3, DATA_TYPE_I16) == 1, "i16");
    TEST_ASSERT(t.initialize(), "init");

    TEST_ASSERT(!t.update_telemetry(-1, nullptr, 0), "negative count");
    TEST_ASSERT(!t.update_telemetry(0, nullptr, 0), "null payload");
    std::uint8_t byte = 0;
    TEST_ASSERT(t.update_telemetry(0, &byte, 0), "zero updates");

    auto both = record<float>(0, {1.5f, -2.25f});
    const auto i16 = record<std::int16_t>(1, {7, 8, 9});
    both.insert(both.end(), i16.begin(), i16.end());
    TEST_ASSERT(t.update_telemetry(2, both.data(), both.size()), "both records");
    TEST_ASSERT_STACKED(check<float>(t, 0, {1.5f, -2.25f}), "f32");
    TEST_ASSERT_STACKED(check<std::int16_t>(t, 1, {7, 8, 9}), "i16");
    TEST_ASSERT(push<std::int16_t>(t, 1, {-1}), "i16 wrap");
    TEST_ASSERT_STACKED(check<float>(t, 0, {1.5f, -2.25f}), "other channel stays put");
    TEST_ASSERT_STACKED(check<std::int16_t>(t, 1, {8, 9, -1}), "i16 dropped its oldest value");

    Telemetry partial;
    TEST_ASSERT(partial.add_telemetry(4, DATA_TYPE_F32) == 0, "partial f32");
    TEST_ASSERT(partial.add_telemetry(3, DATA_TYPE_I16) == 1 && partial.initialize(), "partial init");
    TEST_ASSERT(partial.update_telemetry(1, both.data(), both.size()), "n_updates stops after the first record");
    TEST_ASSERT_STACKED(check<float>(partial, 0, {1.5f, -2.25f}), "first record");
    TEST_ASSERT_STACKED(check<std::int16_t>(partial, 1, {}), "later record was not read");

    Telemetry header;
    i32_channel(header);
    std::vector<std::uint8_t> short_header(3, 0);
    TEST_ASSERT(!header.update_telemetry(1, short_header.data(), short_header.size()), "truncated header");
    TEST_ASSERT_STACKED(check<std::int32_t>(header, 0, {}), "nothing stored");
    TEST_ASSERT_STACKED(reject_after_good({5, 0, 0, 0}, "index out of range"), "bad index");
    TEST_ASSERT_STACKED(reject_after_good({0, 0, 2, 0, 0x11}, "truncated body"), "short body");
    TEST_ASSERT_STACKED(reject_after_good({0, 0, 1}, "header cut off"), "cut header");
    return TEST_SUCCESS;
}

RUN_TESTS()
