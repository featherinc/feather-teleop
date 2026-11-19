#include "../../framework.hpp"
#include "../../../src/robot/services/data_monitor.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

using feather::robot::DataMonitorFileWriter;
using feather::robot::DataMonitorFileWriterConfig;
using feather::robot::DataMonitorService;
using feather::robot::DataMonitorServiceConfig;

namespace {

constexpr std::uint64_t kRecordBytes = sizeof(std::uint64_t);
constexpr std::uint64_t kVarChunk = 256;

struct TempDir {
    std::filesystem::path path;
    explicit TempDir(const char* name)
        : path(std::filesystem::path("/tmp") / (std::string(name) + std::to_string(::getpid()))) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directory(path);
    }
    ~TempDir() { std::filesystem::remove_all(path); }
    std::string file(const char* name) const { return (path / name).string(); }
};

std::vector<std::uint8_t> read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

DataMonitorFileWriterConfig make_cfg(
    const std::string& path, FthrFileFormat format, std::uint64_t chunk, std::uint64_t slots, const char* name
) {
    DataMonitorFileWriterConfig cfg;
    cfg.path = path;
    cfg.data_name = name;
    cfg.format = format;
    cfg.chunk_size = chunk;
    cfg.max_file_size = sizeof(FthrFileHeader) + slots * chunk;
    return cfg;
}

bool header_matches(
    const std::vector<std::uint8_t>& file, const char* name, FthrFileFormat format, std::uint64_t chunk
) {
    if (file.size() < sizeof(FthrFileHeader)) return false;
    FthrFileHeader header{};
    std::memcpy(&header, file.data(), sizeof header);
    return std::memcmp(header.magic, FTHR_MAGIC, 4) == 0 && header.version_major == 1 && header.version_minor == 0 &&
        header.version_micro == 0 && header.version_patch == 0 &&
        std::strcmp(reinterpret_cast<const char*>(header.data_name), name) == 0 && header.file_format == format &&
        header.chunk_size == chunk && header.closed_safely == FTHR_FILE_CLOSED_SAFELY_CLEAN;
}

bool fixed_records_match(const std::vector<std::uint8_t>& file, const std::vector<std::uint64_t>& expect) {
    if (file.size() != sizeof(FthrFileHeader) + expect.size() * kRecordBytes) return false;
    for (std::size_t i = 0; i < expect.size(); ++i) {
        std::uint64_t got = 0;
        std::memcpy(&got, file.data() + sizeof(FthrFileHeader) + i * kRecordBytes, sizeof got);
        if (got != expect[i]) return false;
    }
    return true;
}

// Values written are 1..count. Slot `count % slots` holds the oldest of the last `slots` values.
std::vector<std::uint64_t> fixed_ring(std::uint64_t count, std::uint64_t slots) {
    std::vector<std::uint64_t> out(static_cast<std::size_t>(slots));
    const std::uint64_t oldest = count % slots;
    for (std::uint64_t k = 0; k < slots; ++k) {
        out[static_cast<std::size_t>((oldest + k) % slots)] = count - slots + 1 + k;
    }
    return out;
}

bool write_fixed_values(DataMonitorFileWriter& writer, std::uint64_t count) {
    std::vector<std::uint64_t> values(static_cast<std::size_t>(count));
    for (std::uint64_t i = 0; i < count; ++i) values[i] = i + 1;
    return writer.write(values.data(), values.size() * kRecordBytes);
}

bool run_fixed(const std::string& path, std::uint64_t slots, std::uint64_t count, std::vector<std::uint64_t> expect) {
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_FIXED, kRecordBytes, slots, "fixed"));
        if (!write_fixed_values(writer, count)) return false;
    }
    const auto file = read_file(path);
    return header_matches(file, "fixed", FTHR_FILE_FORMAT_FIXED, kRecordBytes) && fixed_records_match(file, expect);
}

bool rejects_config(DataMonitorFileWriterConfig cfg) {
    try {
        DataMonitorFileWriter writer(cfg);
    } catch (const std::invalid_argument&) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

FthrBlockHeader block_at(const std::vector<std::uint8_t>& file, std::uint64_t slot, std::uint64_t chunk) {
    FthrBlockHeader header{};
    const std::size_t off = sizeof(FthrFileHeader) + static_cast<std::size_t>(slot * chunk);
    if (off + sizeof header <= file.size()) std::memcpy(&header, file.data() + off, sizeof header);
    return header;
}

bool write_var_items(DataMonitorFileWriter& writer, std::uint32_t count, std::size_t payload_size) {
    for (std::uint32_t id = 1; id <= count; ++id) {
        std::vector<std::uint8_t> payload(payload_size, 0);
        if (payload_size >= sizeof id) std::memcpy(payload.data(), &id, sizeof id);
        if (!writer.write(payload.data(), payload.size())) return false;
    }
    return true;
}

bool block_ids(
    const std::vector<std::uint8_t>& file, std::uint64_t slot, std::uint64_t chunk, std::size_t payload,
    std::int64_t num_items, std::uint64_t block_idx, std::initializer_list<std::uint32_t> ids
) {
    const FthrBlockHeader block = block_at(file, slot, chunk);
    if (block.num_items != num_items || block.block_idx != block_idx) return false;
    std::size_t off = sizeof(FthrFileHeader) + static_cast<std::size_t>(slot * chunk) + sizeof(FthrBlockHeader);
    for (std::uint32_t id : ids) {
        if (payload < sizeof id || off + sizeof(std::uint64_t) + payload > file.size()) return false;
        std::uint64_t nbytes = 0;
        std::uint32_t got = 0;
        std::memcpy(&nbytes, file.data() + off, sizeof nbytes);
        std::memcpy(&got, file.data() + off + sizeof nbytes, sizeof got);
        if (nbytes != payload || got != id) return false;
        off += sizeof nbytes + payload;
    }
    return true;
}

}  // namespace

DECLARE_TEST(test_invalid_configs) {
    TempDir dir("fthr_bad_cfg_");
    const auto base = make_cfg(dir.file("ok.fthr"), FTHR_FILE_FORMAT_FIXED, kRecordBytes, 1, "fixed");

    auto cfg = base;
    cfg.path.clear();
    TEST_ASSERT(rejects_config(cfg), "empty path");

    cfg = base;
    cfg.data_name = std::string(32, 'a');
    TEST_ASSERT(rejects_config(cfg), "32-byte name");

    cfg = base;
    cfg.chunk_size = 0;
    TEST_ASSERT(rejects_config(cfg), "chunk size 0");

    cfg = base;
    cfg.max_file_size = sizeof(FthrFileHeader) + kRecordBytes - 1;
    TEST_ASSERT(rejects_config(cfg), "max file size too small");

    cfg = base;
    cfg.write_buffer_bytes = 0;
    TEST_ASSERT(rejects_config(cfg), "buffer size 0");

    cfg = make_cfg(dir.file("ok.fthr"), FTHR_FILE_FORMAT_VARIABLE, sizeof(FthrBlockHeader) + sizeof(std::uint64_t) - 1, 1, "var");
    TEST_ASSERT(rejects_config(cfg), "variable block cannot hold an item");

    const std::string name(31, 'n');
    cfg = make_cfg(dir.file("name.fthr"), FTHR_FILE_FORMAT_FIXED, kRecordBytes, 1, name.c_str());
    {
        DataMonitorFileWriter writer(cfg);
        writer.abort();
    }
    TEST_ASSERT(header_matches(read_file(dir.file("name.fthr")), name.c_str(), FTHR_FILE_FORMAT_FIXED, kRecordBytes), "31-byte name");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_missing_directory) {
    const char* path = "/tmp/fthr_missing_dir_no_such/out.fthr";
    bool threw = false;
    try {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_FIXED, kRecordBytes, 1, "fixed"));
    } catch (const std::runtime_error&) {
        threw = true;
    }
    TEST_ASSERT(threw, "missing directory");
    TEST_ASSERT(!std::filesystem::exists(path), "file was not created");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_empty_close) {
    TempDir dir("fthr_fixed_empty_");
    const std::string path = dir.file("empty.fthr");
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_FIXED, kRecordBytes, 4, "fixed"));
        writer.abort();
        writer.abort();
    }
    const auto file = read_file(path);
    TEST_ASSERT(file.size() == sizeof(FthrFileHeader), "header only");
    TEST_ASSERT(header_matches(file, "fixed", FTHR_FILE_FORMAT_FIXED, kRecordBytes), "header");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_one_record) {
    TempDir dir("fthr_fixed_one_");
    TEST_ASSERT(run_fixed(dir.file("one.fthr"), 4, 1, {1}), "one record");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_many_records) {
    TempDir dir("fthr_fixed_many_");
    TEST_ASSERT(run_fixed(dir.file("many.fthr"), 16, 10, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}), "records");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_one_lap) {
    TempDir dir("fthr_fixed_lap_");
    constexpr std::uint64_t kSlots = 4;
    TEST_ASSERT(run_fixed(dir.file("lap.fthr"), kSlots, kSlots * 2, fixed_ring(kSlots * 2, kSlots)), "second lap");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_one_slot) {
    TempDir dir("fthr_fixed_slot_");
    TEST_ASSERT(run_fixed(dir.file("slot.fthr"), 1, 5, {5}), "last record");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_many_laps) {
    TempDir dir("fthr_fixed_laps_");
    constexpr std::uint64_t kSlots = 4;
    constexpr std::uint64_t kCount = kSlots * 5 + 2;
    TEST_ASSERT(run_fixed(dir.file("laps.fthr"), kSlots, kCount, fixed_ring(kCount, kSlots)), "surviving records");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_rejects_partial_and_empty_write) {
    TempDir dir("fthr_fixed_partial_");
    const std::string path = dir.file("partial.fthr");
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_FIXED, kRecordBytes, 4, "fixed"));
        const std::uint64_t value = 7;
        TEST_ASSERT(writer.write(nullptr, 0), "empty write");
        TEST_ASSERT(!writer.write(&value, 1), "partial record");
        TEST_ASSERT(writer.write(&value, sizeof value), "full record");
    }
    TEST_ASSERT(fixed_records_match(read_file(path), {7}), "only the full record");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_one_write_wraps) {
    TempDir dir("fthr_fixed_wrap_");
    // Records 1..6 into 4 slots: the last two overwrite the first two.
    TEST_ASSERT(run_fixed(dir.file("wrap.fthr"), 4, 6, {5, 6, 3, 4}), "wrapped in one write");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_fixed_unused_tail) {
    TempDir dir("fthr_fixed_tail_");
    const std::string path = dir.file("tail.fthr");
    DataMonitorFileWriterConfig cfg = make_cfg(path, FTHR_FILE_FORMAT_FIXED, kRecordBytes, 2, "fixed");
    cfg.max_file_size += kRecordBytes / 2;
    {
        DataMonitorFileWriter writer(cfg);
        TEST_ASSERT(write_fixed_values(writer, 3), "write");
    }
    const auto file = read_file(path);
    TEST_ASSERT(file.size() == sizeof(FthrFileHeader) + 2 * kRecordBytes, "slack was not written");
    TEST_ASSERT(fixed_records_match(file, {3, 2}), "third record wrapped");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_empty_close) {
    TempDir dir("fthr_var_empty_");
    const std::string path = dir.file("empty.fthr");
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "var"));
        writer.abort();
    }
    const auto file = read_file(path);
    TEST_ASSERT(file.size() == sizeof(FthrFileHeader) + sizeof(FthrBlockHeader), "header and one empty block");
    TEST_ASSERT(header_matches(file, "var", FTHR_FILE_FORMAT_VARIABLE, kVarChunk), "header");
    const FthrBlockHeader block = block_at(file, 0, kVarChunk);
    TEST_ASSERT(block.num_items == 0 && block.block_idx == 0, "empty block is sealed");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_one_item) {
    TempDir dir("fthr_var_one_");
    const std::string path = dir.file("one.fthr");
    constexpr std::size_t kPayload = 4;
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "var"));
        TEST_ASSERT(write_var_items(writer, 1, kPayload), "write");
        const FthrBlockHeader open = block_at(read_file(path), 0, kVarChunk);
        TEST_ASSERT(open.num_items == -1, "open block is not sealed yet");
    }
    const auto file = read_file(path);
    TEST_ASSERT(header_matches(file, "var", FTHR_FILE_FORMAT_VARIABLE, kVarChunk), "header");
    TEST_ASSERT(block_ids(file, 0, kVarChunk, kPayload, 1, 0, {1}), "sealed item");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_many_items) {
    TempDir dir("fthr_var_many_");
    const std::string path = dir.file("many.fthr");
    constexpr std::size_t kPayload = 4;
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "var"));
        TEST_ASSERT(write_var_items(writer, 3, kPayload), "write");
    }
    const auto file = read_file(path);
    TEST_ASSERT(header_matches(file, "var", FTHR_FILE_FORMAT_VARIABLE, kVarChunk), "header");
    TEST_ASSERT(block_ids(file, 0, kVarChunk, kPayload, 3, 0, {1, 2, 3}), "one block");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_one_lap) {
    TempDir dir("fthr_var_lap_");
    const std::string path = dir.file("lap.fthr");
    // Two 56-byte payloads fill a block. Eight items are two laps of a two-block ring.
    constexpr std::size_t kPayload = 56;
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "var"));
        TEST_ASSERT(write_var_items(writer, 8, kPayload), "write");
    }
    const auto file = read_file(path);
    TEST_ASSERT(header_matches(file, "var", FTHR_FILE_FORMAT_VARIABLE, kVarChunk), "header");
    TEST_ASSERT(file.size() == sizeof(FthrFileHeader) + 2 * kVarChunk, "two blocks");
    TEST_ASSERT(block_ids(file, 0, kVarChunk, kPayload, 2, 2, {5, 6}), "slot 0");
    TEST_ASSERT(block_ids(file, 1, kVarChunk, kPayload, 2, 3, {7, 8}), "slot 1");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_one_block) {
    TempDir dir("fthr_var_oneblock_");
    const std::string path = dir.file("one.fthr");
    constexpr std::size_t kPayload = 56;
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 1, "var"));
        TEST_ASSERT(write_var_items(writer, 6, kPayload), "write");
    }
    const auto file = read_file(path);
    TEST_ASSERT(file.size() == sizeof(FthrFileHeader) + kVarChunk, "one block");
    TEST_ASSERT(block_ids(file, 0, kVarChunk, kPayload, 2, 2, {5, 6}), "last block");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_many_laps) {
    TempDir dir("fthr_var_laps_");
    const std::string path = dir.file("laps.fthr");
    constexpr std::size_t kPayload = 56;
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "var"));
        TEST_ASSERT(write_var_items(writer, 20, kPayload), "write");  // 10 blocks, 5 laps.
    }
    const auto file = read_file(path);
    TEST_ASSERT(block_ids(file, 0, kVarChunk, kPayload, 2, 8, {17, 18}), "slot 0");
    TEST_ASSERT(block_ids(file, 1, kVarChunk, kPayload, 2, 9, {19, 20}), "slot 1");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_rejects_oversize_item) {
    TempDir dir("fthr_var_big_");
    const std::string path = dir.file("big.fthr");
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "var"));
        std::uint8_t huge[121];
        std::memset(huge, 0xab, sizeof huge);
        TEST_ASSERT(!writer.write(huge, sizeof huge), "too big");
        TEST_ASSERT(write_var_items(writer, 1, 4), "later item");
    }
    TEST_ASSERT(block_ids(read_file(path), 0, kVarChunk, 4, 1, 0, {1}), "only the accepted item");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_remainder_starts_new_block) {
    TempDir dir("fthr_var_rest_");
    const std::string path = dir.file("rest.fthr");
    // A 32-byte payload is 40 bytes on disk. Three fit in the 128-byte area; the fourth does not.
    constexpr std::size_t kPayload = 32;
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "var"));
        TEST_ASSERT(write_var_items(writer, 4, kPayload), "write");
    }
    const auto file = read_file(path);
    TEST_ASSERT(block_ids(file, 0, kVarChunk, kPayload, 3, 0, {1, 2, 3}), "first block");
    TEST_ASSERT(block_ids(file, 1, kVarChunk, kPayload, 1, 1, {4}), "second block");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_variable_empty_item) {
    TempDir dir("fthr_var_zero_");
    const std::string path = dir.file("zero.fthr");
    constexpr std::uint64_t kChunk = sizeof(FthrBlockHeader) + sizeof(std::uint64_t);
    {
        DataMonitorFileWriter writer(make_cfg(path, FTHR_FILE_FORMAT_VARIABLE, kChunk, 1, "var"));
        std::uint8_t byte = 1;
        TEST_ASSERT(writer.write(nullptr, 0), "empty item");
        TEST_ASSERT(!writer.write(&byte, 1), "one byte does not fit");
    }
    const auto file = read_file(path);
    TEST_ASSERT(header_matches(file, "var", FTHR_FILE_FORMAT_VARIABLE, kChunk), "header");
    const FthrBlockHeader block = block_at(file, 0, kChunk);
    TEST_ASSERT(block.num_items == 1 && block.block_idx == 0, "empty item stored");
    std::uint64_t nbytes = 0;
    const std::size_t off = sizeof(FthrFileHeader) + sizeof(FthrBlockHeader);
    TEST_ASSERT(file.size() >= off + sizeof nbytes, "length prefix");
    std::memcpy(&nbytes, file.data() + off, sizeof nbytes);
    TEST_ASSERT(nbytes == 0, "zero length");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_write_after_close) {
    TempDir dir("fthr_closed_");
    DataMonitorFileWriter writer(make_cfg(dir.file("closed.fthr"), FTHR_FILE_FORMAT_FIXED, kRecordBytes, 2, "fixed"));
    writer.abort();
    const std::uint64_t value = 1;
    TEST_ASSERT(!writer.write(&value, sizeof value), "write after close");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_invalid_format) {
    TempDir dir("fthr_fmt_");
    const std::string path = dir.file("fmt.fthr");
    DataMonitorFileWriterConfig cfg = make_cfg(path, FTHR_FILE_FORMAT_FIXED, kRecordBytes, 1, "fixed");
    cfg.format = static_cast<FthrFileFormat>(9);
    {
        DataMonitorFileWriter writer(cfg);
        const std::uint64_t value = 1;
        TEST_ASSERT(!writer.write(&value, sizeof value), "unknown format");
    }
    const auto file = read_file(path);
    TEST_ASSERT(header_matches(file, "fixed", static_cast<FthrFileFormat>(9), kRecordBytes), "closed cleanly");
    TEST_ASSERT(file.size() == sizeof(FthrFileHeader), "no records");
    return TEST_SUCCESS;
}

// One variable-format item: length prefix, then the bytes passed to write().
bool variable_item_is(const std::vector<std::uint8_t>& file, const void* payload, std::size_t size) {
    const std::size_t off = sizeof(FthrFileHeader) + sizeof(FthrBlockHeader);
    if (file.size() < off + sizeof(std::uint64_t) + size) return false;
    std::uint64_t nbytes = 0;
    std::memcpy(&nbytes, file.data() + off, sizeof nbytes);
    if (nbytes != size) return false;
    return std::memcmp(file.data() + off + sizeof nbytes, payload, size) == 0;
}

DECLARE_TEST(test_service_writes_enabled_streams) {
    TempDir dir("fthr_svc_");
    const std::string candump_path = dir.file("candump.fthr");
    const std::string telemetry_path = dir.file("telemetry.fthr");
    const std::uint8_t candump[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    const std::uint8_t telemetry[] = {0x02, 0x00, 0x01, 0x00, 0x01, 0x00, 0xAB};

    DataMonitorServiceConfig cfg;
    cfg.candump_config = make_cfg(candump_path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "candump");
    cfg.telemetry_config = make_cfg(telemetry_path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "telemetry");
    {
        DataMonitorService service(cfg);
        TEST_ASSERT(service.write_candump(candump, sizeof candump), "candump write");
        TEST_ASSERT(service.write_telemetry(telemetry, sizeof telemetry), "telemetry write");
        TEST_ASSERT(service.write_candump(nullptr, 0), "empty candump");
        TEST_ASSERT(service.write_telemetry(nullptr, 0), "empty telemetry");
    }

    const auto candump_file = read_file(candump_path);
    const auto telemetry_file = read_file(telemetry_path);
    TEST_ASSERT(header_matches(candump_file, "candump", FTHR_FILE_FORMAT_VARIABLE, kVarChunk), "candump header");
    TEST_ASSERT(header_matches(telemetry_file, "telemetry", FTHR_FILE_FORMAT_VARIABLE, kVarChunk), "telemetry header");
    TEST_ASSERT(variable_item_is(candump_file, candump, sizeof candump), "candump bytes");
    TEST_ASSERT(variable_item_is(telemetry_file, telemetry, sizeof telemetry), "telemetry bytes");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_service_skips_unset_writers) {
    TempDir dir("fthr_svc_off_");
    const std::string candump_path = dir.file("candump.fthr");
    DataMonitorServiceConfig cfg;
    cfg.candump_config = make_cfg(candump_path, FTHR_FILE_FORMAT_VARIABLE, kVarChunk, 2, "candump");

    const std::uint8_t byte = 1;
    {
        DataMonitorService service(cfg);
        TEST_ASSERT(service.write_candump(&byte, 1), "candump enabled");
        TEST_ASSERT(service.write_telemetry(&byte, 1), "telemetry disabled");
        TEST_ASSERT(!service.write_candump(nullptr, 1), "null candump payload");
    }
    TEST_ASSERT(header_matches(read_file(candump_path), "candump", FTHR_FILE_FORMAT_VARIABLE, kVarChunk), "candump file");
    TEST_ASSERT(!std::filesystem::exists(dir.file("telemetry.fthr")), "no telemetry file");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_service_abort_rejects_later_writes) {
    TempDir dir("fthr_svc_abort_");
    DataMonitorServiceConfig cfg;
    cfg.telemetry_config = make_cfg(dir.file("telemetry.fthr"), FTHR_FILE_FORMAT_FIXED, kRecordBytes, 2, "telemetry");
    DataMonitorService service(cfg);
    service.abort();
    const std::uint64_t value = 1;
    TEST_ASSERT(!service.write_telemetry(&value, sizeof value), "write after abort");
    TEST_ASSERT(service.write_candump(&value, sizeof value), "unset candump still succeeds");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_service_bad_second_config_closes_first) {
    TempDir dir("fthr_svc_bad_");
    const std::string candump_path = dir.file("candump.fthr");
    DataMonitorServiceConfig cfg;
    cfg.candump_config = make_cfg(candump_path, FTHR_FILE_FORMAT_FIXED, kRecordBytes, 1, "candump");
    cfg.telemetry_config = make_cfg(dir.file("telemetry.fthr"), FTHR_FILE_FORMAT_FIXED, kRecordBytes, 1, "telemetry");
    cfg.telemetry_config->path.clear();

    bool threw = false;
    try {
        DataMonitorService service(cfg);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    TEST_ASSERT(threw, "empty telemetry path");
    TEST_ASSERT(header_matches(read_file(candump_path), "candump", FTHR_FILE_FORMAT_FIXED, kRecordBytes), "candump closed");
    return TEST_SUCCESS;
}

RUN_TESTS()
