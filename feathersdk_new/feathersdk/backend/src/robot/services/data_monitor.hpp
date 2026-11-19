#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <optional>
#include <sys/types.h>
#include "qdefs/fthr_file_qdef.hpp"

namespace feather::robot {

// Everything `DataMonitorFileWriter` needs to create a file.
struct DataMonitorFileWriterConfig {
    std::string path;  // Path to the file to write. Assumes the directory exists.
    std::string data_name;  // Null-terminated name of the data in this file. At most 31 bytes.
    FthrFileFormat format;
    std::uint64_t chunk_size;
    std::uint64_t max_file_size;
    std::uint64_t file_timeout_ns = 1 * 1000000;  // How long to retry after EINTR. The first attempt always runs.
    std::uint64_t write_buffer_bytes = 256 * 1024;  // FILE buffer size. Larger means fewer write syscalls.
};

// Implements the FTHR file format. Write fixed-sized records or variable-size records. Not thread-safe. Uses fwrite()
// so writes are buffered and much faster than direct system calls.
class DataMonitorFileWriter {
public:
    explicit DataMonitorFileWriter(const DataMonitorFileWriterConfig& config);
    ~DataMonitorFileWriter();

    // Disable copying and moving.
    DataMonitorFileWriter(const DataMonitorFileWriter&) = delete;
    DataMonitorFileWriter& operator=(const DataMonitorFileWriter&) = delete;
    DataMonitorFileWriter(DataMonitorFileWriter&&) = delete;
    DataMonitorFileWriter& operator=(DataMonitorFileWriter&&) = delete;

    /**
     * Writes the given data to the file. Handles fixed-size and variable-size records, new blocks, wrapping around
     * ring buffer, checking data size, etc. You can write more than one record at a time sequentially in memory.
     * @param data The data to write.
     * @param size The total number of bytes to write.
     * @return True on success, false otherwise.
     */
    bool write(const void* data, std::size_t size) noexcept;

    // Close the file. Sets closed_safely first unless a file error occurred. A second call is a no-op.
    void abort() noexcept;

private:
    DataMonitorFileWriterConfig config_;
    FILE* file_ = nullptr;
    std::uint64_t block_idx_ = 0;
    std::uint64_t block_start_ = 0;  // File offset of the open variable block.
    std::uint64_t block_used_ = 0;  // Item bytes written into the open block, after its header.
    std::int64_t block_items_ = 0;
    std::uint64_t file_pos_ = 0;
    bool closed_ = false;
    bool file_error_ = false;
    bool block_open_ = false;

    // Writes a new block header at current file position. Flushes file. Increments block_idx_. Returns true on success.
    bool create_new_block() noexcept;

    // Flush the open block, then set num_items. Last write to that block. Returns true on success.
    bool seal_block() noexcept;

    bool write_fixed(const void* data, std::size_t size) noexcept;
    bool write_variable(const void* data, std::size_t size) noexcept;

    // Set the closed_safely flag and flush the file. Mark as closed. Returns true on success.
    // NOTE: this will close the data writer even if marking as closed fails.
    bool set_closed_safely() noexcept;

    // Write data to file. Returns true on success. On error, logs an error message.
    bool dm_write(const void* data, std::size_t size) noexcept;
    // Seek to the given position. Returns true on success. On error, logs an error message.
    bool dm_seek(off_t pos) noexcept;
    // Flush and fsync the file. Returns true on success. On error, logs an error message.
    bool dm_flush() noexcept;
};

// Configuration for the data monitor service.
struct DataMonitorServiceConfig {
    std::optional<DataMonitorFileWriterConfig> candump_config;  // Optional: write candump data to a file.
    std::optional<DataMonitorFileWriterConfig> telemetry_config;  // Optional: write telemetry data to a file.
};

// Writes candump frames and actuator telemetry updates to .fthr files.
// A config left unset skips that writer. Writes are not thread-safe; the Teensy poll thread is the caller.
class DataMonitorService {
public:
    // Opens a writer for each config that is set. Throws if a config is invalid or a file cannot be opened.
    explicit DataMonitorService(const DataMonitorServiceConfig& config);
    ~DataMonitorService() noexcept = default;

    // Append one candump response body: packed CandumpTelemetry entries, with no count header.
    // Success when candump is disabled or size is 0. Returns false when the writer rejects the bytes.
    bool write_candump(const void* data, std::size_t size) noexcept;

    // Append one actuator-telemetry response body, including GetTelemetryPayloadHeader.
    // Success when telemetry is disabled or size is 0. Returns false when the writer rejects the bytes.
    bool write_telemetry(const void* data, std::size_t size) noexcept;

    // Close both files. Safe to call more than once.
    void abort() noexcept;

    bool has_candump() const noexcept { return candump_ != nullptr; }

private:
    bool write_to(DataMonitorFileWriter* writer, const char* what, const void* data, std::size_t size) noexcept;

    std::unique_ptr<DataMonitorFileWriter> candump_;
    std::unique_ptr<DataMonitorFileWriter> telemetry_;
};

}  // namespace feather::robot
