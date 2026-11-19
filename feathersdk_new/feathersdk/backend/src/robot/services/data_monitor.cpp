#include "data_monitor.hpp"

#include "qdefs/fthr_file_qdef.hpp"
#include "src/utils/logging.hpp"
#include "src/utils/system.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <cerrno>
#include <unistd.h>

namespace feather::robot {
namespace detail {

// True if an EINTR should be retried. The clock starts at the first EINTR, so the interrupted call does not consume
// the budget. A timeout of 0 does not retry.
bool eintr_retry(std::uint64_t timeout_ns, bool& started, feather_time_t& start) noexcept {
    if (!started) {
        started = true;
        start = feather::time::now();
        return timeout_ns > 0;
    }
    return feather::time::elapsed(start) <= timeout_ns;
}

void validate_DataMonitorFileWriterConfig(const DataMonitorFileWriterConfig& config) {
    if (config.path.empty()) throw std::invalid_argument("data monitor path is empty");
    if (config.data_name.size() >= sizeof(FthrFileHeader::data_name)) {  // Need terminating null byte.
        throw std::invalid_argument("data monitor data_name must be < 32 bytes");
    }
    if (config.chunk_size == 0) throw std::invalid_argument("data monitor chunk_size is 0");
    if (config.format == FTHR_FILE_FORMAT_VARIABLE &&
        config.chunk_size < sizeof(FthrBlockHeader) + sizeof(std::uint64_t)) {
        throw std::invalid_argument("data monitor block size cannot hold an item");
    }
    if (config.max_file_size < sizeof(FthrFileHeader) + config.chunk_size) {
        throw std::invalid_argument("data monitor max_file_size does not fit one chunk");
    }
    if (config.write_buffer_bytes == 0) throw std::invalid_argument("data monitor write_buffer_bytes is 0");
}

}  // namespace detail

DataMonitorFileWriter::DataMonitorFileWriter(const DataMonitorFileWriterConfig& config) {
    detail::validate_DataMonitorFileWriterConfig(config);
    config_ = config;

    // Open the file for writing.
    file_ = std::fopen(config_.path.c_str(), "w+b");
    if (file_ == nullptr) {
        throw std::runtime_error("data monitor failed to open " + config_.path + ": " + std::strerror(errno));
    }

    try {
        if (std::setvbuf(file_, nullptr, _IOFBF, static_cast<std::size_t>(config_.write_buffer_bytes)) != 0) {
            throw std::runtime_error("data monitor failed to set buffer " + config_.path + ": " + std::strerror(errno));
        }

        // Write the file header.
        FthrFileHeader header{};
        std::memcpy(header.magic, FTHR_MAGIC, sizeof header.magic);
        header.version_major = 1;
        header.version_minor = 0;
        header.version_micro = 0;
        header.version_patch = 0;
        std::memcpy(header.data_name, config_.data_name.data(), config_.data_name.size());
        header.file_format = config_.format;
        header.chunk_size = config_.chunk_size;
        header.closed_safely = FTHR_FILE_CLOSED_SAFELY_TORN;
        if (!dm_write(&header, sizeof header)) {
            throw std::runtime_error("data monitor failed to write " + config_.path + ": " + std::strerror(errno));
        }

        // If the file format is variable, create an initial empty block.
        if (config_.format == FTHR_FILE_FORMAT_VARIABLE) {
            if (!create_new_block()) {
                throw std::runtime_error(
                    std::string("data monitor failed to create initial empty block: ") + std::strerror(errno));
            }
        }
    } catch (...) {
        std::fclose(file_);
        file_ = nullptr;
        throw;
    }
}

bool DataMonitorFileWriter::create_new_block() noexcept {
    block_start_ = file_pos_;
    block_used_ = 0;
    block_items_ = 0;
    block_open_ = true;
    FthrBlockHeader header{.num_items = -1, .block_idx = block_idx_++};
    if (!dm_write(&header, sizeof header) || !dm_flush()) return false;
    return true;
}

bool DataMonitorFileWriter::seal_block() noexcept {
    if (!block_open_) return true;
    if (!dm_flush() || !dm_seek(static_cast<off_t>(block_start_))) return false;
    if (!dm_write(&block_items_, sizeof block_items_) || !dm_flush()) return false;
    block_open_ = false;
    return true;
}

bool DataMonitorFileWriter::write(const void* data, std::size_t size) noexcept {
    ENSURE(file_ != nullptr, false, "attempted data monitor [%s] write with unopened file", config_.data_name.c_str());
    ENSURE(!closed_, false, "attempted to write to data monitor [%s] when file is closed.", config_.data_name.c_str());
    ENSURE(!file_error_, false, "attempted write to data monitor [%s] after a file error.", config_.data_name.c_str());
    ENSURE(config_.format == FTHR_FILE_FORMAT_FIXED || config_.format == FTHR_FILE_FORMAT_VARIABLE, false,
        "data monitor [%s] invalid file format: %d", config_.data_name.c_str(), config_.format);

    if (config_.format == FTHR_FILE_FORMAT_FIXED) return write_fixed(data, size);
    return write_variable(data, size);
}

bool DataMonitorFileWriter::write_fixed(const void* data, std::size_t size) noexcept {
    ENSURE(size % config_.chunk_size == 0, false, 
        "data monitor [%s] write size is not a multiple of chunk size: %zu", config_.data_name.c_str(), size);

    const auto* bytes = static_cast<const std::uint8_t*>(data);
    while (size > 0) {
        // Not enough room for another whole record. Commit this lap, then wrap.
        // A failure after this flush leaves the committed tail in place.
        if (config_.max_file_size - file_pos_ < config_.chunk_size) {
            if (!dm_flush() || !dm_seek(static_cast<off_t>(sizeof(FthrFileHeader)))) return false;
            continue;
        }
        if (!dm_write(bytes, static_cast<std::size_t>(config_.chunk_size))) return false;
        bytes += config_.chunk_size;
        size -= config_.chunk_size;
    }
    return true;
}

bool DataMonitorFileWriter::write_variable(const void* data, std::size_t size) noexcept {
    const std::uint64_t capacity = config_.chunk_size - sizeof(FthrBlockHeader);
    const std::uint64_t item_bytes = sizeof(std::uint64_t) + static_cast<std::uint64_t>(size);
    ENSURE(item_bytes <= capacity, false, 
        "data monitor [%s] item of %zu bytes does not fit in a block", config_.data_name.c_str(), size);

    // If the block is full/not open, seal it and create a new one.
    if (!block_open_ || item_bytes > capacity - block_used_) {
        if (!seal_block()) return false;
        const std::uint64_t next = block_start_ + config_.chunk_size;
        const off_t dest = next + config_.chunk_size > config_.max_file_size
            ? static_cast<off_t>(sizeof(FthrFileHeader))
            : static_cast<off_t>(next);
        if (!dm_seek(dest) || !create_new_block()) return false;
    }

    // Write the item data and update the block stats.
    const auto nbytes = static_cast<std::uint64_t>(size);
    if (!dm_write(&nbytes, sizeof nbytes)) return false;
    if (size > 0 && !dm_write(data, size)) return false;
    block_used_ += item_bytes;
    ++block_items_;
    if (block_used_ == capacity && !seal_block()) return false;
    
    return true;
}

DataMonitorFileWriter::~DataMonitorFileWriter() {
    abort();
}

bool DataMonitorFileWriter::set_closed_safely() noexcept {
    if (file_ == nullptr || closed_) return false;
    closed_ = true;

    // A clean close seals the open variable block before closed_safely is set.
    if (!seal_block()) return false;

    // Flush the file and seek to the closed_safely field.
    if (!dm_flush() || !dm_seek(static_cast<off_t>(offsetof(FthrFileHeader, closed_safely)))) return false;

    // Set the closed_safely flag and flush the file.
    uint8_t closed_safely = FTHR_FILE_CLOSED_SAFELY_CLEAN;
    if (!dm_write(&closed_safely, sizeof(closed_safely)) || !dm_flush()) return false;

    return true;
}

bool DataMonitorFileWriter::dm_seek(off_t pos) noexcept {
    if (::fseeko(file_, pos, SEEK_SET) != 0) {
        ERROR("data monitor [%s] failed seek to %lld: %s", config_.data_name.c_str(), static_cast<long long>(pos), 
              std::strerror(errno));
        file_error_ = true;
        return false;
    }
    file_pos_ = static_cast<std::uint64_t>(pos);
    return true;
}

bool DataMonitorFileWriter::dm_flush() noexcept {
    bool started = false;
    feather_time_t retry_start = 0;
    while (true) {
        if (::fflush(file_) != 0) {
            if (errno == EINTR && detail::eintr_retry(config_.file_timeout_ns, started, retry_start)) {
                std::clearerr(file_);
                continue;
            }
            if (errno == EINTR) {
                ERROR("data monitor [%s] failed sync, timeout: %s", config_.data_name.c_str(), std::strerror(errno));
            } else {
                ERROR("data monitor [%s] failed flush file: %s", config_.data_name.c_str(), std::strerror(errno));
            }
            file_error_ = true;
            return false;
        }

        if (::fsync(::fileno(file_)) == 0) return true;
        if (errno == EINTR && detail::eintr_retry(config_.file_timeout_ns, started, retry_start)) continue;
        if (errno == EINTR) {
            ERROR("data monitor [%s] failed sync file, timeout: %s", config_.data_name.c_str(), std::strerror(errno));
        } else {
            ERROR("data monitor [%s] failed sync file: %s", config_.data_name.c_str(), std::strerror(errno));
        }
        file_error_ = true;
        return false;
    }
}

bool DataMonitorFileWriter::dm_write(const void* data, std::size_t size) noexcept {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::size_t written = 0;
    bool started = false;
    feather_time_t retry_start = 0;

    while (written < size) {
        const std::size_t n = ::fwrite(bytes + written, 1, size - written, file_);
        written += n;
        file_pos_ += n;
        if (written == size) return true;
        if (errno == EINTR && detail::eintr_retry(config_.file_timeout_ns, started, retry_start)) {
            std::clearerr(file_);
            continue;
        }
        if (errno == EINTR) {
            ERROR("data monitor [%s] failed to write %zu bytes, timeout", config_.data_name.c_str(), size);
        } else {
            ERROR("data monitor [%s] failed to write %zu bytes: %s", config_.data_name.c_str(), size, 
                  std::strerror(errno));
        }
        file_error_ = true;
        return false;
    }
    return true;
}

void DataMonitorFileWriter::abort() noexcept {
    if (file_ == nullptr) return;
    if (!file_error_) set_closed_safely();
    std::fclose(file_);
    file_ = nullptr;
}

DataMonitorService::DataMonitorService(const DataMonitorServiceConfig& config) {
    if (config.candump_config) candump_ = std::make_unique<DataMonitorFileWriter>(*config.candump_config);
    if (config.telemetry_config) telemetry_ = std::make_unique<DataMonitorFileWriter>(*config.telemetry_config);
}

bool DataMonitorService::write_to(
    DataMonitorFileWriter* writer, const char* what, const void* data, std::size_t size
) noexcept {
    if (writer == nullptr || size == 0) return true;
    ENSURE(data != nullptr, false, "data monitor %s write has a null payload of %zu bytes", what, size);
    return writer->write(data, size);
}

bool DataMonitorService::write_candump(const void* data, std::size_t size) noexcept {
    return write_to(candump_.get(), "candump", data, size);
}

bool DataMonitorService::write_telemetry(const void* data, std::size_t size) noexcept {
    return write_to(telemetry_.get(), "telemetry", data, size);
}

void DataMonitorService::abort() noexcept {
    if (candump_) candump_->abort();
    if (telemetry_) telemetry_->abort();
}

}  // namespace feather::robot
