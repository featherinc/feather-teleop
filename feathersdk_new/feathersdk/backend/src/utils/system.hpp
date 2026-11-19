#pragma once

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "host is little-endian"
#endif

#include "logging.hpp"

#include <cstdint>
#include <unistd.h>
#include <array>

using Vec3 = std::array<double, 3>;

typedef std::uint64_t feather_time_t;

namespace feather::time {

// Monotonic time since the steady clock's epoch.
feather_time_t now() noexcept;

// Time elapsed since start. start is a value returned by now().
feather_time_t elapsed(feather_time_t start) noexcept;

// Convert milliseconds to feather_time_t.
feather_time_t ms2time(std::uint64_t ms) noexcept;

// Convert seconds to feather_time_t. Zero or less becomes zero.
feather_time_t from_seconds(double seconds) noexcept;

// Convert milliseconds to feather_time_t.
feather_time_t from_millis(std::uint64_t ms) noexcept;

// Convert feather_time_t to whole milliseconds.
std::uint64_t to_ms(feather_time_t time) noexcept;

}  // namespace feather::time

namespace feather::system {

// An open file descriptor that is closed when destroyed.
struct Socket {
    int fd = -1;
    explicit Socket(int fd_in) : fd(fd_in) {}
    ~Socket() { if (fd >= 0) close(fd); }
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;
};

// Current thread's ID.
int thread_id() noexcept;

// Current process's ID.
int process_id() noexcept;

}  // namespace feather::system


#define __PARENT_DIR__ \
    /* The parent directory holding this file */ \
    std::filesystem::path(__FILE__).parent_path()

#define ENSURE(cond, retval, msg, ...) \
    /* Assert the condition is true, otherwise return the given value and log the given message as error. */ \
    if (!(cond)) { \
        ERROR(msg, ##__VA_ARGS__); \
        return retval; \
    }
