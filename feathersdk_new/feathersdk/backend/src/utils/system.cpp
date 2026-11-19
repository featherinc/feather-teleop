#include "system.hpp"

#include <chrono>

#include <sys/syscall.h>
#include <unistd.h>

namespace feather::time {

feather_time_t now() noexcept {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
    return static_cast<feather_time_t>(ns.count());
}

feather_time_t elapsed(feather_time_t start) noexcept {
    return now() - start;
}

feather_time_t ms2time(std::uint64_t ms) noexcept {
    return ms * 1000000;
}

feather_time_t from_seconds(double seconds) noexcept {
    if (seconds <= 0) return 0;
    return static_cast<feather_time_t>(seconds * 1000000000.0);
}

feather_time_t from_millis(std::uint64_t ms) noexcept {
    return ms * 1000000;
}

std::uint64_t to_ms(feather_time_t time) noexcept {
    return time / 1000000;
}

}  // namespace feather::time

namespace feather::system {

int thread_id() noexcept {
    static thread_local const int tid = static_cast<int>(syscall(SYS_gettid));
    return tid;
}

int process_id() noexcept {
    static const int pid = static_cast<int>(getpid());
    return pid;
}

}  // namespace feather::system
