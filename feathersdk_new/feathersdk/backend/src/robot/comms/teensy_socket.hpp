#pragma once

#include "src/utils/system.hpp"

#include <atomic>
#include <cstddef>
#include <string>

namespace feather::comms {

// 127.0.0.1 when emulated, otherwise the IPv4 of TEENSY_HOST.
// Throws if the name does not resolve, or the neighbor MAC is a different device.
std::string find_teensy_host(bool use_emulated_teensy);

// Nonblocking TCP connection to the Teensy command port, with Nagle disabled.
class TeensySocket {
public:
    TeensySocket(bool use_emulated_teensy, feather_time_t timeout);

    // -1 once kill() has shut the connection down. The fd stays open so another thread can still be in poll or recv.
    int fd() const noexcept { return dead_ ? -1 : sock_.fd; }

    // Write every byte. Returns false if they cannot be sent. A partial write shuts the connection down.
    bool write_all(const void* data, std::size_t size) noexcept;

    // Wake poll/recv and reject later sends. Does not close the fd.
    void kill() noexcept;

private:
    feather::system::Socket sock_;
    std::atomic<bool> dead_{false};
};

}  // namespace feather::comms
