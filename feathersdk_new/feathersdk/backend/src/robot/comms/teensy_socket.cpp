#include "teensy_socket.hpp"

#include "logging.hpp"
#include "qdefs/teensy_commands_qdef.hpp"
#include "run.hpp"

#include <cctype>
#include <cerrno>
#include <stdexcept>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace feather::comms {
namespace {

// Returns the MAC address of the neighbor with the given IP address. Empty means no MAC was found.
std::string neighbor_mac(const std::string& ip) {
    try {
        std::string ignored;
        std::string text;
        run({"ping", "-c", "1", "-W", "1", ip}, 2, nullptr, &ignored);  // Ping to populate the neighbor table.
        run({"ip", "neigh", "show", ip}, 2, nullptr, &text);

        for (char& ch : text) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        const std::size_t at = text.find("lladdr ");
        if (at == std::string::npos || at + 24 > text.size()) return {};
        return text.substr(at + 7, 17);
    } catch (const std::exception&) {
        return {};
    }
}

// Nonblocking connect so timeout applies. Returns the fd. Throws on failure.
int connect_tcp(const std::string& ip, feather_time_t timeout) {
    // Build the IP address the socket will connect to.
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(TEENSY_PORT));
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        throw std::runtime_error("invalid teensy address: " + ip);
    }

    // Open a socket to the Teensy. `fail` can be called to clean up the socket and throw an exception.
    const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) throw std::runtime_error("failed to open teensy socket: errno " + std::to_string(errno));
    const auto fail = [&](const std::string& msg) {
        const int err = errno;
        close(fd);
        throw std::runtime_error(msg + ": errno " + std::to_string(err));
    };

    // Set the socket to nonblocking.
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) fail("failed to set teensy socket nonblocking");

    // Attempt to connect to the Teensy.
    const std::string where = ip + ":" + std::to_string(TEENSY_PORT);
    const int rc = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr);
    if (rc != 0 && errno != EINPROGRESS) fail("failed to connect to " + where);

    // Connection is in progress. Poll until it finishes or the deadline passes.
    // Sample the remaining time once per wait: a second elapsed() call can pass the deadline and wrap.
    if (rc != 0) {
        const feather_time_t start = feather::time::now();
        bool ready = false;
        for (;;) {
            const feather_time_t elapsed = feather::time::elapsed(start);
            if (elapsed >= timeout) break;
            const auto left_ms = feather::time::to_ms(timeout - elapsed);

            // poll() takes an int millisecond timeout. Cap one wait, then recompute what remains.
            const int wait_ms = left_ms > 60000 ? 60000 : static_cast<int>(left_ms < 1 ? 1 : left_ms);
            pollfd pfd{.fd = fd, .events = POLLOUT, .revents = 0};
            const int pr = poll(&pfd, 1, wait_ms);

            if (pr > 0) {
                ready = true;
                break;
            }
            if (pr == 0) continue;
            if (errno != EINTR) fail("failed to connect to " + where);
        }

        if (!ready) {
            errno = ETIMEDOUT;
            fail("failed to connect to " + where);
        }

        // Check if the connection failed because of a socket error.
        int err = 0;
        socklen_t len = sizeof err;
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
            if (err != 0) errno = err;
            fail("failed to connect to " + where);
        }
    }

    // Disable Nagle's algorithm to reduce latency.
    const int yes = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes) != 0) fail("failed to disable Nagle");
    return fd;
}

}  // namespace

bool TeensySocket::write_all(const void* data, std::size_t size) noexcept {
    if (dead_ || sock_.fd < 0) return false;
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::size_t sent = 0;

    // A full TCP window must not sit in send() forever. The socket is nonblocking.
    const feather_time_t deadline = feather::time::now() + feather::time::from_millis(1000);

    while (sent < size) {
        // Try to send the next chunk of data.
        const ssize_t n = ::send(sock_.fd, bytes + sent, size - sent, MSG_NOSIGNAL);
        if (n > 0) {
            sent += static_cast<std::size_t>(n);
            continue;
        }
        
        if (n < 0 && errno == EINTR) continue;  // If the send was interrupted, try again.

        // If the send would block, wait for it to be ready. Poll until the deadline passes.
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            const feather_time_t now = feather::time::now();
            if (now >= deadline) break;

            const auto left_ms = feather::time::to_ms(deadline - now);
            pollfd pfd{.fd = sock_.fd, .events = POLLOUT, .revents = 0};
            const int pr = ::poll(&pfd, 1, left_ms < 1 ? 1 : static_cast<int>(left_ms));

            if (pr < 0 && errno == EINTR) continue;  // If the poll was interrupted, try again.
            if (pr < 0) break;
            continue;
        }

        break;
    }

    if (sent == size) return true;

    // A torn frame would be parsed as the next command. Shut the connection down unless nothing was written.
    // shutdown leaves the fd open so poll/recv on another thread does not race with close.
    if (sent > 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) kill();
    return false;
}

void TeensySocket::kill() noexcept {
    if (dead_.exchange(true)) return;
    if (sock_.fd >= 0) ::shutdown(sock_.fd, SHUT_RDWR);
}

std::string find_teensy_host(bool use_emulated_teensy) {
    if (use_emulated_teensy) return "127.0.0.1";

    // Ask the operating system to resolve the Teensy's IP address. Specifically look for an IPv4 TCP address.
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if (getaddrinfo(TEENSY_HOST, nullptr, &hints, &res) != 0 || res == nullptr) {
        throw std::runtime_error(std::string(TEENSY_HOST) + " did not resolve.");
    }

    // Attempt to convert the resolved address to a string. Free the resolved address list.
    char ip[INET_ADDRSTRLEN] = {};
    bool ok = inet_ntop(AF_INET, &reinterpret_cast<sockaddr_in*>(res->ai_addr)->sin_addr, ip, sizeof ip) != nullptr;
    freeaddrinfo(res);
    if (!ok) throw std::runtime_error(std::string(TEENSY_HOST) + " did not resolve.");

    // Make sure MAC address matches the Teensy's MAC address. mDNS can answer for a different machine.
    const std::string mac = neighbor_mac(ip);
    if (!mac.empty() && mac != TEENSY_MAC) {
        throw std::runtime_error(std::string(ip) + " has MAC " + mac + ", expected " + TEENSY_MAC);
    }
    return ip;
}

TeensySocket::TeensySocket(bool use_emulated_teensy, feather_time_t timeout) : sock_(-1) {
    const std::string host = find_teensy_host(use_emulated_teensy);
    NOTICE("Connecting to Teensy at %s:%d", host.c_str(), TEENSY_PORT);
    sock_.fd = connect_tcp(host, timeout);
}

}  // namespace feather::comms
