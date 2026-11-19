#include "flash.hpp"

#include "src/robot/comms/teensy_socket.hpp"
#include "src/utils/logging.hpp"
#include "src/utils/run.hpp"
#include "src/utils/system.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace feather::teensy {
namespace {

constexpr double kBounceDelay = 11;
constexpr double kProbe = 0.25;
constexpr double kResolve = 2;

// A blocking connect ignores timeout_s, so the socket stays nonblocking until connect finishes.
int tcp_connect(const std::string& ip, int port, double timeout_s) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) return -1;

    const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    const int flags = fcntl(fd, F_GETFL, 0);

    bool ok = false;
    if (flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0) {
        const int rc = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr);
        if (rc == 0) ok = true;
        else if (errno == EINPROGRESS) {
            pollfd pfd{fd, POLLOUT, 0};
            int err = 0;
            socklen_t len = sizeof err;
            ok = poll(&pfd, 1, static_cast<int>(timeout_s * 1000)) == 1 &&
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0;
        }
    }
    if (!ok) {
        close(fd);
        return -1;
    }

    fcntl(fd, F_SETFL, flags);
    const int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes);
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_s);
    tv.tv_usec = static_cast<suseconds_t>((timeout_s - static_cast<double>(tv.tv_sec)) * 1e6);
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    return fd;
}

void send_all(int fd, const void* data, size_t n) {
    const auto* p = static_cast<const std::uint8_t*>(data);
    size_t sent = 0;
    while (sent < n) {
        const ssize_t rc = send(fd, p + sent, n - sent, MSG_NOSIGNAL);
        if (rc > 0) sent += static_cast<size_t>(rc);
        else if (rc < 0 && errno == EINTR) continue;
        else throw std::runtime_error("Teensy connection failed while sending");
    }
}

void recv_exact(int fd, void* data, size_t n) {
    auto* p = static_cast<std::uint8_t*>(data);
    size_t got = 0;
    while (got < n) {
        const ssize_t rc = recv(fd, p + got, n - got, 0);
        if (rc > 0) got += static_cast<size_t>(rc);
        else if (rc < 0 && errno == EINTR) continue;
        else if (rc == 0) throw std::runtime_error("Teensy closed the connection while receiving");
        else throw std::runtime_error("Teensy connection failed while receiving");
    }
}

// Header is message id, opcode, payload length.
void expect_success(int fd, std::uint32_t msg_id, const char* what) {
    ClientCommandHeader hdr{};
    recv_exact(fd, &hdr, sizeof hdr);
    if (hdr.payload_size > 4096) throw std::runtime_error("implausible response payload size");
    std::string payload(hdr.payload_size, '\0');
    if (hdr.payload_size != 0) recv_exact(fd, payload.data(), hdr.payload_size);
    if (hdr.opcode == TEENSY_COMMAND_OPCODE_RESP_ERROR) throw std::runtime_error(std::string(what) + ": " + payload);
    if (hdr.msg_id != msg_id || hdr.opcode != TEENSY_COMMAND_OPCODE_RESP_SUCCESS) {
        throw std::runtime_error(std::string(what) + " failed");
    }
}

// The Jetson NIC does not renegotiate when the Teensy PHY resets. Skip loopback.
void bounce_link(const std::string& host) {
    std::string route;
    if (run({"ip", "route", "get", host}, 2, nullptr, &route) != 0) return;
    const auto dev = route.find(" dev ");
    if (dev == std::string::npos) return;
    const auto begin = dev + 5;
    const auto end = route.find_first_of(" \n", begin);
    const std::string iface = end == std::string::npos ? route.substr(begin) : route.substr(begin, end - begin);
    if (iface.empty() || iface == "lo") return;

    NOTICE("Bouncing %s", iface.c_str());
    for (const char* state : {"down", "up"}) {
        if (run({"sudo", "ip", "link", "set", iface, state}, 10) != 0) {
            throw std::runtime_error("failed to set " + iface + " " + state);
        }
    }
}

// Returns once a TCP connection succeeds. Bounces the link if the board stays down.
bool wait_until_up(int port, double reboot_wait) {
    const feather_time_t start = feather::time::now();
    const feather_time_t deadline = feather::time::from_seconds(reboot_wait);
    feather_time_t next_resolve = 0;
    feather_time_t next_bounce = feather::time::from_seconds(kBounceDelay);
    std::optional<std::string> host;

    while (feather::time::elapsed(start) < deadline) {
        const feather_time_t elapsed = feather::time::elapsed(start);
        if (elapsed >= next_resolve) {
            host.reset();
            try { host = feather::comms::find_teensy_host(false); } catch (const std::exception&) {}
            next_resolve = elapsed + feather::time::from_seconds(kResolve);
        }
        if (host && feather::system::Socket(tcp_connect(*host, port, kProbe)).fd >= 0) return true;
        if (host && elapsed >= next_bounce) {
            bounce_link(*host);
            next_bounce = elapsed + feather::time::from_seconds(kBounceDelay);
        } else {
            std::this_thread::sleep_for(std::chrono::duration<double>(kProbe));
        }
    }
    return false;
}

}  // namespace

void flash_ota(const std::string& hex_path, const FlashConfig& cfg) {
    std::ifstream in(hex_path, std::ios::binary);
    if (!in) throw std::runtime_error("hex file not found: " + hex_path);
    std::string data{std::istreambuf_iterator<char>(in), {}};
    const auto start = data.find_first_not_of(" \t\r\n\v\f");
    if (start == std::string::npos || data[start] != ':') throw std::runtime_error("not an Intel HEX file: " + hex_path);
    const size_t file_bytes = data.size();
    if (data.back() != '\n' && data.back() != '\r') data.push_back('\n');

    const std::string host = feather::comms::find_teensy_host(false);
    NOTICE("Connecting to %s:%d", host.c_str(), cfg.port);
    feather::system::Socket sock(tcp_connect(host, cfg.port, cfg.timeout_s));
    if (sock.fd < 0) throw std::runtime_error("failed to connect to " + host + ":" + std::to_string(cfg.port));

    const ClientCommandHeader hdr{
        .msg_id = 1,
        .opcode = TEENSY_COMMAND_OPCODE_CMD_FLASH_TEENSY,
        .payload_size = 0,
    };
    send_all(sock.fd, &hdr, sizeof hdr);
    NOTICE("Waiting for Teensy to prepare flash staging area");
    expect_success(sock.fd, hdr.msg_id, "prepare failed");

    NOTICE("Sending %s (%zu bytes)", fs::path(hex_path).filename().c_str(), file_bytes);
    send_all(sock.fd, data.data(), data.size());
    NOTICE("Waiting for flash confirmation");
    expect_success(sock.fd, hdr.msg_id, "flash failed");

    NOTICE("OTA flash complete; waiting up to %.1fs for Teensy to reboot", cfg.reboot_wait_s);
    if (!wait_until_up(cfg.port, cfg.reboot_wait_s)) {
        // Drop the elf so the next launch does not treat this failed flash as a finished build.
        fs::path elf = hex_path;
        elf.replace_extension(".elf");
        std::error_code ec;
        if (fs::is_regular_file(elf, ec) && fs::remove(elf, ec)) {
            ERROR("Removed %s after reboot wait failure", elf.c_str());
        }
        throw std::runtime_error(
            "Teensy did not accept connections at " + host + ":" + std::to_string(cfg.port));
    }
    NOTICE("Teensy is back up");
}

}  // namespace feather::teensy
