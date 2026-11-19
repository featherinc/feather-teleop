// Start the emulated Teensy, check a 16-byte echo, then send one CAN frame on bus 0.
// Build and run from backend/: make teensy_echo

#include "src/robot/comms/teensy_manager.hpp"
#include "logging.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <optional>

namespace {

int check_response(
    const char* what, const std::optional<feather::comms::TeensyResponse>& resp,
    const std::uint8_t* got, bool poll_err, TeensyCommandOpcode expect
) {
    if (!resp) {
        std::fprintf(stderr, "no %s response (poll_err=%d)\n", what, poll_err ? 1 : 0);
        return 1;
    }
    if (resp->opcode == TEENSY_COMMAND_OPCODE_RESP_ERROR) {
        std::fprintf(stderr, "%s error: %.*s\n", what, static_cast<int>(resp->payload_size),
            reinterpret_cast<const char*>(got));
        return 1;
    }
    if (resp->opcode != expect) {
        std::fprintf(stderr, "%s: opcode %u, %u bytes\n", what, static_cast<unsigned>(resp->opcode), resp->payload_size);
        return 1;
    }
    return 0;
}

}  // namespace

int main() {
    feather::log::Config log_cfg;
    log_cfg.console_enabled = true;
    log_cfg.file_enabled = false;
    feather::log::init(log_cfg);

    try {
        feather::comms::TeensyConfig cfg;
        cfg.use_emulated_teensy = true;
        feather::comms::TeensyManager mgr(cfg);
        std::printf("src hash %llu, %d CAN lines\n",
            static_cast<unsigned long long>(mgr.src_hash()), mgr.num_can_lines);
        std::array<std::uint8_t, TEENSY_MAX_RESPONSE_PAYLOAD_SIZE> got{};

        std::array<std::uint8_t, ECHO_PAYLOAD_SIZE> sent{};
        for (std::uint8_t i = 0; i < ECHO_PAYLOAD_SIZE; ++i) sent[i] = static_cast<std::uint8_t>(0xA0u + i);
        const auto echo = mgr.send_await(
            TEENSY_COMMAND_OPCODE_CMD_ECHO,
            {reinterpret_cast<const char*>(sent.data()), sent.size()},
            feather::time::from_millis(500),
            feather::comms::payload_as(got));
        if (check_response("echo", echo, got.data(), mgr.poll_err(), TEENSY_COMMAND_OPCODE_RESP_ECHO) != 0) return 1;
        if (echo->payload_size != ECHO_PAYLOAD_SIZE || std::memcmp(sent.data(), got.data(), ECHO_PAYLOAD_SIZE) != 0) {
            std::fprintf(stderr, "echo mismatch: %u bytes\n", echo->payload_size);
            return 1;
        }
        std::printf("echo ok\n");

        const std::uint8_t can_data[] = {0x11, 0x22};
        const auto can = mgr.get_response(
            mgr.cansend(0, false, 0x123, {reinterpret_cast<const char*>(can_data), sizeof can_data}),
            feather::time::from_millis(500),
            feather::comms::payload_as(got));
        if (check_response("cansend", can, got.data(), mgr.poll_err(), TEENSY_COMMAND_OPCODE_RESP_SUCCESS) != 0) {
            return 1;
        }
        std::printf("cansend ok\n");
        return 0;
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        return 1;
    }
}
