#pragma once

#include "teensy_manager.hpp"

#include <optional>
#include <string>

namespace feather::comms {

struct CommsManagerConfig {
    TeensyConfig teensy;
};

class CommsManager {
public:
    // `telemetry` receives actuator telemetry replies. Null leaves those replies unapplied.
    // `data_monitor` records candump and telemetry replies. Null leaves them unrecorded.
    explicit CommsManager(
        const CommsManagerConfig& cfg,
        feather::robot::Telemetry* telemetry = nullptr,
        feather::robot::DataMonitorService* data_monitor = nullptr
    );

    // Forwards to TeensyManager::send_message.
    SentTeensyMessage send_teensy_message(TeensyCommandOpcode opcode, std::string_view payload) noexcept;

    // Forwards to TeensyManager::get_response.
    std::optional<TeensyResponse> get_response(
        const SentTeensyMessage& msg, feather_time_t timeout = 0, PayloadOut out = {}
    ) noexcept;

    // Forwards to TeensyManager::send_await.
    std::optional<TeensyResponse> send_await(
        TeensyCommandOpcode opcode, std::string_view payload, feather_time_t timeout = 0, PayloadOut out = {}
    ) noexcept;

    // Forwards to TeensyManager::cansend.
    SentTeensyMessage cansend(int bus, bool extended, int can_id, std::string_view data) noexcept;

    // Forwards to TeensyManager::abort.
    void abort();

    // CAN buses reported by the Teensy. -1 until that reply arrives.
    int num_can_lines() const { return teensy_->num_can_lines; }

    CommsManagerConfig config;
    std::string teensy_bin_path;
    std::string teensy_source_hash;

private:
    // Built after recompile_if_needed. TeensyManager connects in its constructor.
    std::optional<TeensyManager> teensy_;
};

}  // namespace feather::comms
