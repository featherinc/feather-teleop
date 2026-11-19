#include "comms_manager.hpp"

#include "teensy/host/recompile.hpp"

namespace feather::comms {

CommsManager::CommsManager(
    const CommsManagerConfig& cfg,
    feather::robot::Telemetry* telemetry,
    feather::robot::DataMonitorService* data_monitor
) : config(cfg) {
    config.teensy.validate();

    const feather::teensy::RecompileResult built =
        feather::teensy::recompile_if_needed(config.teensy.use_emulated_teensy);
    teensy_bin_path = built.binary;
    teensy_source_hash = std::to_string(built.hash);
    teensy_.emplace(config.teensy, telemetry, data_monitor);
}

SentTeensyMessage CommsManager::send_teensy_message(TeensyCommandOpcode opcode, std::string_view payload) noexcept {
    return teensy_->send_message(opcode, payload);
}

std::optional<TeensyResponse> CommsManager::get_response(
    const SentTeensyMessage& msg, feather_time_t timeout, PayloadOut out
) noexcept {
    return teensy_->get_response(msg, timeout, out);
}

std::optional<TeensyResponse> CommsManager::send_await(
    TeensyCommandOpcode opcode, std::string_view payload, feather_time_t timeout, PayloadOut out
) noexcept {
    return teensy_->send_await(opcode, payload, timeout, out);
}

SentTeensyMessage CommsManager::cansend(int bus, bool extended, int can_id, std::string_view data) noexcept {
    return teensy_->cansend(bus, extended, can_id, data);
}

void CommsManager::abort() {
    teensy_->abort();
}

}  // namespace feather::comms
