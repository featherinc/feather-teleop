// C ABI over TeensyManager and Telemetry, for the Python SDK (feathersdk/robot/comms/cpp_link.py, through ctypes).
//
// ctypes releases the GIL for every call here, and TeensyManager's poll thread never touches Python, so the Teensy's
// replies are received, decoded and written to the telemetry rings whatever the Python process is doing. Every call
// catches C++ exceptions: a function that can fail returns false or null and writes the reason into `err`.

#include "src/robot/comms/teensy_manager.hpp"
#include "src/robot/telemetry.hpp"

#include <cstdio>
#include <cstring>
#include <exception>
#include <new>

using feather::comms::PayloadOut;
using feather::comms::SentTeensyMessage;
using feather::comms::TeensyConfig;
using feather::comms::TeensyManager;
using feather::robot::Telemetry;

namespace {

void set_err(char* err, std::size_t cap, const char* what) {
    if (err != nullptr && cap > 0) std::snprintf(err, cap, "%s", what);
}

}  // namespace

extern "C" {

// The fields of TeensyConfig the Python SDK sets (CommsManagerConfig), in this order. Layout read by ctypes.
struct FbTeensyConfig {
    double teensy_connect_timeout;
    double teensy_telemetry_update_frequency;
    double teensy_candump_update_frequency;
    double teensy_board_clock_sync_hz;
    double max_can_rate;
    double can_leniency;
    std::int32_t enable_teensy_debug_logging;
    std::int32_t teensy_correct_clock_drift;
    std::int32_t teensy_expected_clock_drift_ns;
    std::int32_t teensy_max_clock_sync_error_ns;
    std::int32_t teensy_clock_sync_trials;
    std::int32_t use_emulated_teensy;
    std::int32_t server_run_time_nanos;
    std::int32_t can_read_run_time_nanos;
};

Telemetry* fb_telemetry_new() noexcept { return new (std::nothrow) Telemetry(); }

int fb_telemetry_add(Telemetry* t, std::uint64_t capacity, int dtype) noexcept {
    return t->add_telemetry(capacity, static_cast<DataType>(dtype));
}

bool fb_telemetry_initialize(Telemetry* t) noexcept { return t->initialize(); }

bool fb_telemetry_buffers(Telemetry* t, std::uint8_t** data, std::size_t* data_size, std::uint64_t** starts,
                          std::uint64_t** ends, std::uint64_t** counts, std::size_t* n_channels) noexcept {
    const Telemetry::Buffers b = t->buffers();
    if (b.data == nullptr) return false;
    *data = b.data;
    *data_size = b.data_size;
    *starts = b.starts;
    *ends = b.ends;
    *counts = b.counts;
    *n_channels = b.n_channels;
    return true;
}

// Connects, initializes the Teensy, reads its source hash and CAN lines, and syncs clocks. The caller has rebuilt the
// firmware and started any emulator (manage_firmware is off) and compares fb_teensy_src_hash_text itself.
TeensyManager* fb_teensy_new(const FbTeensyConfig* c, Telemetry* telemetry, char* err, std::size_t err_cap) noexcept {
    try {
        TeensyConfig cfg;
        cfg.teensy_connect_timeout = c->teensy_connect_timeout;
        cfg.teensy_telemetry_update_frequency = c->teensy_telemetry_update_frequency;
        cfg.teensy_candump_update_frequency = c->teensy_candump_update_frequency;
        cfg.teensy_board_clock_sync_hz = c->teensy_board_clock_sync_hz;
        cfg.max_can_rate = c->max_can_rate;
        cfg.can_leniency = c->can_leniency;
        cfg.enable_teensy_debug_logging = c->enable_teensy_debug_logging != 0;
        cfg.teensy_correct_clock_drift = c->teensy_correct_clock_drift != 0;
        cfg.teensy_expected_clock_drift_ns = c->teensy_expected_clock_drift_ns;
        cfg.teensy_max_clock_sync_error_ns = c->teensy_max_clock_sync_error_ns;
        cfg.teensy_clock_sync_trials = c->teensy_clock_sync_trials;
        cfg.use_emulated_teensy = c->use_emulated_teensy != 0;
        cfg.server_run_time_nanos = c->server_run_time_nanos;
        cfg.can_read_run_time_nanos = c->can_read_run_time_nanos;
        cfg.manage_firmware = false;
        return new TeensyManager(cfg, telemetry, nullptr);
    } catch (const std::exception& e) {
        set_err(err, err_cap, e.what());
    } catch (...) {
        set_err(err, err_cap, "unknown C++ exception");
    }
    return nullptr;
}

void fb_teensy_close(TeensyManager* m) noexcept { m->shut_down(); }

bool fb_teensy_poll_err(TeensyManager* m) noexcept { return m->poll_err(); }

int fb_teensy_num_can_lines(TeensyManager* m) noexcept { return m->num_can_lines; }

std::int64_t fb_teensy_clock_sync_bound_ns(TeensyManager* m) noexcept { return m->clock_sync_bound_ns(); }

// Copies the hash text (no terminator). Returns its length, or -1 if it does not fit.
long fb_teensy_src_hash_text(TeensyManager* m, char* out, std::size_t cap) noexcept {
    const std::string text = m->src_hash_text();
    if (text.size() > cap) return -1;
    std::memcpy(out, text.data(), text.size());
    return static_cast<long>(text.size());
}

// Returns the message id, 0 if the send failed (socket closed, payload too large, every reply slot in use).
std::uint32_t fb_teensy_send(TeensyManager* m, int opcode, const char* payload, std::size_t size,
                             bool want_response) noexcept {
    return m->send_message(static_cast<TeensyCommandOpcode>(opcode), {payload, size}, want_response).msg_id;
}

// Waits up to timeout_ns for the reply to msg_id and takes it. Returns 1 with the reply's opcode, payload (up to cap
// bytes; *size is its length) and receive time, 0 if it has not arrived (or was culled).
int fb_teensy_get_response(TeensyManager* m, std::uint32_t msg_id, std::int64_t timeout_ns, int* opcode,
                           std::uint8_t* payload, std::uint32_t cap, std::uint32_t* size,
                           std::int64_t* received_ns) noexcept {
    const SentTeensyMessage msg{.msg_id = msg_id, .mailbox_idx = msg_id & (TEENSY_MAX_RESPONSE_BUFFER_SIZE - 1)};
    const auto resp = m->get_response(msg, timeout_ns > 0 ? static_cast<feather_time_t>(timeout_ns) : 0,
                                      PayloadOut{.data = payload, .size = cap});
    if (!resp) return 0;
    *opcode = static_cast<int>(resp->opcode);
    *size = resp->payload_size;
    *received_ns = static_cast<std::int64_t>(resp->received_ts);
    return 1;
}

void fb_teensy_set_recording(TeensyManager* m, bool telemetry, bool candump, bool messages) noexcept {
    m->records.telemetry = telemetry;
    m->records.candump = candump;
    m->records.messages = messages;
}

std::size_t fb_teensy_drain_records(TeensyManager* m, std::uint8_t* out, std::size_t cap,
                                    std::uint64_t* dropped) noexcept {
    return m->records.drain(out, cap, dropped);
}

}  // extern "C"
