#ifndef _GNU_SOURCE  // Needed for ::ppoll
#define _GNU_SOURCE
#endif

#include "teensy_manager.hpp"
#include "qdefs/telemetry_qdef.hpp"
#include "src/robot/telemetry.hpp"
#include "src/robot/services/data_monitor.hpp"
#include "src/utils/system.hpp"
#include "src/utils/logging.hpp"
#include "src/robot/comms/teensy/host/recompile.hpp"

#include <charconv>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

// Maximum time to wait for a response to be received and processed.
#define MAX_RESPONSE_TIMEOUT feather::time::from_millis(500)

// ethernet.c rejects a command larger than this and drops the connection.
constexpr std::size_t MAX_COMMAND_SIZE = 512;

// Width of the slot index packed into msg_id. The rest of the uint32 is the generation.
constexpr std::uint32_t slot_bit_count(std::uint32_t n) {
    std::uint32_t bits = 0;
    while (n > 1) {
        n >>= 1;
        ++bits;
    }
    return bits;
}
constexpr std::uint32_t SLOT_BITS = slot_bit_count(TEENSY_MAX_RESPONSE_BUFFER_SIZE);
constexpr std::uint32_t GEN_LIMIT = 1u << (32 - SLOT_BITS);

namespace feather::comms {

static_assert((1u << SLOT_BITS) == TEENSY_MAX_RESPONSE_BUFFER_SIZE, "slot bit count must cover every slot");

void TeensyConfig::validate() const {
    if (can_leniency > max_can_rate) {
        throw std::invalid_argument(
            "can_leniency (" + std::to_string(can_leniency) + ") must be <= max_can_rate (" +
            std::to_string(max_can_rate) + ")");
    }
}

TeensyManager::EmulatedProcess::EmulatedProcess(const TeensyConfig& cfg) {
    if (!cfg.use_emulated_teensy || !cfg.manage_firmware) return;

    // Recompile the Teensy firmware if needed, then start the program in another process.
    const std::string binary = feather::teensy::recompile_if_needed(true).binary;
    pid = fork();
    if (pid < 0) throw std::runtime_error("failed to start emulated Teensy");
    if (pid == 0) {
        const int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            if (devnull > STDERR_FILENO) close(devnull);
        }
        execl(binary.c_str(), binary.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    NOTICE("Started emulated Teensy pid %d", pid);

    // Wait and make sure the process starts
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    if (waitpid(pid, nullptr, WNOHANG) == pid) {
        pid = -1;
        throw std::runtime_error("emulated Teensy exited during startup");
    }
}

void TeensyManager::EmulatedProcess::stop() noexcept {
    if (pid <= 0) return;
    const int child = pid;
    pid = -1;
    kill(child, SIGKILL);
    waitpid(child, nullptr, 0);
}

TeensyManager::TeensyManager(
    const TeensyConfig& cfg, feather::robot::Telemetry* telemetry, feather::robot::DataMonitorService* data_monitor
)
    : config(cfg),
      emulator_(cfg),
      sock_(cfg.use_emulated_teensy, feather::time::from_seconds(cfg.teensy_connect_timeout)),
      telemetry_(telemetry),
      data_monitor_(data_monitor),
      response_buffer{} {
    
    config.validate();
    for (std::uint16_t i = 0; i < TEENSY_MAX_RESPONSE_BUFFER_SIZE; ++i) free_slots[i] = i;
    free_count = TEENSY_MAX_RESPONSE_BUFFER_SIZE;

    // The emulator binary was already rebuilt above. A real board is rebuilt here.
    if (!cfg.use_emulated_teensy && cfg.manage_firmware) {
        try {
            expected_src_hash_ = feather::teensy::recompile_if_needed(false).hash;
        } catch (const std::runtime_error& e) {
            ERROR("Failed to recompile Teensy firmware: %s", e.what());
            abort();
            throw;
        }
    }

    poll_running_ = true;
    poll_thread_ = std::thread([this] {
        poll();
        poll_running_ = false;
    });

    try {
        bring_up();
    } catch (...) {
        abort();
        throw;
    }
}

void TeensyManager::bring_up() {
    // Build the initialize command payload.
    InitializeConfig init{};
    init.comms_config.server_run_time_nanos = static_cast<std::uint32_t>(config.server_run_time_nanos);
    init.comms_config.can_read_run_time_nanos = static_cast<std::uint32_t>(config.can_read_run_time_nanos);
    init.comms_config.max_can_rate = static_cast<float>(config.max_can_rate);
    init.comms_config.can_leniency = static_cast<float>(config.can_leniency);

    // Send the initialize command and wait for the response.
    std::array<char, TEENSY_MAX_RESPONSE_PAYLOAD_SIZE> err{};
    const auto init_resp = send_await(
        TEENSY_COMMAND_OPCODE_CMD_INITIALIZE,
        {reinterpret_cast<const char*>(&init), sizeof init},
        MAX_RESPONSE_TIMEOUT,
        {reinterpret_cast<std::uint8_t*>(err.data()), static_cast<std::uint32_t>(err.size())});
    
    if (!init_resp || init_resp->opcode != TEENSY_COMMAND_OPCODE_RESP_SUCCESS) {
        if (init_resp && init_resp->opcode == TEENSY_COMMAND_OPCODE_RESP_ERROR) {
            const std::size_t n = init_resp->payload_size < err.size() ? init_resp->payload_size : err.size();
            throw std::runtime_error("Teensy initialize failed: " + std::string(err.data(), n));
        }
        throw std::runtime_error("Teensy initialize failed");
    }

    // Request the source hash and CAN line count.
    if (send_message(TEENSY_COMMAND_OPCODE_CMD_GET_SRC_HASH, {}).msg_id == 0 ||
        send_message(TEENSY_COMMAND_OPCODE_CMD_GET_NUM_CAN_LINES, {}).msg_id == 0) {
        throw std::runtime_error("Failed to request Teensy source hash or CAN line count");
    }

    // Wait for the source hash and CAN line count to arrive and be processed.
    const feather_time_t start = feather::time::now();
    bool ready = false;
    while (!ready && feather::time::elapsed(start) < MAX_RESPONSE_TIMEOUT) {
        {
            std::lock_guard<std::mutex> lock(resp_buf_mut);
            ready = src_hash_received_ && num_can_lines >= 0;
        }
        if (!ready) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!ready) throw std::runtime_error("Timed out waiting for Teensy source hash and CAN line count");
    NOTICE("Teensy ready: src hash %s, %d CAN lines", src_hash_text().c_str(), num_can_lines);

    // If the hashes do not match, flash the Teensy. Without manage_firmware the caller compares src_hash_text().
    if (config.manage_firmware && !config.use_emulated_teensy &&
        (teensy_src_hash_ == 0 || teensy_src_hash_ != expected_src_hash_)) {
        WARNING("Teensy's source hash (%llu) does not match expected (%llu); flashing is not implemented",
            static_cast<unsigned long long>(teensy_src_hash_), static_cast<unsigned long long>(expected_src_hash_));
        // TODO: implement flashing
    }

    sync_clocks();
}

void TeensyManager::sync_clocks() {
    // Teensy logging off during the trials, for fast replies.
    SetLoggingPayload logging{};
    logging.enabled = false;
    if (!send_await(TEENSY_COMMAND_OPCODE_CMD_SET_LOGGING, {reinterpret_cast<const char*>(&logging), sizeof logging},
                    MAX_RESPONSE_TIMEOUT)) {
        throw std::runtime_error("Teensy did not answer CMD_SET_LOGGING before the clock sync");
    }
    for (int i = 0; i < config.teensy_clock_sync_trials; ++i) {
        if (!send_await(TEENSY_COMMAND_OPCODE_CMD_GET_BOARD_TIME, {}, MAX_RESPONSE_TIMEOUT)) {
            throw std::runtime_error("Teensy did not answer CMD_GET_BOARD_TIME during the clock sync");
        }
    }
    // The poll thread applies each sample; wait for the last offset it pushed to be accepted.
    const feather_time_t start = feather::time::now();
    while (pending_offset_msg_ != 0 && feather::time::elapsed(start) < MAX_RESPONSE_TIMEOUT) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const std::int64_t bound = clock_bound_ns_;
    if (pending_offset_msg_ != 0 || bound > config.teensy_max_clock_sync_error_ns) {
        throw std::runtime_error("Failed to sync clocks: bound " + std::to_string(bound) + " ns > " +
                                 std::to_string(config.teensy_max_clock_sync_error_ns) + " ns, or the offset "
                                 "was not accepted");
    }
    if (config.enable_teensy_debug_logging) {
        logging.enabled = true;
        if (!send_await(TEENSY_COMMAND_OPCODE_CMD_SET_LOGGING,
                        {reinterpret_cast<const char*>(&logging), sizeof logging}, MAX_RESPONSE_TIMEOUT)) {
            throw std::runtime_error("Teensy did not answer CMD_SET_LOGGING after the clock sync");
        }
    }
    clocks_synced_ = true;
}

void TeensyManager::on_board_time(feather_time_t sent, feather_time_t received, std::uint64_t board_ns) noexcept {
    // Initial sync always runs; continuous correction only when enabled. One offset in flight at a time, and a sample
    // sent before the last offset was accepted still shows the old clock.
    if (clocks_synced_ && !config.teensy_correct_clock_drift) return;
    if (pending_offset_msg_ != 0 || sent < offset_accepted_at_) return;

    // Accuracy bound is +-RTT/2. Take it only when tighter than the last bound plus the drift assumed since then.
    const std::int64_t bound = static_cast<std::int64_t>(received - sent) / 2;
    const feather_time_t now = feather::time::now();
    const std::int64_t last_bound = clock_bound_ns_;
    const double drift_ns = static_cast<double>(now - last_clock_sync_) * config.teensy_expected_clock_drift_ns / 1e9;
    if (last_bound != INT64_MAX && static_cast<double>(bound) >= drift_ns + static_cast<double>(last_bound)) return;

    clock_bound_ns_ = bound;
    last_clock_sync_ = now;
    SetTimestampOffsetPayload body{};
    body.offset = clock_offset_ns_ + static_cast<std::int64_t>(sent) + bound - static_cast<std::int64_t>(board_ns);
    pending_offset_ns_ = body.offset;
    // The reply is handled on this thread, so it cannot arrive before pending_offset_msg_ is set.
    pending_offset_msg_ = send_message(
        TEENSY_COMMAND_OPCODE_CMD_SET_TIMESTAMP_OFFSET, {reinterpret_cast<const char*>(&body), sizeof body}, false
    ).msg_id;
}

std::string TeensyManager::src_hash_text() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(resp_buf_mut));
    return src_hash_text_;
}

void RecordQueue::push(RecordKind kind, feather_time_t ts, std::uint32_t msg_id, std::uint16_t opcode,
                       const std::uint8_t* data, std::size_t size) noexcept {
    RecordHeader header{};
    header.kind = kind;
    header.opcode = opcode;
    header.msg_id = msg_id;
    header.ts_ns = static_cast<std::int64_t>(ts);
    header.size = static_cast<std::uint32_t>(size);
    std::lock_guard<std::mutex> lock(mut_);
    if (buf_.size() + sizeof header + size > capacity_) {
        ++dropped_;
        return;
    }
    const auto* h = reinterpret_cast<const std::uint8_t*>(&header);
    buf_.insert(buf_.end(), h, h + sizeof header);
    if (size > 0) buf_.insert(buf_.end(), data, data + size);
}

std::size_t RecordQueue::drain(std::uint8_t* out, std::size_t cap, std::uint64_t* dropped) noexcept {
    std::lock_guard<std::mutex> lock(mut_);
    std::size_t n = 0;
    while (n + sizeof(RecordHeader) <= buf_.size()) {
        RecordHeader header{};
        std::memcpy(&header, buf_.data() + n, sizeof header);
        const std::size_t total = sizeof header + header.size;
        if (n + total > cap) break;
        n += total;
    }
    if (n > 0) {
        std::memcpy(out, buf_.data(), n);
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(n));
    }
    if (dropped != nullptr) *dropped = dropped_;
    dropped_ = 0;
    return n;
}

TeensyManager::~TeensyManager() { abort(); }

void TeensyManager::push_free(std::uint16_t slot) noexcept {
    free_slots[free_tail] = slot;
    free_tail = static_cast<std::uint16_t>((free_tail + 1) & (TEENSY_MAX_RESPONSE_BUFFER_SIZE - 1));
    ++free_count;
}

void TeensyManager::reclaim_expired(feather_time_t now) noexcept {
    for (std::uint16_t i = 0; i < TEENSY_MAX_RESPONSE_BUFFER_SIZE; ++i) {
        TeensyResponse& slot = response_buffer[i];
        if (slot.msg_id == 0 || slot.inserted_ts > now || now - slot.inserted_ts < MAX_RESPONSE_TIMEOUT) continue;
        slot.msg_id = 0;
        slot.received_ts = 0;
        push_free(i);
    }
}

SentTeensyMessage TeensyManager::send_message(
    TeensyCommandOpcode opcode, std::string_view payload, bool want_response
) noexcept {
    ENSURE(sock_.fd() >= 0, (SentTeensyMessage{}), "Teensy socket not connected");
    ENSURE(payload.size() <= MAX_COMMAND_SIZE - sizeof(ClientCommandHeader), (SentTeensyMessage{}), 
           "Command payload too large");

    std::uint32_t msg_id = 0;
    std::uint16_t slot_idx = 0;
    {
        std::lock_guard<std::mutex> lock(resp_buf_mut);

        // If there are no free slots, reclaim expired ones. If there are still no free slots, return a failed send.
        const feather_time_t now = feather::time::now();
        if (free_count == 0) reclaim_expired(now);
        if (free_count == 0) return {.msg_id = 0, .mailbox_idx = 0};

        // Get the next free slot.
        slot_idx = free_slots[free_head];
        free_head = static_cast<std::uint16_t>((free_head + 1) & (TEENSY_MAX_RESPONSE_BUFFER_SIZE - 1));
        --free_count;

        TeensyResponse& slot = response_buffer[slot_idx];

        // Increment the generation. If it overflows, wrap around to 1.
        std::uint32_t gen = slot.generation + 1;
        if (gen >= GEN_LIMIT) gen = 1;
        slot.generation = gen;

        // Build the message id and initialize the slot.
        msg_id = (gen << SLOT_BITS) | slot_idx;
        slot.msg_id = msg_id;
        slot.payload_size = 0;
        slot.received_ts = 0;
        slot.inserted_ts = now;
        slot.discard = !want_response;
    }

    // One mutex covers the whole frame, including a short write retried after EAGAIN.
    {
        std::lock_guard<std::mutex> lock(send_mut);

        // Build the command frame.
        std::uint8_t frame[MAX_COMMAND_SIZE];
        const ClientCommandHeader header{
            .msg_id = msg_id,
            .opcode = opcode,
            .payload_size = static_cast<std::uint32_t>(payload.size()),
        };
        std::memcpy(frame, &header, sizeof header);
        if (!payload.empty()) std::memcpy(frame + sizeof header, payload.data(), payload.size());

        // Write the command frame to the Teensy. If it fails, clear the slot and return a failed send.
        if (!sock_.write_all(frame, sizeof header + payload.size())) {
            std::lock_guard<std::mutex> buf_lock(resp_buf_mut);
            TeensyResponse& slot = response_buffer[slot_idx];
            if (slot.msg_id == msg_id) {
                slot.msg_id = 0;
                slot.received_ts = 0;
                push_free(slot_idx);
            }
            return {.msg_id = 0, .mailbox_idx = 0};
        }
    }

    return {.msg_id = msg_id, .mailbox_idx = slot_idx};
}

std::optional<TeensyResponse> TeensyManager::get_response(
    const SentTeensyMessage& msg, feather_time_t timeout, PayloadOut out
) noexcept {
    // msg_id 0 is a failed send, and it is also what a taken slot is cleared to. Do not index with it.
    if (msg.msg_id == 0 || msg.mailbox_idx >= TEENSY_MAX_RESPONSE_BUFFER_SIZE) return std::nullopt;

    // Sleep on the condition variable, not a spin: a waiting caller must not compete with the poll thread for the lock.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::nanoseconds(timeout);
    std::unique_lock<std::mutex> lock(resp_buf_mut);
    TeensyResponse& slot = response_buffer[msg.mailbox_idx];
    const auto settled = [&] { return slot.msg_id != msg.msg_id || slot.received_ts != 0; };
    if (timeout > 0) resp_cv_.wait_until(lock, deadline, settled);
    if (slot.msg_id != msg.msg_id || slot.received_ts == 0) return std::nullopt;

    // A short destination must not consume the payload. The caller can retry with a larger one.
    if (out.data != nullptr && (
        slot.payload_size > out.size || slot.payload_size > TEENSY_MAX_RESPONSE_PAYLOAD_SIZE
    )) {
        WARNING("Payload destination too small for response payload size");
        return std::nullopt;
    }

    TeensyResponse resp = slot;
    if (out.data != nullptr && slot.payload_size > 0) {
        std::memcpy(out.data, payload_buffer[msg.mailbox_idx], slot.payload_size);
    }
    slot.msg_id = 0;
    slot.received_ts = 0;
    push_free(static_cast<std::uint16_t>(msg.mailbox_idx));
    return resp;
}

std::optional<TeensyResponse> TeensyManager::send_await(
    TeensyCommandOpcode opcode, std::string_view payload, feather_time_t timeout, PayloadOut out
) noexcept {
    return get_response(send_message(opcode, payload), timeout, out);
}

SentTeensyMessage TeensyManager::cansend(int bus, bool extended, int can_id, std::string_view data) noexcept {
    ENSURE(data.size() <= 8, (SentTeensyMessage{}), "CAN data is %zu bytes", data.size());
    SendCanPayload body{};
    body.can_bus = static_cast<std::uint8_t>(bus);
    body.extended = extended;
    body.dlc = static_cast<std::uint8_t>(data.size());
    body.can_id = static_cast<std::uint32_t>(can_id);
    if (!data.empty()) std::memcpy(body.data, data.data(), data.size());
    return send_message(TEENSY_COMMAND_OPCODE_CMD_SEND_CAN, {reinterpret_cast<const char*>(&body), sizeof body});
}

void TeensyManager::abort() {
    aborted_ = true;
    if (poll_thread_.joinable()) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
        while (poll_running_ && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (!poll_running_) poll_thread_.join();
        else {
            ERROR("Teensy poll thread did not exit within 0.25 s");
            poll_thread_.detach();
        }
    }
    emulator_.stop();
}

void TeensyManager::poll() {
    // The stream is no longer frame-aligned, or the socket failed. Stop senders too.
    const auto fail = [&] {
        poll_err_ = true;
        sock_.kill();
    };

    feather_time_t last_telemetry = 0;
    feather_time_t last_candump = 0;
    feather_time_t last_board_time = 0;

    while (!aborted_) {
        const int fd = sock_.fd();
        ENSURE(fd >= 0, fail(), "Teensy poll stopped: socket is closed");

        // Request telemetry and candump updates if needed
        const bool ready = telemetry_ == nullptr || telemetry_->initialized();
        const feather_time_t now = feather::time::now();
        feather_time_t wait = feather::time::from_millis(ready ? 30 : 5);

        const auto request = [&](bool enabled, feather_time_t& last, double hz, TeensyCommandOpcode opcode) {
            if (!enabled || !(hz > 0.0)) return;
            const feather_time_t period = feather::time::from_seconds(1.0 / hz);
            const feather_time_t elapsed = now - last;

            if (elapsed >= period) {
                last = now;
                send_message(opcode, {}, false);  // handled here, nobody takes the reply
                if (period < wait) wait = period;
            } else if (period - elapsed < wait) {
                wait = period - elapsed;
            }
        };

        if (ready) {
            request(
                telemetry_ != nullptr, last_telemetry, config.teensy_telemetry_update_frequency,
                TEENSY_COMMAND_OPCODE_CMD_GET_ACTUATOR_TELEMETRY
            );
            request(
                (data_monitor_ != nullptr && data_monitor_->has_candump()) || records.candump, last_candump,
                config.teensy_candump_update_frequency, TEENSY_COMMAND_OPCODE_CMD_GET_CANDUMP
            );
        }
        // Continuous drift correction once the initial sync is done (sync_clocks asks for its own samples).
        request(
            clocks_synced_ && config.teensy_correct_clock_drift, last_board_time, config.teensy_board_clock_sync_hz,
            TEENSY_COMMAND_OPCODE_CMD_GET_BOARD_TIME
        );

        // Poll for Teensy responses
        pollfd pfd{.fd = fd, .events = POLLIN, .revents = 0};
        const timespec timeout{
            .tv_sec = static_cast<time_t>(wait / 1000000000ULL),
            .tv_nsec = static_cast<long>(wait % 1000000000ULL),
        };
        const int pr = ::ppoll(&pfd, 1, &timeout, nullptr);

        if (pr < 0 && errno == EINTR) continue;
        ENSURE(pr >= 0, fail(), "Teensy poll failed: errno %d", errno);
        if (pr == 0) continue;
        ENSURE((pfd.revents & POLLIN) != 0, fail(), "Teensy connection closed while poll");

        const ssize_t n = ::recv(fd, rx_buf_.data() + rx_len_, rx_buf_.size() - rx_len_, 0);
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        ENSURE(n != 0, fail(), "Teensy connection closed while recv");
        ENSURE(n > 0, fail(), "Teensy recv failed: errno %d", errno);
        rx_len_ += static_cast<std::size_t>(n);

        // Parse every complete frame in this read.
        std::size_t off = 0;
        while (!aborted_ && rx_len_ - off >= sizeof(ClientCommandHeader)) {
            ClientCommandHeader header{};
            std::memcpy(&header, rx_buf_.data() + off, sizeof header);
            const std::size_t total = sizeof header + static_cast<std::size_t>(header.payload_size);

            // If the response payload is too large for the buffer, abort the connection.
            if (total > rx_buf_.size()) {
                WARNING("Teensy response payload is %u bytes", header.payload_size);
                return fail();
            }
            
            if (rx_len_ - off < total) break;
            handle_response(header, rx_buf_.data() + off + sizeof header);
            off += total;
        }

        // Slide an unfinised frame to the front.
        if (off > 0) {
            rx_len_ -= off;
            if (rx_len_ > 0) std::memmove(rx_buf_.data(), rx_buf_.data() + off, rx_len_);
        }
    }
}

void TeensyManager::handle_response(const ClientCommandHeader& header, const std::uint8_t* payload) {
    // Low bits of msg_id are the slot. A mismatch is a reply this manager did not send.
    const std::uint16_t slot_idx =
        static_cast<std::uint16_t>(header.msg_id & (TEENSY_MAX_RESPONSE_BUFFER_SIZE - 1));
    const feather_time_t received = feather::time::now();

    // Recorded replies leave the telemetry and candump bodies out: they have their own recordings. Successes are not
    // recorded (as comms_manager.py).
    if (records.messages && header.opcode != TEENSY_COMMAND_OPCODE_RESP_SUCCESS) {
        const bool own_recording = header.opcode == TEENSY_COMMAND_OPCODE_RESP_ACTUATOR_TELEMETRY ||
            header.opcode == TEENSY_COMMAND_OPCODE_RESP_CANDUMP;
        records.push(RECORD_RX_MESSAGE, received, header.msg_id, static_cast<std::uint16_t>(header.opcode), payload,
                     own_recording ? 0 : header.payload_size);
    }

    // A board time sample is applied after the lock is released: a better offset is sent, which takes it.
    bool board_sample = false;
    feather_time_t sample_sent = 0;
    std::uint64_t sample_board_ns = 0;
    {
        // Lock the response buffer to access the slot. Update the slot's state.
        std::lock_guard<std::mutex> lock(resp_buf_mut);
        TeensyResponse& slot = response_buffer[slot_idx];
        if (header.msg_id == 0 || slot.msg_id != header.msg_id) {
            const std::string text = "Received message with ID `" + std::to_string(header.msg_id) + "` (opcode=" +
                std::to_string(static_cast<unsigned>(header.opcode)) + ") but it was not in sent messages; dropping";
            records.push(RECORD_WARNING, received, header.msg_id, static_cast<std::uint16_t>(header.opcode),
                         reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
            return;
        }

        slot.opcode = header.opcode;
        slot.payload_size = 0;
        slot.received_ts = received;

        // The offset sync_clocks/on_board_time pushed: accepted, or rejected so the next sample can try again.
        if (header.msg_id == pending_offset_msg_) {
            if (header.opcode == TEENSY_COMMAND_OPCODE_RESP_SUCCESS) {
                clock_offset_ns_ = pending_offset_ns_;
                offset_accepted_at_ = received;
            } else {
                clock_bound_ns_ = INT64_MAX;
            }
            pending_offset_msg_ = 0;
        }

        // Handle the response
        switch (header.opcode) {
        case TEENSY_COMMAND_OPCODE_RESP_SUCCESS:  // Nothing else to do for a success response
            break;

        case TEENSY_COMMAND_OPCODE_RESP_ERROR:
        case TEENSY_COMMAND_OPCODE_RESP_ECHO:
            if (header.payload_size > TEENSY_MAX_RESPONSE_PAYLOAD_SIZE) {
                ERROR("Teensy string response is %u bytes", header.payload_size);
                break;
            }
            if (header.payload_size > 0) std::memcpy(payload_buffer[slot_idx], payload, header.payload_size);
            slot.payload_size = header.payload_size;
            if (header.opcode == TEENSY_COMMAND_OPCODE_RESP_ERROR) {  // logged even if a caller takes it
                const std::string text = "Received error response for message `" + std::to_string(header.msg_id) +
                    "`: " + std::string(reinterpret_cast<const char*>(payload), header.payload_size);
                records.push(RECORD_ERROR, received, header.msg_id, static_cast<std::uint16_t>(header.opcode),
                             reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
            }
            break;

        case TEENSY_COMMAND_OPCODE_RESP_ACTUATOR_TELEMETRY: {
            if (header.payload_size < sizeof(GetTelemetryPayloadHeader)) {
                ERROR("Teensy actuator telemetry response is %u bytes", header.payload_size);
                break;
            }

            GetTelemetryPayloadHeader body{};
            std::memcpy(&body, payload, sizeof body);
            const std::size_t up_size = header.payload_size - sizeof body;
            if (telemetry_ == nullptr) {
                DEBUG("Actuator telemetry received with no telemetry manager");
            } else {
                telemetry_->update_telemetry(
                    static_cast<int>(body.num_telemetry_updates), up_size == 0 ? payload : payload + sizeof body,
                    up_size
                );
            }
            // Record the response body, header included, only when it carries updates.
            if (body.num_telemetry_updates > 0 && data_monitor_ != nullptr) {
                data_monitor_->write_telemetry(payload, header.payload_size);
            }
            if (body.num_telemetry_updates > 0 && records.telemetry) {
                records.push(RECORD_TELEMETRY, received, header.msg_id, body.num_telemetry_updates,
                             payload + sizeof body, up_size);
            }
            break;
        }

        case TEENSY_COMMAND_OPCODE_RESP_CANDUMP:
            // Payload is a packed CandumpTelemetry array. Zero means the ring was empty.
            if (header.payload_size > 0 && data_monitor_ != nullptr) {
                data_monitor_->write_candump(payload, header.payload_size);
            }
            if (header.payload_size > 0 && records.candump) {
                records.push(RECORD_CANDUMP, received, header.msg_id, 0, payload, header.payload_size);
            }
            break;

        case TEENSY_COMMAND_OPCODE_RESP_BOARD_TIME: {
            if (header.payload_size != sizeof(GetBoardTimeResponsePayload)) {
                ERROR("Teensy board time response is %u bytes", header.payload_size);
                break;
            }
            std::memcpy(payload_buffer[slot_idx], payload, header.payload_size);
            slot.payload_size = header.payload_size;
            GetBoardTimeResponsePayload body{};
            std::memcpy(&body, payload, sizeof body);
            board_sample = true;
            sample_sent = slot.inserted_ts;
            sample_board_ns = body.board_time_nanos;
            break;
        }

        case TEENSY_COMMAND_OPCODE_RESP_SRC_HASH: {
            constexpr std::string_view start = SRC_HASH_START;
            constexpr std::string_view end = SRC_HASH_END;
            const std::string_view text(reinterpret_cast<const char*>(payload), header.payload_size);

            // Check if the response is wrapped in the start and end markers, then parse the decimal hash.
            const bool wrapped = text.size() > start.size() + end.size() && text.substr(0, start.size()) == start &&
                text.substr(text.size() - end.size()) == end;
            const auto digits = wrapped ? text.substr(start.size(), text.size() - start.size() - end.size()) :
                std::string_view{};
            src_hash_text_ = std::string(text);
            src_hash_received_ = true;
            std::uint64_t hash = 0;
            const auto parsed = wrapped ? std::from_chars(digits.data(), digits.data() + digits.size(), hash) :
                std::from_chars_result{};

            // The Python SDK's firmware hash is wider than 64 bits: without manage_firmware the caller checks the text.
            if (!wrapped || parsed.ec != std::errc{} || parsed.ptr != digits.data() + digits.size()) {
                if (config.manage_firmware) ERROR("Teensy source hash response is malformed");
                break;
            }
            teensy_src_hash_ = hash;
            break;
        }

        case TEENSY_COMMAND_OPCODE_RESP_NUM_CAN_LINES: {
            if (header.payload_size != sizeof(GetNumCanLinesResponsePayload)) {
                ERROR("Teensy num CAN lines response is %u bytes", header.payload_size);
                break;
            }
            GetNumCanLinesResponsePayload body{};
            std::memcpy(&body, payload, sizeof body);
            num_can_lines = static_cast<std::int32_t>(body.num_can_lines);
            break;
        }

        default: {
            const std::string text = "Unknown response code: `" + std::to_string(static_cast<unsigned>(header.opcode)) +
                "` for message " + std::to_string(header.msg_id);
            records.push(RECORD_ERROR, received, header.msg_id, static_cast<std::uint16_t>(header.opcode),
                         reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
            break;
        }
        }

        // Callers read these back, unless nobody will. Everything else was handled above.
        const bool keep = !slot.discard && (header.opcode == TEENSY_COMMAND_OPCODE_RESP_SUCCESS ||
            header.opcode == TEENSY_COMMAND_OPCODE_RESP_ERROR ||
            header.opcode == TEENSY_COMMAND_OPCODE_RESP_ECHO ||
            header.opcode == TEENSY_COMMAND_OPCODE_RESP_BOARD_TIME);
        if (!keep) {
            slot.msg_id = 0;
            slot.received_ts = 0;
            push_free(slot_idx);
        }
    }
    resp_cv_.notify_all();  // a waiter on a freed slot also wakes, and returns nothing at once

    if (board_sample) on_board_time(sample_sent, received, sample_board_ns);
}

}  // namespace feather::comms
