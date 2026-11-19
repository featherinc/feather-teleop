#pragma once

#include "qdefs/teensy_commands_qdef.hpp"
#include "system.hpp"
#include "teensy_socket.hpp"

#include <optional>
#include <string_view>
#include <cstdint>
#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <type_traits>

// Maximum size of a response payload. Max size at time of writing is 267 bytes.
#define TEENSY_MAX_RESPONSE_PAYLOAD_SIZE 512

// Maximum number of responses to buffer.
// Must be a power of two. The slot index is the low bits of msg_id, and the free-slot ring
// advances with a mask of size - 1.
#define TEENSY_MAX_RESPONSE_BUFFER_SIZE 1024

namespace feather::robot {
class Telemetry;
class DataMonitorService;
}

static_assert(
    TEENSY_MAX_RESPONSE_BUFFER_SIZE > 0 &&
        (TEENSY_MAX_RESPONSE_BUFFER_SIZE & (TEENSY_MAX_RESPONSE_BUFFER_SIZE - 1)) == 0,
    "TEENSY_MAX_RESPONSE_BUFFER_SIZE must be a power of two"
);

namespace feather::comms {

// Teensy connection and runtime settings. Defaults match CommsManagerConfig in comms_manager.py.
struct TeensyConfig {
    // Seconds to wait for the Teensy to connect. Range [0.1, 60].
    double teensy_connect_timeout = 10.0;
    // How often to process results from the Teensy socket, in Hz. Range [1, 1000].
    double teensy_operating_frequency = 300.0;
    // How often to request motor telemetry, in Hz. Range [1, 1000].
    double teensy_telemetry_update_frequency = 300.0;
    // How often to request candump updates, in Hz. Range [1, 1000].
    double teensy_candump_update_frequency = 30.0;
    // Enable debug logging from the Teensy over serial.
    bool enable_teensy_debug_logging = false;
    // Correct Teensy clock drift against the host.
    bool teensy_correct_clock_drift = true;
    // How often to sync the Teensy clock to the host, in Hz. Range [1, 1000].
    double teensy_board_clock_sync_hz = 10.0;
    // Expected Teensy drift, in nanoseconds per second. Range [1000, 10000000].
    int teensy_expected_clock_drift_ns = 100000;
    // Largest acceptable Teensy/host clock error, in nanoseconds. Range [1000, 20000000].
    int teensy_max_clock_sync_error_ns = 12000000;
    // Attempts to sync the Teensy clock. Range [1, 1000].
    int teensy_clock_sync_trials = 30;
    // Maximum message responses to process per tick. Range [1, 1000].
    int teensy_max_rx_messages_per_tick = 32;
    // Use the host emulator instead of a connected Teensy 4.1.
    bool use_emulated_teensy = true;
    // Rebuild the firmware, start the emulator and check the source hash here. False when the caller already did the
    // first two (the Python SDK, whose firmware hash is not a uint64): bring_up then only waits for the hash text,
    // src_hash_text(), and the caller compares it.
    bool manage_firmware = true;

    // Maximum CAN messages per second. Sent to the Teensy on initialize. Range [10, 3500].
    double max_can_rate = 3500.0;
    // Leniency for CAN overload checks. Must be <= max_can_rate. Range [1, 3500].
    double can_leniency = 100.0;
    // Max nanoseconds the Teensy may spend in the ethernet server loop per tick. Range [1, 30000000].
    int server_run_time_nanos = 100000;
    // Max nanoseconds the Teensy may spend polling CAN buses per tick. Range [1, 30000000].
    int can_read_run_time_nanos = 100000;

    // How often the result thread drains the socket buffer, in Hz. Range [1, 1000].
    double result_operating_frequency = 200.0;
    // Seconds to wait for the processing thread to start. Range [0.1, 60].
    double await_thread_start_timeout = 1.0;
    // Seconds to wait for the processing thread to toggle state. Range [0.01, 3].
    double processing_toggle_timeout = 0.3;

    // Throws if invalid configs are passed
    void validate() const;
};

// Response from the Teensy not already handled by the TeensyManager. So, largest message will be an ERROR string.
struct TeensyResponse {
    std::uint32_t msg_id;  // (generation << slot bits) | slot. 0 means this slot is free.
    TeensyCommandOpcode opcode;
    std::uint32_t payload_size;
    std::uint32_t generation;  // Survives a free so the next id for this slot differs. Fills padding before timestamps.
    feather_time_t inserted_ts;  // Timestamp when the response was inserted into the buffer.
    feather_time_t received_ts;  // Timestamp when the response was received from the Teensy. 0 if not received yet.
    bool discard;  // Nobody will take this reply: it is freed on arrival (an error reply is recorded first).
};

// What the poll thread records for the caller to write out (the Python SDK's data monitor files and its log). Each
// record is a RecordHeader then `size` payload bytes.
enum RecordKind : std::uint8_t {
    RECORD_TELEMETRY = 0,  // An actuator telemetry body after its update count; opcode holds the count.
    RECORD_CANDUMP = 1,    // A candump body.
    RECORD_RX_MESSAGE = 2, // A reply as it arrived (telemetry and candump bodies left out, as they are recorded above).
    RECORD_ERROR = 3,      // A message to log as an error: an error reply, an unknown reply opcode.
    RECORD_WARNING = 4,    // A message to log as a warning: a reply to a message this manager did not send.
};

struct RecordHeader {
    std::uint8_t kind;
    std::uint8_t pad0;
    std::uint16_t opcode;
    std::uint32_t msg_id;
    std::int64_t ts_ns;  // Host monotonic time it was received.
    std::uint32_t size;
    std::uint32_t pad1;
};
static_assert(sizeof(RecordHeader) == 24, "RecordHeader is read by the Python SDK as '<BxHIqI4x'");

// Bounded byte queue of records, filled by the poll thread and drained by the caller. When full, records are dropped
// and counted, so a slow drain never holds up receiving.
class RecordQueue {
public:
    explicit RecordQueue(std::size_t capacity) : capacity_(capacity) { buf_.reserve(capacity); }
    // Which kinds to keep. Telemetry, candump and messages are off until the caller turns them on.
    std::atomic<bool> telemetry{false}, candump{false}, messages{false};
    void push(RecordKind kind, feather_time_t ts, std::uint32_t msg_id, std::uint16_t opcode, const std::uint8_t* data,
              std::size_t size) noexcept;
    // Copies whole records into out, oldest first, and removes them. Returns the bytes copied; *dropped gets the
    // records dropped since the last drain.
    std::size_t drain(std::uint8_t* out, std::size_t cap, std::uint64_t* dropped) noexcept;

private:
    std::mutex mut_;
    std::vector<std::uint8_t> buf_;
    std::size_t capacity_;
    std::uint64_t dropped_ = 0;
};

// Contains info about a message sent to the Teensy. Pass to get_response() to wait for the response to this message.
struct SentTeensyMessage {
    std::uint32_t msg_id;  // Unique id for the message sent to the Teensy. If 0, then sending the message failed.
    std::uint32_t mailbox_idx;  // Low bits of msg_id. Index of the slot the reply is written to.
};

// Where get_response copies a payload. A null data pointer ignores the payload.
struct PayloadOut {
    std::uint8_t* data = nullptr;
    std::uint32_t size = 0;  // Capacity of data, in bytes.
};

// Bind a trivially copyable object as the payload destination. sizeof(T) is the capacity.
template <typename T>
PayloadOut payload_as(T& dst) {
    static_assert(std::is_trivially_copyable<T>::value, "response payload must be trivially copyable");
    return {.data = reinterpret_cast<std::uint8_t*>(&dst), .size = static_cast<std::uint32_t>(sizeof(T))};
}

// Host-side owner of the Teensy connection. Construction opens the socket, initializes the Teensy, and waits for source
// hash and CAN line count. When use_emulated_teensy is set, the emulator process is started before the socket connects.
class TeensyManager {
public:
    // telemetry receives RESP_ACTUATOR_TELEMETRY. Null leaves those replies unapplied.
    // data_monitor records RESP_CANDUMP and RESP_ACTUATOR_TELEMETRY. Null leaves them unrecorded.
    explicit TeensyManager(
        const TeensyConfig& config,
        feather::robot::Telemetry* telemetry = nullptr,
        feather::robot::DataMonitorService* data_monitor = nullptr
    );
    ~TeensyManager();

    // Send a message to the Teensy. Returns a struct with message info that can be passed to get_response() to wait 
    // for the response.
    // want_response false: nobody will take the reply, so its slot is freed as soon as it arrives (otherwise it is held
    // until taken or for MAX_RESPONSE_TIMEOUT, and fire-and-forget commands at hundreds per second run out of slots).
    SentTeensyMessage send_message(
        TeensyCommandOpcode opcode, std::string_view payload, bool want_response = true
    ) noexcept;

    // Response for message_id, or nullopt if it has not arrived. A successful take removes it.
    // out receives the payload. The default ignores it. A payload larger than out.size leaves the slot in place.
    std::optional<TeensyResponse> get_response(
        const SentTeensyMessage& msg, feather_time_t timeout = 0, PayloadOut out = {}
    ) noexcept;

    // Send a message to the Teensy and wait for the response. Returns the response, or nullopt if it has not arrived
    // by the time the timeout expires. out receives the payload, same as get_response.
    std::optional<TeensyResponse> send_await(
        TeensyCommandOpcode opcode, std::string_view payload, feather_time_t timeout = 0, PayloadOut out = {}
    ) noexcept;

    // Send one CAN frame. Returns the sent-message ticket, or msg_id 0 if the send failed.
    SentTeensyMessage cansend(int bus, bool extended, int can_id, std::string_view data) noexcept;

    // Stop the poll thread. Waits up to 0.25 s for it to exit.
    void abort();

    // abort(), then shut the connection down (the Teensy sees the client leave). Safe while other threads send:
    // their sends fail with msg_id 0, since the socket is shut down, not closed, and this object stays alive.
    void shut_down() noexcept {
        abort();
        sock_.kill();
    }

    // True if poll() stopped because of a socket or framing error.
    bool poll_err() const noexcept { return poll_err_; }

    // CAN lines reported by the Teensy. -1 until RESP_NUM_CAN_LINES arrives.
    std::int32_t num_can_lines = -1;

    // Firmware source hash from RESP_SRC_HASH. 0 until a valid response arrives.
    std::uint64_t src_hash() const noexcept { return teensy_src_hash_; }

    // The source hash reply as the board sent it, markers included. Empty until RESP_SRC_HASH arrives.
    std::string src_hash_text() const;

    // The clock sync's accuracy bound in nanoseconds (half the best round trip it used). INT64_MAX before any.
    std::int64_t clock_sync_bound_ns() const noexcept { return clock_bound_ns_.load(); }

    // Records for the caller to write out. See RecordQueue.
    RecordQueue records{4 * 1024 * 1024};

private:
    // Initialize the Teensy, then wait for the source hash and CAN line count.
    void bring_up();
    // One-shot Cristian sync of the board clock to the host's monotonic clock, as comms_manager.py's _sync_clocks.
    void sync_clocks();
    // A board time sample: maybe push a better offset (comms_manager.py's _maybe_update_clock_offset). Poll thread
    // only, without resp_buf_mut held (it sends).
    void on_board_time(feather_time_t sent, feather_time_t received, std::uint64_t board_ns) noexcept;
    // Read and parse Teensy responses until abort() sets aborted_. Also continually requests actuator telemetry and
    // candump updates.
    void poll();
    // Dispatch one complete response.
    void handle_response(const ClientCommandHeader& header, const std::uint8_t* payload);

    // Caller holds resp_buf_mut. Returns a checked-out slot to the free queue.
    void push_free(std::uint16_t slot) noexcept;
    // Caller holds resp_buf_mut. Pushes slots older than the response timeout. Used only when the queue is empty.
    void reclaim_expired(feather_time_t now) noexcept;

    // Owns emulated Teensy process. pid stays -1 for a real board. Declared before socket so the process is up first.
    struct EmulatedProcess {
        int pid = -1;
        explicit EmulatedProcess(const TeensyConfig& cfg);
        ~EmulatedProcess() { stop(); }
        void stop() noexcept;
        EmulatedProcess(const EmulatedProcess&) = delete;
        EmulatedProcess& operator=(const EmulatedProcess&) = delete;
    };

    TeensyConfig config;
    EmulatedProcess emulator_;
    TeensySocket sock_;
    feather::robot::Telemetry* telemetry_ = nullptr;
    feather::robot::DataMonitorService* data_monitor_ = nullptr;

    // Stores unhandled responses from the Teensy until they are handled or culled.
    std::array<TeensyResponse, TEENSY_MAX_RESPONSE_BUFFER_SIZE> response_buffer;
    std::array<std::uint8_t[TEENSY_MAX_RESPONSE_PAYLOAD_SIZE], TEENSY_MAX_RESPONSE_BUFFER_SIZE> payload_buffer;

    // Free slot indexes, oldest at free_head. free_count separates empty from full when head == tail.
    std::array<std::uint16_t, TEENSY_MAX_RESPONSE_BUFFER_SIZE> free_slots{};
    std::uint16_t free_head = 0;
    std::uint16_t free_tail = 0;
    std::uint16_t free_count = 0;

    std::mutex send_mut;
    std::mutex resp_buf_mut;
    // Signalled under resp_buf_mut when a reply a caller may take arrives.
    std::condition_variable resp_cv_;

    // Clock sync state, poll thread only except the atomics.
    std::atomic<bool> clocks_synced_{false};
    std::atomic<std::int64_t> clock_bound_ns_{INT64_MAX};
    std::int64_t clock_offset_ns_ = 0;          // Offset the board has accepted.
    std::int64_t pending_offset_ns_ = 0;        // Offset in flight.
    std::atomic<std::uint32_t> pending_offset_msg_{0};  // Its message id, 0 when none is in flight.
    feather_time_t offset_accepted_at_ = 0;
    feather_time_t last_clock_sync_ = 0;
    std::string src_hash_text_;  // Guarded by resp_buf_mut.
    bool src_hash_received_ = false;  // Guarded by resp_buf_mut.

    // Set by abort(). poll() loads it between socket waits.
    std::atomic<bool> aborted_{false};
    // True until poll() returns. abort() uses this to stop waiting.
    std::atomic<bool> poll_running_{false};
    std::thread poll_thread_;
    // Set when poll() stops on a socket or framing error.
    std::atomic<bool> poll_err_{false};

    std::uint64_t teensy_src_hash_ = 0;  // Teensy's stored source hash. 0 until a valid response arrives.
    std::uint64_t expected_src_hash_ = 0;  // Expected source hash from current source code. 0 until computed.

    // One inbound frame is assembled here, then parsed. 100 KiB matches the Python result buffer.
    static constexpr std::size_t RX_BUFFER_SIZE = 100 * 1024;
    std::array<std::uint8_t, RX_BUFFER_SIZE> rx_buf_;
    std::size_t rx_len_ = 0;
};

}  // namespace feather::comms
