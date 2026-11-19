#include "can.hpp"
#include "comms.hpp"
#include "../logging.hpp"
#include <assert.h>
#include <string.h>
#include <math.h>
#include "../commands/robstride.hpp"
#include "../commands/ezmotion.hpp"
#include "../commands/realhand.hpp"
#include "../commands/soc.hpp"
#include "../commands/all_commands.hpp"
#include "ethernet.hpp"
#include "../system.hpp"
#include <errno.h>


#ifdef COMPILE_FOR_TEENSY

    #include <FlexCAN_T4.h>

    #define MAX_NUM_CAN_LINES 3
    static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_line_0;
    static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_line_1;
    static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_line_2;
    
    bool initialize_can(void) {
        can_line_0.begin();
        can_line_0.setBaudRate(CAN_BAUD_RATE);
        can_line_1.begin();
        can_line_1.setBaudRate(CAN_BAUD_RATE);
        can_line_2.begin();
        can_line_2.setBaudRate(CAN_BAUD_RATE);
        feather_log("CAN lines initialized.");
        return true;
    }

    uint8_t get_num_can_lines(void) {
        return MAX_NUM_CAN_LINES;
    }

    int write_physical_can(uint8_t bus, const CAN_message_t* msg) {
        switch (bus) {
            case 0: return can_line_0.write(*msg);
            case 1: return can_line_1.write(*msg);
            case 2: return can_line_2.write(*msg);
            default: return 0;
        }
    }

    int read_physical_can(uint8_t bus, CAN_message_t* msg) {
        switch (bus) {
            case 0: return can_line_0.read(*msg);
            case 1: return can_line_1.read(*msg);
            case 2: return can_line_2.read(*msg);
            default: return 0;
        }
    }

#else

/////////////////////////////////
// Emulator-specific CAN Mutex //
/////////////////////////////////

#include <pthread.h>
#include <unistd.h>

#define ACQUIRE_POLL_LOCK_MAX_ATTEMPTS 50
#define ACQUIRE_POLL_LOCK_SLEEP_US 100

runtime_static pthread_mutex_t can_poll_mutex = PTHREAD_MUTEX_INITIALIZER;

bool acquire_poll_lock(void) {
    for (int i = 0; i < ACQUIRE_POLL_LOCK_MAX_ATTEMPTS; i++) {
        if (pthread_mutex_trylock(&can_poll_mutex) == 0) return true;
        usleep(ACQUIRE_POLL_LOCK_SLEEP_US);
    }
    return false;
}

void release_poll_lock(void) {
    (void)pthread_mutex_unlock(&can_poll_mutex);
}

#endif


///////////////////////
// Candump Telemetry //
///////////////////////


// --- Ring buffer storage ---
runtime_static CandumpTelemetry candump_telemetry[CANDUMP_TELEMETRY_SIZE];
runtime_static uint32_t candump_telem_start = 0;
runtime_static uint32_t candump_telem_end = 0;

void clear_candump_telemetry(void) {
    candump_telem_start = 0;
    candump_telem_end = 0;
}

// Insert a candump telemetry value into the buffer.
runtime_static void _insert_candump_telemetry(uint8_t bus, bool sent, const CAN_message_t* msg) {
    uint8_t num_can_lines = get_num_can_lines();
    ASSERT_GLOBAL(bus < num_can_lines, ;, "Invalid CAN bus %u! Valid buses are 0-%u.", bus, num_can_lines - 1);
    ASSERT_GLOBAL(!msg->flags.extended || msg->id <= 0x1FFFFFFF, ;, "Extended CAN ID not in range [0, 0x1FFFFFFF].", msg->id);
    ASSERT_GLOBAL(msg->flags.extended || msg->id <= 0x7FF, ;, "Standard CAN ID not in range [0, 0x7FF].", msg->id);
    ASSERT_GLOBAL(msg->len <= 8, ;, "Invalid CAN data length `%u`. Should be in range [0, 8]", msg->len);

    ASSERT_GLOBAL(ACQUIRE_POLL_LOCK(), ;, "Failed to acquire CAN poll lock.");
    // Build the candump telemetry value.
    uint8_t info = (uint8_t)(((unsigned)bus << 5) | ((sent ? 1u : 0u) << 4) | (unsigned)msg->len);
    CandumpTelemetry telem = {.info = info, .can_id = msg->id, .data = {0}, .timestamp_ns = currtime_ns()};
    memcpy(telem.data, msg->buf, msg->len);
    candump_telemetry[candump_telem_end] = telem;

    // Update the ring buffer and check for overflow.
    candump_telem_end = (candump_telem_end + 1) % CANDUMP_TELEMETRY_SIZE;
    if (candump_telem_end == candump_telem_start) {
        set_global_error_msg("Candump telemetry buffer is full! Dropping data.");
        candump_telem_start = (candump_telem_start + 1) % CANDUMP_TELEMETRY_SIZE;
    }
    RELEASE_POLL_LOCK();
}

void insert_candump_telemetry_rx(uint8_t bus, const CAN_message_t* msg) {
    _insert_candump_telemetry(bus, false, msg);
}


/////////////////
// CAN Writing //
/////////////////


// --- Can Throttler ---
runtime_static float can_last_loads[MAX_NUM_CAN_LINES] = {0.0};
runtime_static uint64_t can_last_times_ns[MAX_NUM_CAN_LINES] = {0};

// Writes a CAN message to the given bus index. Returns >0 on success, 0 on failure/invalid bus
static int can_write(uint8_t bus, const CAN_message_t* msg) {
    int result = write_physical_can(bus, msg);
    if (result > 0) _insert_candump_telemetry(bus, true, msg);
    return result;
}

/**
 * Checks if the given CAN bus would overload.
 *
 * If it would overload, then the message rates will not be updated under the assumption that the message will not be
 * sent. Otherwise it is updated and assumed we will send the message.
 *
 * We use an exponential decay to calculate the new load. See the feathersdk python documentation for the
 * `EventThrottler` class for more information.
 *
 * @param bus The bus index in ``[0, get_num_can_lines())``
 * @param now_ns Current time in nanoseconds (typically ``currtime_ns()``). Tests may reuse a fixed timestamp to force
 *     a zero inter-event interval.
 * @return True if the bus would overload, false otherwise
 */
runtime_static bool can_would_overload(uint8_t bus, uint64_t now_ns) {
    float leniency = comms_config.can_leniency;
    float time_diff = ((float)(now_ns - can_last_times_ns[bus])) * 1e-9f;

    float new_load = can_last_loads[bus] * (float)exp(-(time_diff * leniency)) + leniency;
    if (new_load > comms_config.max_can_rate) return true;

    can_last_loads[bus] = new_load;
    can_last_times_ns[bus] = now_ns;
    return false;
}

bool cansend(uint8_t bus, bool extended, uint32_t can_id, const uint8_t* data, uint8_t len) {
    uint8_t num_can_lines = get_num_can_lines();
    ASSERT_LOCAL(len <= 8, false, "CAN data length too long (%u bytes)! Max is 8.", len);
    ASSERT_LOCAL(bus < num_can_lines, false, "Invalid CAN bus %u! Valid buses are 0-%u.", bus, num_can_lines - 1);
    if (extended) {
        ASSERT_LOCAL(can_id <= 0x1FFFFFFF, false, "CAN ID too large (%u)! Max is 0x1FFFFFFF for extended.", can_id);
    } else {
        ASSERT_LOCAL(can_id <= 0x7FF, false, "CAN ID too large (%u)! Max is 0x7FF for standard IDs.", can_id);
    }

    // Check for CAN bus overload
    ASSERT_LOCAL(!can_would_overload(bus, currtime_ns()), false, "CAN bus %u is overloaded.", bus);

    // Build the can message.
    CAN_message_t msg = {0};
    msg.id = can_id;
    msg.flags.extended = extended;
    msg.len = len;
    memcpy(msg.buf, data, len);

    ASSERT_LOCAL(can_write(bus, &msg), false, "Failed to send CAN message on bus %u! Errno: %s", bus, strerror(errno));
    return true;
}

bool opencan_send_sdo(
    uint8_t bus, SDOCommand cmd, uint8_t node_id, uint16_t idx, uint8_t subindex, DataType dtype, const void* value
) {
    ASSERT_LOCAL(cmd == SDO_READ || cmd == SDO_WRITE, false, "Invalid SDO command: %u", cmd);
    ASSERT_LOCAL(node_id <= 127, false, "Invalid node ID: %u", node_id);

    if (cmd == SDO_READ) {
        ASSERT_LOCAL(value == NULL, false, "Value must be NULL when reading.");
    }
    else {
        ASSERT_LOCAL(value != NULL, false, "Value is NULL when writing.");
    }

    // Expedited SDO: read request is always 0x40. Write CS encodes unused data bytes:
    // 1B=0x2F, 2B=0x2B, 4B=0x23. 
    uint8_t cs = 0x40;
    if (cmd == SDO_WRITE) {
        switch (dtype_size(dtype)) {
            case 1: cs = 0x2F; break;
            case 2: cs = 0x2B; break;
            case 4: cs = 0x23; break;
            default: cs = 0; break;
        }
    }

    ASSERT_LOCAL(cs != 0, false, "Unhandled sdo cmd/data type: cmd=%u, dtype=%u", cmd, dtype);

    // Index is little-endian (idx_lo, idx_hi). Only update the unused bytes if writing, reads have all 0's
    uint8_t data[8] = {cs, (uint8_t)(idx & 0xFF), (uint8_t)(idx >> 8), subindex, 0, 0, 0, 0};
    if (cmd == SDO_WRITE) {
        memcpy(data + 4, value, dtype_size(dtype));
    }
    return cansend(bus, false, 0x600 | node_id, data, 8);
}


//////////////////////////
// CAN Reading/Handling //
//////////////////////////


void handle_can_message(uint8_t bus, const CAN_message_t* msg) {
    if (msg->flags.extended) {
        handle_robstride_can_message(bus, msg);
    } else if (msg->id == SOCBRIDGE_CAN_ID) {
        handle_soc_can_message(bus, msg);
    } else if (is_realhand_slot(msg->id)) {
        handle_realhand_can_message(bus, msg);
    } else {
        handle_ezmotion_can_message(bus, msg);
    }
}

void poll_can(void) {
    uint64_t start_time = currtime_ns();

    // Round-robin across the lines, reading one message per line per pass. Stop once a full pass reads
    // nothing (all lines empty) or the time budget is exhausted.
    bool any_received = true;
    while (any_received && (currtime_ns() - start_time < comms_config.can_read_run_time_nanos)) {
        any_received = false;
        for (uint8_t bus = 0; bus < get_num_can_lines(); bus++) {
            CAN_message_t msg;

            int result = read_physical_can(bus, &msg);
            if (result > 0) {
                any_received = true;
                _insert_candump_telemetry(bus, false, &msg);
                handle_can_message(bus, &msg);
            }
        }
    }
}


//////////////////////////////////////
// Handle Candump Telemetry Command //
//////////////////////////////////////


static inline bool _handle_get_candump_locked(uint32_t msg_id, void* payload) {
    (void)payload;

    const uint32_t framing_size = sizeof(ClientCommandHeader);
    uint32_t remaining_space = eth_write_buffer_remaining_space();

    // Empty dump: zero-length payload (ClientCommandHeader only).
    if (candump_telem_start == candump_telem_end) {
        if (remaining_space < framing_size) {
            set_local_error_msg("Not enough space in the buffer to send a candump response.");
            return false;
        }
        eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_CANDUMP, NULL, 0);
        return true;
    }

    // Need room for framing + at least one CandumpTelemetry.
    if (remaining_space < framing_size + sizeof(CandumpTelemetry)) {
        set_local_error_msg("Not enough space in the buffer to send a candump telemetry response.");
        return false;
    }
    remaining_space -= framing_size;

    uint32_t pending = (candump_telem_end + CANDUMP_TELEMETRY_SIZE - candump_telem_start) % CANDUMP_TELEMETRY_SIZE;
    uint32_t num_msgs = MIN(remaining_space / (uint32_t)sizeof(CandumpTelemetry), pending);

    uint32_t payload_size = (uint32_t)(num_msgs * sizeof(CandumpTelemetry));
    ClientCommandHeader cmd_header = {msg_id, TEENSY_COMMAND_OPCODE_RESP_CANDUMP, payload_size};
    eth_write_buffer_insert(&cmd_header, sizeof(ClientCommandHeader));

    // Copy ring contents in 1–2 contiguous chunks (source wrap), not one insert per message.
    uint32_t first = MIN(num_msgs, CANDUMP_TELEMETRY_SIZE - candump_telem_start);
    eth_write_buffer_insert(&candump_telemetry[candump_telem_start], first * sizeof(CandumpTelemetry));
    if (num_msgs > first) {
        eth_write_buffer_insert(&candump_telemetry[0], (num_msgs - first) * sizeof(CandumpTelemetry));
    }
    candump_telem_start = (candump_telem_start + num_msgs) % CANDUMP_TELEMETRY_SIZE;

    return true;
}

bool handle_get_candump(uint32_t msg_id, void* payload) {
    ASSERT_GLOBAL(ACQUIRE_POLL_LOCK(), false, "Failed to acquire CAN poll lock.");
    bool result = _handle_get_candump_locked(msg_id, payload);
    RELEASE_POLL_LOCK();
    return result;
}
