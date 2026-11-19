#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "../system.hpp"
#include "qdefs/telemetry_qdef.hpp"

#define CAN_BAUD_RATE 1000000

#ifdef COMPILE_FOR_TEENSY
    bool initialize_can(void);
    uint8_t get_num_can_lines(void);
    int write_physical_can(uint8_t bus, const CAN_message_t* msg);
    int read_physical_can(uint8_t bus, CAN_message_t* msg);

    #define POLL_CAN() poll_can()
    #define ACQUIRE_POLL_LOCK() true
    #define RELEASE_POLL_LOCK()
#else
    #include "../../emulator/em_can.hpp"

    #define POLL_CAN() /* No-op on emulator since we launch background thread, otherwise calls poll_can() on teensy. */;
    #define ACQUIRE_POLL_LOCK() \
        /* No-op `true` on teensy since single-threaded. On emulator, gets a mutex lock for updating CAN telemetry, returns bool success. */ \
        acquire_poll_lock()
    #define RELEASE_POLL_LOCK() \
        /* No-op on teensy since single-threaded. On emulator, releases the mutex lock for updating CAN telemetry, no return value. */ \
        release_poll_lock()
#endif


/**
 * DO NOT CALL! Use the ACQUIRE_POLL_LOCK() and RELEASE_POLL_LOCK() macros instead.
 */
bool acquire_poll_lock(void);

/**
 * DO NOT CALL! Use the ACQUIRE_POLL_LOCK() and RELEASE_POLL_LOCK() macros instead.
 */
void release_poll_lock(void);


/**
 * DO NOT CALL! Use the POLL_CAN() macro instead.
 * Polls all CAN lines for incoming messages and dispatches them to handle_can_message().
 * Loops through each CAN line one at a time, reading a single message per line per pass, until either
 * all lines are empty or `can_read_run_time_nanos` nanoseconds have elapsed.
 */
void poll_can(void);
 
/**
 * Sends a CAN message on the specified bus. Will set the error message if an error occurs.
 *
 * @param bus  CAN bus index in ``[0, get_num_can_lines())`` (Teensy: 0–2; host: up to
 *     ``MAX_NUM_CAN_LINES`` successfully opened SocketCAN interfaces)
 * @param extended  Whether the message is an extended (29-bit) frame, or a standard (11-bit) frame
 * @param can_id  CAN ID of the message
 * @param data  Data bytes to send (up to 8)
 * @param len  Number of data bytes (0–8)
 * @return True if the message was sent successfully, false otherwise
 */
bool cansend(uint8_t bus, bool extended, uint32_t can_id, const uint8_t* data, uint8_t len);

// Whether the SDO is a read or write command
enum {
    SDO_READ = 0,
    SDO_WRITE = 1,
};
typedef uint8_t SDOCommand;

/**
 * Sends an OpenCAN SDO request.
 *
 * @param bus  CAN bus index in ``[0, get_num_can_lines())`` (same range as ``cansend``)
 * @param cmd  Whether the SDO is a read or write command
 * @param node_id  Node ID of the device to send the SDO to
 * @param idx  Index of the parameter to send
 * @param subindex  Subindex of the parameter to send
 * @param dtype  Data type of the parameter to send
 * @param value  Value of the parameter to send when writing, or NULL when reading
 * @return True if the SDO was sent successfully, false otherwise
 */
bool opencan_send_sdo(
    uint8_t bus, SDOCommand cmd, uint8_t node_id, uint16_t idx, uint8_t subindex, DataType dtype, const void* value
);

// --- Candump Telemetry ---
#define CANDUMP_TELEMETRY_SIZE (1024)  // ~21 KB with timestamp
#define MAX_CANDUMP_UPDATE_SIZE (sizeof(CandumpTelemetry) * CANDUMP_TELEMETRY_SIZE)

/**
 * Drops all pending candump ring entries. Called on board re-initialize so a
 * reconnect does not drain stale frames (and timestamps) from a prior run.
 */
void clear_candump_telemetry(void);

/**
 * Handles the GET_CANDUMP command.
 *
 * Response payload is a flat packed array of ``CandumpTelemetry`` entries (no count header).
 * An empty ring yields a zero-length payload.
 *
 * @param msg_id The message id of the command
 * @param payload The payload of the command
 */
bool handle_get_candump(uint32_t msg_id, void* payload);


/**
 * Handles a single incoming CAN message.
 *
 * @param bus The bus index the message was received on in ``[0, get_num_can_lines())``
 * @param msg The received CAN message
 */
void handle_can_message(uint8_t bus, const CAN_message_t* msg);

/**
 * Records a received frame in the candump ring. Used by the emulator background CAN poll thread.
 */
void insert_candump_telemetry_rx(uint8_t bus, const CAN_message_t* msg);
