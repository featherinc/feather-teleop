#pragma once

#include <stdint.h>
#include "../commands/all_commands.hpp"
#include "can.hpp"

// --- Ethernet Hard-coded Configs ---
#define IP_ADDRESS_INT(a, b, c, d) \
    ((uint32_t)(a & 0xFF) << 24 | (uint32_t)(b & 0xFF) << 16 | (uint32_t)(c & 0xFF) << 8 | (uint32_t)(d & 0xFF))
#define TEENSY_IP      IP_ADDRESS_INT(192, 168, 10, 2)
#define TEENSY_SUBNET  IP_ADDRESS_INT(255, 255, 255, 0)
#define TEENSY_GATEWAY IP_ADDRESS_INT(192, 168, 10, 1)
#define TEENSY_MAC_ADDR {0x04, 0xE9, 0xE5, 0x00, 0x00, 0x01}
#define TEENSY_HOSTNAME "feather-teensy"  /* mDNS name is `feather-teensy.local` */

// Enough to store the largest possible full response. Can override/MAX with other commands if needed.
#define MAX_RESP_PAYLOAD_SIZE (MAX(MAX_TELEMETRY_UPDATE_SIZE, MAX_CANDUMP_UPDATE_SIZE))
#define MAX_ETH_RESPONSE_SIZE (MAX_RESP_PAYLOAD_SIZE + sizeof(ClientCommandHeader))
#define ETH_WRITE_BUF_SIZE ((uint32_t)((MAX_ETH_RESPONSE_SIZE) * 5u))  // Be able to hold multiple full responses

/**
 * Initializes the Ethernet server.
 */
void initialize_ethernet(void);

/**
 * Handles all ethernet server logic: checking for new client, checking for client disconnect, and handling messages.
 * Only does it as bytes are available, up to `server_run_time_nanos` nanoseconds of time.
 */
void poll_ethernet(void);

/**
 * Writes data to the ethernet write buffer, to be written to the client in a later call to eth_write_buffer_flush().
 * 
 * @param msg_id The message ID
 * @param opcode The opcode
 * @param payload The payload
 * @param payload_size The size of the payload
 * @return 0 if the data was written successfully, -1 if the buffer is full
 */
int eth_buffer_write(uint32_t msg_id, uint32_t opcode, const void* payload, uint32_t payload_size);

/**
 * Inserts data into the ethernet write buffer, wrapping around the end of the buffer if needed.
 * 
 * @param data The data to insert
 * @param size The size of the data to insert
 */
void eth_write_buffer_insert(const void* data, uint32_t size);

/**
 * Returns the remaining space in the ethernet write buffer.
 * 
 * @return The remaining space in the ethernet write buffer
 */
uint32_t eth_write_buffer_remaining_space(void);

/**
 * Attempt to write as much data as possible from the write buffer to the client. Only writes if there is space
 * available so we don't block if the TX queue is full. Flushes the TCP stack so small responses are not held by
 * QNEthernet's send timer (~200-250 ms).
 */
void eth_write_buffer_flush(void);