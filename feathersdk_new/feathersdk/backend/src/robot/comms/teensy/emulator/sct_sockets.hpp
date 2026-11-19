// Handles the creation and management of sockets
#pragma once

#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <pthread.h>
#include <poll.h>


#define SOCKET_NAME_MAX_LEN 32
#define SCT_CAN_DATA_SIZE 8

typedef struct __attribute__((packed)) CanSocketResult {
    uint64_t timestamp;
    uint8_t direction;
    uint8_t dlc;
    uint32_t can_id;
    uint8_t data[SCT_CAN_DATA_SIZE];
} CanSocketResult;

typedef struct SocketEntry {
    bool stop_polling;
    pthread_t polling_thread;

    int sockfd;
    char name[SOCKET_NAME_MAX_LEN];
} SocketEntry;

/**
 * @brief Adds a new CAN interface to the sockets array.
 * 
 * @param sock_idx The index of the socket to add the endpoint to
 * @param ifname The name of the interface to add (e.g. "can0", "can1", etc.)
 * @return True on success, false on error
 */
bool add_endpoint(int sock_idx, const char *ifname);

/** Returns true if ``sock_idx`` has an open, bound CAN socket (via ``add_endpoint``). */
bool sct_socket_is_open(int sock_idx);

// Snapshot of the circular buffer for a socket. Used to pass to the python thread.
typedef struct SocketCircularBufferSnapshot {
    void *buffer;    // Pointer to the buffer
    uint64_t start;  // Index of first result that hasn't been processed by python yet. Only modified by python thread.
    uint64_t count;  // Last atomic count value
    uint32_t size;   // Size of the buffer
} SocketCircularBufferSnapshot;

/**
 * @brief Gets the result buffer for a socket.
 * 
 * @param sock_idx The index of the socket to get the buffer for
 * @return A snapshot of the circular buffer
 */
SocketCircularBufferSnapshot get_socket_result_buffer_snapshot(int sock_idx);

/**
 * @brief Sends a CAN message on the specified interface.
 * 
 * Creates or retrieves a CAN socket for the interface and sends a CAN frame with the specified
 * parameters. Performs exponential backoff retry on buffer full conditions.
 * 
 * @param sock_idx Index of the socket to send the message on
 * @param extended Whether to use extended CAN ID format (29-bit) or standard (11-bit)
 * @param can_id CAN identifier (max 0x7FF for standard, 0x1FFFFFFF for extended)
 * @param data Pointer to data buffer containing the CAN payload
 * @param dlc Data length code (number of bytes to send, max 8)
 * @return True on success, false on error
 */
bool sct_cansend(int sock_idx, bool extended, uint32_t can_id, const uint8_t* data, uint8_t dlc);
