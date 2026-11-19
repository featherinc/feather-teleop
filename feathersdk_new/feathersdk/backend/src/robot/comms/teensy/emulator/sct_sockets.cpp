#include "sct_sockets.hpp"
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
#include <atomic>
#include <poll.h>
#include <errno.h>
#include "../src/logging.hpp"
#include "../src/system.hpp"

#define POLL_TIMEOUT_MS 1

#define MAX_SOCKETS 8
#define CAN_RECV_BUF_SIZE (1024 * 8)

// Holds all currently open sockets, indexed by CAN bus number.
SocketEntry sockets[MAX_SOCKETS] = {0};

typedef struct SocketCircularBuffer {
    void *buffer;
    uint64_t start;  // Index of first result that hasn't been processed by python yet. Only modified by python thread.
    // Number of results that have ever been written to buffer. Both C and python threads use this. Only increments.
    // Make sure the Python thread reads this atomically *before* any processing, and C thread writes *after* completion
    std::atomic<uint64_t> count;
} SocketCircularBuffer;

// Buffers to quickly store results from the polling loop to be processed by the handler.
SocketCircularBuffer socket_bufs[MAX_SOCKETS] = {0};

// Adds a new socket to the sockets array. Returns true on success, false on error.
// Assumes you have acquired the sockets mutex already
static bool insert_socket(int sockfd, int sock_idx, const char *name) {
    ASSERT_GLOBAL(sock_idx >= 0 && sock_idx < MAX_SOCKETS, false, "Invalid socket index: %d", sock_idx);
    ASSERT_GLOBAL(sockets[sock_idx].name[0] == '\0', false, "Socket already exists: %d", sock_idx);

    void *buffer = malloc(sizeof(CanSocketResult) * CAN_RECV_BUF_SIZE);
    ASSERT_GLOBAL(buffer != NULL, false, "Failed to allocate memory for buffer: %s: %s", name, strerror(errno));

    sockets[sock_idx].stop_polling = false;
    sockets[sock_idx].sockfd = sockfd;
    socket_bufs[sock_idx].buffer = buffer;
    socket_bufs[sock_idx].start = 0;
    socket_bufs[sock_idx].count = 0;
    strncpy(sockets[sock_idx].name, name, SOCKET_NAME_MAX_LEN - 1);
    sockets[sock_idx].name[SOCKET_NAME_MAX_LEN - 1] = '\0';

    return true;
}

// Creates a new CAN socket and returns the index in the sockets array.
// Assumes you have acquired the sockets mutex already
// Returns true on success, false on error
static bool create_can_socket(int sock_idx, const char *ifname) {
    for (int i = 0; i < MAX_SOCKETS; i++) {
        if (sockets[i].name[0] != '\0') {
            ASSERT_GLOBAL(strcmp(sockets[i].name, ifname) != 0, false, "Socket already exists: \"%s\"", ifname);
        }
    }

    ASSERT_GLOBAL(strnlen(ifname, IFNAMSIZ) < IFNAMSIZ, false, "CAN interface name too long: %s", ifname);

    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    ASSERT_GLOBAL(sockfd >= 0, false, "Failed to create CAN socket: %s", ifname);

    if (strncmp(ifname, "vcan", 4) == 0) {
        int loopback = 0;  // 1 = loopback, 0 = no loopback
        if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &loopback, sizeof(loopback)) < 0) {
            close(sockfd);
            ASSERT_GLOBAL(false, false, "Failed to set loopback mode for CAN socket: %s", ifname);
        }
    }
    
    // Get the system-level index of the CAN interface
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        close(sockfd);
        ASSERT_GLOBAL(false, false, "Failed ioctl on CAN socket: %s", ifname);
    }

    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sockfd);
        ASSERT_GLOBAL(false, false, "Failed to bind CAN socket: %s", ifname);
    }

    if (!insert_socket(sockfd, sock_idx, ifname)) {
        close(sockfd);
        return false;
    }
    return true;
}

// Reads a CAN message from the socket and stores the result in the recv_buf.
static void handle_can_message(int sock_idx) {
    struct can_frame frame;
    ssize_t nbytes = read(sockets[sock_idx].sockfd, &frame, sizeof(frame));

    if (nbytes < 0) {
        if (ERRNO_IS_EAGAIN_OR_EWOULDBLOCK(errno)) return;
        ASSERT_GLOBAL(false, ;, "Failed to read CAN frame: %s", sockets[sock_idx].name);
    }
    ASSERT_GLOBAL(nbytes >= (int)sizeof(struct can_frame), ;, "Incomplete CAN frame: %s", sockets[sock_idx].name);
    ASSERT_GLOBAL(frame.can_dlc <= 8, ;, "CAN DLC %d exceeds max %d: %s", frame.can_dlc, 8, sockets[sock_idx].name);

    CanSocketResult *buf = (CanSocketResult *)socket_bufs[sock_idx].buffer;
    uint64_t count = socket_bufs[sock_idx].count.load(std::memory_order_relaxed);
    int buf_idx = count % CAN_RECV_BUF_SIZE;

    buf[buf_idx].timestamp = currtime_ns();
    buf[buf_idx].direction = 0;  // 0 = received, 1 = sent (sent is logged from Python)
    buf[buf_idx].can_id = frame.can_id;
    buf[buf_idx].dlc = frame.can_dlc;
    memcpy(buf[buf_idx].data, frame.data, frame.can_dlc);
    if (frame.can_dlc < 8) memset(buf[buf_idx].data + frame.can_dlc, 0, 8 - frame.can_dlc);
    socket_bufs[sock_idx].count.fetch_add(1, std::memory_order_release);
}

// Background thread to poll for new messages, then store them into the recv_buf
static void* bg_poll_thread(void* arg) {
    int sock_idx = (int)(intptr_t)arg;

    struct pollfd poll_fds[1];
    poll_fds[0].fd = sockets[sock_idx].sockfd;
    poll_fds[0].events = POLLIN;

    while (!sockets[sock_idx].stop_polling) {
        // Poll for new data. Return after POLL_TIMEOUT_MS milliseconds if no data to check continue polling
        int poll_result = poll(poll_fds, 1, POLL_TIMEOUT_MS);
        if (poll_result < 0) {
            set_global_error_msg("Poll error: %s\n", strerror(errno));
            continue;
        }

        if (!(poll_fds[0].revents & POLLIN)) continue;  // No data available, continue polling

        // Handle the message from the socket
        handle_can_message(sock_idx);
    }
    return NULL;
}

bool sct_socket_is_open(int sock_idx) {
    if (sock_idx < 0 || sock_idx >= MAX_SOCKETS) return false;
    return sockets[sock_idx].name[0] != '\0' && sockets[sock_idx].sockfd > 0;
}

bool add_endpoint(int sock_idx, const char *ifname) {
    ASSERT_GLOBAL(create_can_socket(sock_idx, ifname), false, "Failed to create socket: %s", ifname);

    // Start the background polling thread
    int result = pthread_create(&sockets[sock_idx].polling_thread, NULL, bg_poll_thread, (void*)(intptr_t)sock_idx);
    ASSERT_GLOBAL(result == 0, false, "Failed to create background polling thread for socket: \"%s\"", ifname);

    return true;
}

SocketCircularBufferSnapshot get_socket_result_buffer_snapshot(int sock_idx) {
    // Update the start index to the next unprocessed result
    uint64_t count = socket_bufs[sock_idx].count.load(std::memory_order_acquire);
    uint64_t start = socket_bufs[sock_idx].start;
    socket_bufs[sock_idx].start = count;

    return (SocketCircularBufferSnapshot){
        .buffer = socket_bufs[sock_idx].buffer, .start = start, .count = count, .size = CAN_RECV_BUF_SIZE
    };
}

bool sct_cansend(int sock_idx, bool extended, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    ASSERT_LOCAL(sock_idx >= 0 && sock_idx < MAX_SOCKETS, false, "Invalid socket index: %d", sock_idx);

    ASSERT_LOCAL(dlc <= 8, false, "CAN data length too long: %d", dlc);
    if (extended) {
        ASSERT_LOCAL(can_id <= 0x1FFFFFFF, false, "Extended CAN ID too large: %d", can_id);
    } else {
        ASSERT_LOCAL(can_id <= 0x7FF, false, "Standard CAN ID too large: %d", can_id);
    }
    
    struct can_frame frame = {0};
    frame.can_id = extended ? (can_id | CAN_EFF_FLAG) : can_id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);

    int n_attempts = 15;
    for (int i = 0; i < n_attempts; i++) {
        errno = 0;  // Clear the errno before writing
        ssize_t nbytes = write(sockets[sock_idx].sockfd, &frame, sizeof(struct can_frame));
        if (nbytes == sizeof(struct can_frame)) return true;  // Success, return now

        if (errno == ENOBUFS && i < n_attempts - 1) {
            usleep(2000);  // Sleep 2ms between attempts
            continue;
        }
        ASSERT_LOCAL(false, false, "Failed to write CAN frame to endpoint `%s` after %d attempts: %s", 
              sockets[sock_idx].name, i + 1, strerror(errno));
    }

    return false;
}
