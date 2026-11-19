#include "em_can.hpp"
#include "../src/comms/can.hpp"
#include "../src/system.hpp"
#include "../src/logging.hpp"
#include "sct_sockets.hpp"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>


#define MAX_CAN_IFACES_DISCOVER 32
#define EM_CAN_BAUD_RATE 1000000u

#ifndef ARPHRD_CAN
#define ARPHRD_CAN 280
#endif

runtime_static uint8_t num_can_lines = 0;
runtime_static bool _testing_can_buf_used = false;

// Checks if the given interface is a CAN interface by checking the type file in /sys/class/net/<iface>/type.
// open/read/close, not fopen: -fanalyzer loses FILE* across fscanf and reports a leak.
static bool is_can_iface(const char* name) {
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/net/%s/type", name);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return false;

    char buf[16];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return false;
    buf[n] = '\0';
    return atoi(buf) == ARPHRD_CAN;
}

// Used to qsort the interface names so order is deterministic
static int can_iface_name_cmp(const void* a, const void* b) {
    return strncmp((const char*)a, (const char*)b, IFNAMSIZ);
}

static pthread_t can_poll_thread;
static volatile sig_atomic_t can_poll_stop = 0;
static bool can_poll_thread_running = false;

static CAN_message_t can_result_to_message(uint8_t bus, const CanSocketResult* result) {
    CAN_message_t msg = {0};
    msg.bus = bus;
    msg.flags.extended = (result->can_id & CAN_EFF_FLAG) != 0;
    msg.flags.remote = (result->can_id & CAN_RTR_FLAG) != 0;
    msg.id = result->can_id & (msg.flags.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    msg.len = result->dlc;
    memcpy(msg.buf, result->data, result->dlc);
    return msg;
}

static bool process_socket_snapshot(uint8_t bus, SocketCircularBufferSnapshot snapshot) {
    if (snapshot.buffer == NULL || snapshot.start >= snapshot.count) return false;

    CanSocketResult* results = (CanSocketResult*)snapshot.buffer;
    for (uint64_t i = snapshot.start; i < snapshot.count; i++) {
        CanSocketResult* result = &results[i % snapshot.size];
        CAN_message_t msg = can_result_to_message(bus, result);
        insert_candump_telemetry_rx(bus, &msg);
        handle_can_message(bus, &msg);
    }
    return true;
}

static void* can_poll_thread_main(void* arg) {
    (void)arg;
    while (!can_poll_stop) {
        if (num_can_lines == 0 || _testing_can_buf_used) {
            usleep(500);
            continue;
        }

        bool any_processed = false;
        for (uint8_t bus = 0; bus < num_can_lines; bus++) {
            SocketCircularBufferSnapshot snapshot = get_socket_result_buffer_snapshot(bus);
            if (process_socket_snapshot(bus, snapshot)) any_processed = true;
        }

        if (!any_processed) usleep(500);
    }
    return NULL;
}

static bool start_can_poll_thread(void) {
    can_poll_stop = 0;
    if (pthread_create(&can_poll_thread, NULL, can_poll_thread_main, NULL) != 0) {
        feather_log("Failed to start CAN background poll thread: %s", strerror(errno));
        return false;
    }
    can_poll_thread_running = true;
    return true;
}

// Names only. closedir happens before any later return so -fanalyzer can see the DIR* released.
static bool discover_can_ifaces(char ifnames[][IFNAMSIZ], size_t max_ifaces, size_t* out_count) {
    DIR* dir = opendir("/sys/class/net");
    if (dir == NULL) {
        feather_log("Failed to open /sys/class/net: %s", strerror(errno));
        return false;
    }

    size_t nifaces = 0;
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL && nifaces < max_ifaces) {
        if (ent->d_name[0] == '.' || !is_can_iface(ent->d_name)) continue;
        strncpy(ifnames[nifaces], ent->d_name, IFNAMSIZ - 1);
        ifnames[nifaces][IFNAMSIZ - 1] = '\0';
        nifaces++;
    }
    closedir(dir);
    *out_count = nifaces;
    return true;
}

bool initialize_can(void) {
    // All linux CAN interfaces exist as directories in /sys/class/net (along with other network interfaces).
    char ifnames[MAX_CAN_IFACES_DISCOVER][IFNAMSIZ];
    size_t nifaces = 0;
    if (!discover_can_ifaces(ifnames, MAX_CAN_IFACES_DISCOVER, &nifaces)) return false;

    // Sort interfaces by names so order is deterministic
    qsort(ifnames, nifaces, IFNAMSIZ, can_iface_name_cmp);

    // Open a RAW socket for each CAN interface.
    for (size_t i = 0; i < nifaces; i++) {
        const char* ifname = ifnames[i];
        if (num_can_lines >= MAX_NUM_CAN_LINES) {
            feather_log("Skipping CAN interface %s: already at max %u lines", ifname, MAX_NUM_CAN_LINES);
            continue;
        }

        if (!add_endpoint(num_can_lines, ifname)) {
            feather_log("Failed to add endpoint %s to CAN bus %u", ifname, num_can_lines);
            return false;
        }

        feather_log("Initialized CAN bus %u on interface %s", num_can_lines, ifname);
        num_can_lines++;
    }

    if (num_can_lines == 0) {
        feather_log("No CAN interfaces found");
        return false;
    }

    if (!start_can_poll_thread()) {
        feather_log("Failed to start CAN background poll thread");
        num_can_lines = 0;
        return false;
    }

    feather_log("CAN lines initialized: %u available", num_can_lines);
    return true;
}

uint8_t get_num_can_lines(void) {
    return num_can_lines;
}

// Allows us to override CAN behavior during unit tests
runtime_static uint8_t _testing_can_buf[MAX_NUM_CAN_LINES][sizeof(struct can_frame)] = {0};

int read_physical_can(uint8_t bus, CAN_message_t* msg) {
    (void)bus;
    (void)msg;
    // RX is handled by the background poll thread draining sct socket buffers.
    return 0;
}

int write_physical_can(uint8_t bus, const CAN_message_t* msg) {
    if (bus >= num_can_lines) return 0;

    if (_testing_can_buf_used) {
        struct can_frame frame = {0};
        frame.can_id = msg->flags.extended ? ((msg->id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (msg->id & CAN_SFF_MASK);
        if (msg->flags.remote) frame.can_id |= CAN_RTR_FLAG;
        frame.can_dlc = msg->len;
        memcpy(frame.data, msg->buf, msg->len);
        memcpy(_testing_can_buf[bus], &frame, sizeof(frame));
        return 1;
    }

    if (!sct_socket_is_open(bus)) return 0;

    if (!sct_cansend(bus, msg->flags.extended, msg->id, msg->buf, msg->len)) return 0;
    return 1;
}
