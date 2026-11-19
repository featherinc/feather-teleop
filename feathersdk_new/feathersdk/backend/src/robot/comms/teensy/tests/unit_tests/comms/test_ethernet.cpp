#include "../../../src/logging.cpp"
#include "../../../src/system.cpp"
#include "../../../src/comms/comms.cpp"
#include "../../../src/comms/can.cpp"
#include "../../../src/actuator.cpp"
#include "../../../src/commands/robstride.cpp"
#include "../../../src/commands/ezmotion.cpp"
#include "../../../src/commands/realhand.cpp"
#include "../../../src/commands/soc.cpp"
#include "../../../src/commands/all_commands.cpp"
#include "../../../src/ota_flash/FlashTxx.cpp"
#include "../../../src/ota_flash/FXUtil.cpp"
#include "../../../src/comms/ethernet.cpp"
#include "../../framework.hpp"
#include <fcntl.h>
#include <sys/socket.h>

extern uint8_t eth_read_buf[];
extern uint32_t eth_read_buf_len;
extern uint32_t eth_read_needed_bytes;
extern ClientCommandHeader* eth_read_header;

extern void clear_eth_read_buffer_state(void);
extern void send_error_response(uint32_t msg_id, const char* err_msg);
extern void send_success_response(uint32_t msg_id);
extern void stop_connection(void);
extern void handle_command(ClientCommandHeader* header);

/** Write ``header`` + optional payload into ``eth_read_buf`` and set read state. */
static ClientCommandHeader* seed_read_command(
    uint32_t msg_id, uint32_t opcode, uint32_t payload_size, const void* payload, uint32_t read_len
) {
    ClientCommandHeader* hdr = (ClientCommandHeader*)eth_read_buf;
    hdr->msg_id = msg_id;
    hdr->opcode = (TeensyCommandOpcode)opcode;
    hdr->payload_size = payload_size;
    if (payload != NULL && payload_size > 0) {
        memcpy(eth_read_buf + sizeof(ClientCommandHeader), payload, payload_size);
    }
    eth_read_buf_len = read_len;
    eth_read_header = hdr;
    return hdr;
}

/**
 * In-memory "StringIO" capture for ethernet_client writes.
 *
 * socketpair(2) creates two connected UNIX-domain socket fds that behave like a
 * bidirectional pipe. We assign one end to ethernet_client.fd so flush uses the
 * real emulator write path (send(2) on that fd). Bytes written by the firmware
 * can be read from the peer end into a user buffer — same idea as Python's
 * io.BytesIO or Java's ByteArrayOutputStream, but wired through the kernel's
 * socket layer so we exercise the actual fd-based client code.
 */
typedef struct {
    int write_fd;
    int read_fd;
    uint8_t buf[ETH_WRITE_BUF_SIZE];
    size_t len;
} eth_capture_t;

static bool eth_capture_open(eth_capture_t* cap) {
    int fds[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) return false;
    cap->write_fd = fds[0];
    cap->read_fd = fds[1];
    cap->len = 0;
    ethernet_client.fd = cap->write_fd;
    return true;
}

static test_result_t eth_capture_drain(eth_capture_t* cap) {
    cap->len = 0;
    int saved_flags = fcntl(cap->read_fd, F_GETFL, 0);
    TEST_ASSERT(saved_flags >= 0, "fcntl F_GETFL failed");
    TEST_ASSERT(fcntl(cap->read_fd, F_SETFL, saved_flags | O_NONBLOCK) == 0, "fcntl O_NONBLOCK failed");

    while (cap->len < sizeof(cap->buf)) {
        ssize_t n = read(cap->read_fd, cap->buf + cap->len, sizeof(cap->buf) - cap->len);
        if (n > 0) {
            cap->len += (size_t)n;
            continue;
        }
        if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) break;
        TEST_ASSERT(0, "read on capture fd failed: %s", strerror(errno));
    }

    TEST_ASSERT(fcntl(cap->read_fd, F_SETFL, saved_flags) == 0, "fcntl restore failed");
    return TEST_SUCCESS;
}

static void eth_capture_close(eth_capture_t* cap) {
    if (cap->write_fd >= 0) close(cap->write_fd);
    if (cap->read_fd >= 0) close(cap->read_fd);
    cap->write_fd = cap->read_fd = -1;
    ethernet_client.fd = -1;
}

/** Pack a command header (+ optional payload) into ``out``; returns total bytes. */
static uint32_t pack_command(
    uint8_t* out, uint32_t msg_id, uint32_t opcode, const void* payload, uint32_t payload_size
) {
    ClientCommandHeader hdr = {
        .msg_id = msg_id, .opcode = (TeensyCommandOpcode)opcode, .payload_size = payload_size
    };
    memcpy(out, &hdr, sizeof(hdr));
    if (payload_size > 0 && payload != NULL) memcpy(out + sizeof(hdr), payload, payload_size);
    return (uint32_t)sizeof(hdr) + payload_size;
}

static test_result_t peer_send(eth_capture_t* cap, const void* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(cap->read_fd, (const uint8_t*)data + sent, len - sent);
        TEST_ASSERT(n > 0, "peer write failed: %s", strerror(errno));
        sent += (size_t)n;
    }
    return TEST_SUCCESS;
}

/** Assert drained peer bytes match a header + payload response. */
static test_result_t assert_peer_response(
    eth_capture_t* cap, uint32_t msg_id, uint32_t opcode, const void* payload, uint32_t payload_size
) {
    TEST_ASSERT_STACKED(eth_capture_drain(cap), "drain peer");
    uint32_t expect_len = (uint32_t)sizeof(ClientCommandHeader) + payload_size;
    TEST_ASSERT(cap->len == expect_len, "peer got %zu bytes, expected %u", cap->len, expect_len);

    ClientCommandHeader hdr;
    memcpy(&hdr, cap->buf, sizeof(hdr));
    TEST_ASSERT(hdr.msg_id == msg_id && hdr.opcode == opcode && hdr.payload_size == payload_size,
        "response hdr mismatch");
    if (payload_size > 0) {
        TEST_ASSERT(memcmp(cap->buf + sizeof(hdr), payload, payload_size) == 0, "response payload mismatch");
    }
    return TEST_SUCCESS;
}

/** stop_connection() closes ethernet_client.fd; drop our copy so we don't double-close. */
static void eth_capture_note_stopped(eth_capture_t* cap) {
    cap->write_fd = -1;
}

static int eth_read_always_fail(uint8_t* buf, size_t size) {
    (void)buf;
    (void)size;
    return -1;
}

/** Assert the unread portion of the write ring matches ``expected``. */
static test_result_t assert_eth_write_ring_eq(const uint8_t* expected, uint32_t size) {
    uint32_t used = (eth_write_buf_end - eth_write_buf_start + ETH_WRITE_BUF_SIZE) % ETH_WRITE_BUF_SIZE;
    TEST_ASSERT(used == size, "ring used %u, expected %u", used, size);

    uint8_t got[ETH_WRITE_BUF_SIZE];
    if (eth_write_buf_end >= eth_write_buf_start) {
        memcpy(got, eth_write_buf + eth_write_buf_start, size);
    } else {
        uint32_t first = ETH_WRITE_BUF_SIZE - eth_write_buf_start;
        memcpy(got, eth_write_buf + eth_write_buf_start, first);
        memcpy(got + first, eth_write_buf, eth_write_buf_end);
    }
    TEST_ASSERT(memcmp(got, expected, size) == 0, "ring content mismatch");
    return TEST_SUCCESS;
}


DECLARE_TEST(test_eth_write_buffer_insert_and_remaining_space) {
    eth_write_buf_clear();
    TEST_ASSERT(eth_write_buffer_remaining_space() == ETH_WRITE_BUF_SIZE - 1u, "empty buf");

    const uint8_t chunk[] = {1, 2, 3};
    eth_write_buffer_insert(chunk, 3);
    TEST_ASSERT(eth_write_buf_start == 0 && eth_write_buf_end == 3, "end after linear insert");
    TEST_ASSERT(eth_write_buffer_remaining_space() == ETH_WRITE_BUF_SIZE - 1u - 3, "remaining after insert");
    TEST_ASSERT_STACKED(assert_eth_write_ring_eq(chunk, 3), "linear insert");

    // Wrap insert across the physical end of the ring (empty buf with indices near the end).
    eth_write_buf_clear();
    eth_write_buf_start = eth_write_buf_end = ETH_WRITE_BUF_SIZE - 2u;
    const uint8_t wrap[] = {0xA, 0xB, 0xC, 0xD};
    eth_write_buffer_insert(wrap, 4);
    TEST_ASSERT(eth_write_buf_end == 2, "end wraps to 2");
    TEST_ASSERT(eth_write_buf[ETH_WRITE_BUF_SIZE - 2] == 0xA && eth_write_buf[ETH_WRITE_BUF_SIZE - 1] == 0xB,
        "tail chunk at physical end");
    TEST_ASSERT(eth_write_buf[0] == 0xC && eth_write_buf[1] == 0xD, "head chunk after wrap");
    TEST_ASSERT_STACKED(assert_eth_write_ring_eq(wrap, 4), "wrap insert");

    // remaining_space respects consumed data (start offset).
    eth_write_buf_start = 5;
    eth_write_buf_end = 10;
    TEST_ASSERT(eth_write_buffer_remaining_space() == ETH_WRITE_BUF_SIZE - 1u - 5u, "used with start offset");

    eth_write_buf_start = 10;
    eth_write_buf_end = 5;
    TEST_ASSERT(eth_write_buffer_remaining_space() == ETH_WRITE_BUF_SIZE - 1u -
        ((ETH_WRITE_BUF_SIZE - 10u) + 5u), "used when ring wraps");

    return TEST_SUCCESS;
}


DECLARE_TEST(test_eth_buffer_write) {
    eth_write_buf_clear();
    TEST_ASSERT(eth_buffer_write(7, TEENSY_COMMAND_OPCODE_RESP_ECHO, "hi", 2) == 0, "write with payload");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(7, TEENSY_COMMAND_OPCODE_RESP_ECHO, "hi", 2), "payload in buf");

    eth_write_buf_clear();
    TEST_ASSERT(eth_buffer_write(8, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0) == 0, "empty payload write");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(8, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0), "empty payload in buf");

    // Buffer full.
    eth_write_buf_clear();
    eth_write_buf_leave_space(sizeof(ClientCommandHeader) + 3);
    TEST_ASSERT(eth_buffer_write(1, TEENSY_COMMAND_OPCODE_RESP_ERROR, "abcd", 4) == -1, "full buf");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Ethernet writebuf full");

    // NULL payload with non-zero size.
    eth_write_buf_clear();
    TEST_ASSERT(eth_buffer_write(2, TEENSY_COMMAND_OPCODE_RESP_ERROR, NULL, 5) == -1, "null payload");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Payload NULL");

    return TEST_SUCCESS;
}


DECLARE_TEST(test_clear_eth_read_buffer_state) {
    eth_read_buf_len = 100;
    eth_read_needed_bytes = 50;
    eth_read_header = (ClientCommandHeader*)eth_read_buf;
    set_local_error_msg("leftover error");

    clear_eth_read_buffer_state();

    TEST_ASSERT(eth_read_buf_len == 0, "len cleared");
    TEST_ASSERT(eth_read_needed_bytes == sizeof(ClientCommandHeader), "needed reset");
    TEST_ASSERT(eth_read_header == NULL, "header cleared");
    TEST_ASSERT(get_local_error_msg() == NULL, "local err cleared");

    return TEST_SUCCESS;
}


DECLARE_TEST(test_send_error_and_success_response) {
    eth_write_buf_clear();
    send_success_response(42);
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(42, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0), "success response");

    eth_write_buf_clear();
    send_error_response(99, "bad cmd");
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(99, TEENSY_COMMAND_OPCODE_RESP_ERROR, "bad cmd", 7), "error response");

    return TEST_SUCCESS;
}


DECLARE_TEST(test_eth_write_buffer_flush) {
    eth_capture_t cap = {.write_fd = -1, .read_fd = -1};

    // Empty ring: flush is a no-op.
    eth_write_buf_clear();
    TEST_ASSERT(eth_capture_open(&cap), "socketpair failed");
    eth_write_buffer_flush();
    TEST_ASSERT_STACKED(eth_capture_drain(&cap), "drain empty");
    TEST_ASSERT(cap.len == 0, "empty buf must not write");
    eth_capture_close(&cap);

    // Linear ring: one contiguous segment drains fully to the capture fd.
    TEST_ASSERT(eth_capture_open(&cap), "socketpair failed");
    eth_write_buf_clear();
    const uint8_t linear[] = {0x10, 0x20, 0x30};
    eth_write_buffer_insert(linear, 3);
    eth_write_buffer_flush();
    TEST_ASSERT_STACKED(eth_capture_drain(&cap), "drain linear");
    TEST_ASSERT(cap.len == 3 && memcmp(cap.buf, linear, 3) == 0, "linear flush output");
    TEST_ASSERT(eth_write_buf_start == eth_write_buf_end, "linear ring drained");
    eth_capture_close(&cap);

    // Wrapped ring: flush must stitch two physically-contiguous segments in order.
    TEST_ASSERT(eth_capture_open(&cap), "socketpair failed");
    eth_write_buf_clear();
    eth_write_buf_start = eth_write_buf_end = ETH_WRITE_BUF_SIZE - 2u;
    const uint8_t wrap[] = {0xA, 0xB, 0xC, 0xD};
    eth_write_buffer_insert(wrap, 4);
    eth_write_buffer_flush();
    TEST_ASSERT_STACKED(eth_capture_drain(&cap), "drain wrap");
    TEST_ASSERT(cap.len == 4 && memcmp(cap.buf, wrap, 4) == 0, "wrapped flush output");
    TEST_ASSERT(eth_write_buf_start == eth_write_buf_end, "wrapped ring drained");
    eth_capture_close(&cap);

    // No valid client fd: data stays queued (availableForWrite/write both bail out).
    eth_write_buf_clear();
    eth_write_buffer_insert("keep", 4);
    ethernet_client.fd = -1;
    eth_write_buffer_flush();
    TEST_ASSERT(eth_write_buf_start != eth_write_buf_end, "must retain data with no client");
    TEST_ASSERT_STACKED(assert_eth_write_ring_eq((const uint8_t*)"keep", 4), "unchanged ring");

    return TEST_SUCCESS;
}


DECLARE_TEST(test_handle_command) {
    const uint32_t msg_id = 55;
    uint8_t echo_payload[ECHO_PAYLOAD_SIZE];
    for (uint32_t i = 0; i < ECHO_PAYLOAD_SIZE; i++) echo_payload[i] = (uint8_t)(0xA0u + i);

    // Echo: handler writes its own response; no default success/error wrapper.
    eth_write_buf_clear();
    ClientCommandHeader* hdr = seed_read_command(
        msg_id, TEENSY_COMMAND_OPCODE_CMD_ECHO, ECHO_PAYLOAD_SIZE, echo_payload,
        (uint32_t)(sizeof(ClientCommandHeader) + ECHO_PAYLOAD_SIZE)
    );
    handle_command(hdr);
    TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_ECHO, echo_payload, ECHO_PAYLOAD_SIZE), "echo response");
    TEST_ASSERT(eth_read_buf_len == 0 && eth_read_header == NULL, "read state cleared after echo");

    // Unknown opcode: error response, no handler call.
    eth_write_buf_clear();
    hdr = seed_read_command(msg_id, NUM_COMMANDS + 50, 0, NULL, sizeof(ClientCommandHeader));
    handle_command(hdr);
    {
        char expect_err[64];
        snprintf(expect_err, sizeof(expect_err), "Invalid command opcode: %u", (uint32_t)(NUM_COMMANDS + 50));
        TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_ERROR, expect_err, (uint32_t)strlen(expect_err)),
            "invalid opcode");
    }
    TEST_ASSERT(eth_read_buf_len == 0 && eth_read_header == NULL, "read state cleared after invalid opcode");

    // Size mismatch: error response, echo handler not invoked.
    eth_write_buf_clear();
    hdr = seed_read_command(
        msg_id, TEENSY_COMMAND_OPCODE_CMD_ECHO, ECHO_PAYLOAD_SIZE, echo_payload,
        (uint32_t)(sizeof(ClientCommandHeader) + ECHO_PAYLOAD_SIZE - 1)
    );
    handle_command(hdr);
    {
        char expect_err[128];
        snprintf(expect_err, sizeof(expect_err),
            "Payload size for command TEENSY_COMMAND_OPCODE_CMD_ECHO does not match expected size. Got %u, expected %zu.",
            (uint32_t)(sizeof(ClientCommandHeader) + ECHO_PAYLOAD_SIZE - 1),
            sizeof(ClientCommandHeader) + ECHO_PAYLOAD_SIZE);
        TEST_ASSERT_STACKED(check_eth_write_buf_eq(msg_id, TEENSY_COMMAND_OPCODE_RESP_ERROR, expect_err, (uint32_t)strlen(expect_err)),
            "size mismatch error");
    }
    TEST_ASSERT(eth_read_buf_len == 0 && eth_read_header == NULL, "read state cleared after size mismatch");

    return TEST_SUCCESS;
}


DECLARE_TEST(test_stop_connection) {
    int fds[2];
    TEST_ASSERT(socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0, "socketpair failed");

    ethernet_client.fd = fds[0];
    eth_write_buf_clear();
    eth_write_buffer_insert("xy", 2);
    eth_read_buf_len = 20;
    eth_read_needed_bytes = 0;
    eth_read_header = (ClientCommandHeader*)eth_read_buf;
    set_local_error_msg("stale");

    stop_connection();

    TEST_ASSERT(ethernet_client.fd == -1, "client stopped");
    TEST_ASSERT(eth_write_buf_start == 0 && eth_write_buf_end == 0, "write buf cleared");
    TEST_ASSERT(eth_read_buf_len == 0 && eth_read_header == NULL, "read state cleared");
    TEST_ASSERT(get_local_error_msg() == NULL, "local err cleared");

    close(fds[1]);
    return TEST_SUCCESS;
}


DECLARE_TEST(test_poll_ethernet) {
    // Give the read loop plenty of wall time; default 100us is tight under load.
    CommsConfig cfg = DEFAULT_COMMS_CONFIG;
    cfg.server_run_time_nanos = 10 * 1000 * 1000;  // 10 ms
    set_comms_config(&cfg);

    eth_capture_t cap = {.write_fd = -1, .read_fd = -1};
    uint8_t cmd[sizeof(ClientCommandHeader) + ECHO_PAYLOAD_SIZE];
    uint8_t echo_payload[ECHO_PAYLOAD_SIZE];
    for (uint32_t i = 0; i < ECHO_PAYLOAD_SIZE; i++) echo_payload[i] = (uint8_t)(0xB0u + i);
    const uint32_t msg_id = 77;

    // --- No client: early return ---
    ethernet_client.fd = -1;
    ethernet_server.listen_fd = -1;
    clear_eth_read_buffer_state();
    eth_write_buf_clear();
    poll_ethernet();
    TEST_ASSERT(ethernet_client.fd == -1, "no client must stay disconnected");

    // --- Connected + flush of pending write data (no inbound bytes) ---
    TEST_ASSERT(eth_capture_open(&cap), "socketpair failed");
    clear_eth_read_buffer_state();
    eth_write_buf_clear();
    eth_write_buffer_insert("zz", 2);
    poll_ethernet();
    TEST_ASSERT_STACKED(eth_capture_drain(&cap), "drain flush-only");
    TEST_ASSERT(cap.len == 2 && memcmp(cap.buf, "zz", 2) == 0, "pending write flushed");
    TEST_ASSERT(eth_write_buf_start == eth_write_buf_end, "write ring drained");

    // --- Full echo command in one poll ---
    clear_eth_read_buffer_state();
    eth_write_buf_clear();
    uint32_t cmd_len = pack_command(cmd, msg_id, TEENSY_COMMAND_OPCODE_CMD_ECHO, echo_payload, ECHO_PAYLOAD_SIZE);
    TEST_ASSERT_STACKED(peer_send(&cap, cmd, cmd_len), "send echo");
    poll_ethernet();
    TEST_ASSERT_STACKED(assert_peer_response(&cap, msg_id, TEENSY_COMMAND_OPCODE_RESP_ECHO, echo_payload, ECHO_PAYLOAD_SIZE),
        "echo response");
    TEST_ASSERT(eth_read_buf_len == 0 && eth_read_header == NULL, "read state cleared after echo");

    // --- Partial header, then remainder + payload across polls ---
    clear_eth_read_buffer_state();
    eth_write_buf_clear();
    TEST_ASSERT_STACKED(peer_send(&cap, cmd, 5), "send partial hdr");
    poll_ethernet();
    TEST_ASSERT(eth_read_buf_len == 5 && eth_read_header == NULL, "waiting for full header");
    TEST_ASSERT_STACKED(eth_capture_drain(&cap), "no response yet");
    TEST_ASSERT(cap.len == 0, "partial must not respond");

    TEST_ASSERT_STACKED(peer_send(&cap, cmd + 5, cmd_len - 5), "send remainder");
    poll_ethernet();
    TEST_ASSERT_STACKED(assert_peer_response(&cap, msg_id, TEENSY_COMMAND_OPCODE_RESP_ECHO, echo_payload, ECHO_PAYLOAD_SIZE),
        "echo after partial");

    // --- Oversized payload: error response + stop ---
    clear_eth_read_buffer_state();
    eth_write_buf_clear();
    ClientCommandHeader oversized = {
        .msg_id = msg_id, .opcode = TEENSY_COMMAND_OPCODE_CMD_ECHO, .payload_size = 1000  // > MAX_COMMAND_SIZE - hdr
    };
    TEST_ASSERT_STACKED(peer_send(&cap, &oversized, sizeof(oversized)), "send oversized hdr");
    poll_ethernet();
    {
        char expect_err[64];
        snprintf(expect_err, sizeof(expect_err), "Command payload too large (%u bytes).", 1000u);
        TEST_ASSERT_STACKED(assert_peer_response(&cap, msg_id, TEENSY_COMMAND_OPCODE_RESP_ERROR, expect_err, (uint32_t)strlen(expect_err)),
            "oversized error response");
    }
    TEST_ASSERT(ethernet_client.fd == -1, "oversized must stop connection");
    eth_capture_note_stopped(&cap);
    eth_capture_close(&cap);

    // --- Client disconnect ---
    TEST_ASSERT(eth_capture_open(&cap), "socketpair failed");
    clear_eth_read_buffer_state();
    eth_write_buf_clear();
    close(cap.read_fd);
    cap.read_fd = -1;
    poll_ethernet();
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Client-side disconnect");
    TEST_ASSERT(ethernet_client.fd == -1, "disconnect must stop connection");
    eth_capture_note_stopped(&cap);
    eth_capture_close(&cap);

    // --- Negative read: kill connection ---
    TEST_ASSERT(eth_capture_open(&cap), "socketpair failed");
    clear_eth_read_buffer_state();
    eth_write_buf_clear();
    uint8_t one = 0;
    TEST_ASSERT_STACKED(peer_send(&cap, &one, 1), "seed byte");
    int (*saved_read)(uint8_t*, size_t) = ethernet_client.read;
    ethernet_client.read = eth_read_always_fail;
    poll_ethernet();
    ethernet_client.read = saved_read;
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Got a negative value for ethernet_client.read()");
    TEST_ASSERT(ethernet_client.fd == -1, "negative read must stop connection");
    eth_capture_note_stopped(&cap);
    eth_capture_close(&cap);

    CommsConfig defaults = DEFAULT_COMMS_CONFIG;
    set_comms_config(&defaults);
    return TEST_SUCCESS;
}


RUN_TESTS();
