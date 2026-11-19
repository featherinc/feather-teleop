#include "ethernet.hpp"
#include "comms.hpp"
#include <string.h>
#include "../commands/all_commands.hpp"

#ifdef COMPILE_FOR_TEENSY
// Single shared instances (see externs in comms.h). Emulator defines its own in em_ethernet.c.
EthernetServer ethernet_server(IP_PORT);
EthernetClient ethernet_client;
#endif


/////////////////////////////
// Ethernet Initialization //
/////////////////////////////


// --- Current Ethernet Buffer State ---
#define MAX_COMMAND_SIZE 512  // TODO: Make this dynamic based on the largest command size
runtime_static uint8_t eth_read_buf[MAX_COMMAND_SIZE] = {0};
runtime_static uint32_t eth_read_buf_len = 0;
runtime_static uint32_t eth_read_needed_bytes = sizeof(ClientCommandHeader);
runtime_static ClientCommandHeader* eth_read_header = NULL;


#ifdef COMPILE_FOR_TEENSY
// True once DHCP has given us an address and the command server is listening. Until then, poll_ethernet() only keeps
// the network stack (DHCP) running, so the main loop is never blocked waiting for a lease.
static bool eth_server_started = false;
#endif


/**
 * Starts the command server (and mDNS on the Teensy) and logs our address. Called once we have an IP address.
 */
static void start_ethernet_server(void) {
#if defined(COMPILE_FOR_TEENSY) && LWIP_MDNS_RESPONDER
    if (!MDNS.begin(TEENSY_HOSTNAME) || !MDNS.addService("_feather", "_tcp", IP_PORT)) {
        feather_log("mDNS announcement failed");
    }
#endif
    ethernet_server.begin();

    uint8_t ip[4];
#ifdef COMPILE_FOR_TEENSY
    IPAddress local_ip = Ethernet.localIP();
    ip[0] = local_ip[0];
    ip[1] = local_ip[1];
    ip[2] = local_ip[2];
    ip[3] = local_ip[3];
#else
    uint32_t local_ip = Ethernet._local_ip.addr;
    ip[0] = (uint8_t)(local_ip >> 24);
    ip[1] = (uint8_t)(local_ip >> 16);
    ip[2] = (uint8_t)(local_ip >> 8);
    ip[3] = (uint8_t)local_ip;
#endif
    feather_log("Ethernet initialized (link %s, ip %u.%u.%u.%u)",
                Ethernet.linkState() ? "up" : "down", ip[0], ip[1], ip[2], ip[3]);
}


void initialize_ethernet(void) {
    feather_log("Initializing Ethernet...");
    uint8_t mac[6] = TEENSY_MAC_ADDR;
    Ethernet.setMACAddress(mac);

#ifdef COMPILE_FOR_TEENSY
    // Start DHCP without waiting for it. poll_ethernet() starts the server once a lease arrives, however long it takes.
    Ethernet.setHostname(TEENSY_HOSTNAME);
    feather_log("ethernet mac/hostname set, requesting dhcp lease...");
    if (!Ethernet.begin()) {
        set_global_error_msg("Failed to start Ethernet!");
    }
#else
    if (!Ethernet.begin(BUILD_IP_ADDR(TEENSY_IP), BUILD_IP_ADDR(TEENSY_SUBNET), BUILD_IP_ADDR(TEENSY_GATEWAY))) {
        set_global_error_msg("Failed to configure Ethernet!");
        return;
    }
    start_ethernet_server();
#endif
}


///////////////////////////
// Ethernet Write Buffer //
///////////////////////////


// --- Response Buffer is a Ring Buffer ---
runtime_static uint8_t eth_write_buf[ETH_WRITE_BUF_SIZE] = {0};
runtime_static uint32_t eth_write_buf_start = 0;
runtime_static uint32_t eth_write_buf_end = 0;

void eth_write_buffer_insert(const void* data, uint32_t size) {
    uint32_t first_chunk = MIN(size, (uint32_t)(ETH_WRITE_BUF_SIZE - eth_write_buf_end));
    memcpy(&eth_write_buf[eth_write_buf_end], data, first_chunk);
    if (size > first_chunk) {
        memcpy(&eth_write_buf[0], (const uint8_t*)data + first_chunk, size - first_chunk);
    }
    eth_write_buf_end = (eth_write_buf_end + size) % ETH_WRITE_BUF_SIZE;
}

uint32_t eth_write_buffer_remaining_space(void) {
    uint32_t used = (eth_write_buf_end - eth_write_buf_start + ETH_WRITE_BUF_SIZE) % ETH_WRITE_BUF_SIZE;
    return ETH_WRITE_BUF_SIZE - 1u - used;
}

int eth_buffer_write(uint32_t msg_id, uint32_t opcode, const void* payload, uint32_t payload_size) {
    ClientCommandHeader header = {
        .msg_id = msg_id, .opcode = (TeensyCommandOpcode)opcode, .payload_size = payload_size
    };
    uint32_t size = sizeof(ClientCommandHeader) + payload_size;

    uint32_t remaining = eth_write_buffer_remaining_space();
    ASSERT_GLOBAL(remaining >= size, -1, "Ethernet writebuf full, dropping. Needed: %u, Available: %u", size, remaining);
    ASSERT_GLOBAL(payload != NULL || payload_size == 0, -1, "Payload NULL but payload size is %u.", payload_size);

    eth_write_buffer_insert(&header, sizeof(ClientCommandHeader));
    eth_write_buffer_insert(payload, payload_size);
    return 0;
}

void eth_write_buffer_flush(void) {
    // Drain as much contiguous data as the TX queue can accept, without ever blocking.
    bool wrote = false;
    while (true) {
        uint32_t used = (uint32_t)((eth_write_buf_end - eth_write_buf_start + ETH_WRITE_BUF_SIZE) % ETH_WRITE_BUF_SIZE);
        if (used == 0) break;

        int available_bytes = ethernet_client.availableForWrite();
        if (available_bytes <= 0) break;

        // write() can only take a physically-contiguous run. If the data wraps around the end of the
        // buffer, write up to the physical end now and let the next iteration handle the wrapped remainder.
        uint32_t contiguous = (eth_write_buf_start < eth_write_buf_end) ? used : (ETH_WRITE_BUF_SIZE - eth_write_buf_start);
        uint32_t to_write = MIN((uint32_t)available_bytes, contiguous);

        int written = ethernet_client.write(&eth_write_buf[eth_write_buf_start], to_write);
        if (written <= 0) break;
        wrote = true;
        eth_write_buf_start = (eth_write_buf_start + (uint32_t)written) % ETH_WRITE_BUF_SIZE;
    }
    if (wrote) ethernet_client.flush();
}


//////////////////////////////////
// Ethernet Connection Handling //
//////////////////////////////////


/**
 * Clears the current buffer state.
 */
runtime_static void clear_eth_read_buffer_state(void) {
    eth_read_buf_len = 0;
    eth_read_needed_bytes = sizeof(ClientCommandHeader);
    eth_read_header = NULL;
    clear_local_error_msg();
}

/**
 * Sends an error response to the client.
 * 
 * @param msg_id The message id of the command that is being responded to
 * @param err_msg The error message to send
 */
runtime_static void send_error_response(uint32_t msg_id, const char* err_msg) {
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_ERROR, err_msg, (uint32_t)strlen(err_msg));
}

/**
 * Sends a success response to the client.
 * 
 * @param msg_id The message id of the command that is being responded to
 */
runtime_static void send_success_response(uint32_t msg_id) {
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0);
}

/**
* If we don't have a client, attempts to connect to a new available client.
* 
* @return True if we have a client, false otherwise
*/
static bool check_for_ethernet_client(void) {
    if (ETHERNET_CLIENT_CONNECTED) return true;
    if (ETHERNET_CLIENT_AVAILABLE) {
        // Disable Nagle so small command/response packets go out promptly.
        ethernet_client.setNoDelay(true);
        feather_log("New Ethernet client connection accepted.");
        return true;
    }
    return false;
}

/**
 * Flushes the current connection and stops it.
 */
runtime_static void stop_connection(void) {
    eth_write_buffer_flush();
    ethernet_client.flush();
    ethernet_client.stop();
    clear_eth_read_buffer_state();

    // Discard any unsent buffered response data so it can't leak into the next client's stream.
    eth_write_buf_start = 0;
    eth_write_buf_end = 0;
}

/**
 * Handles a single command. The current command will be cleared after the command is handled.
 * 
 * @param header The header of the command
 */
runtime_static void handle_command(ClientCommandHeader* header) {
    feather_log("Received command: opcode=%u, payload_size=%u, read_buf_len=%u", header->opcode, header->payload_size, eth_read_buf_len);
    const ClientCommand* current_command = get_command(header->opcode);

    // If command not found, do nothing (error set by get_command()). If size mismatch, set error and do nothing.
    // Otherwise, command is good, handle it. `response_written` is true if the handler already wrote its own response.
    bool response_written = false;
    if (current_command == NULL) {}
    else if (eth_read_buf_len != sizeof(ClientCommandHeader) + current_command->num_bytes) {
        set_local_error_msg("Payload size for command %s does not match expected size. Got %u, expected %zu.", 
            current_command->name, eth_read_buf_len, sizeof(ClientCommandHeader) + current_command->num_bytes);
    }
    else response_written = current_command->handler_function(header->msg_id, &eth_read_buf[sizeof(ClientCommandHeader)]);

    // Send a response based on whether there was an error. If the handler already wrote its own response, only send
    // an error response if one occurred, otherwise send nothing further.
    const char* err_msg = get_local_error_msg();
    if (err_msg != NULL) send_error_response(header->msg_id, err_msg);
    else if (!response_written) send_success_response(header->msg_id);

    clear_eth_read_buffer_state();
}

void poll_ethernet(void) {
#ifdef COMPILE_FOR_TEENSY
    // Keeps DHCP and mDNS running before any client is connected. Cheap: the stack self-limits its poll rate.
    Ethernet.loop();

    // Nothing to serve until DHCP gives us an address. Check once per loop instead of blocking in initialization.
    if (!eth_server_started) {
        if ((uint32_t)Ethernet.localIP() == 0u) return;
        start_ethernet_server();
        eth_server_started = true;
    }
#endif
    if (!check_for_ethernet_client()) return;

    if (!ethernet_client.connected()) {
        set_global_error_msg("Client-side disconnect detected. Stopping connection.");
        stop_connection();
        return;
    }

    // Read bytes and process them into commands as needed
    // Only do it as bytes are available, up to `server_run_time_nanos` nanoseconds of time
    uint64_t start_time = currtime_ns();
    while (ethernet_client.available() && (currtime_ns() - start_time < comms_config.server_run_time_nanos)) {
        // Read as many bytes as available, up to the needed bytes
        int read_bytes = ethernet_client.read(&eth_read_buf[eth_read_buf_len], eth_read_needed_bytes);
        if (read_bytes < 0) {
            set_global_error_msg("Got a negative value for ethernet_client.read()! Killing connection.");
            stop_connection();
            return;    
        }
        eth_read_buf_len += (uint32_t)read_bytes;
        eth_read_needed_bytes -= (uint32_t)read_bytes;

        // If we don't have a header yet, continue reading until we have it
        if (eth_read_buf_len < sizeof(ClientCommandHeader)) continue;

        // If we haven't set the header yet, set it now
        if (eth_read_header == NULL) {
            eth_read_header = (ClientCommandHeader*)eth_read_buf;

            // Reject anything that wouldn't fit in our buffer before we attempt to read it
            if (sizeof(ClientCommandHeader) + eth_read_header->payload_size > MAX_COMMAND_SIZE) {
                set_local_error_msg("Command payload too large (%u bytes).", eth_read_header->payload_size);
                send_error_response(eth_read_header->msg_id, get_local_error_msg());
                stop_connection();
                return;
            }

            eth_read_needed_bytes = eth_read_header->payload_size;
        }

        // If we have all the bytes we need, handle the command
        if (eth_read_needed_bytes == 0) handle_command(eth_read_header);
    }

    // Write any data that is in the write buffer to the client
    eth_write_buffer_flush();
}
