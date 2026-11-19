#include "all_commands.hpp"

#include "../actuator.hpp"
#include "../comms/comms.hpp"
#include "../comms/can.hpp"
#include "../comms/ethernet.hpp"
#include "../system.hpp"
#include "string.h"
#include "../ota_flash/FlashTxx.hpp"
#include "../ota_flash/FXUtil.hpp"
#include "../teensy_source_hash.hpp"

bool handle_echo(uint32_t msg_id, void* payload) {
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_ECHO, payload, ECHO_PAYLOAD_SIZE);
    return true;
}

bool handle_initialize(uint32_t msg_id, void* payload) {
    (void)msg_id;  /* Suppress unused parameter warning. */
    InitializeConfig* initialize_payload = (InitializeConfig*)payload;

    // Handle the communication configuration
    CommsConfig comconf = initialize_payload->comms_config;
    if (comconf.server_run_time_nanos > 30000000u) {
        set_local_error_msg("Server run time nanos out of range [0, 30000000]: %u", comconf.server_run_time_nanos);
        return false;
    }
    if (comconf.can_read_run_time_nanos > 30000000u) {
        set_local_error_msg("CAN read run time nanos out of range [0, 30000000]: %u", comconf.can_read_run_time_nanos);
        return false;
    }
    if (comconf.max_can_rate < 10.0 || comconf.max_can_rate > 3500.0) {
        set_local_error_msg("Max CAN rate out of range [10.0, 3500.0]: %f", comconf.max_can_rate);
        return false;
    }
    if (comconf.can_leniency < 1.0 || comconf.can_leniency > comconf.max_can_rate) {
        set_local_error_msg("CAN leniency out of range [1.0, %f]: %f", comconf.max_can_rate, comconf.can_leniency);
        return false;
    }
    set_comms_config(&comconf);

    ASSERT_LOCAL(ACQUIRE_POLL_LOCK(), false, "Failed to acquire CAN poll lock.");
    // Drop any previously configured actuators so a reconnect starts clean.
    reset_actuators();
    
    // Drop stale CAN frames from a prior run (wrong timestamps after clock re-sync).
    clear_candump_telemetry();
    RELEASE_POLL_LOCK();

    // Fresh clock offset on every initialize so sync starts from raw board time.
    (void)set_timestamp_offset_ns(0);

    return false;  // Let the caller send the default success response
}

bool handle_get_global_error_msg(uint32_t msg_id, void* payload) {
    (void)payload;  /* Suppress unused parameter warning. */
    
    const char* global_error_msg = get_global_error_msg();

    if (global_error_msg == NULL) eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0);
    else eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_ERROR, global_error_msg, (uint32_t)strlen(global_error_msg));

    clear_global_error_msg();
    return true;
}

bool handle_set_logging(uint32_t msg_id, void* payload) {
    (void)msg_id;
    SetLoggingPayload* set_logging_payload = (SetLoggingPayload*)payload;
    set_logging_enabled(set_logging_payload->enabled);
    return false;
}

bool handle_flash_teensy(uint32_t msg_id, void* payload) {
    (void)msg_id; (void)payload;

    // Initialize the firmware buffer, and clear flash
    flash_elem_t* buf_addr = NULL;
    size_t buf_size = 0;
    firmware_buffer_init(&buf_addr, &buf_size);
    firmware_buffer_free(buf_addr, buf_size);

    // Send a success response to let the caller know we're ready to flash.
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0);
    eth_write_buffer_flush();

    if (!prepare_teensy_flash(&ethernet_client, buf_addr, buf_size)) {
        firmware_buffer_free(buf_addr, buf_size);
        return false;
    }

    // Send a final success response to let the caller know we're ready to reboot.
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0);
    eth_write_buffer_flush();
    flash_noreturn(buf_addr);
    return true;  // Note: this will never be reached because flash_noreturn() will reboot.
}

bool handle_reboot(uint32_t msg_id, void* payload) {
    (void)payload;

    // Ack first so the host sees success before the link drops.
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_SUCCESS, NULL, 0);
    eth_write_buffer_flush();

    REBOOT;
    for (;;) {}  // Wait until the system reset takes effect.
    return true;
}

bool handle_get_src_hash(uint32_t msg_id, void* payload) {
    (void)payload;
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_SRC_HASH, teensy_source_hash_str, (uint32_t)strlen(teensy_source_hash_str));
    return true;
}

bool handle_get_num_can_lines(uint32_t msg_id, void* payload) {
    (void)payload;
    GetNumCanLinesResponsePayload response = {.num_can_lines = get_num_can_lines()};
    eth_buffer_write(msg_id, TEENSY_COMMAND_OPCODE_RESP_NUM_CAN_LINES, &response, sizeof(response));
    return true;
}

bool handle_send_can(uint32_t msg_id, void* payload) {
    (void)msg_id;
    SendCanPayload* pyld = (SendCanPayload*)payload;
    cansend(pyld->can_bus, pyld->extended, pyld->can_id, pyld->data, pyld->dlc);
    return false;
}