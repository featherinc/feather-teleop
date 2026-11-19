#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../system.hpp"
#include "../comms/can.hpp"
#include "qdefs/soc_qdef.hpp"
#include "qdefs/soc_bridge_qdef.hpp"


bool soc_handle_initialize_soc(uint32_t msg_id, void* payload);
bool soc_handle_read_param(uint32_t msg_id, void* payload);
bool soc_handle_write_param(uint32_t msg_id, void* payload);

/**
 * Handles one incoming frame on ``SOCBRIDGE_CAN_ID``.
 *
 * Read responses are stored in the SOC actuator's telemetry. Write acknowledgements and
 * looped-back commands (status ``CMD_SENT``) are ignored.
 */
void handle_soc_can_message(uint8_t bus, const CAN_message_t* msg);
