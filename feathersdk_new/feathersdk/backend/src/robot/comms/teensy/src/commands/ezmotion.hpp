#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../system.hpp"
#include "../comms/comms.hpp"
#include "../comms/can.hpp"
#include "qdefs/ezmotion_qdef.hpp"


bool ezmotion_handle_initialize_motor(uint32_t msg_id, void* payload);
bool ezmotion_handle_read_param(uint32_t msg_id, void* payload);
bool ezmotion_handle_write_param(uint32_t msg_id, void* payload);
bool ezmotion_handle_set_control_word(uint32_t msg_id, void* payload);
bool ezmotion_handle_set_admin_params(uint32_t msg_id, void* payload);

/**
 * Handles a single incoming EZMotion CAN message.
 *
 * @param bus The bus index the message was received on (``[0, get_num_can_lines())``)
 * @param msg The received CAN message
 */
void handle_ezmotion_can_message(uint8_t bus, const CAN_message_t* msg);

/**
 * Converts a EZMotion param ID to a pcode index.
 *
 * Params are too spread out, so have to use a switch statement.
 *
 * @param param_id The EZMotion param ID to convert.
 * @param subindex The subindex of the param to convert.
 * @return The pcode index, or NUM_EZMotionParamCode if the param ID is not found.
 */
static inline uint8_t ezmotion_id_to_pcode(uint16_t param_id, uint8_t subindex) {
    switch ((param_id << 8) | subindex) {
        EZMOTION_ID_TO_PCODE_SWITCH
        default: return NUM_EZMotionParamCode;
    }
}
