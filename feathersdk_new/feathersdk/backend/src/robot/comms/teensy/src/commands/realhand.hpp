#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../system.hpp"
#include "../comms/can.hpp"
#include "qdefs/realhand_qdef.hpp"


bool realhand_handle_initialize_motor(uint32_t msg_id, void* payload);
bool realhand_handle_read_param(uint32_t msg_id, void* payload);
bool realhand_handle_write_param(uint32_t msg_id, void* payload);

/**
 * Handles one incoming RealHand frame. The arbitration id is the actuator slot.
 *
 * @param bus The bus index the frame arrived on
 * @param msg The received CAN frame
 */
void handle_realhand_can_message(uint8_t bus, const CAN_message_t* msg);

/** Frame property to pcode. Version is property ``0xC2``. */
static inline uint8_t realhand_property_to_pcode(uint8_t property) {
    switch (property) {
        REALHAND_ID_TO_PCODE_SWITCH
        default: return NUM_RealHandParamCode;
    }
}
