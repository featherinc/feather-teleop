#pragma once

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "../system.hpp"
#include "../comms/comms.hpp"
#include "../logging.hpp"
#include "../comms/can.hpp"
#include "qdefs/robstride_qdef.hpp"


bool rob_handle_initialize_motor(uint32_t msg_id, void* payload);
bool rob_handle_get_device_id(uint32_t msg_id, void* payload);
bool rob_handle_get_firmware_version(uint32_t msg_id, void* payload);
bool rob_handle_operation_control(uint32_t msg_id, void* payload);
bool rob_handle_enable(uint32_t msg_id, void* payload);
bool rob_handle_disable(uint32_t msg_id, void* payload);
bool rob_handle_zero_position(uint32_t msg_id, void* payload);
bool rob_handle_set_id(uint32_t msg_id, void* payload);
bool rob_handle_begin_flash(uint32_t msg_id, void* payload);
bool rob_handle_flash_file_info(uint32_t msg_id, void* payload);
bool rob_handle_flash_data(uint32_t msg_id, void* payload);
bool rob_handle_end_flash(uint32_t msg_id, void* payload);
bool rob_handle_read_param(uint32_t msg_id, void* payload);
bool rob_handle_write_param(uint32_t msg_id, void* payload);
bool rob_handle_save_motor_data(uint32_t msg_id, void* payload);
bool rob_handle_set_active_reporting(uint32_t msg_id, void* payload);
bool rob_handle_set_skip_write_param_checks(uint32_t msg_id, void* payload);

void handle_robstride_can_message(uint8_t bus, const CAN_message_t* msg);

/**
 * Converts a Robstride param ID to a pcode index.
 *
 * We subtract 0x7000 from the param ID to get the index into the map for quick lookup.
 *
 * @param param_id The Robstride param ID to convert.
 * @return The pcode index, or NUM_RobParamCode if the param ID is not found.
 */
static inline uint8_t rob_id_to_pcode(uint16_t param_id) {
    uint16_t pcode_map_size = sizeof(rob_id_pcode_map) / sizeof(rob_id_pcode_map[0]);
    
    // Use a wide index before bounds-checking. Casting to uint8_t first would mis-map
    // IDs like 0x7105 -> index 5 (RUN_MODE) instead of "unknown".
    uint16_t index = (uint16_t)(param_id - (uint16_t)ROB_PARAM_BASE);
    if (param_id < (uint16_t)ROB_PARAM_BASE || index >= pcode_map_size) return NUM_RobParamCode;
    return rob_id_pcode_map[index];
}

/**
 * If value is outside [min_val, max_val]: clamp when `clamp` is true, otherwise return NAN and
 * set an error message.
 */
static inline float maybe_clamp(float value, float min_val, float max_val, bool clamp) {
    if (value < min_val || value > max_val) {
        if (clamp) value = MAX(min_val, MIN(value, max_val));
        else {
            set_local_error_msg("Value out of motor config range: %f not in [%f, %f]", value, min_val, max_val);
            return NAN;
        }
    }
    return value;
}
