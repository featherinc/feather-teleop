#include "robstride.hpp"

#include "../system.hpp"  // for MIN()/MAX()
#include <stdint.h>
#include <string.h>  // for memcpy()/memset()
#include <stdlib.h>  // for calloc()
#include <assert.h>  // for assert()
#include "../logging.hpp"
#include "../comms/can.hpp"
#include "../actuator.hpp"

/**
 * Gets a pointer to the Robstride motor configuration for a given motor ID.
 *
 * @param motor_id The ID of the motor to get the configuration for.
 * @param scope The scope of the calling function for error messages.
 * @return A pointer to the Robstride motor configuration, or NULL if out of bounds / uninitialized.
 */
static RobMotorConfig* get_rob_motor_config(uint8_t motor_id, ErrorScope scope) {
    return (RobMotorConfig*)get_actuator_config(motor_id, scope);
}

// Sticky error tally. Not part of RobMotorConfig: the host does not send it, and re-init must not restore it
// from the init payload. Cleared by disable with clear_faults, and by a successful initialize.
typedef struct RobMotorState {
    uint16_t errors;
} RobMotorState;

static RobMotorState rob_motor_state[MAX_NUM_MOTORS];

// Wire bit -> RobMotorError bit index in the uint16 telemetry word.
typedef struct {
    uint8_t wire_bit;
    uint8_t enum_bit;
} _RobErrorBit;

// Low 6 bits of the feedback extra byte. Bits 0-4 match RobMotorError. Bit 5 is undefined.
static const _RobErrorBit _motor_feedback_error_bits[] = {
    {0, ROB_MOTOR_ERROR_UNDER_VOLTAGE},
    {1, ROB_MOTOR_ERROR_OVER_CURRENT},
    {2, ROB_MOTOR_ERROR_OVER_TEMPERATURE},
    {3, ROB_MOTOR_ERROR_MAGNETIC_ENCODING},
    {4, ROB_MOTOR_ERROR_GRIDLOCK_OVERLOAD},
};

// Little-endian fault word. These wire bits are not the telemetry bit positions.
static const _RobErrorBit _fault_feedback_error_bits[] = {
    {0,  ROB_MOTOR_ERROR_OVER_TEMPERATURE},
    {1,  ROB_MOTOR_ERROR_DRIVER_CHIP_FAULT},
    {2,  ROB_MOTOR_ERROR_UNDER_VOLTAGE},
    {3,  ROB_MOTOR_ERROR_OVER_VOLTAGE},
    {4,  ROB_MOTOR_ERROR_B_PHASE_OVERCURRENT},
    {5,  ROB_MOTOR_ERROR_C_PHASE_OVERCURRENT},
    {7,  ROB_MOTOR_ERROR_ENCODER_NOT_CALIBRATED},
    {8,  ROB_MOTOR_ERROR_HARDWARE_IDENTIFICATION_FAULT},
    {9,  ROB_MOTOR_ERROR_POSITION_INITIALIZATION_FAULT},
    {14, ROB_MOTOR_ERROR_GRIDLOCK_OVERLOAD},
    {16, ROB_MOTOR_ERROR_A_PHASE_OVERCURRENT},
};

// Map set wire bits onto RobMotorError positions. Unknown bits are ignored.
static uint16_t _map_error_bits(uint32_t wire, const _RobErrorBit* map, size_t count) {
    uint32_t known = 0;
    uint16_t packed = 0;
    for (size_t i = 0; i < count; i++) {
        known |= (1u << map[i].wire_bit);
        if ((wire & (1u << map[i].wire_bit)) != 0u) {
            packed = (uint16_t)(packed | (uint16_t)(1u << map[i].enum_bit));
        }
    }

    return packed;
}

// OR newly reported bits into the motor's sticky tally and publish that tally.
static void _store_rob_errors(uint8_t motor_id, uint16_t new_bits) {
    rob_motor_state[motor_id].errors = (uint16_t)(rob_motor_state[motor_id].errors | new_bits);
    uint16_t errors = rob_motor_state[motor_id].errors;
    insert_telemetry_value(motor_id, ROB_PARAM_CODE_FB_ERRORS, &errors, DATA_TYPE_U16, false);
}

// Macro to check if a value is within a range (inclusive) and set an error message/return false if not.
#define CHECK_IN_RANGE(value, min, max) \
    if (value < min || value > max) { \
        set_local_error_msg("Value \"%s\" out of range: `%f` not in [%f, %f]", #value, value, min, max); \
        return false; \
    }

bool rob_handle_initialize_motor(uint32_t msg_id, void* payload) {
    (void)msg_id;  // Suppress unused parameter compiler warning.

    RobMotorInitPayload* init_payload = (RobMotorInitPayload*)payload;
    RobMotorConfig* new_config = &init_payload->config;

    // Do parameter checks to make sure they are within range
    if (new_config->version >= NUM_RobVersion) {
        set_local_error_msg("Value \"new_config->version\" out of range: `%u` not in [0, %u]",
            new_config->version, NUM_RobVersion - 1);
        return false;
    }
    CHECK_IN_RANGE(new_config->max_velocity, 0, rob_spd_lim[new_config->version]);
    CHECK_IN_RANGE(new_config->max_torque, 0, rob_torque_lim[new_config->version]);
    CHECK_IN_RANGE(new_config->max_acceleration, 0, ROB_MAX_ACCELERATION);
    CHECK_IN_RANGE(new_config->cal_range_min, ROB_ANGLE_MIN, ROB_ANGLE_MAX);
    CHECK_IN_RANGE(new_config->cal_range_max, ROB_ANGLE_MIN, ROB_ANGLE_MAX);
    CHECK_IN_RANGE(new_config->homing_offset, ROB_ANGLE_MIN, ROB_ANGLE_MAX);
    if (new_config->direction != 1 && new_config->direction != -1) {
        set_local_error_msg("Invalid \"new_config->direction\": `%d`, must be 1 or -1", new_config->direction);
        return false;
    }

    // Create the new actuator
    uint8_t actuator_id = init_payload->actuator_id;
    bool allow_reinit = init_payload->allow_reinit;
    if (!init_actuator(actuator_id, allow_reinit, new_config, sizeof(RobMotorConfig), NUM_RobParamCode)) return false;
    rob_motor_state[actuator_id].errors = 0;

    // Create all of the telemetries for the actuator
    for (uint8_t i = 0; i < NUM_RobParamCode; i++) {
        RobParam param = rob_all_params[i];
        uint16_t telem_size = MIN(param.max_telemetry_size, init_payload->max_telemetry_size);
        if (!add_actuator_param_telemetry(
            actuator_id, i, param.dtype, telem_size, init_payload->telem_inds[i]
        )) {
            free_actuator(actuator_id);
            return false;
        }
    }

    feather_log("Initialized Robstride motor %u: version=%u", actuator_id, new_config->version);
    return false;
}

/**
 * Sends a CAN message to the Robstride motor.
 *
 * @param bus The bus to send the message on.
 * @param op The operation to send.
 * @param e_byte The extra byte to send.
 * @param host_id The host ID to send.
 * @param motor_id The motor ID to send.
 * @param data The data to send.
 */
runtime_static void rob_cansend(
    uint8_t bus, RobOperation op, uint8_t e_byte, uint8_t host_id, uint8_t motor_id, const uint8_t* data
) {
    if (bus >= get_num_can_lines()) {
        set_local_error_msg("Invalid CAN bus %u! Valid buses are 0-%u.", bus, get_num_can_lines() - 1);
        return;
    }
    if (motor_id >= MAX_NUM_MOTORS) {
        set_local_error_msg("Motor ID out of bounds: %u! Valid IDs are 0-%u.", motor_id, MAX_NUM_MOTORS - 1);
        return;
    }

    uint32_t can_id = ((uint32_t)op << 24) | ((uint32_t)e_byte << 16) | ((uint32_t)host_id << 8) | motor_id;
    feather_log("Sending CAN message: bus=%u, op=%u, e_byte=%u, host_id=%u, motor_id=%u, can_id=%u",
        bus, op, e_byte, host_id, motor_id, can_id);
    cansend(bus, true, can_id, data, 8);
}

///////////////////////
// Handler Functions //
///////////////////////


#define ROB_HOST_ID 0xAA

// Macro to get the motor and payload from the payload. Stores into `motor`, and
// `rob_payload`. If the motor is not found, returns early.
#define GET_MOTOR(payload, type)                                                               \
    (void)msg_id;  /* Suppress unused parameter warning. */                                    \
    type* rob_payload = (type*)(payload);                                                      \
    RobMotorConfig* motor = get_rob_motor_config(rob_payload->motor_id, ERROR_SCOPE_LOCAL);    \
    if (motor == NULL) return false;  /* error message already set */


bool rob_handle_get_device_id(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobGetDeviceIdPayload); (void)motor;
    uint8_t data[8] = {0};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_GET_DEVICE_ID, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_get_firmware_version(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobGetFirmwareVersionPayload); (void)motor;

    // Same frame as disable, with the firmware-version request bytes in the data field.
    uint8_t data[8] = {0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_DISABLE, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_enable(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobEnablePayload); (void)motor;
    uint8_t data[8] = {0};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_ENABLE, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_disable(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobDisablePayload);

    uint8_t clear_faults = rob_payload->clear_faults ? 1u : 0u;
    uint8_t data[8] = {clear_faults, 0, 0, 0, 0, 0, 0, 0};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_DISABLE, 0, ROB_HOST_ID, rob_payload->motor_id, data);

    if (rob_payload->clear_faults) {
        rob_motor_state[rob_payload->motor_id].errors = 0;
        uint16_t errors = 0;
        insert_telemetry_value(rob_payload->motor_id, ROB_PARAM_CODE_FB_ERRORS, &errors, DATA_TYPE_U16, false);
    }
    return false;
}

bool rob_handle_save_motor_data(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobSaveMotorDataPayload); (void)motor;
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_SAVE_MOTOR_DATA, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_set_active_reporting(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobSetActiveReportingPayload);

    uint8_t active = rob_payload->active ? 1u : 0u;
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, active, 8};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_SET_ACTIVE_REPORTING, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_set_skip_write_param_checks(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobSetSkipWriteParamChecksPayload);
    motor->skip_write_param_checks = rob_payload->skip_write_param_checks;
    return false;
}

bool rob_handle_zero_position(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobZeroPositionPayload); (void)motor;
    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_ZERO_POS, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_read_param(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobReadParamPayload);

    if (rob_payload->pcode >= NUM_RobParamCode) {
        set_local_error_msg("Invalid param code: %u", rob_payload->pcode);
        return false;
    }

    const RobParam param = rob_all_params[rob_payload->pcode];
    if (param.param_id == 0x0) {
        set_local_error_msg("Invalid param code: %u", rob_payload->pcode);
        return false;
    }
    if (!param.readable) {
        set_local_error_msg("Param code %u is not readable", rob_payload->pcode);
        return false;
    }

    uint8_t data[8] = {(uint8_t)(param.param_id & 0xFF), (uint8_t)(param.param_id >> 8), 0, 0, 0, 0, 0, 0};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_READ_PARAM, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

// Handles bounds checking and clamping for params differently depending on their type
bool rob_handle_write_param(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobWriteParamPayload);

    if (rob_payload->pcode >= NUM_RobParamCode) {
        set_local_error_msg("Invalid param code: %u", rob_payload->pcode);
        return false;
    }

    RobParamCode pcode = (RobParamCode)rob_payload->pcode;
    const RobParam param = rob_all_params[pcode];
    if (param.param_id == 0x0) {
        set_local_error_msg("Invalid param code: %u", rob_payload->pcode);
        return false;
    }
    if (!param.writable) {
        set_local_error_msg("Param code %u is not writable", pcode);
        return false;
    }

    uint8_t data[8] = {(uint8_t)(param.param_id & 0xFF), (uint8_t)(param.param_id >> 8), 0, 0, 0, 0, 0, 0};

    if (param.dtype == DATA_TYPE_F32) {
        float value = rob_payload->value;

        // We don't allow sending nan's since we need it to check if values are within range later
        if (isnan(value)) {
            set_local_error_msg("Cannot send NaN values");
            return false;
        }

        // If we are not forcing the write through, do checks to make sure it is within range, clamping if needed.
        // We assume that any config limits are within parameter range.
        if (!motor->skip_write_param_checks) {
            if (pcode == ROB_PARAM_CODE_LIMIT_SPD || pcode == ROB_PARAM_CODE_VEL_MAX || pcode == ROB_PARAM_CODE_SPD_REF) {
                value = maybe_clamp(value, -motor->max_velocity, motor->max_velocity, rob_payload->clamp);
            } else if (pcode == ROB_PARAM_CODE_LIMIT_TORQUE) {
                value = maybe_clamp(value, -motor->max_torque, motor->max_torque, rob_payload->clamp);
            } else if (pcode == ROB_PARAM_CODE_ACC_RAD || pcode == ROB_PARAM_CODE_ACC_SET) {
                value = maybe_clamp(value, -motor->max_acceleration, motor->max_acceleration, rob_payload->clamp);
            } else if (pcode == ROB_PARAM_CODE_LOC_REF) {
                value = maybe_clamp(value, motor->cal_range_min, motor->cal_range_max, rob_payload->clamp);
            } else {  // The default parameter limit
                float min_default = param.min_float[motor->version], max_default = param.max_float[motor->version];
                value = maybe_clamp(value, min_default, max_default, rob_payload->clamp);
            }

            if (isnan(value)) return false;  // Error message already set
        }

        // Need to do this since Robstride motors don't update vel_max register value when you send 0 (even though
        // it does stop the motor). But, if you pass a very small value, it still stops and updates.
        if (pcode == ROB_PARAM_CODE_VEL_MAX && value == 0.0f) value = 1e-30f;

        // Add homing offset to position. Offset is calculated pre-direction so do it before direction correction
        if (param.is_location_param) value = value + motor->homing_offset;

        // Take into account our direction
        if (param.is_direction_param) value = value * (float)motor->direction;

        memcpy(data + 4, &value, 4);
    }

    else {
        uint32_t value = (uint32_t)(rob_payload->value);

        if (!motor->skip_write_param_checks) {
            // Check the value is within range of the main parameter
            uint64_t min_param = param.min_int[motor->version], max_param = param.max_int[motor->version];
            if (value < min_param || value > max_param) {
                set_local_error_msg("Value out of parameter range: %u not in [%u, %u]", value, min_param, max_param);
                return false;
            }
        }

        // Copy the value into the data array based on the data type, left-justified in the 4 bytes
        if (param.dtype == DATA_TYPE_U8) {
            data[4] = (uint8_t)value;
        } else if (param.dtype == DATA_TYPE_U16) {
            data[4] = (uint8_t)(value & 0xFF);
            data[5] = (uint8_t)(value >> 8);
        } else if (param.dtype == DATA_TYPE_U32) {
            data[4] = (uint8_t)((value >> 0) & 0xFF);
            data[5] = (uint8_t)((value >> 8) & 0xFF);
            data[6] = (uint8_t)((value >> 16) & 0xFF);
            data[7] = (uint8_t)((value >> 24) & 0xFF);
        } else {
            set_local_error_msg("Invalid data type: %d", param.dtype);
            return false;
        }
    }

    rob_cansend(rob_payload->can_bus, ROB_OPERATION_WRITE_PARAM, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

// Maps a value to a uint16_t based on the range for this version, and stores result at given location.
// Assumes value is already clamped to the range. Big-endian, matching Robstride wire format.
runtime_static void opcom_map_insert(float value, float min, float max, uint8_t* data) {
    uint16_t mapped = (uint16_t)((value - min) / (max - min) * 65535.0f);
    data[0] = (uint8_t)(mapped >> 8);
    data[1] = (uint8_t)(mapped & 0xFF);
}

// Inverse of opcom_map_insert: decode a big-endian uint16 sample into engineering units.
runtime_static float opcom_map_extract(const uint8_t* data, float min, float max) {
    uint16_t mapped = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return ((float)mapped / 65535.0f) * (max - min) + min;
}

bool rob_handle_operation_control(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobOperationControlPayload);

    // Macro to clamp and check a value. Will store the clamped value in the variable `name`.
    #define CLAMP_AND_CHECK(name, value, min, max)                                                            \
        if (isnan(value)) {                                                                                   \
            set_local_error_msg("Cannot send NaN values for " #name);                                         \
            return false;                                                                                     \
        }                                                                                                     \
        float name = maybe_clamp(value, min, max, rob_payload->clamp);                                        \
        if (isnan(name)) {                                                                                    \
            /* Override clamp's error message with our own */                                                 \
            set_local_error_msg("Value out of range for `" #name "`: %f not in [%f, %f]", value, min, max);   \
            return false;                                                                                     \
        }

    // Clamp all the values and make sure they are valid. Will set error and return early if not.
    CLAMP_AND_CHECK(torque,   rob_payload->torque,   -motor->max_torque,          motor->max_torque         );
    CLAMP_AND_CHECK(angle,    rob_payload->angle,     motor->cal_range_min,       motor->cal_range_max      );
    CLAMP_AND_CHECK(velocity, rob_payload->velocity, -motor->max_velocity,        motor->max_velocity       );
    CLAMP_AND_CHECK(kp,       rob_payload->kp,        0,                          rob_kp_lim[motor->version]);
    CLAMP_AND_CHECK(kd,       rob_payload->kd,        0,                          rob_kd_lim[motor->version]);

    // Apply homing offset (pre-direction) and direction, matching the python `operation_command`.
    angle = (float)motor->direction * (angle + motor->homing_offset);
    velocity = (float)motor->direction * velocity;
    torque = (float)motor->direction * torque;

    uint8_t data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Map the values to uint16_t based on the range for this version, and store result at given location.
    opcom_map_insert(angle,    ROB_ANGLE_MIN,                    ROB_ANGLE_MAX,                  data + 0);
    opcom_map_insert(velocity, rob_n_spd_lim[motor->version],    rob_spd_lim[motor->version],    data + 2);
    opcom_map_insert(kp,       0,                                rob_kp_lim[motor->version],     data + 4);
    opcom_map_insert(kd,       0,                                rob_kd_lim[motor->version],     data + 6);
    opcom_map_insert(torque,   rob_n_torque_lim[motor->version], rob_torque_lim[motor->version], data + 8);

    rob_cansend(rob_payload->can_bus, ROB_OPERATION_OPERATION_CONTROL, data[8], data[9], rob_payload->motor_id, data);
    return false;
}


////////////////////
// Admin Commands //
////////////////////


bool rob_handle_set_id(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobSetIdPayload);

    // Robstride allows ids up to 0xFD, but the Teensy actuator table only addresses [0, MAX_NUM_MOTORS).
    // An id outside that table cannot be commanded again.
    if (rob_payload->new_id >= MAX_NUM_MOTORS) {
        set_local_error_msg("Invalid new ID, must be < %u: %u", MAX_NUM_MOTORS, rob_payload->new_id);
        return false;
    }

    uint8_t data[8] = {0};
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_SET_ID, rob_payload->new_id, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_begin_flash(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobBeginFlashPayload);
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_BEGIN_FLASH, 0, ROB_HOST_ID, rob_payload->motor_id, rob_payload->device_id);
    return false;
}

bool rob_handle_flash_file_info(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobFlashFileInfoPayload);

    uint8_t data[8];
    memcpy(data,     &rob_payload->file_size,   4);
    memcpy(data + 4, &rob_payload->num_packets, 4);

    rob_cansend(rob_payload->can_bus, ROB_OPERATION_FLASH_FILE_INFO, 0, ROB_HOST_ID, rob_payload->motor_id, data);
    return false;
}

bool rob_handle_flash_data(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobFlashDataPayload);

    uint8_t e_byte = (uint8_t)(rob_payload->packet_index >> 8);
    uint8_t h_byte = (uint8_t)(rob_payload->packet_index & 0xFF);
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_FLASH_DATA, e_byte, h_byte, rob_payload->motor_id, rob_payload->data);
    return false;
}

bool rob_handle_end_flash(uint32_t msg_id, void* payload) {
    GET_MOTOR(payload, RobEndFlashPayload);

    uint8_t data[8] = {0};
    memcpy(data, &rob_payload->num_packets, 4);
    rob_cansend(rob_payload->can_bus, ROB_OPERATION_END_FLASH, 0, 0, rob_payload->motor_id, data);
    return false;
}


//////////////////////////
// CAN Message Handling //
//////////////////////////


runtime_static inline uint8_t _get_robstride_repsonse_motor_id(const CAN_message_t* msg) {
    return (uint8_t)((msg->id >> 8) & 0xFF);
}

runtime_static inline uint8_t _get_robstride_response_host_id(const CAN_message_t* msg) {
    return (uint8_t)(msg->id & 0xFF);
}

runtime_static inline uint8_t _get_robstride_opcode(const CAN_message_t* msg) {
    return (uint8_t)((msg->id >> 24) & 0x1F);
}

runtime_static inline uint8_t _get_robstride_e_byte(const CAN_message_t* msg) {
    return (uint8_t)((msg->id >> 16) & 0xFF);
}

runtime_static void _handle_read_param_response(uint8_t bus, uint8_t motor_id, const CAN_message_t* msg) {
    RobMotorConfig* motor = get_rob_motor_config(motor_id, ERROR_SCOPE_GLOBAL);
    if (motor == NULL) return;

    // Param ID is little-endian in bytes 0-1.
    uint16_t param_id;
    memcpy(&param_id, &msg->buf[0], sizeof(param_id));

    uint8_t pcode = rob_id_to_pcode(param_id);
    if (pcode >= NUM_RobParamCode) {
        set_global_error_msg("Unknown RobStride param ID 0x%X on bus %u for motor 0x%X", param_id, bus, motor_id);
        return;
    }

    const RobParam param = rob_all_params[pcode];
    if (param.param_id == 0x0 || !param.readable) {
        set_global_error_msg("Invalid RobStride param ID 0x%X on bus %u for motor 0x%X", param_id, bus, motor_id);
        return;
    }

    const void* value_loc = &msg->buf[4];
    DataType from_dtype = param.dtype;
    double value;

    // Float values should be non-null, and direction/location params need to be modified.
    if (param.dtype == DATA_TYPE_F32 || param.dtype == DATA_TYPE_F64) {
        if (!dtype_cast(&value, DATA_TYPE_F64, &msg->buf[4], param.dtype, ERROR_SCOPE_GLOBAL)) return;
        ASSERT_GLOBAL(!isnan(value), ;, "RobStride NaN value for pcode: %u, motor 0x%X", pcode, motor_id);

        // Apply direction and homing offset to the value, if needed.
        if (param.is_direction_param) value = (double)motor->direction * value;
        if (param.is_location_param) value = value - motor->homing_offset;

        value_loc = &value;
        from_dtype = DATA_TYPE_F64;
    }

    insert_telemetry_value(motor_id, pcode, value_loc, from_dtype, true);
}

/**
 * Parse a motor feedback / active-reporting frame into FB_* telemetries.
 * Wire format matches feathersdk: BE uint16 angle/vel/torque/temp, errors+mode in e_byte.
 */
runtime_static void _handle_motor_feedback(uint8_t bus, uint8_t motor_id, const CAN_message_t* msg) {
    (void)bus;
    RobMotorConfig* motor = get_rob_motor_config(motor_id, ERROR_SCOPE_GLOBAL);
    if (motor == NULL) return;

    // Firmware-version replies arrive as feedback frames starting with 0x00 0xC4 'V'.
    if (msg->buf[0] == 0x00 && msg->buf[1] == 0xC4 && msg->buf[2] == 'V') {
        uint64_t firmware_version;
        memcpy(&firmware_version, msg->buf, sizeof(firmware_version));
        insert_telemetry_value(
            motor_id, ROB_PARAM_CODE_FIRMWARE_VERSION, &firmware_version, DATA_TYPE_U64, true
        );
        return;
    }

    if (motor->version >= NUM_RobVersion) {
        set_global_error_msg("Invalid RobStride version %u on motor 0x%X", motor->version, motor_id);
        return;
    }

    uint8_t e_byte = _get_robstride_e_byte(msg);
    float angle = opcom_map_extract(&msg->buf[0], ROB_ANGLE_MIN, ROB_ANGLE_MAX);
    float velocity = opcom_map_extract(
        &msg->buf[2], rob_n_spd_lim[motor->version], rob_spd_lim[motor->version]);
    float torque = opcom_map_extract(
        &msg->buf[4], rob_n_torque_lim[motor->version], rob_torque_lim[motor->version]);
    float temp = (float)(((uint16_t)msg->buf[6] << 8) | msg->buf[7]) / 10.0f;

    // Convert motor frame → host frame (inverse of operation_control / write transforms).
    angle = (float)motor->direction * angle - motor->homing_offset;
    velocity = (float)motor->direction * velocity;
    torque = (float)motor->direction * torque;

    uint8_t mode = (uint8_t)((e_byte >> 6) & 0x3u);
    uint16_t new_errors = _map_error_bits(
        (uint32_t)(e_byte & 0x3Fu), _motor_feedback_error_bits,
        sizeof(_motor_feedback_error_bits) / sizeof(_motor_feedback_error_bits[0])
    );
    double timestamp = (double)currtime_ns() * 1e-9;

    insert_telemetry_value(motor_id, ROB_PARAM_CODE_FB_POSITION,    &angle,     DATA_TYPE_F32, false);
    insert_telemetry_value(motor_id, ROB_PARAM_CODE_FB_VELOCITY,    &velocity,  DATA_TYPE_F32, false);
    insert_telemetry_value(motor_id, ROB_PARAM_CODE_FB_TORQUE,      &torque,    DATA_TYPE_F32, false);
    insert_telemetry_value(motor_id, ROB_PARAM_CODE_FB_TEMP,        &temp,      DATA_TYPE_F32, false);
    _store_rob_errors(motor_id, new_errors);
    insert_telemetry_value(motor_id, ROB_PARAM_CODE_FB_MODE,        &mode,      DATA_TYPE_U8,  false);
    insert_telemetry_value(motor_id, ROB_PARAM_CODE_FEEDBACK_TS, &timestamp, DATA_TYPE_F64, false);
}

// Fault word is a little-endian uint32 in the first four data bytes. High four bytes are unused.
runtime_static void _handle_fault_feedback(uint8_t bus, uint8_t motor_id, const CAN_message_t* msg) {
    (void)bus;
    if (get_rob_motor_config(motor_id, ERROR_SCOPE_GLOBAL) == NULL) return;

    // We currently ignore the warning value in the last 4 bytes
    uint32_t fault_word = 0;
    memcpy(&fault_word, msg->buf, sizeof(fault_word));

    uint16_t new_errors = _map_error_bits(
        fault_word, _fault_feedback_error_bits,
        sizeof(_fault_feedback_error_bits) / sizeof(_fault_feedback_error_bits[0])
    );
    _store_rob_errors(motor_id, new_errors);
}

// The 8 data bytes are the device id, big-endian.
runtime_static void _handle_get_device_id_response(uint8_t bus, uint8_t motor_id, const CAN_message_t* msg) {
    (void)bus;
    uint64_t device_id = 0;
    for (uint8_t i = 0; i < 8; i++) device_id = (device_id << 8) | msg->buf[i];
    insert_telemetry_value(motor_id, ROB_PARAM_CODE_DEVICE_ID, &device_id, DATA_TYPE_U64, true);
}

void handle_robstride_can_message(uint8_t bus, const CAN_message_t* msg) {
    // Check that the host_id is the same id we've been sending. Should always be true, but just in case.
    //uint8_t host_id = _get_robstride_response_host_id(msg);  // Host_id is currently unused

    // Make sure this is a known motor
    uint8_t motor_id = _get_robstride_repsonse_motor_id(msg);
    RobMotorConfig* motor = get_rob_motor_config(motor_id, ERROR_SCOPE_GLOBAL);
    if (motor == NULL) return;

    // Robstride's messages are always 8 bytes.
    if (msg->len != 8) {
        set_global_error_msg("Invalid RobStride msg len on bus %u for motor 0x%X: %u != 8", bus, motor_id, msg->len);
        return;
    }

    // Handle the message based on the opcode.
    uint8_t opcode = _get_robstride_opcode(msg);
    switch (opcode) {
        case ROB_OPERATION_READ_PARAM:
            _handle_read_param_response(bus, motor_id, msg);
            break;
        case ROB_OPERATION_MOTOR_FEEDBACK:
        case ROB_OPERATION_SET_ACTIVE_REPORTING:
            _handle_motor_feedback(bus, motor_id, msg);
            break;
        case ROB_OPERATION_GET_DEVICE_ID:
            _handle_get_device_id_response(bus, motor_id, msg);
            break;
        case ROB_OPERATION_BEGIN_FLASH:
        case ROB_OPERATION_FLASH_FILE_INFO:
        case ROB_OPERATION_FLASH_DATA:
        case ROB_OPERATION_END_FLASH:
            // Flash replies are acknowledged by storing the operation code for the host to wait on.
            insert_telemetry_value(motor_id, ROB_PARAM_CODE_FLASH_INFO, &opcode, DATA_TYPE_U8, true);
            break;
        case ROB_OPERATION_FAULT_FEEDBACK:
            _handle_fault_feedback(bus, motor_id, msg);
            break;
        default:
            set_global_error_msg("Unknown RobStride opcode 0x%X on bus %u for motor 0x%X", opcode, bus, motor_id);
            break;
    }
}
