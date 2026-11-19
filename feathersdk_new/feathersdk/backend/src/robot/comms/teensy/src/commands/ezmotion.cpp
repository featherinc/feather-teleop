#include "ezmotion.hpp"
#include "../comms/can.hpp"
#include "../actuator.hpp"
#include "../system.hpp"
#include <stdint.h>


bool ezmotion_handle_initialize_motor(uint32_t msg_id, void* payload) {
    (void)msg_id;  // Suppress unused parameter compiler warning.

    EZMotionMotorInitPayload* init_payload = (EZMotionMotorInitPayload*)payload;
    EZMotionMotorConfig* new_config = &init_payload->config;

    // Create the new actuator
    uint8_t act_id = init_payload->actuator_id;
    if (!init_actuator(
        act_id, init_payload->allow_reinit, new_config, sizeof(EZMotionMotorConfig), NUM_EZMotionParamCode
    )) return false;

    // Create all of the telemetries for the actuator
    for (uint8_t i = 0; i < NUM_EZMotionParamCode; i++) {
        EZMotionParam param = ezmotion_all_params[i];
        uint16_t telem_size = MIN(param.max_telemetry_size, init_payload->max_telemetry_size);
        if (!add_actuator_param_telemetry(
            act_id, i, param.dtype, telem_size, init_payload->telem_inds[i]
        )) {
            free_actuator(act_id);
            return false;
        }
    }

    feather_log("Initialized EZMotion motor %u", act_id);
    return false;
}

// Macro to get an EZMotion parameter. Returns `false` early if the parameter is invalid.
#define GET_EZ_PARAM(_pcode)                                                                           \
    ASSERT_LOCAL(_pcode < NUM_EZMotionParamCode, false, "Invalid EZMotion parameter code %u", _pcode);  \
    EZMotionParam param = ezmotion_all_params[_pcode];                                                 \
    ASSERT_LOCAL(param.param_id != 0, false, "Invalid EZMotion parameter code %u", _pcode);            

bool ezmotion_handle_read_param(uint32_t msg_id, void* payload) {
    GET_ACTUATOR(EZMotionMotorConfig, EZMotionReadParamPayload, payload, ERROR_SCOPE_LOCAL); (void)act;
    GET_EZ_PARAM(pyld->pcode);
    ASSERT_LOCAL(param.readable, false, "EZMotion parameter code %u is not readable", pyld->pcode);
    opencan_send_sdo(pyld->can_bus, SDO_READ, pyld->actuator_id, param.param_id, param.subindex, param.dtype, NULL);
    return false;
}

// Send a write SDO to the EZMotion motor.
runtime_static bool ez_write(uint8_t can_bus, uint8_t node_id, EZMotionParam param, int64_t value, int16_t pcode) {
    ASSERT_LOCAL(param.writable, false, "EZMotion parameter code %d is not writable", pcode);
    ASSERT_LOCAL(value >= param.min_value && value <= param.max_value, false,
        "EZMotion pcode %d (param_id: %u) value %lld out of range [%lld, %lld]",
        pcode, param.param_id, (long long)value, (long long)param.min_value, (long long)param.max_value);
    
    // Make sure the value is aligned to 8 bytes for the send. Cast incoming int64 to param dtype and send.
    uint8_t casted_value[8] __attribute__((aligned(8)));
    if (!dtype_cast(casted_value, param.dtype, &value, DATA_TYPE_I64, ERROR_SCOPE_LOCAL)) return false;
    return opencan_send_sdo(can_bus, SDO_WRITE, node_id, param.param_id, param.subindex, param.dtype, casted_value);
}

bool ezmotion_handle_write_param(uint32_t msg_id, void* payload) {
    GET_ACTUATOR(EZMotionMotorConfig, EZMotionWriteParamPayload, payload, ERROR_SCOPE_LOCAL); (void)act;
    ASSERT_LOCAL(pyld->pcode != EZ_MOTION_PARAM_CODE_CONTROL_WORD, false, 
        "Cannot write to control word parameter, use the set_control_word command instead");
    GET_EZ_PARAM(pyld->pcode);
    ez_write(pyld->can_bus, pyld->actuator_id, param, pyld->value, pyld->pcode);
    return false;
}

bool ezmotion_handle_set_control_word(uint32_t msg_id, void* payload) {
    GET_ACTUATOR(EZMotionMotorConfig, EZMotionSetControlWordPayload, payload, ERROR_SCOPE_LOCAL); (void)act;

    uint16_t control_word = 
        (pyld->halt                ? (1 << 8) : 0) |
        (pyld->fault_reset         ? (1 << 7) : 0) |
        (pyld->transition_position ? (1 << 4) : 0) |
        (pyld->enable_operation    ? (1 << 3) : 0) |
        (pyld->quick_stop          ? (1 << 2) : 0) |
        (pyld->disable_power       ? 0 : (1 << 1)) |  // Flipped around
        (pyld->switch_on           ? (1 << 0) : 0);

    EZMotionParam param = ezmotion_all_params[EZ_MOTION_PARAM_CODE_CONTROL_WORD];
    ez_write(pyld->can_bus, pyld->actuator_id, param, control_word, EZ_MOTION_PARAM_CODE_CONTROL_WORD);
    return false;
}

bool ezmotion_handle_set_admin_params(uint32_t msg_id, void* payload) {
    GET_ACTUATOR(EZMotionMotorConfig, EZMotionSetAdminParamsPayload, payload, ERROR_SCOPE_LOCAL); (void)act;

    uint8_t act_id = pyld->actuator_id;
    if (!ez_write(pyld->can_bus, act_id, EZParamVoltageLowerLimit, pyld->voltage_lower_limit, -1)) return false;
    if (!ez_write(pyld->can_bus, act_id, EZParamVoltageUpperLimit, pyld->voltage_upper_limit, -1)) return false;
    if (!ez_write(pyld->can_bus, act_id, EZParamNodeId,            pyld->node_id,             -1)) return false;
    if (!ez_write(pyld->can_bus, act_id, EZParamStoreParameters,   EZ_MOTION_STRPARAM_VALUE,  -1)) return false;

    return false;
}

runtime_static void handle_ezmotion_sdo_response(uint8_t act_id, const CAN_message_t* msg) {
    ASSERT_GLOBAL(msg->len == 8, ;, "EZMotion invalid SDO response length for motor %u: %u", act_id, msg->len);

    // Check to make sure the response is a success response
    uint8_t status = msg->buf[0];
    ASSERT_GLOBAL(status != 0x80, ;, "EZMotion SDO ERROR response for motor %u", act_id);

    // Get the parameter index/subindex and make sure they are valid
    uint8_t pcode = ezmotion_id_to_pcode(msg->buf[2] << 8 | msg->buf[1], msg->buf[3]);
    ASSERT_GLOBAL(pcode < NUM_EZMotionParamCode, ;, "EZMotion unknown SDO pcode %u for motor %u", pcode, act_id);
    EZMotionParam param = ezmotion_all_params[pcode];
    ASSERT_GLOBAL(param.param_id != 0, ;, "EZMotion invalid SDO pcode %u for motor %u", pcode, act_id);

    // Check the status code and make sure we know how to process it
    if (status == 0x60) return;  // write-response success. Ignore it for now.
    ASSERT_GLOBAL(status == 0x4F || status == 0x4B || status == 0x43, ;, 
        "EZMotion SDO status code unknown for motor %u: 0x%X", act_id, status);

    uint32_t expected_size = (uint32_t)dtype_size(param.dtype);
    uint32_t data_size = status == 0x4F ? 1u : status == 0x4B ? 2u : 4u;
    ASSERT_GLOBAL(expected_size == data_size, ;, 
        "EZMotion SDO data size mismatch for motor %u: expected %u, got %u", act_id, expected_size, data_size);

    insert_telemetry_value(act_id, pcode, &msg->buf[4], param.dtype, true);
}

void handle_ezmotion_can_message(uint8_t bus, const CAN_message_t* msg) {
    (void)bus;

    // Response to a read/write SDO command.
    if (msg->id > 0x580 && msg->id <= (0x580 + 0x7F)) {
        uint8_t act_id = (uint8_t)(msg->id - 0x580u);
        EZMotionMotorConfig* motor = (EZMotionMotorConfig*)get_actuator_config(act_id, ERROR_SCOPE_GLOBAL);
        ASSERT_GLOBAL(motor != NULL, ;, "EZMotion motor %u not found", act_id);
        handle_ezmotion_sdo_response(act_id, msg);

    // Coming online message. Currently, just check that it's a known motor and ignore it.
    } else if (msg->id > 0x700 && msg->id <= (0x700 + 0x7F)) {
        uint8_t act_id = (uint8_t)(msg->id - 0x700u);
        EZMotionMotorConfig* motor = (EZMotionMotorConfig*)get_actuator_config(act_id, ERROR_SCOPE_GLOBAL);
        ASSERT_GLOBAL(motor != NULL, ;, "EZMotion motor %u not found", act_id);
    } else {
        set_global_error_msg("EZMotion message on bus %u invalid/unhandled canopen cmd 0x%X", bus, msg->id);
    }
}
