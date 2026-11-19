#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "system.hpp"
#include "logging.hpp"
#include "qdefs/telemetry_qdef.hpp"

// The max we will allow for a telemetry buffer, just as a sanity check.
#define MAX_TELEMETRY_SIZE 150

#define MAX_TELEMETRY_UPDATE_SIZE (sizeof(GetTelemetryPayloadHeader) + \
    (sizeof(GetTelemetrySingleTelemetryHeader) + sizeof(uint64_t)) * MAX_TELEMETRY_SIZE)

typedef struct ActuatorParamTelemetry {
    DataType dtype;
    uint16_t max_size;
    uint16_t start_ind;
    uint16_t end_ind;
    uint16_t host_telem_id;
    void* values;
} ActuatorParamTelemetry;

typedef struct ActuatorTelemetries {
    uint8_t num_telems;
    ActuatorParamTelemetry* telemetry_list;
} ActuatorTelemetries;

typedef struct Actuator {
    uint8_t id;
    void* config;
    ActuatorTelemetries telemetries;
} Actuator;

/**
 * Initializes a new actuator.
 * 
 * @param actuator_id The ID of the actuator to initialize. Valid IDs are [0, MAX_NUM_ACTUATORS).
 * @param allow_reinit Whether to allow re-initialization of the actuator.
 * @param config A pointer to the (stack-local) actuator configuration. We will handle mallocing/freeing on our end, so
 *               just pass a pointer to the stack-local struct.
 * @param config_size The size of the actuator configuration
 * @param num_telems The number of telemetries to initialize
 * @return True if the actuator was initialized successfully, false otherwise
 */
bool init_actuator(uint8_t actuator_id, bool allow_reinit, void* config, uint16_t config_size, uint8_t num_telems);

/**
 * Adds a new parameter telemetry to the actuator.
 * 
 * @param actuator_id The ID of the actuator to add the telemetry to
 * @param telem_idx The index of the telemetry to add
 * @param dtype The data type of the parameter telemetry
 * @param max_size The size of the telemetry ring buffer for this parameter. Max number of values we can store
 * @param host_telem_id The ID of the telemetry on the python-side
 * @return True if the parameter telemetry was added successfully, false otherwise
 */
bool add_actuator_param_telemetry(
    uint8_t actuator_id, uint16_t telem_idx, DataType dtype, uint16_t max_size, uint16_t host_telem_id
);

/**
 * Inserts a value into an actuator parameter telemetry ring buffer.
 * On overflow, drops the oldest sample, sets a global error/warning, and still returns true.
 *
 * @param actuator_id The actuator ID
 * @param telem_idx The telemetry index
 * @param value Pointer to a value of the telemetry's dtype
 * @param from_dtype The data type of the value to insert. Value will be cast into telemetry's dtype before insertion.
 *                   Ignores any overflow on conversion.
 * @param include_timestamp Whether to also store the current timestamp in the telemetry. Assumes the telemetry's dtype
 *                          is DATA_TYPE_F64, and should be stored in `telem_idx + 1`.
 * @return True if the value was inserted, false on invalid args / uninitialized telemetry
 */
bool insert_telemetry_value(
    uint8_t actuator_id, uint16_t telem_idx, const void* value, DataType from_dtype, bool include_timestamp
);

/**
 * Resets all actuators.
 */
void reset_actuators(void);

/**
 * Frees all heap allocations for one actuator and clears its slot.
 * Safe to call on a partially-initialized actuator.
 * 
 * @param actuator_id The ID of the actuator to free
 */
void free_actuator(uint8_t actuator_id);

/**
 * Marks a slot as a RealHand. The slot id is the hand's standard CAN id.
 * ``free_actuator`` clears the mark, which is how standard frames are told apart from EZMotion.
 */
void set_realhand_slot(uint8_t actuator_id, bool is_realhand);

/** True when ``can_id`` is a live RealHand slot. */
bool is_realhand_slot(uint32_t can_id);

/**
 * Gets a pointer to the actuator configuration.
 *
 * @param actuator_id The actuator ID
 * @param scope The scope of the calling function for error messages.
 * @return A pointer to the actuator configuration, or NULL if the actuator is out of bounds or not initialized
 */
void* get_actuator_config(uint8_t actuator_id, ErrorScope scope);

/**
 * Gets a pointer to one parameter telemetry slot (for draining / inspection).
 *
 * @param actuator_id The actuator ID
 * @param telem_idx The telemetry index
 * @param scope The scope of the calling function for error messages.
 * @return Pointer to the telemetry slot, or NULL if invalid / not added yet
 */
ActuatorParamTelemetry* get_actuator_param_telemetry(uint8_t actuator_id, uint16_t telem_idx, ErrorScope scope);

/**
 * Handles a request for actuator telemetry.
 *
 * @param msg_id The message ID
 * @param payload Pointer to the payload
 * @return True if the telemetry was sent successfully, false otherwise
 */
bool handle_get_actuator_telemetry(uint32_t msg_id, void* payload);


// Macro to get the actuator config and payload from the payload. Stores the actuator config in `act`, and
// the payload in `pyld`. If the actuator is not found, returns `false` early after setting an error message.
#define GET_ACTUATOR(_act_type, _payload_type, _payload, _scope)                      \
    (void)msg_id;  /* Suppress unused parameter warning. */                           \
    _payload_type* pyld = (_payload_type*)(_payload);                                 \
    _act_type* act = (_act_type*)get_actuator_config(pyld->actuator_id, _scope);      \
    if (act == NULL) return false;  /* error message already set */
