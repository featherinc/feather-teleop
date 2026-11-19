#include "actuator.hpp"
#include "commands/all_commands.hpp"
#include "logging.hpp"
#include "comms/ethernet.hpp"
#include "system.hpp"
#include <stdlib.h>
#include <string.h>

// List of all possible actuators
runtime_static Actuator actuators[MAX_NUM_ACTUATORS] = {0};

// RealHand replies use a standard id equal to the actuator slot, same shape as other standard frames.
// This mark is what lets the CAN router send those frames to the RealHand parser.
runtime_static bool realhand_slots[MAX_NUM_ACTUATORS] = {0};

void set_realhand_slot(uint8_t actuator_id, bool is_realhand) {
    if (actuator_id >= MAX_NUM_ACTUATORS) return;
    realhand_slots[actuator_id] = is_realhand;
}

bool is_realhand_slot(uint32_t can_id) {
    return can_id < MAX_NUM_ACTUATORS && realhand_slots[can_id];
}

void free_actuator(uint8_t actuator_id) {
    if (actuator_id >= MAX_NUM_ACTUATORS) return;
    realhand_slots[actuator_id] = false;

    Actuator* actuator = &actuators[actuator_id];

    if (actuator->telemetries.telemetry_list != NULL) {
        for (uint16_t j = 0; j < actuator->telemetries.num_telems; j++) {
            free(actuator->telemetries.telemetry_list[j].values);
            actuator->telemetries.telemetry_list[j].values = NULL;
        }
        free(actuator->telemetries.telemetry_list);
        actuator->telemetries.telemetry_list = NULL;
    }

    free(actuator->config);
    actuator->config = NULL;
    actuator->id = 0;
    actuator->telemetries.num_telems = 0;
}

runtime_static bool _check_actuator_id(uint8_t actuator_id, ErrorScope scope, bool allow_uninitialized) {
    if (actuator_id >= MAX_NUM_ACTUATORS) {
        set_error_msg(scope, "Invalid actuator ID %u, must be in [0, %u]", actuator_id, MAX_NUM_ACTUATORS - 1);
        return false;
    }
    if (!allow_uninitialized && actuators[actuator_id].config == NULL) {
        set_error_msg(scope, "Actuator %d not initialized", actuator_id);
        return false;
    }
    return true;
}

runtime_static bool _check_telem_idx(uint16_t telem_idx, uint16_t num_telems, ErrorScope scope) {
    if (num_telems == 0) {
        set_error_msg(scope, "Actuator has 0 num_telems");
        return false;
    }
    if (telem_idx >= num_telems) {
        set_error_msg(scope, "Invalid telemetry idx %u for actuator. Max is %u", telem_idx, num_telems - 1);
        return false;
    }
    return true;
}

bool init_actuator(uint8_t actuator_id, bool allow_reinit, void* config, uint16_t config_size, uint8_t num_telems) {
    // Sanity checks. allow_uninitialized=true: this call creates the actuator.
    if (!_check_actuator_id(actuator_id, ERROR_SCOPE_LOCAL, true)) return false;
    if (num_telems > MAX_NUM_ACTUATOR_PARAM_TELEMETRIES) {
        set_local_error_msg("Invalid number of telemetries");
        return false;
    }
    if (actuators[actuator_id].config != NULL) {  // If actuator already initialized
        if (!allow_reinit) {  // If not allowed to re-initialize, error
            set_local_error_msg("Actuator %d already initialized", actuator_id);
            return false;
        }
        free_actuator(actuator_id);  // If allowed to re-initialize, free the existing actuator
    }
    if (config == NULL || config_size == 0) {
        set_local_error_msg("Invalid config");
        return false;
    }

    Actuator* actuator = &actuators[actuator_id];
    actuator->id = actuator_id;
    actuator->telemetries.num_telems = num_telems;
    actuator->telemetries.telemetry_list = NULL;

    // Copy config onto the heap
    actuator->config = malloc(config_size);
    if (actuator->config == NULL) {
        free_actuator(actuator_id);
        set_local_error_msg("Failed to allocate memory for actuator config");
        return false;
    }
    memcpy(actuator->config, config, config_size);

    // Allocate the telemetry slot list (values buffers are allocated later via add_*)
    if (num_telems > 0) {
        actuator->telemetries.telemetry_list =
            (ActuatorParamTelemetry*)calloc(num_telems, sizeof(ActuatorParamTelemetry));
        if (actuator->telemetries.telemetry_list == NULL) {
            free_actuator(actuator_id);
            set_local_error_msg("Failed to allocate memory for actuator telemetry list");
            return false;
        }
    }

    return true;
}

bool add_actuator_param_telemetry(
    uint8_t actuator_id, uint16_t telem_idx, DataType dtype, uint16_t max_size, uint16_t host_telem_id
) {
    // Sanity checks
    if (!_check_actuator_id(actuator_id, ERROR_SCOPE_LOCAL, false)) return false;
    if (!_check_telem_idx(telem_idx, actuators[actuator_id].telemetries.num_telems, ERROR_SCOPE_LOCAL)) return false;
    if (actuators[actuator_id].telemetries.telemetry_list == NULL) {
        set_local_error_msg("Actuator %u has no telemetry list", actuator_id);
        return false;
    }

    ActuatorParamTelemetry* telem = &actuators[actuator_id].telemetries.telemetry_list[telem_idx];
    if (telem->values != NULL) {
        set_local_error_msg("Telemetry %u already added for actuator %u", telem_idx, actuator_id);
        return false;
    }
    if (max_size == 0 || max_size > MAX_TELEMETRY_SIZE) {
        set_local_error_msg("Invalid telemetry max size %u for actuator %u. Expected in range [1, %u]", 
            max_size, actuator_id, MAX_TELEMETRY_SIZE);
        return false;
    }
    if (dtype_size(dtype) == 0) {
        set_local_error_msg("Unknown telemetry data type %u", dtype);
        return false;
    }

    // Allocate the ring buffer before committing metadata so failure leaves the slot unused
    void* values = malloc((size_t)max_size * (size_t)dtype_size(dtype));
    if (values == NULL) {
        set_local_error_msg("Failed to allocate memory for telemetry %u for actuator %u", telem_idx, actuator_id);
        return false;
    }

    telem->dtype = dtype;
    telem->max_size = max_size;
    telem->start_ind = 0;
    telem->end_ind = 0;
    telem->host_telem_id = host_telem_id;
    telem->values = values;

    return true;
}

void reset_actuators(void) {
    for (uint8_t i = 0; i < MAX_NUM_ACTUATORS; i++) {
        if (actuators[i].config == NULL && actuators[i].telemetries.telemetry_list == NULL) continue;
        free_actuator(i);
    }
}

runtime_static bool _insert_telemetry_value_locked(
    uint8_t actuator_id, uint16_t telem_idx, const void* value, DataType from_dtype, bool include_timestamp
) {
    // Sanity checks
    if (!_check_actuator_id(actuator_id, ERROR_SCOPE_GLOBAL, false)) return false;
    if (!_check_telem_idx(telem_idx, actuators[actuator_id].telemetries.num_telems, ERROR_SCOPE_GLOBAL)) return false;

    ActuatorParamTelemetry* telem = &actuators[actuator_id].telemetries.telemetry_list[telem_idx];
    ASSERT_GLOBAL(telem->values != NULL, false, "Telemetry %u not added for actuator %u", telem_idx, actuator_id);
    ASSERT_GLOBAL(value != NULL, false, "Null value pointer for actuator %u telemetry %u", actuator_id, telem_idx);

    // Case the value into correct dtype and insert into the telemetry buffer
    uint8_t* slot = (uint8_t*)telem->values + (size_t)telem->end_ind * (size_t)dtype_size(telem->dtype);
    if (!dtype_cast(slot, telem->dtype, value, from_dtype, ERROR_SCOPE_GLOBAL)) return false;
    telem->end_ind = (uint16_t)((telem->end_ind + 1u) % telem->max_size);

    // On overflow, drop the oldest sample and warn, but still succeed
    if (telem->end_ind == telem->start_ind) {
        set_global_error_msg("Telemetry %u for actuator %u overflowed", telem_idx, actuator_id);
        telem->start_ind = (uint16_t)((telem->start_ind + 1u) % telem->max_size);
    }

    // If requested, insert a timestamp
    if (include_timestamp) {
        //telem_idx + 1u < NUM_EZMotionParamCode && ezmotion_all_params[telem_idx + 1u].dtype == DATA_TYPE_F64
        double timestamp = (double)currtime_ns() * 1e-9;
        if (!_insert_telemetry_value_locked(actuator_id, (uint16_t)(telem_idx + 1u), &timestamp, DATA_TYPE_F64, false))
            return false;
    }

    return true;
}

bool insert_telemetry_value(
    uint8_t actuator_id, uint16_t telem_idx, const void* value, DataType from_dtype, bool include_timestamp
) {
    ASSERT_GLOBAL(ACQUIRE_POLL_LOCK(), false, "Failed to acquire actuator poll lock.");
    bool result = _insert_telemetry_value_locked(actuator_id, telem_idx, value, from_dtype, include_timestamp);
    RELEASE_POLL_LOCK();
    return result;
}

void* get_actuator_config(uint8_t actuator_id, ErrorScope scope) {
    if (!_check_actuator_id(actuator_id, scope, false)) return NULL;
    return actuators[actuator_id].config;
}

ActuatorParamTelemetry* get_actuator_param_telemetry(uint8_t actuator_id, uint16_t telem_idx, ErrorScope scope) {
    if (!_check_actuator_id(actuator_id, scope, false)) return NULL;
    if (!_check_telem_idx(telem_idx, actuators[actuator_id].telemetries.num_telems, scope)) return NULL;

    ActuatorParamTelemetry* telem = &actuators[actuator_id].telemetries.telemetry_list[telem_idx];
    if (telem->values == NULL) {
        set_error_msg(scope, "Telemetry %u not added for actuator %u", telem_idx, actuator_id);
        return NULL;
    }
    return telem;
}

// Number of samples currently buffered (0 when empty). Safe for wrap-around with unsigned indices.
static uint32_t _telem_num_values(const ActuatorParamTelemetry* telem) {
    return ((uint32_t)telem->end_ind + telem->max_size - telem->start_ind) % telem->max_size;
}

/**
 * Writes the data for a single telemetry update to the ethernet write buffer, accounting for wrapping around the ring
 * buffer in the telemetry.
 *
 * @param telem The telemetry to write the data for
 */
runtime_static void _ethbuf_write_telem_data(ActuatorParamTelemetry* telem) {
    uint32_t elem_size = (uint32_t)dtype_size(telem->dtype);
    uint8_t* values = (uint8_t*)telem->values;

    if (telem->end_ind >= telem->start_ind) {
        uint32_t num_values = (uint32_t)telem->end_ind - telem->start_ind;
        eth_write_buffer_insert(values + (size_t)telem->start_ind * elem_size, num_values * elem_size);
    } else {
        // Wrapped: [start_ind, max_size) then [0, end_ind)
        uint32_t first = (uint32_t)telem->max_size - telem->start_ind;
        eth_write_buffer_insert(values + (size_t)telem->start_ind * elem_size, first * elem_size);
        eth_write_buffer_insert(values, (uint32_t)telem->end_ind * elem_size);
    }
}

runtime_static bool _handle_get_actuator_telemetry_locked(uint32_t msg_id, void* payload) {
    (void)payload;

    uint32_t remaining_space = eth_write_buffer_remaining_space();
    const uint32_t framing_size =
        sizeof(ClientCommandHeader) + sizeof(GetTelemetryPayloadHeader);

    // Empty dump only needs framing; non-empty dumps check per-update size below.
    if (remaining_space < framing_size) {
        set_local_error_msg("Not enough space for get_actuator_telemetry payload headers");
        return false;
    }
    remaining_space -= framing_size;

    // Pass 1: decide how many non-empty telemetries fit and track payload body size.
    uint16_t num_telemetry_updates = 0;
    uint32_t payload_body_size = 0;

    for (uint8_t i = 0; i < MAX_NUM_ACTUATORS; i++) {
        if (actuators[i].config == NULL || actuators[i].telemetries.telemetry_list == NULL) continue;

        for (uint8_t j = 0; j < actuators[i].telemetries.num_telems; j++) {
            ActuatorParamTelemetry* telem = &actuators[i].telemetries.telemetry_list[j];
            if (telem->values == NULL) continue;
            if (telem->end_ind == telem->start_ind) continue;

            uint32_t num_values = _telem_num_values(telem);
            uint32_t new_size = (uint32_t)(sizeof(GetTelemetrySingleTelemetryHeader) +
                                 dtype_size(telem->dtype) * num_values);

            if (remaining_space < new_size) {
                if (num_telemetry_updates == 0) {
                    set_local_error_msg("No space to fit any telemetries with updates");
                    return false;
                }
                goto write_response;
            }
            remaining_space -= new_size;
            payload_body_size += new_size;
            num_telemetry_updates++;
        }
    }

write_response: ;
    // Write the response headers
    uint32_t payload_size = sizeof(GetTelemetryPayloadHeader) + payload_body_size;
    ClientCommandHeader header = {msg_id, TEENSY_COMMAND_OPCODE_RESP_ACTUATOR_TELEMETRY, payload_size};
    eth_write_buffer_insert(&header, sizeof(ClientCommandHeader));
    eth_write_buffer_insert(&num_telemetry_updates, sizeof(num_telemetry_updates));

    // Pass 2: write exactly the planned updates (do not re-use a ran_out_of_space flag).
    uint16_t written = 0;
    for (uint8_t i = 0; i < MAX_NUM_ACTUATORS && written < num_telemetry_updates; i++) {
        if (actuators[i].config == NULL || actuators[i].telemetries.telemetry_list == NULL) continue;

        for (uint8_t j = 0; j < actuators[i].telemetries.num_telems && written < num_telemetry_updates; j++) {
            ActuatorParamTelemetry* telem = &actuators[i].telemetries.telemetry_list[j];
            if (telem->values == NULL) continue;
            if (telem->end_ind == telem->start_ind) continue;

            uint32_t num_values = _telem_num_values(telem);
            GetTelemetrySingleTelemetryHeader telem_header = {
                .telemetry_id = telem->host_telem_id,
                .num_values = (uint16_t)num_values
            };
            eth_write_buffer_insert(&telem_header, sizeof(GetTelemetrySingleTelemetryHeader));
            _ethbuf_write_telem_data(telem);

            // Drain so these samples are not resent on the next poll.
            telem->start_ind = telem->end_ind;
            written++;
        }
    }

    return true;
}

bool handle_get_actuator_telemetry(uint32_t msg_id, void* payload) {
    ASSERT_GLOBAL(ACQUIRE_POLL_LOCK(), false, "Failed to acquire actuator poll lock.");
    bool result = _handle_get_actuator_telemetry_locked(msg_id, payload);
    RELEASE_POLL_LOCK();
    return result;
}
