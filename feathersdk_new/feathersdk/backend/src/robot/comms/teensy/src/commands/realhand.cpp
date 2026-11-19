#include "realhand.hpp"

#include "../actuator.hpp"
#include "../comms/can.hpp"
#include "../system.hpp"
#include <string.h>


bool realhand_handle_initialize_motor(uint32_t msg_id, void* payload) {
    (void)msg_id;

    RealHandMotorInitPayload* init_payload = (RealHandMotorInitPayload*)payload;
    uint8_t act_id = init_payload->actuator_id;
    if (!init_actuator(
        act_id, init_payload->allow_reinit, &init_payload->config, sizeof(RealHandMotorConfig), NUM_RealHandParamCode
    )) return false;

    for (uint8_t i = 0; i < NUM_RealHandParamCode; i++) {
        RealHandParam param = realhand_all_params[i];
        uint16_t telem_size = MIN(param.max_telemetry_size, init_payload->max_telemetry_size);
        if (!add_actuator_param_telemetry(act_id, i, param.dtype, telem_size, init_payload->telem_inds[i])) {
            free_actuator(act_id);
            return false;
        }
    }

    set_realhand_slot(act_id, true);
    feather_log("Initialized RealHand %u", act_id);
    return false;
}

// Rejects a pcode that is not a real frame property (including timestamp companions, whose param_id is 0).
#define GET_REALHAND_PARAM(_pcode)                                                                          \
    ASSERT_LOCAL((_pcode) < NUM_RealHandParamCode, false, "Invalid RealHand parameter code %u", (_pcode));  \
    RealHandParam param = realhand_all_params[(_pcode)];                                                    \
    ASSERT_LOCAL(param.param_id != 0, false, "Invalid RealHand parameter code %u", (_pcode));

bool realhand_handle_read_param(uint32_t msg_id, void* payload) {
    GET_ACTUATOR(RealHandMotorConfig, RealHandReadParamPayload, payload, ERROR_SCOPE_LOCAL);
    (void)act;
    GET_REALHAND_PARAM(pyld->pcode);
    ASSERT_LOCAL(param.readable, false, "RealHand parameter code %u is not readable", pyld->pcode);

    uint8_t frame[1] = {param.param_id};
    cansend(pyld->can_bus, false, pyld->actuator_id, frame, 1);
    return false;
}

// Six finger bytes, big-endian, sitting in the low 48 bits. Thumb flexion is the first byte on the wire.
runtime_static void pack_finger_frame(uint8_t property, uint64_t value, uint8_t frame[7]) {
    frame[0] = property;
    for (uint8_t i = 0; i < REALHAND_FINGER_COUNT; i++) {
        uint8_t shift = (uint8_t)(8u * (REALHAND_FINGER_COUNT - 1u - i));
        frame[1u + i] = (uint8_t)((value >> shift) & 0xFFu);
    }
}

bool realhand_handle_write_param(uint32_t msg_id, void* payload) {
    GET_ACTUATOR(RealHandMotorConfig, RealHandWriteParamPayload, payload, ERROR_SCOPE_LOCAL);
    (void)act;
    GET_REALHAND_PARAM(pyld->pcode);
    ASSERT_LOCAL(param.writable, false, "RealHand parameter code %u is not writable", pyld->pcode);
    ASSERT_LOCAL(pyld->value >= param.min_value && pyld->value <= param.max_value, false,
        "RealHand pcode %u value out of range", pyld->pcode);

    uint8_t frame[7];
    pack_finger_frame(param.param_id, pyld->value, frame);
    cansend(pyld->can_bus, false, pyld->actuator_id, frame, 7);
    return false;
}

// Bytes after the frame property, big-endian, matching the host's int.from_bytes(..., "big").
runtime_static uint64_t unpack_reply_value(const uint8_t* data, uint8_t len) {
    uint64_t value = 0;
    for (uint8_t i = 0; i < len; i++) value = (value << 8) | (uint64_t)data[i];
    return value;
}

void handle_realhand_can_message(uint8_t bus, const CAN_message_t* msg) {
    ASSERT_GLOBAL(msg->id < MAX_NUM_ACTUATORS, ;, "RealHand CAN id 0x%X on bus %u is not an actuator slot", msg->id, bus);
    uint8_t act_id = (uint8_t)msg->id;
    if (get_actuator_config(act_id, ERROR_SCOPE_GLOBAL) == NULL) return;

    ASSERT_GLOBAL(msg->len >= 1 && msg->len <= 8, ;,
        "RealHand invalid frame length %u on actuator %u", msg->len, act_id);

    // A query echo is the property byte alone. It carries no value, so it must not land in telemetry.
    if (msg->len == 1) return;

    // Vendor error frame: ASCII "ERROR" followed by a big-endian code. Not a parameter reply.
    if (msg->len >= 5 && memcmp(msg->buf, "ERROR", 5) == 0) {
        uint32_t code = 0;
        for (uint8_t i = 5; i < msg->len; i++) code = (code << 8) | (uint32_t)msg->buf[i];
        set_global_error_msg("RealHand error on actuator %u: 0x%06X", act_id, code);
        return;
    }

    uint8_t pcode = realhand_property_to_pcode(msg->buf[0]);
    ASSERT_GLOBAL(pcode < NUM_RealHandParamCode, ;,
        "RealHand unknown frame property 0x%02X on actuator %u", msg->buf[0], act_id);

    uint64_t value = unpack_reply_value(&msg->buf[1], (uint8_t)(msg->len - 1u));
    insert_telemetry_value(act_id, pcode, &value, DATA_TYPE_U64, true);
}
