#include "soc.hpp"
#include "../system.hpp"
#include "../actuator.hpp"
#include "../comms/comms.hpp"
#include "../comms/can.hpp"
#include "../logging.hpp"
#include "qdefs/soc_bridge_qdef.hpp"

// Actuator id assigned by CMD_SOC_INITIALIZE_SOC. Incoming bridge frames are stored there.
runtime_static uint8_t soc_actuator_id = 0;
runtime_static bool soc_initialized = false;


bool soc_handle_initialize_soc(uint32_t msg_id, void* payload) {
    (void)msg_id;  // Suppress unused parameter compiler warning.

    SOCInitPayload* init_payload = (SOCInitPayload*)payload;
    SOCConfig* new_config = &init_payload->config;

    // Create the new actuator
    uint8_t act_id = init_payload->actuator_id;
    if (!init_actuator(act_id, init_payload->allow_reinit, new_config, sizeof(SOCConfig), NUM_SOCParamCode)) {
        return false;
    }

    // Create all of the telemetries for the actuator
    for (uint8_t i = 0; i < NUM_SOCParamCode; i++) {
        SOCParam param = soc_all_params[i];
        if (!add_actuator_param_telemetry(act_id, i, param.dtype, SOC_TELEM_SIZE, init_payload->telem_inds[i])) {
            free_actuator(act_id);
            return false;
        }
    }

    soc_actuator_id = act_id;
    soc_initialized = true;
    feather_log("Initialized SOC with id: %u", act_id);
    return false;
}


// Find the readable/writable pcode that owns this I2C register. Timestamp slots are skipped.
runtime_static uint8_t soc_pcode_for_register(uint8_t slave_addr, uint8_t reg_addr) {
    for (uint8_t i = 0; i < NUM_SOCParamCode; i++) {
        SOCParam param = soc_all_params[i];
        if (param.slave_addr == slave_addr && param.reg_addr == reg_addr && (param.readable || param.writable)) {
            return i;
        }
    }
    return NUM_SOCParamCode;
}


// READ_REGISTER is 4 bytes; WRITE_REGISTER is 6. value_ab is wire order and only used for writes.
runtime_static bool soc_send_register(
    uint8_t bus, SOCBridgeCmd cmd, SOCParam param, const uint8_t value_ab[2]
) {
    uint8_t data[6] = {
        cmd, SOC_BRIDGE_STATUS_CMD_SENT, param.slave_addr, param.reg_addr, 0, 0
    };
    uint8_t len = 4;
    if (cmd == SOC_BRIDGE_CMD_WRITE_REGISTER) {
        data[4] = value_ab[0];
        data[5] = value_ab[1];
        len = 6;
    }
    return cansend(bus, false, SOCBRIDGE_CAN_ID, data, len);
}


bool soc_handle_read_param(uint32_t msg_id, void* payload) {
    (void)msg_id;  // Suppress unused parameter compiler warning.

    SOCReadParamPayload* pyld = (SOCReadParamPayload*)payload;
    (void)pyld->node_id;  // Bridge CAN ID is fixed; the host field is unused.
    ASSERT_LOCAL(pyld->pcode < NUM_SOCParamCode, false, "Unknown SOC pcode: %u", pyld->pcode);
    SOCParam param = soc_all_params[pyld->pcode];
    ASSERT_LOCAL(param.readable, false, "SOC parameter %u is not readable", pyld->pcode);

    soc_send_register(pyld->bus, SOC_BRIDGE_CMD_READ_REGISTER, param, NULL);
    return false;
}

bool soc_handle_write_param(uint32_t msg_id, void* payload) {
    (void)msg_id;  // Suppress unused parameter compiler warning.

    SOCWriteParamPayload* pyld = (SOCWriteParamPayload*)payload;
    (void)pyld->node_id;
    ASSERT_LOCAL(pyld->pcode < NUM_SOCParamCode, false, "Unknown SOC pcode: %u", pyld->pcode);
    SOCParam param = soc_all_params[pyld->pcode];
    ASSERT_LOCAL(param.writable, false, "SOC parameter %u is not writable", pyld->pcode);
    ASSERT_LOCAL(dtype_size(param.dtype) == 2, false, "SOC parameter %u is not a 2-byte register", pyld->pcode);

    uint16_t value = (uint16_t)pyld->value;
    uint8_t wire[2];
    if (param.big_endian) {
        wire[0] = (uint8_t)(value >> 8);
        wire[1] = (uint8_t)(value & 0xFF);
    } else {
        wire[0] = (uint8_t)(value & 0xFF);
        wire[1] = (uint8_t)(value >> 8);
    }
    soc_send_register(pyld->bus, SOC_BRIDGE_CMD_WRITE_REGISTER, param, wire);
    return false;
}


runtime_static void handle_soc_register_response(SOCBridgeCmd cmd, const CAN_message_t* msg) {
    ASSERT_GLOBAL(msg->len >= 6, ;, "SOC bridge register response too short: %u", msg->len);

    uint8_t slave_addr = msg->buf[2];
    uint8_t reg_addr = msg->buf[3];
    uint8_t pcode = soc_pcode_for_register(slave_addr, reg_addr);
    ASSERT_GLOBAL(pcode < NUM_SOCParamCode, ;,
        "SOC bridge response for unknown register 0x%02X/0x%02X", slave_addr, reg_addr);

    // Write acks echo the written bytes. Telemetry is updated from read responses only.
    if (cmd == SOC_BRIDGE_CMD_WRITE_REGISTER) return;

    ASSERT_GLOBAL(soc_initialized, ;, "SOC bridge response with no initialized SOC");
    ASSERT_GLOBAL(get_actuator_config(soc_actuator_id, ERROR_SCOPE_GLOBAL) != NULL, ;,
        "SOC actuator %u not found", soc_actuator_id);

    SOCParam param = soc_all_params[pcode];
    uint8_t value_a = msg->buf[4];
    uint8_t value_b = msg->buf[5];
    uint16_t raw = (uint16_t)(param.big_endian
        ? ((unsigned)value_a << 8) | value_b
        : (unsigned)value_a | ((unsigned)value_b << 8));
    insert_telemetry_value(soc_actuator_id, pcode, &raw, DATA_TYPE_U16, true);
}


void handle_soc_can_message(uint8_t bus, const CAN_message_t* msg) {
    (void)bus;
    ASSERT_GLOBAL(msg->id == SOCBRIDGE_CAN_ID, ;, "SOC message with unexpected CAN ID 0x%X", msg->id);
    ASSERT_GLOBAL(msg->len >= sizeof(SOCBridgePayloadHeader), ;, "SOC bridge frame too short: %u", msg->len);

    uint8_t cmd = msg->buf[0];
    uint8_t status = msg->buf[1];
    // TX loopback of our own command. The bridge ignores these too.
    if (status == SOC_BRIDGE_STATUS_CMD_SENT) return;

    ASSERT_GLOBAL(status == SOC_BRIDGE_STATUS_OK, ;, "SOC bridge error status %u for cmd %u", status, cmd);

    if (cmd == SOC_BRIDGE_CMD_READ_REGISTER || cmd == SOC_BRIDGE_CMD_WRITE_REGISTER) {
        handle_soc_register_response((SOCBridgeCmd)cmd, msg);
        return;
    }
    set_global_error_msg("SOC bridge unhandled cmd %u", cmd);
}
