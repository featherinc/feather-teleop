/**
 * Aggregates all command definitions, and defines the global array of commands, and command header/struct.
 *
 * Create a new command by importing any necessary handler functions/payload struct definitions, and adding the command
 * to the `client_commands` array using the `CMD` macro.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "../system.hpp"
#include "../logging.hpp"
#include "../actuator.hpp"
#include "../comms/can.hpp"
#include "robstride.hpp"
#include "ezmotion.hpp"
#include "realhand.hpp"
#include "soc.hpp"
#include "qdefs/teensy_commands_qdef.hpp"


typedef struct ClientCommand {
    const char* name;
    uint32_t num_bytes;
    // Returns true if the handler already wrote a response to the buffer, false otherwise (in which case the caller
    // sends a default success/error response).
    bool (*handler_function) (uint32_t msg_id, void* payload);
} ClientCommand;


bool handle_echo(uint32_t msg_id, void* payload);
bool handle_initialize(uint32_t msg_id, void* payload);
bool handle_get_global_error_msg(uint32_t msg_id, void* payload);
bool handle_set_logging(uint32_t msg_id, void* payload);
bool handle_flash_teensy(uint32_t msg_id, void* payload);
bool handle_reboot(uint32_t msg_id, void* payload);
bool handle_get_src_hash(uint32_t msg_id, void* payload);
bool handle_get_num_can_lines(uint32_t msg_id, void* payload);
bool handle_send_can(uint32_t msg_id, void* payload);


/**
 * Creates a new client command. Should only be using when defining a new command in the `client_commands` array.
 * 
 * @param name The name of the command
 * @param opcode The opcode of the command
 * @param num_bytes The number of bytes the command takes (not including header)
 * @param handler_function The function to handle the command. See the `ClientCommand` struct for the function
 *     signature. Should set an error message if there was an error, which will be sent to the client.
 */
#define CMD(opcode, num_bytes, handler_function) [opcode] = {#opcode, num_bytes, handler_function},

#define RESERVED_FOR_FUTURE_USE(opcode) CMD(opcode, 0, NULL)

static const ClientCommand client_commands[] = {
    // Responses ------------------------------
    CMD(TEENSY_COMMAND_OPCODE_RESP_SUCCESS,               0, NULL)
    CMD(TEENSY_COMMAND_OPCODE_RESP_ERROR,                 0, NULL)
    CMD(TEENSY_COMMAND_OPCODE_RESP_ECHO,                  0, NULL)
    CMD(TEENSY_COMMAND_OPCODE_RESP_ACTUATOR_TELEMETRY,    0, NULL)
    CMD(TEENSY_COMMAND_OPCODE_RESP_CANDUMP,               0, NULL)
    CMD(TEENSY_COMMAND_OPCODE_RESP_BOARD_TIME,            0, NULL)
    CMD(TEENSY_COMMAND_OPCODE_RESP_SRC_HASH,              0, NULL)
    CMD(TEENSY_COMMAND_OPCODE_RESP_NUM_CAN_LINES,         0, NULL)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_8)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_9)

    // Utility commands -----------------------
    CMD(TEENSY_COMMAND_OPCODE_CMD_ECHO,                      ECHO_PAYLOAD_SIZE,                 handle_echo                  )
    CMD(TEENSY_COMMAND_OPCODE_CMD_INITIALIZE,                sizeof(InitializeConfig),          handle_initialize            )
    CMD(TEENSY_COMMAND_OPCODE_CMD_GET_GLOBAL_ERROR_MSG,      0,                                 handle_get_global_error_msg  )
    CMD(TEENSY_COMMAND_OPCODE_CMD_GET_ACTUATOR_TELEMETRY,    0,                                 handle_get_actuator_telemetry)
    CMD(TEENSY_COMMAND_OPCODE_CMD_GET_CANDUMP,               0,                                 handle_get_candump           )
    CMD(TEENSY_COMMAND_OPCODE_CMD_GET_BOARD_TIME,            0,                                 handle_get_board_time        )
    CMD(TEENSY_COMMAND_OPCODE_CMD_SET_LOGGING,               sizeof(SetLoggingPayload),         handle_set_logging           )
    CMD(TEENSY_COMMAND_OPCODE_CMD_SET_TIMESTAMP_OFFSET,      sizeof(SetTimestampOffsetPayload), handle_set_timestamp_offset  )
    CMD(TEENSY_COMMAND_OPCODE_CMD_FLASH_TEENSY,              0,                                 handle_flash_teensy          )
    CMD(TEENSY_COMMAND_OPCODE_CMD_REBOOT,                    0,                                 handle_reboot                )
    CMD(TEENSY_COMMAND_OPCODE_CMD_GET_SRC_HASH,              0,                                 handle_get_src_hash          )
    CMD(TEENSY_COMMAND_OPCODE_CMD_GET_NUM_CAN_LINES,         0,                                 handle_get_num_can_lines     )
    CMD(TEENSY_COMMAND_OPCODE_CMD_SEND_CAN,                  sizeof(SendCanPayload),            handle_send_can             )
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_23)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_24)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_25)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_26)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_27)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_28)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_29)

    // Robstride commands ---------------------
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_INITIALIZE_MOTOR,            sizeof(RobMotorInitPayload),               rob_handle_initialize_motor           )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_GET_DEVICE_ID,               sizeof(RobGetDeviceIdPayload),             rob_handle_get_device_id              )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_OPERATION_CONTROL,           sizeof(RobOperationControlPayload),        rob_handle_operation_control          )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_ENABLE,                      sizeof(RobEnablePayload),                  rob_handle_enable                     )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_DISABLE,                     sizeof(RobDisablePayload),                 rob_handle_disable                    )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_ZERO_POSITION,               sizeof(RobZeroPositionPayload),            rob_handle_zero_position              )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_SET_ID,                      sizeof(RobSetIdPayload),                   rob_handle_set_id                     )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_BEGIN_FLASH,                 sizeof(RobBeginFlashPayload),              rob_handle_begin_flash                )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_FLASH_FILE_INFO,             sizeof(RobFlashFileInfoPayload),           rob_handle_flash_file_info            )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_FLASH_DATA,                  sizeof(RobFlashDataPayload),               rob_handle_flash_data                 )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_END_FLASH,                   sizeof(RobEndFlashPayload),                rob_handle_end_flash                  )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_READ_PARAM,                  sizeof(RobReadParamPayload),               rob_handle_read_param                 )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_WRITE_PARAM,                 sizeof(RobWriteParamPayload),              rob_handle_write_param                )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_SAVE_MOTOR_DATA,             sizeof(RobSaveMotorDataPayload),           rob_handle_save_motor_data            )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_SET_ACTIVE_REPORTING,        sizeof(RobSetActiveReportingPayload),      rob_handle_set_active_reporting       )
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_SET_SKIP_WRITE_PARAM_CHECKS, sizeof(RobSetSkipWriteParamChecksPayload), rob_handle_set_skip_write_param_checks)
    CMD(TEENSY_COMMAND_OPCODE_CMD_ROB_GET_FIRMWARE_VERSION,        sizeof(RobGetFirmwareVersionPayload),      rob_handle_get_firmware_version       )
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_47)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_48)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_49)

    // EZMotion commands ---------------------
    CMD(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_INITIALIZE_MOTOR,  sizeof(EZMotionMotorInitPayload),      ezmotion_handle_initialize_motor)
    CMD(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_READ_PARAM,        sizeof(EZMotionReadParamPayload),      ezmotion_handle_read_param      )
    CMD(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_WRITE_PARAM,       sizeof(EZMotionWriteParamPayload),     ezmotion_handle_write_param     )
    CMD(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_SET_CONTROL_WORD,  sizeof(EZMotionSetControlWordPayload), ezmotion_handle_set_control_word)
    CMD(TEENSY_COMMAND_OPCODE_CMD_EZ_MOTION_SET_ADMIN_PARAMS,  sizeof(EZMotionSetAdminParamsPayload), ezmotion_handle_set_admin_params)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_55)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_56)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_57)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_58)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_59)

    // SOC commands -------------------------
    CMD(TEENSY_COMMAND_OPCODE_CMD_SOC_INITIALIZE_SOC,  sizeof(SOCInitPayload),       soc_handle_initialize_soc)
    CMD(TEENSY_COMMAND_OPCODE_CMD_SOC_READ_PARAM,      sizeof(SOCReadParamPayload),  soc_handle_read_param    )
    CMD(TEENSY_COMMAND_OPCODE_CMD_SOC_WRITE_PARAM,     sizeof(SOCWriteParamPayload), soc_handle_write_param   )
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_63)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_64)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_65)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_66)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_67)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_68)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_69)

    // RealHand commands --------------------
    CMD(TEENSY_COMMAND_OPCODE_CMD_REALHAND_INITIALIZE_MOTOR, sizeof(RealHandMotorInitPayload), realhand_handle_initialize_motor)
    CMD(TEENSY_COMMAND_OPCODE_CMD_REALHAND_READ_PARAM,       sizeof(RealHandReadParamPayload), realhand_handle_read_param)
    CMD(TEENSY_COMMAND_OPCODE_CMD_REALHAND_WRITE_PARAM,      sizeof(RealHandWriteParamPayload), realhand_handle_write_param)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_73)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_74)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_75)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_76)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_77)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_78)
    RESERVED_FOR_FUTURE_USE(TEENSY_COMMAND_OPCODE_RESERVED_FOR_FUTURE_USE_79)

};

#define NUM_COMMANDS (sizeof(client_commands) / sizeof(client_commands[0]))

/**
 * Gets a pointer to a command given its opcode.
 * 
 * @param opcode The opcode of the command
 * @return A pointer to the command, or NULL if there was an error.
 */
static inline const ClientCommand* get_command(uint32_t opcode) {
    if (opcode >= NUM_COMMANDS) {
        set_local_error_msg("Invalid command opcode: %u", opcode);
        return NULL;
    }

    const ClientCommand* command = &client_commands[opcode];

    if (command->name == NULL || command->handler_function == NULL) {
        set_local_error_msg("Unknown command opcode, or handler function not implemented: %u", opcode);
        return NULL;
    }

    return command;
}
