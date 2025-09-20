/*
  Q8Commands.cpp - Implementation of command utilities and validation
  High-level command system for autonomous Q8bot control
*/

#include "Q8Commands.h"

uint8_t Q8CommandUtils::calculateChecksum(const HighLevelCommand& cmd) {
    uint8_t checksum = 0;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&cmd);
    
    // Calculate checksum for all bytes except the checksum field itself
    for (size_t i = 0; i < sizeof(HighLevelCommand) - 3; i++) { // -3 to skip checksum and reserved bytes
        if (i != offsetof(HighLevelCommand, checksum)) {
            checksum ^= data[i];
        }
    }
    
    return checksum;
}

uint8_t Q8CommandUtils::calculateChecksum(const StatusResponse& status) {
    uint8_t checksum = 0;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&status);
    
    // Calculate checksum for all bytes except the checksum field
    for (size_t i = 0; i < sizeof(StatusResponse) - 1; i++) {
        if (i != offsetof(StatusResponse, checksum)) {
            checksum ^= data[i];
        }
    }
    
    return checksum;
}

bool Q8CommandUtils::validateCommand(const HighLevelCommand& cmd) {
    // Check command type validity
    if (cmd.cmd == CMD_INVALID) {
        return false;
    }
    
    // Verify checksum
    uint8_t expected_checksum = calculateChecksum(cmd);
    if (cmd.checksum != expected_checksum) {
        return false;
    }
    
    // Validate parameters based on command type
    if (isMovementCommand(cmd.cmd)) {
        if (!validateSpeed(cmd.param1)) {
            return false;
        }
        if (cmd.param2 > 0 && !validateDuration(cmd.param2)) {
            return false;
        }
        if (!validateGaitType(cmd.gait_type)) {
            return false;
        }
    }
    
    return true;
}

bool Q8CommandUtils::validateStatus(const StatusResponse& status) {
    // Verify checksum
    uint8_t expected_checksum = calculateChecksum(status);
    return (status.checksum == expected_checksum);
}

bool Q8CommandUtils::isMovementCommand(CommandType cmd) {
    return (cmd >= CMD_MOVE_FORWARD && cmd <= CMD_STRAFE_RIGHT);
}

bool Q8CommandUtils::isSystemCommand(CommandType cmd) {
    return (cmd >= CMD_GET_STATUS && cmd <= CMD_SET_DEBUG_MODE);
}

bool Q8CommandUtils::requiresParameters(CommandType cmd) {
    switch (cmd) {
        case CMD_MOVE_FORWARD:
        case CMD_MOVE_BACKWARD:
        case CMD_TURN_LEFT:
        case CMD_TURN_RIGHT:
        case CMD_MOVE_DIAGONAL_FL:
        case CMD_MOVE_DIAGONAL_FR:
        case CMD_STRAFE_LEFT:
        case CMD_STRAFE_RIGHT:
        case CMD_SET_SPEED:
        case CMD_CHANGE_GAIT:
            return true;
        default:
            return false;
    }
}

bool Q8CommandUtils::validateSpeed(float speed) {
    return (speed >= 0.0 && speed <= 2.0);
}

bool Q8CommandUtils::validateDuration(float duration) {
    return (duration >= 0.0 && duration <= 3600.0); // Max 1 hour
}

bool Q8CommandUtils::validateGaitType(uint8_t gait) {
    return (gait <= GAIT_CMD_PRONK);
}

HighLevelCommand Q8CommandUtils::createMovementCommand(CommandType cmd, float speed, float duration, GaitTypeCmd gait) {
    HighLevelCommand command = {};
    
    command.cmd = cmd;
    command.param1 = speed;
    command.param2 = duration;
    command.gait_type = gait;
    command.flags = (duration > 0) ? FLAG_NONE : FLAG_CONTINUOUS;
    command.sequence_id = 0; // Will be set by sender
    command.checksum = calculateChecksum(command);
    
    return command;
}

HighLevelCommand Q8CommandUtils::createSystemCommand(CommandType cmd, uint8_t flags) {
    HighLevelCommand command = {};
    
    command.cmd = cmd;
    command.param1 = 0.0;
    command.param2 = 0.0;
    command.gait_type = GAIT_CMD_AMBER;
    command.flags = flags;
    command.sequence_id = 0; // Will be set by sender
    command.checksum = calculateChecksum(command);
    
    return command;
}

HighLevelCommand Q8CommandUtils::createGaitCommand(GaitTypeCmd gait, float speed) {
    HighLevelCommand command = {};
    
    command.cmd = CMD_CHANGE_GAIT;
    command.param1 = speed;
    command.param2 = 0.0;
    command.gait_type = gait;
    command.flags = FLAG_NONE;
    command.sequence_id = 0; // Will be set by sender
    command.checksum = calculateChecksum(command);
    
    return command;
}

StatusResponse Q8CommandUtils::createStatusResponse(uint8_t gait, float battery, float speed, bool torque_enabled) {
    StatusResponse status = {};
    
    status.current_gait = gait;
    status.battery_voltage = battery;
    status.current_speed = speed;
    status.torque_enabled = torque_enabled ? 1 : 0;
    status.error_flags = ERROR_NONE;
    status.uptime_seconds = 0; // Will be set by robot
    status.sequence_id = 0; // Will be set by robot
    
    // Initialize joint temperatures to reasonable values
    for (int i = 0; i < 8; i++) {
        status.joint_temperatures[i] = 25; // 25°C default
    }
    
    status.checksum = calculateChecksum(status);
    
    return status;
}

void Q8CommandUtils::setErrorFlag(StatusResponse& status, ErrorFlags error) {
    status.error_flags |= error;
    // Recalculate checksum after modifying error flags
    status.checksum = calculateChecksum(status);
}

bool Q8CommandUtils::hasError(const StatusResponse& status, ErrorFlags error) {
    return (status.error_flags & error) != 0;
}