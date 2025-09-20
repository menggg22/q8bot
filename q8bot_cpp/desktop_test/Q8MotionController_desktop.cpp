/*
  Desktop implementation of Q8MotionController for testing
*/

#include "Q8MotionController_desktop.h"
#include <iostream>
#include <chrono>
#include <cstring>

Q8MotionController::Q8MotionController(q8Dynamixel* dxl) 
    : kinematics(19.5, 25.0, 40.0, 25.0, 40.0),
      gaitGenerator(&kinematics),
      motorController(dxl),
      currentState(MOTION_IDLE),
      currentGait(GAIT_AMBER),
      currentDirection(DIR_FORWARD),
      currentSpeed(RobotConfig::DEFAULT_SPEED),
      torqueEnabled(false),
      gaitLength(0),
      currentStep(0),
      stepStartTime(0),
      stepDuration(50),
      gaitActive(false),
      lastControlUpdate(0),
      lastStatusUpdate(0),
      lastHeartbeat(0),
      lastCommandTime(0),
      errorFlags(ERROR_NONE),
      consecutiveErrors(0) {
    
    activeCommand = {};
    activeCommand.cmd = CMD_IDLE;
    
    robotStatus = {};
    robotStatus.current_gait = GAIT_CMD_AMBER;
    robotStatus.battery_voltage = 3.7;
    robotStatus.current_speed = 0.0;
    robotStatus.torque_enabled = 0;
    robotStatus.error_flags = ERROR_NONE;
    robotStatus.uptime_seconds = 0;
    
    for (int i = 0; i < 8; i++) {
        robotStatus.joint_temperatures[i] = 25;
    }
}

bool Q8MotionController::init() {
    std::cout << "Q8MotionController: Initializing..." << std::endl;
    
    if (!motorController) {
        std::cout << "ERROR: Motor controller not provided" << std::endl;
        errorFlags |= ERROR_SYSTEM_FAULT;
        return false;
    }
    
    setIdlePosition();
    
    lastControlUpdate = millis();
    lastStatusUpdate = millis();
    lastHeartbeat = millis();
    lastCommandTime = millis();
    
    clearErrors();
    
    std::cout << "Q8MotionController: Initialization complete" << std::endl;
    return true;
}

void Q8MotionController::update() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastControlUpdate >= 1) { // 1ms = 1000Hz
        updateControlLoop();
        lastControlUpdate = currentTime;
    }
    
    if (currentTime - lastStatusUpdate >= RobotConfig::STATUS_PERIOD_MS) {
        updateRobotStatus();
        lastStatusUpdate = currentTime;
    }
    
    if (currentTime - lastHeartbeat >= RobotConfig::HEARTBEAT_PERIOD_MS) {
        robotStatus.uptime_seconds = currentTime / 1000;
        lastHeartbeat = currentTime;
    }
    
    if (currentTime - lastCommandTime > RobotConfig::COMMAND_TIMEOUT_MS) {
        if (currentState == MOTION_MOVING || currentState == MOTION_TURNING) {
            transitionToIdle();
        }
    }
}

void Q8MotionController::updateControlLoop() {
    checkSafetyConditions();
    
    if (currentState == MOTION_EMERGENCY) {
        handleEmergencyStop();
        return;
    }
    
    if (gaitActive && currentState != MOTION_IDLE) {
        executeGaitStep();
    }
}

void Q8MotionController::executeGaitStep() {
    if (!gaitActive || gaitLength == 0) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    if (currentTime - stepStartTime >= stepDuration) {
        currentStep++;
        
        if (currentStep >= gaitLength) {
            if (activeCommand.flags & FLAG_CONTINUOUS) {
                currentStep = 0;
            } else {
                stopGaitExecution();
                transitionToIdle();
                return;
            }
        }
        
        stepStartTime = currentTime;
    }
    
    if (currentStep < gaitLength && validateJointPositions(gaitPositions[currentStep])) {
        float positions[8];
        for (int i = 0; i < 8; i++) {
            positions[i] = gaitPositions[currentStep][i];
        }
        
        limitJointPositions(positions);
        
        for (int i = 0; i < 8; i++) {
            motorController->setJointPosition(i, positions[i]);
        }
    } else {
        consecutiveErrors++;
        if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
            emergencyStop();
        }
    }
}

bool Q8MotionController::executeCommand(const HighLevelCommand& cmd) {
    if (!Q8CommandUtils::validateCommand(cmd)) {
        std::cout << "ERROR: Invalid command received" << std::endl;
        return false;
    }
    
    std::cout << "Executing command: " << (int)cmd.cmd << std::endl;
    
    lastCommandTime = millis();
    activeCommand = cmd;
    
    bool success = false;
    
    if (Q8CommandUtils::isMovementCommand(cmd.cmd)) {
        success = executeMovementCommand(cmd);
    } else if (Q8CommandUtils::isSystemCommand(cmd.cmd)) {
        success = executeSystemCommand(cmd);
    } else {
        // Handle individual commands that might not be classified correctly
        switch (cmd.cmd) {
            case CMD_IDLE:
                success = executeSystemCommand(cmd);
                break;
            case CMD_CHANGE_GAIT:
            case CMD_SET_SPEED:
            case CMD_EMERGENCY_STOP:
                success = executeSystemCommand(cmd);
                break;
            default:
                success = executeSpecialCommand(cmd);
                break;
        }
    }
    
    updateRobotStatus();
    return success;
}

bool Q8MotionController::executeMovementCommand(const HighLevelCommand& cmd) {
    if (!torqueEnabled) {
        std::cout << "WARNING: Movement command received but torque disabled" << std::endl;
        errorFlags |= ERROR_SYSTEM_FAULT;
        return false;
    }
    
    Direction newDirection = MotionUtils::commandToDirection(cmd.cmd);
    
    currentSpeed = cmd.param1;
    if (currentSpeed < RobotConfig::MIN_SPEED) currentSpeed = RobotConfig::MIN_SPEED;
    if (currentSpeed > RobotConfig::MAX_SPEED) currentSpeed = RobotConfig::MAX_SPEED;
    
    if (cmd.gait_type <= GAIT_CMD_PRONK) {
        currentGait = MotionUtils::commandToGaitType((GaitTypeCmd)cmd.gait_type);
    }
    
    if (!generateNewGait(currentGait, newDirection)) {
        std::cout << "ERROR: Failed to generate gait" << std::endl;
        errorFlags |= ERROR_GAIT_FAILED;
        return false;
    }
    
    currentDirection = newDirection;
    currentState = (cmd.cmd == CMD_TURN_LEFT || cmd.cmd == CMD_TURN_RIGHT) ? 
                   MOTION_TURNING : MOTION_MOVING;
    
    startGaitExecution();
    
    std::cout << "Motion started: gait=" << (int)currentGait 
              << ", direction=" << (int)currentDirection 
              << ", speed=" << currentSpeed << std::endl;
    
    return true;
}

bool Q8MotionController::executeSystemCommand(const HighLevelCommand& cmd) {
    switch (cmd.cmd) {
        case CMD_IDLE:
            transitionToIdle();
            return true;
            
        case CMD_ENABLE_TORQUE:
            return enableTorque();
            
        case CMD_DISABLE_TORQUE:
            return disableTorque();
            
        case CMD_RESET_POSITION:
            transitionToIdle();
            setIdlePosition();
            return true;
            
        case CMD_EMERGENCY_STOP:
            emergencyStop();
            return true;
            
        case CMD_SET_SPEED:
            setSpeed(cmd.param1);
            return true;
            
        case CMD_CHANGE_GAIT:
            if (cmd.gait_type <= GAIT_CMD_PRONK) {
                setGait(MotionUtils::commandToGaitType((GaitTypeCmd)cmd.gait_type));
                return true;
            }
            return false;
            
        case CMD_GET_STATUS:
        case CMD_GET_BATTERY:
            updateRobotStatus();
            return true;
            
        default:
            std::cout << "WARNING: Unhandled system command: " << (int)cmd.cmd << std::endl;
            return false;
    }
}

bool Q8MotionController::executeSpecialCommand(const HighLevelCommand& cmd) {
    switch (cmd.cmd) {
        case CMD_JUMP:
            std::cout << "JUMP command - not yet implemented" << std::endl;
            return false;
            
        case CMD_DANCE:
            std::cout << "DANCE command - not yet implemented" << std::endl;
            return false;
            
        case CMD_GREETING:
            std::cout << "GREETING command - not yet implemented" << std::endl;
            return false;
            
        default:
            std::cout << "WARNING: Unknown special command" << std::endl;
            return false;
    }
}

bool Q8MotionController::generateNewGait(GaitType gait, Direction dir) {
    bool success = gaitGenerator.generateGait(gait, dir, gaitPositions, gaitLength);
    
    if (!success || gaitLength == 0) {
        std::cout << "ERROR: Gait generation failed" << std::endl;
        return false;
    }
    
    stepDuration = MotionUtils::calculateStepDuration(currentSpeed, gait);
    
    std::cout << "Generated gait: " << gaitLength << " steps, " 
              << stepDuration << "ms per step" << std::endl;
    
    return true;
}

void Q8MotionController::startGaitExecution() {
    currentStep = 0;
    stepStartTime = millis();
    gaitActive = true;
    consecutiveErrors = 0;
}

void Q8MotionController::stopGaitExecution() {
    gaitActive = false;
    currentStep = 0;
}

void Q8MotionController::setIdlePosition() {
    float idlePositions[8] = {45, 135, 45, 135, 45, 135, 45, 135};
    
    if (torqueEnabled) {
        for (int i = 0; i < 8; i++) {
            motorController->setJointPosition(i, idlePositions[i]);
        }
    }
}

void Q8MotionController::transitionToIdle() {
    currentState = MOTION_IDLE;
    stopGaitExecution();
    activeCommand.cmd = CMD_IDLE;
    robotStatus.current_speed = 0.0;
    std::cout << "Transitioned to IDLE state" << std::endl;
}

void Q8MotionController::emergencyStop() {
    std::cout << "EMERGENCY STOP ACTIVATED" << std::endl;
    
    currentState = MOTION_EMERGENCY;
    stopGaitExecution();
    disableTorque();
    
    errorFlags |= ERROR_EMERGENCY_STOP;
    robotStatus.error_flags = errorFlags;
    
    activeCommand.cmd = CMD_EMERGENCY_STOP;
}

void Q8MotionController::handleEmergencyStop() {
    // Emergency stop handling - keep system in safe state
    if (torqueEnabled) {
        disableTorque();
    }
    
    // Update status to reflect emergency state
    robotStatus.current_speed = 0.0;
    robotStatus.torque_enabled = 0;
    robotStatus.error_flags = errorFlags;
}

void Q8MotionController::updateRobotStatus() {
    robotStatus.current_gait = (uint8_t)currentGait;
    robotStatus.current_speed = gaitActive ? currentSpeed : 0.0;
    robotStatus.torque_enabled = torqueEnabled ? 1 : 0;
    robotStatus.error_flags = errorFlags;
    robotStatus.battery_voltage = 3.7; // Mock value
    
    for (int i = 0; i < 8; i++) {
        robotStatus.joint_temperatures[i] = 25 + (i * 2); // Mock values
    }
    
    robotStatus.checksum = Q8CommandUtils::calculateChecksum(robotStatus);
}

void Q8MotionController::checkSafetyConditions() {
    if (!checkBatteryLevel()) {
        errorFlags |= ERROR_LOW_BATTERY;
    }
    
    if (!checkMotorTemperatures()) {
        errorFlags |= ERROR_MOTOR_OVERTEMP;
    }
    
    robotStatus.error_flags = errorFlags;
}

bool Q8MotionController::checkBatteryLevel() {
    return robotStatus.battery_voltage > RobotConfig::BATTERY_LOW_THRESHOLD;
}

bool Q8MotionController::checkMotorTemperatures() {
    for (int i = 0; i < 8; i++) {
        if (robotStatus.joint_temperatures[i] > RobotConfig::MOTOR_TEMP_CRITICAL) {
            return false;
        }
    }
    return true;
}

bool Q8MotionController::validateJointPositions(const float positions[8]) {
    for (int i = 0; i < 8; i++) {
        if (positions[i] < 0 || positions[i] > 180) {
            return false;
        }
    }
    return true;
}

void Q8MotionController::limitJointPositions(float positions[8]) {
    for (int i = 0; i < 8; i++) {
        if (positions[i] < 0) positions[i] = 0;
        if (positions[i] > 180) positions[i] = 180;
    }
}

bool Q8MotionController::enableTorque() {
    if (motorController) {
        motorController->enableTorque();
        torqueEnabled = true;
        robotStatus.torque_enabled = 1;  // Update status immediately
        std::cout << "Torque ENABLED" << std::endl;
        return true;
    }
    return false;
}

bool Q8MotionController::disableTorque() {
    if (motorController) {
        motorController->disableTorque();
        torqueEnabled = false;
        robotStatus.torque_enabled = 0;  // Update status immediately
        std::cout << "Torque DISABLED" << std::endl;
        return true;
    }
    return false;
}

void Q8MotionController::setSpeed(float speed) {
    if (speed >= RobotConfig::MIN_SPEED && speed <= RobotConfig::MAX_SPEED) {
        currentSpeed = speed;
        std::cout << "Speed set to: " << speed << std::endl;
    }
}

void Q8MotionController::setGait(GaitType gait) {
    currentGait = gait;
    robotStatus.current_gait = (uint8_t)gait;
    std::cout << "Gait set to: " << (int)gait << std::endl;
}

MotionState Q8MotionController::getState() const {
    return currentState;
}

bool Q8MotionController::isMoving() const {
    return (currentState == MOTION_MOVING || currentState == MOTION_TURNING) && gaitActive;
}

bool Q8MotionController::isSafe() const {
    return (errorFlags == ERROR_NONE) && (currentState != MOTION_EMERGENCY);
}

StatusResponse Q8MotionController::getStatus() const {
    return robotStatus;
}

HighLevelCommand Q8MotionController::getCurrentCommand() const {
    return activeCommand;
}

uint8_t Q8MotionController::getErrorFlags() const {
    return errorFlags;
}

void Q8MotionController::clearErrors() {
    errorFlags = ERROR_NONE;
    consecutiveErrors = 0;
    robotStatus.error_flags = ERROR_NONE;
    
    if (currentState == MOTION_EMERGENCY) {
        transitionToIdle();
    }
    
    // Recalculate checksum after clearing errors
    robotStatus.checksum = Q8CommandUtils::calculateChecksum(robotStatus);
}

void Q8MotionController::printStatus() const {
    std::cout << "=== Q8 Motion Controller Status ===" << std::endl;
    std::cout << "State: " << (int)currentState << std::endl;
    std::cout << "Gait: " << (int)currentGait << std::endl;
    std::cout << "Speed: " << currentSpeed << std::endl;
    std::cout << "Torque: " << (torqueEnabled ? "ON" : "OFF") << std::endl;
    std::cout << "Active: " << (gaitActive ? "YES" : "NO") << std::endl;
    std::cout << "Errors: 0x" << std::hex << (int)errorFlags << std::dec << std::endl;
    std::cout << "==================================" << std::endl;
}

// Utility function implementations
namespace MotionUtils {
    GaitType commandToGaitType(GaitTypeCmd cmdGait) {
        return (GaitType)cmdGait;
    }
    
    Direction commandToDirection(CommandType cmd) {
        switch (cmd) {
            case CMD_MOVE_FORWARD: return DIR_FORWARD;
            case CMD_MOVE_BACKWARD: return DIR_BACKWARD;
            case CMD_TURN_LEFT: return DIR_LEFT;
            case CMD_TURN_RIGHT: return DIR_RIGHT;
            case CMD_MOVE_DIAGONAL_FL: return DIR_FORWARD_LEFT;
            case CMD_MOVE_DIAGONAL_FR: return DIR_FORWARD_RIGHT;
            case CMD_STRAFE_LEFT: return DIR_LEFT;
            case CMD_STRAFE_RIGHT: return DIR_RIGHT;
            default: return DIR_FORWARD;
        }
    }
    
    unsigned long calculateStepDuration(float speed, GaitType gait) {
        unsigned long baseDuration;
        switch (gait) {
            case GAIT_AMBER: baseDuration = 50; break;
            case GAIT_AMBER_HIGH: baseDuration = 55; break;
            case GAIT_AMBER_LOW: baseDuration = 45; break;
            case GAIT_AMBER_FAST: baseDuration = 40; break;
            case GAIT_WALK: baseDuration = 35; break;
            case GAIT_BOUND: baseDuration = 60; break;
            case GAIT_PRONK: baseDuration = 70; break;
            default: baseDuration = 50; break;
        }
        
        return (unsigned long)(baseDuration / speed);
    }
    
    void interpolatePositions(const float start[8], const float end[8], 
                             float t, float result[8]) {
        for (int i = 0; i < 8; i++) {
            result[i] = start[i] + t * (end[i] - start[i]);
        }
    }
    
    bool isStablePose(const float positions[8]) {
        for (int i = 0; i < 8; i++) {
            if (positions[i] < 10 || positions[i] > 170) {
                return false;
            }
        }
        return true;
    }
}