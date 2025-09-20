/*
  Q8MotionController Implementation
  Autonomous motion control system integrating all Q8bot subsystems
*/

#include "Q8MotionController.h"
#include <Arduino.h>

Q8MotionController::Q8MotionController(q8Dynamixel* dxl) 
    : kinematics(19.5, 25.0, 40.0, 25.0, 40.0),
      gaitGenerator(&kinematics),
      motorController(dxl),
      currentState(MOTION_IDLE),
      currentGait(GAIT_AMBER),
      currentDirection(FORWARD),
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
    
    // Initialize active command
    activeCommand = {};
    activeCommand.cmd = CMD_IDLE;
    
    // Initialize robot status
    robotStatus = {};
    robotStatus.current_gait = GAIT_CMD_AMBER;
    robotStatus.battery_voltage = 0.0;
    robotStatus.current_speed = 0.0;
    robotStatus.torque_enabled = 0;
    robotStatus.error_flags = ERROR_NONE;
    robotStatus.uptime_seconds = 0;
    
    // Initialize joint temperatures
    for (int i = 0; i < 8; i++) {
        robotStatus.joint_temperatures[i] = 25;
    }
}

bool Q8MotionController::init() {
    Serial.println("Q8MotionController: Initializing...");
    
    // Initialize motor controller
    if (!motorController) {
        Serial.println("ERROR: Motor controller not provided");
        errorFlags |= ERROR_SYSTEM_FAULT;
        return false;
    }
    
    // Set initial idle position
    setIdlePosition();
    
    // Initialize timing
    lastControlUpdate = micros();
    lastStatusUpdate = millis();
    lastHeartbeat = millis();
    lastCommandTime = millis();
    
    // Clear any existing errors
    clearErrors();
    
    Serial.println("Q8MotionController: Initialization complete");
    return true;
}

void Q8MotionController::update() {
    unsigned long currentTime = micros();
    
    // Check if it's time for control update (1000Hz)
    if (currentTime - lastControlUpdate >= RobotConfig::CONTROL_PERIOD_US) {
        updateControlLoop();
        lastControlUpdate = currentTime;
    }
    
    // Check for status updates (10Hz)
    unsigned long currentMillis = millis();
    if (currentMillis - lastStatusUpdate >= RobotConfig::STATUS_PERIOD_MS) {
        updateRobotStatus();
        lastStatusUpdate = currentMillis;
    }
    
    // Check for heartbeat (1Hz)
    if (currentMillis - lastHeartbeat >= RobotConfig::HEARTBEAT_PERIOD_MS) {
        robotStatus.uptime_seconds = currentMillis / 1000;
        lastHeartbeat = currentMillis;
    }
    
    // Check for command timeout
    if (currentMillis - lastCommandTime > RobotConfig::COMMAND_TIMEOUT_MS) {
        if (currentState == MOTION_MOVING || currentState == MOTION_TURNING) {
            Serial.println("Command timeout - stopping motion");
            transitionToIdle();
        }
    }
}

void Q8MotionController::updateControlLoop() {
    // Check safety conditions first
    checkSafetyConditions();
    
    // Handle emergency state
    if (currentState == MOTION_EMERGENCY) {
        handleEmergencyStop();
        return;
    }
    
    // Execute current gait step if active
    if (gaitActive && currentState != MOTION_IDLE) {
        executeGaitStep();
    }
    
    // Update motor positions based on current state
    if (torqueEnabled && currentState != MOTION_EMERGENCY) {
        // Motors are handled by executeGaitStep() or setIdlePosition()
    }
}

void Q8MotionController::executeGaitStep() {
    if (!gaitActive || gaitLength == 0) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Check if it's time for the next step
    if (currentTime - stepStartTime >= stepDuration) {
        currentStep++;
        
        // Check if gait is complete
        if (currentStep >= gaitLength) {
            // For continuous movement, restart gait
            if (activeCommand.flags & FLAG_CONTINUOUS) {
                currentStep = 0;
            } else {
                // Motion complete, transition to idle
                stopGaitExecution();
                transitionToIdle();
                return;
            }
        }
        
        stepStartTime = currentTime;
    }
    
    // Send current step positions to motors
    if (currentStep < gaitLength && validateJointPositions(gaitPositions[currentStep])) {
        float positions[8];
        for (int i = 0; i < 8; i++) {
            positions[i] = gaitPositions[currentStep][i];
        }
        
        limitJointPositions(positions);
        
        // Send to motors
        for (int i = 0; i < 8; i++) {
            motorController->setJointPosition(i, positions[i]);
        }
    } else {
        Serial.println("WARNING: Invalid joint positions detected");
        consecutiveErrors++;
        if (consecutiveErrors >= MAX_CONSECUTIVE_ERRORS) {
            emergencyStop();
        }
    }
}

bool Q8MotionController::executeCommand(const HighLevelCommand& cmd) {
    // Validate command first
    if (!Q8CommandUtils::validateCommand(cmd)) {
        Serial.println("ERROR: Invalid command received");
        return false;
    }
    
    Serial.print("Executing command: ");
    Serial.println((int)cmd.cmd);
    
    // Update last command time
    lastCommandTime = millis();
    
    // Store active command
    activeCommand = cmd;
    
    // Execute based on command type
    bool success = false;
    
    if (Q8CommandUtils::isMovementCommand(cmd.cmd)) {
        success = executeMovementCommand(cmd);
    } else if (Q8CommandUtils::isSystemCommand(cmd.cmd)) {
        success = executeSystemCommand(cmd);
    } else {
        success = executeSpecialCommand(cmd);
    }
    
    // Update robot status
    updateRobotStatus();
    
    return success;
}

bool Q8MotionController::executeMovementCommand(const HighLevelCommand& cmd) {
    // Check if torque is enabled
    if (!torqueEnabled) {
        Serial.println("WARNING: Movement command received but torque disabled");
        errorFlags |= ERROR_SYSTEM_FAULT;
        return false;
    }
    
    // Convert command to direction
    Direction newDirection = MotionUtils::commandToDirection(cmd.cmd);
    
    // Update motion parameters
    currentSpeed = cmd.param1;
    if (currentSpeed < RobotConfig::MIN_SPEED) currentSpeed = RobotConfig::MIN_SPEED;
    if (currentSpeed > RobotConfig::MAX_SPEED) currentSpeed = RobotConfig::MAX_SPEED;
    
    // Update gait if specified
    if (cmd.gait_type <= GAIT_CMD_PRONK) {
        currentGait = MotionUtils::commandToGaitType((GaitTypeCmd)cmd.gait_type);
    }
    
    // Generate new gait
    if (!generateNewGait(currentGait, newDirection)) {
        Serial.println("ERROR: Failed to generate gait");
        errorFlags |= ERROR_GAIT_FAILED;
        return false;
    }
    
    // Update state
    currentDirection = newDirection;
    currentState = (cmd.cmd == CMD_TURN_LEFT || cmd.cmd == CMD_TURN_RIGHT) ? 
                   MOTION_TURNING : MOTION_MOVING;
    
    // Start gait execution  
    startGaitExecution();
    
    Serial.print("Motion started: gait=");
    Serial.print((int)currentGait);
    Serial.print(", direction=");
    Serial.print((int)currentDirection);
    Serial.print(", speed=");
    Serial.println(currentSpeed);
    
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
            Serial.println("WARNING: Unhandled system command");
            return false;
    }
}

bool Q8MotionController::executeSpecialCommand(const HighLevelCommand& cmd) {
    switch (cmd.cmd) {
        case CMD_JUMP:
            Serial.println("JUMP command - not yet implemented");
            return false;  // TODO: Implement jump sequence
            
        case CMD_DANCE:
            Serial.println("DANCE command - not yet implemented");
            return false;  // TODO: Implement dance routine
            
        case CMD_GREETING:
            Serial.println("GREETING command - not yet implemented");
            return false;  // TODO: Implement greeting gesture
            
        default:
            Serial.println("WARNING: Unknown special command");
            return false;
    }
}

bool Q8MotionController::generateNewGait(GaitType gait, Direction dir) {
    // Generate gait trajectory
    bool success = gaitGenerator.generateGait(gait, dir, gaitPositions, gaitLength);
    
    if (!success || gaitLength == 0) {
        Serial.println("ERROR: Gait generation failed");
        return false;
    }
    
    // Calculate step duration based on speed
    stepDuration = MotionUtils::calculateStepDuration(currentSpeed, gait);
    
    Serial.print("Generated gait: ");
    Serial.print(gaitLength);
    Serial.print(" steps, ");
    Serial.print(stepDuration);
    Serial.println("ms per step");
    
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
    // Set all joints to idle position (45°, 135° per leg)
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
    
    // Update status
    robotStatus.current_speed = 0.0;
    
    Serial.println("Transitioned to IDLE state");
}

void Q8MotionController::emergencyStop() {
    Serial.println("EMERGENCY STOP ACTIVATED");
    
    currentState = MOTION_EMERGENCY;
    stopGaitExecution();
    
    // Disable torque immediately
    disableTorque();
    
    // Set error flag
    errorFlags |= ERROR_EMERGENCY_STOP;
    robotStatus.error_flags = errorFlags;
    
    // Clear active command
    activeCommand.cmd = CMD_EMERGENCY_STOP;
}

void Q8MotionController::updateRobotStatus() {
    // Update basic status
    robotStatus.current_gait = (uint8_t)currentGait;
    robotStatus.current_speed = gaitActive ? currentSpeed : 0.0;
    robotStatus.torque_enabled = torqueEnabled ? 1 : 0;
    robotStatus.error_flags = errorFlags;
    
    // Read battery voltage (mock for now)
    robotStatus.battery_voltage = 3.7; // TODO: Read actual battery voltage
    
    // Read motor temperatures (mock for now)
    for (int i = 0; i < 8; i++) {
        robotStatus.joint_temperatures[i] = 25 + (i * 2); // TODO: Read actual temperatures
    }
    
    // Calculate checksum
    robotStatus.checksum = Q8CommandUtils::calculateChecksum(robotStatus);
}

void Q8MotionController::checkSafetyConditions() {
    // Check battery level
    if (!checkBatteryLevel()) {
        errorFlags |= ERROR_LOW_BATTERY;
    }
    
    // Check motor temperatures
    if (!checkMotorTemperatures()) {
        errorFlags |= ERROR_MOTOR_OVERTEMP;
    }
    
    // Update error flags in status
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
        Serial.println("Torque ENABLED");
        return true;
    }
    return false;
}

bool Q8MotionController::disableTorque() {
    if (motorController) {
        motorController->disableTorque();
        torqueEnabled = false;
        Serial.println("Torque DISABLED");
        return true;
    }
    return false;
}

void Q8MotionController::setSpeed(float speed) {
    if (speed >= RobotConfig::MIN_SPEED && speed <= RobotConfig::MAX_SPEED) {
        currentSpeed = speed;
        Serial.print("Speed set to: ");
        Serial.println(speed);
    }
}

void Q8MotionController::setGait(GaitType gait) {
    currentGait = gait;
    robotStatus.current_gait = (uint8_t)gait;
    Serial.print("Gait set to: ");
    Serial.println((int)gait);
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
    
    // If we were in emergency state and errors are cleared, go to idle
    if (currentState == MOTION_EMERGENCY) {
        transitionToIdle();
    }
}

void Q8MotionController::printStatus() const {
    Serial.println("=== Q8 Motion Controller Status ===");
    Serial.print("State: ");
    Serial.println((int)currentState);
    Serial.print("Gait: ");
    Serial.println((int)currentGait);
    Serial.print("Speed: ");
    Serial.println(currentSpeed);
    Serial.print("Torque: ");
    Serial.println(torqueEnabled ? "ON" : "OFF");
    Serial.print("Active: ");
    Serial.println(gaitActive ? "YES" : "NO");
    Serial.print("Errors: 0x");
    Serial.println(errorFlags, HEX);
    Serial.println("==================================");
}

// Utility function implementations
namespace MotionUtils {
    GaitType commandToGaitType(GaitTypeCmd cmdGait) {
        return (GaitType)cmdGait;  // Direct mapping
    }
    
    Direction commandToDirection(CommandType cmd) {
        switch (cmd) {
            case CMD_MOVE_FORWARD: return FORWARD;
            case CMD_MOVE_BACKWARD: return BACKWARD;
            case CMD_TURN_LEFT: return LEFT;
            case CMD_TURN_RIGHT: return RIGHT;
            case CMD_MOVE_DIAGONAL_FL: return FORWARD_LEFT;
            case CMD_MOVE_DIAGONAL_FR: return FORWARD_RIGHT;
            case CMD_STRAFE_LEFT: return LEFT;
            case CMD_STRAFE_RIGHT: return RIGHT;
            default: return FORWARD;
        }
    }
    
    unsigned long calculateStepDuration(float speed, GaitType gait) {
        // Base step duration (ms) for different gaits at speed 1.0
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
        
        // Adjust for speed (faster speed = shorter duration)
        return (unsigned long)(baseDuration / speed);
    }
    
    void interpolatePositions(const float start[8], const float end[8], 
                             float t, float result[8]) {
        for (int i = 0; i < 8; i++) {
            result[i] = start[i] + t * (end[i] - start[i]);
        }
    }
    
    bool isStablePose(const float positions[8]) {
        // Simple stability check - all joints within reasonable range
        for (int i = 0; i < 8; i++) {
            if (positions[i] < 10 || positions[i] > 170) {
                return false;
            }
        }
        return true;
    }
}