/*
  Autonomous Control Loop Integration
  Integration code to be added to main.cpp for full autonomous control
*/

#ifndef AUTONOMOUS_CONTROL_H
#define AUTONOMOUS_CONTROL_H

#include "Q8MotionController.h"
#include "Q8Commands.h"
#include "q8Dynamixel.h"
#include <Arduino.h>

// Global autonomous control system
extern Q8MotionController* motionController;
extern q8Dynamixel* dxlController;

// Communication buffers
extern char commandBuffer[256];
extern int bufferIndex;
extern bool commandReady;

// Function declarations for autonomous control
void initAutonomousControl();
void autonomousControlLoop();
void handleIncomingCommand();
void sendStatusUpdate();
void processSerialCommand();
void sendResponse(const char* response);

// Integration with existing main.cpp:
/*

// Add these includes to main.cpp:
#include "autonomous_control.h"

// Add these global variables to main.cpp:
Q8MotionController* motionController = nullptr;
q8Dynamixel* dxlController = nullptr; // Should reference existing dxl object
char commandBuffer[256];
int bufferIndex = 0;
bool commandReady = false;

// Modify setup() function in main.cpp:
void setup() {
    // ... existing setup code ...
    
    // Initialize autonomous control system
    initAutonomousControl();
}

// Modify loop() function in main.cpp:
void loop() {
    // High priority: Autonomous control loop (1000Hz)
    autonomousControlLoop();
    
    // Medium priority: Handle incoming commands
    if (Serial.available()) {
        handleIncomingCommand();
    }
    
    // Low priority: Send status updates (10Hz)
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus >= 100) {
        sendStatusUpdate();
        lastStatus = millis();
    }
    
    // ... any other existing loop code ...
}

*/

#endif

// Implementation of autonomous control functions
void initAutonomousControl() {
    Serial.println("Initializing Autonomous Control System...");
    
    // Initialize motion controller with existing DXL controller
    if (dxlController) {
        motionController = new Q8MotionController(dxlController);
        
        if (motionController->init()) {
            Serial.println("✅ Autonomous Control System initialized successfully");
        } else {
            Serial.println("❌ Failed to initialize Autonomous Control System");
        }
    } else {
        Serial.println("❌ DXL Controller not available for autonomous control");
    }
    
    // Initialize command buffer
    bufferIndex = 0;
    commandReady = false;
    memset(commandBuffer, 0, sizeof(commandBuffer));
}

void autonomousControlLoop() {
    static unsigned long lastUpdate = 0;
    const unsigned long CONTROL_PERIOD = 1000; // 1ms = 1000Hz in microseconds
    
    unsigned long currentTime = micros();
    if (currentTime - lastUpdate >= CONTROL_PERIOD) {
        if (motionController) {
            motionController->update();
        }
        lastUpdate = currentTime;
    }
}

void handleIncomingCommand() {
    while (Serial.available() && !commandReady) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                commandBuffer[bufferIndex] = '\0';
                commandReady = true;
                processSerialCommand();
            }
        } else if (bufferIndex < sizeof(commandBuffer) - 1) {
            commandBuffer[bufferIndex++] = c;
        }
    }
}

void processSerialCommand() {
    if (!commandReady || !motionController) {
        return;
    }
    
    // Parse command from buffer
    Serial.print("Received command: ");
    Serial.println(commandBuffer);
    
    // Simple command parsing - expect format: "CMD_TYPE,param1,param2,flags"
    char* token = strtok(commandBuffer, ",");
    if (!token) {
        sendResponse("ERROR: Invalid command format");
        goto cleanup;
    }
    
    // Parse command type
    CommandType cmdType = CMD_INVALID;
    if (strcmp(token, "CMD_MOVE_FORWARD") == 0) cmdType = CMD_MOVE_FORWARD;
    else if (strcmp(token, "CMD_MOVE_BACKWARD") == 0) cmdType = CMD_MOVE_BACKWARD;
    else if (strcmp(token, "CMD_TURN_LEFT") == 0) cmdType = CMD_TURN_LEFT;
    else if (strcmp(token, "CMD_TURN_RIGHT") == 0) cmdType = CMD_TURN_RIGHT;
    else if (strcmp(token, "CMD_MOVE_DIAGONAL_FL") == 0) cmdType = CMD_MOVE_DIAGONAL_FL;
    else if (strcmp(token, "CMD_MOVE_DIAGONAL_FR") == 0) cmdType = CMD_MOVE_DIAGONAL_FR;
    else if (strcmp(token, "CMD_STRAFE_LEFT") == 0) cmdType = CMD_STRAFE_LEFT;
    else if (strcmp(token, "CMD_STRAFE_RIGHT") == 0) cmdType = CMD_STRAFE_RIGHT;
    else if (strcmp(token, "CMD_CHANGE_GAIT") == 0) cmdType = CMD_CHANGE_GAIT;
    else if (strcmp(token, "CMD_SET_SPEED") == 0) cmdType = CMD_SET_SPEED;
    else if (strcmp(token, "CMD_EMERGENCY_STOP") == 0) cmdType = CMD_EMERGENCY_STOP;
    else if (strcmp(token, "CMD_JUMP") == 0) cmdType = CMD_JUMP;
    else if (strcmp(token, "CMD_GET_STATUS") == 0) cmdType = CMD_GET_STATUS;
    else if (strcmp(token, "CMD_ENABLE_TORQUE") == 0) cmdType = CMD_ENABLE_TORQUE;
    else if (strcmp(token, "CMD_DISABLE_TORQUE") == 0) cmdType = CMD_DISABLE_TORQUE;
    else if (strcmp(token, "CMD_RESET_POSITION") == 0) cmdType = CMD_RESET_POSITION;
    else if (strcmp(token, "CMD_IDLE") == 0) cmdType = CMD_IDLE;
    
    if (cmdType == CMD_INVALID) {
        sendResponse("ERROR: Unknown command type");
        goto cleanup;
    }
    
    // Parse parameters
    float param1 = 0.0, param2 = 0.0;
    uint8_t flags = 0;
    
    token = strtok(NULL, ",");
    if (token) param1 = atof(token);
    
    token = strtok(NULL, ",");
    if (token) param2 = atof(token);
    
    token = strtok(NULL, ",");
    if (token) flags = atoi(token);
    
    // Create command structure
    HighLevelCommand cmd = Q8CommandUtils::createMovementCommand(cmdType, param1, param2, GAIT_CMD_AMBER);
    cmd.flags = flags;
    
    // Execute command
    bool success = motionController->executeCommand(cmd);
    
    if (success) {
        sendResponse("OK: Command executed");
    } else {
        sendResponse("ERROR: Command execution failed");
    }
    
cleanup:
    // Reset buffer
    bufferIndex = 0;
    commandReady = false;
    memset(commandBuffer, 0, sizeof(commandBuffer));
}

void sendStatusUpdate() {
    if (!motionController) {
        return;
    }
    
    StatusResponse status = motionController->getStatus();
    
    // Send status as JSON-like format for easy parsing
    Serial.print("{\"status\":{");
    Serial.print("\"gait\":");
    Serial.print(status.current_gait);
    Serial.print(",\"battery\":");
    Serial.print(status.battery_voltage, 2);
    Serial.print(",\"speed\":");
    Serial.print(status.current_speed, 2);
    Serial.print(",\"torque\":");
    Serial.print(status.torque_enabled);
    Serial.print(",\"errors\":");
    Serial.print(status.error_flags);
    Serial.print(",\"uptime\":");
    Serial.print(status.uptime_seconds);
    Serial.print(",\"state\":");
    Serial.print((int)motionController->getState());
    Serial.print(",\"moving\":");
    Serial.print(motionController->isMoving() ? 1 : 0);
    Serial.print(",\"safe\":");
    Serial.print(motionController->isSafe() ? 1 : 0);
    Serial.println("}}");
}

void sendResponse(const char* response) {
    Serial.print("{\"response\":\"");
    Serial.print(response);
    Serial.println("\"}");
}