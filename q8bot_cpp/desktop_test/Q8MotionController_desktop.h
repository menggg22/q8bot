/*
  Desktop version of Q8MotionController for testing without Arduino hardware
*/

#ifndef Q8MotionController_desktop_h
#define Q8MotionController_desktop_h

#include "Q8Kinematics_desktop.h"
#include "Q8GaitGenerator_desktop.h"
#include "Q8Commands_desktop.h"
#include "arduino_mock.h"
#include <iostream>
#include <chrono>

// Mock q8Dynamixel for desktop testing
class q8Dynamixel {
private:
    float jointPositions[8];
    bool torqueEnabled;
    
public:
    q8Dynamixel() : torqueEnabled(false) {
        for (int i = 0; i < 8; i++) {
            jointPositions[i] = (i % 2 == 0) ? 45.0 : 135.0; // Default idle position
        }
    }
    
    void setJointPosition(int joint, float position) {
        if (joint >= 0 && joint < 8) {
            jointPositions[joint] = position;
        }
    }
    
    float getJointPosition(int joint) {
        if (joint >= 0 && joint < 8) {
            return jointPositions[joint];
        }
        return 0.0;
    }
    
    void enableTorque() {
        torqueEnabled = true;
    }
    
    void disableTorque() {
        torqueEnabled = false;
    }
    
    bool isTorqueEnabled() const {
        return torqueEnabled;
    }
    
    void printPositions() const {
        std::cout << "Joint positions: ";
        for (int i = 0; i < 8; i++) {
            std::cout << jointPositions[i] << "° ";
        }
        std::cout << std::endl;
    }
};

// Motion state enumeration
enum MotionState {
    MOTION_IDLE = 0,
    MOTION_MOVING = 1,
    MOTION_TURNING = 2,
    MOTION_SPECIAL = 3,
    MOTION_EMERGENCY = 4,
    MOTION_CALIBRATING = 5
};

// Robot configuration constants (desktop version)
struct RobotConfig {
    static constexpr float BODY_LENGTH = 180.0;
    static constexpr float BODY_WIDTH = 120.0;
    static constexpr float LEG_OFFSET_X = 60.0;
    static constexpr float LEG_OFFSET_Y = 90.0;
    
    static constexpr float DEFAULT_SPEED = 1.0;
    static constexpr float MAX_SPEED = 2.0;
    static constexpr float MIN_SPEED = 0.1;
    
    static constexpr unsigned long CONTROL_PERIOD_US = 1000;
    static constexpr unsigned long STATUS_PERIOD_MS = 100;
    static constexpr unsigned long HEARTBEAT_PERIOD_MS = 1000;
    
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 5000;
    static constexpr float BATTERY_LOW_THRESHOLD = 3.3;
    static constexpr float MOTOR_TEMP_WARNING = 60.0;
    static constexpr float MOTOR_TEMP_CRITICAL = 75.0;
};

class Q8MotionController {
private:
    Q8Kinematics kinematics;
    Q8GaitGenerator gaitGenerator;
    q8Dynamixel* motorController;
    
    MotionState currentState;
    HighLevelCommand activeCommand;
    StatusResponse robotStatus;
    
    GaitType currentGait;
    Direction currentDirection;
    float currentSpeed;
    bool torqueEnabled;
    
    float gaitPositions[200][8];
    int gaitLength;
    int currentStep;
    unsigned long stepStartTime;
    unsigned long stepDuration;
    bool gaitActive;
    
    unsigned long lastControlUpdate;
    unsigned long lastStatusUpdate;
    unsigned long lastHeartbeat;
    unsigned long lastCommandTime;
    
    uint8_t errorFlags;
    uint8_t consecutiveErrors;
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 5;
    
    void updateControlLoop();
    void executeGaitStep();
    void updateRobotStatus();
    void checkSafetyConditions();
    void handleEmergencyStop();
    void transitionToIdle();
    
    bool executeMovementCommand(const HighLevelCommand& cmd);
    bool executeSystemCommand(const HighLevelCommand& cmd);
    bool executeSpecialCommand(const HighLevelCommand& cmd);
    
    bool generateNewGait(GaitType gait, Direction dir);
    void startGaitExecution();
    void stopGaitExecution();
    void setIdlePosition();
    
    bool validateJointPositions(const float positions[8]);
    void limitJointPositions(float positions[8]);
    bool checkBatteryLevel();
    bool checkMotorTemperatures();
    
public:
    Q8MotionController(q8Dynamixel* dxl);
    
    bool init();
    void update();
    void shutdown();
    
    bool executeCommand(const HighLevelCommand& cmd);
    HighLevelCommand getCurrentCommand() const;
    StatusResponse getStatus() const;
    
    MotionState getState() const;
    bool isMoving() const;
    bool isSafe() const;
    
    void setSpeed(float speed);
    void setGait(GaitType gait);
    bool enableTorque();
    bool disableTorque();
    
    void emergencyStop();
    void resetSystem();
    
    void updateStatus();
    uint8_t getErrorFlags() const;
    void clearErrors();
    
    bool calibrateSystem();
    bool testMotors();
    
    void printStatus() const;
    void printGaitInfo() const;
    void setDebugMode(bool enabled);
};

namespace MotionUtils {
    GaitType commandToGaitType(GaitTypeCmd cmdGait);
    Direction commandToDirection(CommandType cmd);
    unsigned long calculateStepDuration(float speed, GaitType gait);
    void interpolatePositions(const float start[8], const float end[8], 
                             float t, float result[8]);
    bool isStablePose(const float positions[8]);
}

#endif