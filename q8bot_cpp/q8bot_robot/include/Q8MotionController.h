/*
  Q8MotionController - Autonomous Motion Control System
  Integrates kinematics, gait generation, and command handling for autonomous robot control
*/

#ifndef Q8MotionController_h
#define Q8MotionController_h

#include "Q8Kinematics.h"
#include "Q8GaitGenerator.h"
#include "Q8Commands.h"
#include "q8Dynamixel.h"
#include <Arduino.h>

// Motion state enumeration
enum MotionState {
    MOTION_IDLE = 0,
    MOTION_MOVING = 1,
    MOTION_TURNING = 2,
    MOTION_SPECIAL = 3,     // Jump, dance, etc.
    MOTION_EMERGENCY = 4,
    MOTION_CALIBRATING = 5
};

// Robot configuration constants
struct RobotConfig {
    // Physical parameters
    static constexpr float BODY_LENGTH = 180.0;   // mm
    static constexpr float BODY_WIDTH = 120.0;    // mm
    static constexpr float LEG_OFFSET_X = 60.0;   // mm from center
    static constexpr float LEG_OFFSET_Y = 90.0;   // mm from center
    
    // Control parameters
    static constexpr float DEFAULT_SPEED = 1.0;
    static constexpr float MAX_SPEED = 2.0;
    static constexpr float MIN_SPEED = 0.1;
    
    // Timing parameters
    static constexpr unsigned long CONTROL_PERIOD_US = 1000;  // 1ms = 1000Hz
    static constexpr unsigned long STATUS_PERIOD_MS = 100;    // 10Hz status updates
    static constexpr unsigned long HEARTBEAT_PERIOD_MS = 1000; // 1Hz heartbeat
    
    // Safety parameters
    static constexpr unsigned long COMMAND_TIMEOUT_MS = 5000;  // 5 second timeout
    static constexpr float BATTERY_LOW_THRESHOLD = 3.3;        // Volts
    static constexpr float MOTOR_TEMP_WARNING = 60.0;         // Celsius
    static constexpr float MOTOR_TEMP_CRITICAL = 75.0;        // Celsius
};

class Q8MotionController {
private:
    // Core systems
    Q8Kinematics kinematics;
    Q8GaitGenerator gaitGenerator;
    q8Dynamixel* motorController;
    
    // Current state
    MotionState currentState;
    HighLevelCommand activeCommand;
    StatusResponse robotStatus;
    
    // Motion parameters
    GaitType currentGait;
    Direction currentDirection;
    float currentSpeed;
    bool torqueEnabled;
    
    // Gait execution state
    float gaitPositions[200][8];  // Pre-calculated joint positions
    int gaitLength;
    int currentStep;
    unsigned long stepStartTime;
    unsigned long stepDuration;
    bool gaitActive;
    
    // Timing control
    unsigned long lastControlUpdate;
    unsigned long lastStatusUpdate;
    unsigned long lastHeartbeat;
    unsigned long lastCommandTime;
    
    // Error handling
    uint8_t errorFlags;
    uint8_t consecutiveErrors;
    static constexpr uint8_t MAX_CONSECUTIVE_ERRORS = 5;
    
    // Internal methods
    void updateControlLoop();
    void executeGaitStep();
    void updateRobotStatus();
    void checkSafetyConditions();
    void handleEmergencyStop();
    void transitionToIdle();
    
    // Command execution methods
    bool executeMovementCommand(const HighLevelCommand& cmd);
    bool executeSystemCommand(const HighLevelCommand& cmd);
    bool executeSpecialCommand(const HighLevelCommand& cmd);
    
    // Gait management
    bool generateNewGait(GaitType gait, Direction dir);
    void startGaitExecution();
    void stopGaitExecution();
    void setIdlePosition();
    
    // Safety and validation
    bool validateJointPositions(const float positions[8]);
    void limitJointPositions(float positions[8]);
    bool checkBatteryLevel();
    bool checkMotorTemperatures();
    
public:
    Q8MotionController(q8Dynamixel* dxl);
    
    // Initialization and control
    bool init();
    void update();  // Main update loop - call at 1000Hz
    void shutdown();
    
    // Command interface
    bool executeCommand(const HighLevelCommand& cmd);
    HighLevelCommand getCurrentCommand() const;
    StatusResponse getStatus() const;
    
    // State management
    MotionState getState() const;
    bool isMoving() const;
    bool isSafe() const;
    
    // Configuration
    void setSpeed(float speed);
    void setGait(GaitType gait);
    bool enableTorque();
    bool disableTorque();
    
    // Emergency controls
    void emergencyStop();
    void resetSystem();
    
    // Status and diagnostics
    void updateStatus();
    uint8_t getErrorFlags() const;
    void clearErrors();
    
    // Calibration
    bool calibrateSystem();
    bool testMotors();
    
    // Debug and monitoring
    void printStatus() const;
    void printGaitInfo() const;
    void setDebugMode(bool enabled);
};

// Utility functions for motion control
namespace MotionUtils {
    // Convert gait command enum to internal gait type
    GaitType commandToGaitType(GaitTypeCmd cmdGait);
    
    // Convert movement command to direction
    Direction commandToDirection(CommandType cmd);
    
    // Calculate step duration from speed
    unsigned long calculateStepDuration(float speed, GaitType gait);
    
    // Interpolate between two joint positions
    void interpolatePositions(const float start[8], const float end[8], 
                             float t, float result[8]);
    
    // Validate robot pose is stable
    bool isStablePose(const float positions[8]);
}

#endif