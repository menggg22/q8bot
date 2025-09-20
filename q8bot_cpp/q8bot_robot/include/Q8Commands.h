/*
  Q8Commands.h - High-level command definitions for autonomous Q8bot control
  Replaces low-level joint position streaming with autonomous commands
  Created for Phase 2 implementation
*/

#ifndef Q8Commands_h
#define Q8Commands_h

#include <stdint.h>

// High-level command types for autonomous control
enum CommandType : uint8_t {
    // Movement commands (0x01-0x0F)
    CMD_IDLE = 0x00,                    // Stop all movement, maintain position
    CMD_MOVE_FORWARD = 0x01,            // Move forward at specified speed
    CMD_MOVE_BACKWARD = 0x02,           // Move backward at specified speed
    CMD_TURN_LEFT = 0x03,               // Turn left (rotate in place)
    CMD_TURN_RIGHT = 0x04,              // Turn right (rotate in place)
    CMD_MOVE_DIAGONAL_FL = 0x05,        // Move forward-left diagonal
    CMD_MOVE_DIAGONAL_FR = 0x06,        // Move forward-right diagonal
    CMD_STRAFE_LEFT = 0x07,             // Strafe left (sideways movement)
    CMD_STRAFE_RIGHT = 0x08,            // Strafe right (sideways movement)
    
    // Gait control commands (0x10-0x1F)
    CMD_CHANGE_GAIT = 0x10,             // Change gait type (AMBER, WALK, etc.)
    CMD_SET_SPEED = 0x11,               // Set movement speed multiplier
    CMD_SET_GAIT_PARAMS = 0x12,         // Custom gait parameters
    
    // Special movement commands (0x20-0x2F)
    CMD_EMERGENCY_STOP = 0x20,          // Immediate stop and disable torque
    CMD_JUMP = 0x21,                    // Execute jump sequence
    CMD_DANCE = 0x22,                   // Execute dance routine
    CMD_GREETING = 0x23,                // Execute greeting gesture
    
    // System commands (0x30-0x3F)
    CMD_GET_STATUS = 0x30,              // Request robot status
    CMD_GET_BATTERY = 0x31,             // Request battery level
    CMD_GET_SENSOR_DATA = 0x32,         // Request sensor readings
    CMD_CALIBRATE = 0x33,               // Run calibration routine
    
    // Control commands (0x40-0x4F)
    CMD_ENABLE_TORQUE = 0x40,           // Enable motor torque
    CMD_DISABLE_TORQUE = 0x41,          // Disable motor torque
    CMD_RESET_POSITION = 0x42,          // Reset to idle position
    CMD_SET_DEBUG_MODE = 0x43,          // Enable/disable debug output
    
    // Data commands (0x50-0x5F)
    CMD_START_RECORDING = 0x50,         // Start sensor data recording
    CMD_STOP_RECORDING = 0x51,          // Stop recording and send data
    CMD_STREAM_DATA = 0x52,             // Start real-time data streaming
    
    CMD_INVALID = 0xFF                  // Invalid command marker
};

// Gait type enumeration (matches Q8GaitGenerator)
enum GaitTypeCmd : uint8_t {
    GAIT_CMD_AMBER = 0,
    GAIT_CMD_AMBER_HIGH = 1,
    GAIT_CMD_AMBER_LOW = 2,
    GAIT_CMD_AMBER_FAST = 3,
    GAIT_CMD_WALK = 4,
    GAIT_CMD_BOUND = 5,
    GAIT_CMD_PRONK = 6
};

// Command packet structure - optimized for ESPNow (max 250 bytes)
struct HighLevelCommand {
    CommandType cmd;          // Command type (1 byte)
    float param1;            // Primary parameter (4 bytes) - speed/angle/duration
    float param2;            // Secondary parameter (4 bytes) - duration/range/unused
    uint8_t gait_type;       // Gait type for movement commands (1 byte)
    uint8_t flags;           // Additional flags (1 byte)
    uint16_t sequence_id;    // Command sequence number (2 bytes)
    uint8_t checksum;        // Data integrity check (1 byte)
    uint8_t reserved[2];     // Reserved for future use (2 bytes)
} __attribute__((packed));   // Total: 16 bytes

// Status response structure
struct StatusResponse {
    uint8_t current_gait;       // Currently active gait
    float battery_voltage;      // Battery voltage (V)
    float current_speed;        // Current movement speed
    uint8_t torque_enabled;     // Torque enable status (0/1)
    uint8_t error_flags;        // Error condition flags
    uint16_t uptime_seconds;    // Robot uptime in seconds
    uint8_t joint_temperatures[8]; // Joint temperatures (°C)
    uint16_t sequence_id;       // Response sequence number
    uint8_t checksum;           // Data integrity check
} __attribute__((packed));      // Total: 24 bytes

// Error flags for status response
enum ErrorFlags : uint8_t {
    ERROR_NONE = 0x00,
    ERROR_IK_FAILED = 0x01,
    ERROR_GAIT_FAILED = 0x02,
    ERROR_MOTOR_OVERTEMP = 0x04,
    ERROR_LOW_BATTERY = 0x08,
    ERROR_COMMUNICATION = 0x10,
    ERROR_SENSOR_FAULT = 0x20,
    ERROR_EMERGENCY_STOP = 0x40,
    ERROR_SYSTEM_FAULT = 0x80
};

// Command flags
enum CommandFlags : uint8_t {
    FLAG_NONE = 0x00,
    FLAG_CONTINUOUS = 0x01,      // Continue until stopped
    FLAG_PRECISE_TIMING = 0x02,  // Use precise timing control
    FLAG_GENTLE_START = 0x04,    // Gradual acceleration
    FLAG_GENTLE_STOP = 0x08,     // Gradual deceleration
    FLAG_RETURN_DATA = 0x10,     // Return sensor data with response
    FLAG_DEBUG_MODE = 0x20       // Enable debug output
};

// Protocol constants
static const uint16_t PROTOCOL_VERSION = 0x0200;  // Version 2.0
static const uint16_t MAX_COMMAND_QUEUE = 10;     // Maximum queued commands
static const uint32_t COMMAND_TIMEOUT_MS = 5000;  // Command timeout (5 seconds)
static const uint32_t HEARTBEAT_INTERVAL_MS = 1000; // Status update interval

// Utility functions for command validation and processing
class Q8CommandUtils {
public:
    // Calculate checksum for command packet
    static uint8_t calculateChecksum(const HighLevelCommand& cmd);
    static uint8_t calculateChecksum(const StatusResponse& status);
    
    // Validate command packet
    static bool validateCommand(const HighLevelCommand& cmd);
    static bool validateStatus(const StatusResponse& status);
    
    // Command type validation
    static bool isMovementCommand(CommandType cmd);
    static bool isSystemCommand(CommandType cmd);
    static bool requiresParameters(CommandType cmd);
    
    // Parameter validation
    static bool validateSpeed(float speed);          // 0.0 to 2.0
    static bool validateDuration(float duration);    // 0.0 to 3600.0 seconds
    static bool validateGaitType(uint8_t gait);      // 0 to 6
    
    // Command formatting helpers
    static HighLevelCommand createMovementCommand(CommandType cmd, float speed, float duration = 0.0, GaitTypeCmd gait = GAIT_CMD_AMBER);
    static HighLevelCommand createSystemCommand(CommandType cmd, uint8_t flags = FLAG_NONE);
    static HighLevelCommand createGaitCommand(GaitTypeCmd gait, float speed = 1.0);
    
    // Status response helpers
    static StatusResponse createStatusResponse(uint8_t gait, float battery, float speed, bool torque_enabled);
    static void setErrorFlag(StatusResponse& status, ErrorFlags error);
    static bool hasError(const StatusResponse& status, ErrorFlags error);
};

#endif