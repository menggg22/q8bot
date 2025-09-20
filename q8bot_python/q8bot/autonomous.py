"""
Q8bot Autonomous Commander - PC-side high-level command interface
Replaces low-level joint position streaming with autonomous commands
Created for Phase 2 implementation
"""

import serial
import struct
import time
import threading
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Callable, Dict, Any

# Command type enumeration (matches C++ definitions)
class CommandType(IntEnum):
    # Movement commands
    CMD_IDLE = 0x00
    CMD_MOVE_FORWARD = 0x01
    CMD_MOVE_BACKWARD = 0x02
    CMD_TURN_LEFT = 0x03
    CMD_TURN_RIGHT = 0x04
    CMD_MOVE_DIAGONAL_FL = 0x05
    CMD_MOVE_DIAGONAL_FR = 0x06
    CMD_STRAFE_LEFT = 0x07
    CMD_STRAFE_RIGHT = 0x08
    
    # Gait control
    CMD_CHANGE_GAIT = 0x10
    CMD_SET_SPEED = 0x11
    
    # Special movements
    CMD_EMERGENCY_STOP = 0x20
    CMD_JUMP = 0x21
    CMD_DANCE = 0x22
    CMD_GREETING = 0x23
    
    # System commands
    CMD_GET_STATUS = 0x30
    CMD_GET_BATTERY = 0x31
    CMD_ENABLE_TORQUE = 0x40
    CMD_DISABLE_TORQUE = 0x41
    CMD_RESET_POSITION = 0x42
    CMD_SET_DEBUG_MODE = 0x43

class GaitType(IntEnum):
    AMBER = 0
    AMBER_HIGH = 1
    AMBER_LOW = 2
    AMBER_FAST = 3
    WALK = 4
    BOUND = 5
    PRONK = 6

class CommandFlags(IntEnum):
    FLAG_NONE = 0x00
    FLAG_CONTINUOUS = 0x01
    FLAG_PRECISE_TIMING = 0x02
    FLAG_GENTLE_START = 0x04
    FLAG_GENTLE_STOP = 0x08
    FLAG_RETURN_DATA = 0x10
    FLAG_DEBUG_MODE = 0x20

@dataclass
class RobotStatus:
    """Robot status information"""
    current_gait: int
    battery_voltage: float
    current_speed: float
    torque_enabled: bool
    error_flags: int
    uptime_seconds: int
    joint_temperatures: list
    sequence_id: int

class Q8AutonomousCommander:
    """High-level command interface for autonomous Q8bot control"""
    
    # Command packet format: B f f B B H B BB (16 bytes)
    COMMAND_FORMAT = '<B f f B B H B BB'
    COMMAND_SIZE = 16
    
    # Status response format: B f f B B H 8B H B (24 bytes)  
    STATUS_FORMAT = '<B f f B B H 8B H B'
    STATUS_SIZE = 24
    
    def __init__(self, port: str, baud_rate: int = 115200):
        """
        Initialize the autonomous commander
        
        Args:
            port: Serial port for ESP32C3 controller
            baud_rate: Communication baud rate
        """
        self.port = port
        self.baud_rate = baud_rate
        self.serial_handler = None
        self.sequence_counter = 0
        self.last_status = None
        self.status_callback = None
        self.debug_mode = False
        
        # Connection status
        self.connected = False
        self.command_timeout = 5.0  # seconds
        
        # Background status monitoring
        self.status_thread = None
        self.stop_monitoring = False
        
    def connect(self) -> bool:
        """Establish connection to robot controller"""
        try:
            self.serial_handler = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            time.sleep(0.1)  # Allow connection to stabilize
            
            # Test connection with status request
            if self.get_status():
                self.connected = True
                print(f"Connected to Q8bot on {self.port}")
                return True
            else:
                self.serial_handler.close()
                return False
                
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close connection to robot"""
        self.stop_status_monitoring()
        
        if self.serial_handler and self.serial_handler.is_open:
            self.serial_handler.close()
        
        self.connected = False
        print("Disconnected from Q8bot")
    
    def _calculate_checksum(self, data: bytes) -> int:
        """Calculate XOR checksum for data integrity"""
        checksum = 0
        for byte in data[:-3]:  # Exclude checksum and reserved bytes
            checksum ^= byte
        return checksum & 0xFF
    
    def _send_command(self, cmd_type: CommandType, param1: float = 0.0, param2: float = 0.0, 
                     gait_type: GaitType = GaitType.AMBER, flags: CommandFlags = CommandFlags.FLAG_NONE) -> bool:
        """Send a high-level command to the robot"""
        if not self.connected or not self.serial_handler:
            print("Error: Not connected to robot")
            return False
        
        try:
            # Increment sequence counter
            self.sequence_counter = (self.sequence_counter + 1) % 65536
            
            # Pack command data (without checksum initially)
            cmd_data = struct.pack('<B f f B B H BB', 
                                 cmd_type, param1, param2, gait_type, flags,
                                 self.sequence_counter, 0, 0, 0)  # checksum=0, reserved=0,0
            
            # Calculate and insert checksum
            checksum = self._calculate_checksum(cmd_data)
            cmd_data = cmd_data[:-3] + struct.pack('BBB', checksum, 0, 0)  # Insert checksum, keep reserved
            
            # Send command
            self.serial_handler.write(cmd_data)
            
            if self.debug_mode:
                print(f"Sent command: {cmd_type.name} with params ({param1}, {param2})")
            
            return True
            
        except Exception as e:
            print(f"Command send error: {e}")
            return False
    
    def _parse_status_response(self, data: bytes) -> Optional[RobotStatus]:
        """Parse status response from robot"""
        if len(data) != self.STATUS_SIZE:
            return None
            
        try:
            unpacked = struct.unpack(self.STATUS_FORMAT, data)
            
            # Verify checksum
            expected_checksum = self._calculate_checksum(data)
            if unpacked[-1] != expected_checksum:
                print("Status checksum mismatch")
                return None
            
            return RobotStatus(
                current_gait=unpacked[0],
                battery_voltage=unpacked[1],
                current_speed=unpacked[2],
                torque_enabled=bool(unpacked[3]),
                error_flags=unpacked[4],
                uptime_seconds=unpacked[5],
                joint_temperatures=list(unpacked[6:14]),
                sequence_id=unpacked[14]
            )
            
        except Exception as e:
            print(f"Status parsing error: {e}")
            return None
    
    # ===== Movement Commands =====
    
    def move_forward(self, speed: float = 1.0, duration: float = 0.0, gait: GaitType = GaitType.AMBER) -> bool:
        """Move robot forward"""
        return self._send_command(CommandType.CMD_MOVE_FORWARD, speed, duration, gait, 
                                CommandFlags.FLAG_CONTINUOUS if duration <= 0 else CommandFlags.FLAG_NONE)
    
    def move_backward(self, speed: float = 1.0, duration: float = 0.0, gait: GaitType = GaitType.AMBER) -> bool:
        """Move robot backward"""
        return self._send_command(CommandType.CMD_MOVE_BACKWARD, speed, duration, gait,
                                CommandFlags.FLAG_CONTINUOUS if duration <= 0 else CommandFlags.FLAG_NONE)
    
    def turn_left(self, speed: float = 1.0, angle: float = 90.0, gait: GaitType = GaitType.AMBER) -> bool:
        """Turn robot left (degrees)"""
        duration = angle / (90.0 * speed)  # Estimate duration based on angle
        return self._send_command(CommandType.CMD_TURN_LEFT, speed, duration, gait)
    
    def turn_right(self, speed: float = 1.0, angle: float = 90.0, gait: GaitType = GaitType.AMBER) -> bool:
        """Turn robot right (degrees)"""
        duration = angle / (90.0 * speed)  # Estimate duration based on angle
        return self._send_command(CommandType.CMD_TURN_RIGHT, speed, duration, gait)
    
    def move_diagonal_forward_left(self, speed: float = 1.0, duration: float = 0.0) -> bool:
        """Move diagonally forward-left"""
        return self._send_command(CommandType.CMD_MOVE_DIAGONAL_FL, speed, duration, GaitType.AMBER)
    
    def move_diagonal_forward_right(self, speed: float = 1.0, duration: float = 0.0) -> bool:
        """Move diagonally forward-right"""
        return self._send_command(CommandType.CMD_MOVE_DIAGONAL_FR, speed, duration, GaitType.AMBER)
    
    def strafe_left(self, speed: float = 1.0, duration: float = 0.0) -> bool:
        """Strafe (sidestep) left"""
        return self._send_command(CommandType.CMD_STRAFE_LEFT, speed, duration, GaitType.AMBER)
    
    def strafe_right(self, speed: float = 1.0, duration: float = 0.0) -> bool:
        """Strafe (sidestep) right"""
        return self._send_command(CommandType.CMD_STRAFE_RIGHT, speed, duration, GaitType.AMBER)
    
    def stop(self) -> bool:
        """Stop all movement"""
        return self._send_command(CommandType.CMD_IDLE)
    
    # ===== Gait Control =====
    
    def change_gait(self, gait: GaitType, speed: float = 1.0) -> bool:
        """Change the robot's gait pattern"""
        return self._send_command(CommandType.CMD_CHANGE_GAIT, speed, 0.0, gait)
    
    def set_speed(self, speed: float) -> bool:
        """Set default movement speed"""
        return self._send_command(CommandType.CMD_SET_SPEED, speed)
    
    # ===== Special Commands =====
    
    def emergency_stop(self) -> bool:
        """Emergency stop - immediately disable all motors"""
        return self._send_command(CommandType.CMD_EMERGENCY_STOP)
    
    def jump(self) -> bool:
        """Execute jump sequence"""
        return self._send_command(CommandType.CMD_JUMP)
    
    def dance(self) -> bool:
        """Execute dance routine"""
        return self._send_command(CommandType.CMD_DANCE)
    
    def greeting(self) -> bool:
        """Execute greeting gesture"""
        return self._send_command(CommandType.CMD_GREETING)
    
    # ===== System Commands =====
    
    def get_status(self) -> Optional[RobotStatus]:
        """Request robot status"""
        if not self._send_command(CommandType.CMD_GET_STATUS):
            return None
        
        # Wait for response
        try:
            start_time = time.time()
            while time.time() - start_time < self.command_timeout:
                if self.serial_handler.in_waiting >= self.STATUS_SIZE:
                    data = self.serial_handler.read(self.STATUS_SIZE)
                    status = self._parse_status_response(data)
                    if status:
                        self.last_status = status
                        return status
                time.sleep(0.01)
        except Exception as e:
            print(f"Status request error: {e}")
        
        return None
    
    def get_battery(self) -> Optional[float]:
        """Get battery voltage"""
        if self._send_command(CommandType.CMD_GET_BATTERY):
            status = self.get_status()
            return status.battery_voltage if status else None
        return None
    
    def enable_torque(self) -> bool:
        """Enable motor torque"""
        return self._send_command(CommandType.CMD_ENABLE_TORQUE)
    
    def disable_torque(self) -> bool:
        """Disable motor torque"""
        return self._send_command(CommandType.CMD_DISABLE_TORQUE)
    
    def reset_position(self) -> bool:
        """Reset robot to idle standing position"""
        return self._send_command(CommandType.CMD_RESET_POSITION)
    
    def set_debug_mode(self, enable: bool) -> bool:
        """Enable/disable debug output from robot"""
        flags = CommandFlags.FLAG_DEBUG_MODE if enable else CommandFlags.FLAG_NONE
        success = self._send_command(CommandType.CMD_SET_DEBUG_MODE, flags=flags)
        if success:
            self.debug_mode = enable
        return success
    
    # ===== Status Monitoring =====
    
    def start_status_monitoring(self, callback: Callable[[RobotStatus], None], interval: float = 1.0):
        """Start background status monitoring"""
        self.status_callback = callback
        self.stop_monitoring = False
        
        def monitor_status():
            while not self.stop_monitoring and self.connected:
                status = self.get_status()
                if status and self.status_callback:
                    self.status_callback(status)
                time.sleep(interval)
        
        self.status_thread = threading.Thread(target=monitor_status, daemon=True)
        self.status_thread.start()
    
    def stop_status_monitoring(self):
        """Stop background status monitoring"""
        self.stop_monitoring = True
        if self.status_thread:
            self.status_thread.join(timeout=1.0)
    
    # ===== Utility Methods =====
    
    def is_connected(self) -> bool:
        """Check if connected to robot"""
        return self.connected and self.serial_handler and self.serial_handler.is_open
    
    def get_last_status(self) -> Optional[RobotStatus]:
        """Get last received status"""
        return self.last_status
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()