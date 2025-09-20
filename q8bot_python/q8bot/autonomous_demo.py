#!/usr/bin/env python3
"""
Q8bot Autonomous Control Demo
Demonstrates the new high-level command interface
"""

import sys
import time
import pygame
from q8bot.autonomous import Q8AutonomousCommander, GaitType, RobotStatus

def print_status(status: RobotStatus):
    """Print robot status information"""
    print(f"\n--- Robot Status ---")
    print(f"Gait: {status.current_gait} | Speed: {status.current_speed:.1f}")
    print(f"Battery: {status.battery_voltage:.1f}V | Uptime: {status.uptime_seconds}s")
    print(f"Torque: {'ON' if status.torque_enabled else 'OFF'}")
    if status.error_flags != 0:
        print(f"⚠️  Errors: 0x{status.error_flags:02X}")
    print(f"Joint Temps: {[f'{t}°C' for t in status.joint_temperatures[:4]]}")

def main():
    # Check for port argument
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = input("Enter COM port (e.g., COM14, /dev/ttyUSB0): ").strip()
    
    print("Q8bot Autonomous Control Demo")
    print("=============================")
    print(f"Connecting to {port}...")
    
    # Initialize robot commander
    with Q8AutonomousCommander(port) as robot:
        if not robot.connect():
            print("❌ Failed to connect to robot")
            return
        
        # Enable debug mode
        robot.set_debug_mode(True)
        
        # Get initial status
        print("\n📊 Getting robot status...")
        status = robot.get_status()
        if status:
            print_status(status)
        else:
            print("⚠️  Could not get robot status")
        
        # Initialize pygame for keyboard control
        pygame.init()
        screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Q8bot Autonomous Control")
        clock = pygame.time.Clock()
        
        print("\n🎮 Keyboard Controls:")
        print("  W/S - Forward/Backward")
        print("  A/D - Turn Left/Right") 
        print("  Q/E - Diagonal Left/Right")
        print("  Z/X - Strafe Left/Right")
        print("  G   - Change Gait")
        print("  J   - Jump")
        print("  R   - Reset Position")
        print("  B   - Battery Check")
        print("  T   - Toggle Torque")
        print("  ESC - Emergency Stop & Exit")
        print("\nRobot ready! Use keyboard to control...")
        
        # Control variables
        current_gait = GaitType.AMBER
        gait_names = ['AMBER', 'AMBER_HIGH', 'AMBER_LOW', 'AMBER_FAST', 'WALK', 'BOUND', 'PRONK']
        speed = 1.0
        
        # Start status monitoring
        robot.start_status_monitoring(print_status, interval=2.0)
        
        running = True
        movement_active = False
        
        while running:
            clock.tick(30)  # 30 FPS
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.KEYDOWN:
                    key = event.key
                    
                    # Movement commands
                    if key == pygame.K_w:
                        print("🔼 Moving forward...")
                        robot.move_forward(speed=speed, gait=current_gait)
                        movement_active = True
                        
                    elif key == pygame.K_s:
                        print("🔽 Moving backward...")
                        robot.move_backward(speed=speed, gait=current_gait)
                        movement_active = True
                        
                    elif key == pygame.K_a:
                        print("↪️ Turning left...")
                        robot.turn_left(speed=speed, angle=45, gait=current_gait)
                        movement_active = True
                        
                    elif key == pygame.K_d:
                        print("↩️ Turning right...")
                        robot.turn_right(speed=speed, angle=45, gait=current_gait)
                        movement_active = True
                        
                    elif key == pygame.K_q:
                        print("↖️ Moving diagonal forward-left...")
                        robot.move_diagonal_forward_left(speed=speed)
                        movement_active = True
                        
                    elif key == pygame.K_e:
                        print("↗️ Moving diagonal forward-right...")
                        robot.move_diagonal_forward_right(speed=speed)
                        movement_active = True
                        
                    elif key == pygame.K_z:
                        print("⬅️ Strafing left...")
                        robot.strafe_left(speed=speed)
                        movement_active = True
                        
                    elif key == pygame.K_x:
                        print("➡️ Strafing right...")
                        robot.strafe_right(speed=speed)
                        movement_active = True
                    
                    # Gait control
                    elif key == pygame.K_g:
                        current_gait = GaitType((current_gait + 1) % len(gait_names))
                        print(f"🚶 Changed gait to: {gait_names[current_gait]}")
                        robot.change_gait(current_gait, speed)
                    
                    # Special commands
                    elif key == pygame.K_j:
                        print("🦘 Jumping...")
                        robot.jump()
                        time.sleep(0.1)  # Prevent multiple jumps
                        
                    elif key == pygame.K_r:
                        print("🏠 Resetting to idle position...")
                        robot.reset_position()
                        movement_active = False
                        
                    elif key == pygame.K_b:
                        print("🔋 Checking battery...")
                        battery = robot.get_battery()
                        if battery:
                            print(f"   Battery: {battery:.1f}V")
                        
                    elif key == pygame.K_t:
                        print("⚡ Toggling torque...")
                        status = robot.get_status()
                        if status and status.torque_enabled:
                            robot.disable_torque()
                            print("   Torque disabled")
                        else:
                            robot.enable_torque()
                            print("   Torque enabled")
                    
                    # Speed control
                    elif key == pygame.K_PLUS or key == pygame.K_EQUALS:
                        speed = min(2.0, speed + 0.1)
                        print(f"⚡ Speed: {speed:.1f}")
                        robot.set_speed(speed)
                        
                    elif key == pygame.K_MINUS:
                        speed = max(0.1, speed - 0.1)
                        print(f"🐌 Speed: {speed:.1f}")
                        robot.set_speed(speed)
                    
                    # Emergency stop
                    elif key == pygame.K_ESCAPE:
                        print("🛑 EMERGENCY STOP")
                        robot.emergency_stop()
                        running = False
                        
                elif event.type == pygame.KEYUP:
                    # Stop movement when key is released
                    if movement_active and event.key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, 
                                                        pygame.K_q, pygame.K_e, pygame.K_z, pygame.K_x]:
                        print("⏹️ Stopping...")
                        robot.stop()
                        movement_active = False
            
            # Update display
            screen.fill((0, 0, 0))
            pygame.display.flip()
        
        print("\n👋 Shutting down...")
        robot.stop()
        robot.disable_torque()
        
    pygame.quit()
    print("✅ Demo completed")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()