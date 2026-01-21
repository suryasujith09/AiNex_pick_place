#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auto Pick & Place Script for AiNex Humanoid Robot
Complete automation for picking, carrying, and placing objects
"""

import os
import sys
import time
import sqlite3
import subprocess
from typing import Optional, List, Tuple

# FIRST: Check if running inside Docker or on host (before any imports)
if not os.path.exists('/.dockerenv'):
    # We're on the host, need to relaunch in Docker
    print("╔" + "═" * 58 + "╗")
    print("║  Detected host environment - relaunching in Docker...   ║")
    print("╚" + "═" * 58 + "╝")
    print()
    
    script_path = os.path.abspath(__file__)
    docker_script_path = "/home/ubuntu/ros_ws/src/pick_place/auto_pick_place.py"
    
    try:
        # Check Docker is available
        if subprocess.run(['docker', 'ps'], capture_output=True).returncode != 0:
            print("✗ Error: Docker not available")
            sys.exit(1)
        
        # Check ainex container is running
        result = subprocess.run(['docker', 'ps'], capture_output=True, text=True)
        if 'ainex' not in result.stdout:
            print("✗ Error: Docker container 'ainex' is not running")
            print("  Start it with: docker start ainex")
            sys.exit(1)
        
        # Copy script to Docker
        print("→ Copying script to Docker container...")
        subprocess.run(['docker', 'cp', script_path, f'ainex:{docker_script_path}'], 
                      check=True, capture_output=True)
        
        # Make executable
        subprocess.run(['docker', 'exec', '-u', 'ubuntu', 'ainex', 
                       'chmod', '+x', docker_script_path],
                      check=True, capture_output=True)
        
        # Execute in Docker
        print("→ Executing in Docker container...")
        print("━" * 60)
        print()
        
        result = subprocess.run(
            ['docker', 'exec', '-it', '-u', 'ubuntu', 'ainex', 
             'python3', docker_script_path],
            check=False
        )
        
        sys.exit(result.returncode)
        
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to launch in Docker: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Error: {e}")
        sys.exit(1)

# SECOND: We're in Docker - now import Board SDK
sys.path.insert(0, '/home/ubuntu/software/ainex_controller')

try:
    from ros_robot_controller_sdk import Board
    print("✓ Board SDK imported successfully")
except ImportError as e:
    print(f"✗ Failed to import Board SDK: {e}")
    print("  SDK path issue - check installation at:")
    print("  /home/ubuntu/software/ainex_controller")
    sys.exit(1)


class Robot:
    """
    AiNex Humanoid Robot Controller with Gripper Hold Functionality
    """
    
    # Action file search paths (in priority order)
    ACTION_PATHS = [
        '/home/ubuntu/humanoid_interaction_project/actions/custom',
        '/home/ubuntu/software/ainex_controller/ActionGroups',
    ]
    
    # Servo ID for left gripper (must be skipped when holding)
    GRIPPER_SERVO_ID = 24
    
    def __init__(self, device="/dev/rrc", baudrate=1000000):
        """Initialize the robot board"""
        print("═" * 60)
        print("  AiNex Robot - Automatic Pick & Place System")
        print("═" * 60)
        
        try:
            self.board = Board(device=device, baudrate=baudrate)
            print(f"✓ Board initialized on {device}")
            self.speak("Robot initialized")
        except Exception as e:
            print(f"✗ Failed to initialize board: {e}")
            self.speak("Robot initialization failed")
            raise
        
        self.holding = False  # Track if gripper is holding object
        self.current_action = None
    
    def speak(self, message: str):
        """Announce status using espeak"""
        try:
            os.system(f'espeak "{message}" 2>/dev/null &')
        except:
            pass  # Silently fail if espeak not available
    
    def find_action_file(self, action_name: str) -> Optional[str]:
        """Find action file in search paths"""
        for path in self.ACTION_PATHS:
            action_file = os.path.join(path, f"{action_name}.d6a")
            if os.path.exists(action_file):
                return action_file
        return None
    
    def play(self, action_name: str, hold_gripper: bool = False):
        """
        Execute an action with optional gripper hold
        
        Args:
            action_name: Name of the action file (without .d6a)
            hold_gripper: If True, skip commands to servo 24 (left gripper)
                         to maintain grip on object
        """
        self.current_action = action_name
        
        # Update holding state
        if hold_gripper:
            self.holding = True
        elif action_name in ['wave', 'hand_open', 'hands_open']:
            self.holding = False
        
        # Find action file
        action_file = self.find_action_file(action_name)
        if not action_file:
            print(f"✗ Action file not found: {action_name}")
            self.speak(f"Action {action_name} not found")
            return False
        
        print(f"► Executing: {action_name} (hold_gripper={hold_gripper})")
        self.speak(f"Executing {action_name}")
        
        try:
            # Read action from SQLite database
            connection = sqlite3.connect(action_file)
            cursor = connection.cursor()
            cursor.execute("SELECT * FROM ActionGroup")
            
            # Execute each frame
            while True:
                frame = cursor.fetchone()
                if frame is None:
                    break
                
                # Extract timing and servo positions
                duration_ms = frame[1]  # Duration in milliseconds
                servo_positions = []
                
                # Build servo command list (skip gripper if holding)
                for i in range(0, len(frame) - 2):
                    servo_id = i + 1
                    position = frame[2 + i]
                    
                    # CRITICAL: Skip gripper servo when holding object
                    if hold_gripper and servo_id == self.GRIPPER_SERVO_ID:
                        print(f"  → Skipping servo {servo_id} (holding object)")
                        continue
                    
                    servo_positions.append([servo_id, position])
                
                # Send command to board
                if servo_positions:
                    duration_sec = duration_ms / 1000.0
                    self.board.bus_servo_set_position(duration_sec, servo_positions)
                    time.sleep(duration_sec)
            
            cursor.close()
            connection.close()
            
            print(f"✓ Action '{action_name}' completed")
            return True
            
        except Exception as e:
            print(f"✗ Error executing '{action_name}': {e}")
            self.speak(f"Error executing {action_name}")
            return False
    
    def emergency_stop(self):
        """Stop all servos immediately"""
        print("🚨 EMERGENCY STOP")
        self.speak("Emergency stop")
        try:
            servo_ids = list(range(1, 25))  # All 24 servos
            self.board.stopBusServo(servo_ids)
        except Exception as e:
            print(f"✗ Emergency stop error: {e}")
    
    def execute_pick_place_sequence(self):
        """
        Execute the complete pick and place sequence:
        1. Squat down to object level
        2. Grab object with left gripper
        3. Stand up while holding
        4. Turn left while holding
        5. Release object (wave motion)
        6. Return to stand position
        """
        print("\n" + "═" * 60)
        print("  Starting Pick & Place Sequence")
        print("═" * 60)
        self.speak("Starting pick and place sequence")
        
        sequence = [
            ("squat", False, "Bending down to object"),
            ("clamp_left", False, "Grabbing object"),
            ("stand", True, "Standing up with object"),
            ("turn_left", True, "Turning left with object"),
            ("wave", False, "Releasing object"),
            ("stand", False, "Returning to neutral"),
        ]
        
        for i, (action, hold, description) in enumerate(sequence, 1):
            print(f"\n[Step {i}/{len(sequence)}] {description}")
            print("─" * 60)
            
            # Announce step
            self.speak(description)
            
            # Execute action
            success = self.play(action, hold_gripper=hold)
            
            if not success:
                print(f"✗ Sequence failed at step {i}")
                self.speak("Sequence failed")
                return False
            
            # Brief pause between actions
            time.sleep(0.5)
        
        print("\n" + "═" * 60)
        print("  ✓ Pick & Place Sequence Completed Successfully!")
        print("═" * 60)
        self.speak("Pick and place sequence completed successfully")
        return True
    
    def shutdown(self):
        """Clean shutdown"""
        print("\nShutting down robot controller...")
        try:
            # Return to safe position
            self.play("stand", hold_gripper=False)
        except:
            pass


def stop_ainex_controller():
    """Stop the ainex_controller service to free the serial port"""
    print("Stopping ainex_controller service...")
    try:
        result = subprocess.run(
            ['sudo', 'systemctl', 'stop', 'ainex_controller'],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            print("✓ ainex_controller stopped")
            time.sleep(2)  # Wait for port to be released
            return True
        else:
            print(f"⚠ Failed to stop service: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("⚠ Timeout stopping service")
        return False
    except Exception as e:
        print(f"⚠ Error stopping service: {e}")
        return False


def main():
    """Main entry point"""
    print("\n" + "╔" + "═" * 58 + "╗")
    print("║" + " " * 15 + "AiNex Auto Pick & Place" + " " * 20 + "║")
    print("╚" + "═" * 58 + "╝")
    print()
    
    # Step 1: Stop competing service
    print("[1/3] Preparing environment...")
    stop_ainex_controller()
    
    # Step 2: Initialize robot
    print("\n[2/3] Initializing robot...")
    robot = None
    try:
        robot = Robot()
    except Exception as e:
        print(f"\n✗ Fatal error during initialization: {e}")
        return 1
    
    # Step 3: Execute sequence
    print("\n[3/3] Executing pick & place sequence...")
    try:
        success = robot.execute_pick_place_sequence()
        
        if success:
            print("\n" + "🎉 " * 20)
            print("\n  SUCCESS! Pick & Place operation completed.\n")
            print("🎉 " * 20)
            return 0
        else:
            print("\n⚠ Sequence incomplete")
            return 1
            
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
        if robot:
            robot.emergency_stop()
        return 130
        
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        if robot:
            robot.emergency_stop()
        return 1
        
    finally:
        if robot:
            robot.shutdown()
        print("\n" + "─" * 60)
        print("  Program terminated")
        print("─" * 60 + "\n")


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
