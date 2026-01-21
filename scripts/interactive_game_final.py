#!/usr/bin/env python3
"""
Interactive Pick & Place Game - FINAL VERSION with MotionManager
Proper action calling using MotionManager like mimic_node.py
"""

import cv2
import numpy as np
import sys
import time
import subprocess
from pathlib import Path

# Add ROS and AiNex paths
sys.path.insert(0, '/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/src')
sys.path.insert(0, "/home/ubuntu/ros_ws/devel/lib/python3/dist-packages")
sys.path.insert(0, '/home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/src')

import rospy
from ainex_kinematics.motion_manager import MotionManager
from ainex_kinematics.gait_manager import GaitManager

# Configuration
ACTION_GROUPS_PATH = '/home/ubuntu/software/ainex_controller/ActionGroups'
STABILITY_FRAMES = 30  # 1 second at 30 FPS
COLOR_THRESHOLD = 0.15

# ROI Zones (x, y, width, height)
LEFT_ROI = (50, 50, 150, 150)
RIGHT_ROI = (440, 50, 150, 150)

# HSV Color Ranges
COLOR_RANGES = {
    'red': ([0, 120, 70], [10, 255, 255]),
    'green': ([35, 100, 100], [85, 255, 255]),
    'blue': ([100, 100, 100], [130, 255, 255]),
    'yellow': ([20, 100, 100], [35, 255, 255])
}

class InteractiveGame:
    """Interactive Pick & Place Game with MotionManager"""
    
    def __init__(self):
        """Initialize the game"""
        print("\n" + "="*60)
        print("🎮 AINEX INTERACTIVE GAME - MOTION MANAGER VERSION")
        print("="*60 + "\n")
        
        # Initialize ROS node
        rospy.init_node('interactive_game', anonymous=True)
        rospy.loginfo('Interactive Game node started')
        
        # Wait for servo controller services
        rospy.loginfo('Waiting for servo controller services...')
        try:
            rospy.wait_for_service('ros_robot_controller/bus_servo/get_position', timeout=30)
            rospy.loginfo('✅ Servo controller services ready')
        except rospy.ROSException:
            rospy.logerr('❌ Timeout waiting for servo controller services')
            rospy.logerr('   Make sure base.launch is included and running!')
            raise Exception('Servo controller not available')
        
        # Initialize managers
        try:
            time.sleep(0.1)
            self.gait_manager = GaitManager()
            self.motion_manager = MotionManager(ACTION_GROUPS_PATH)
            rospy.loginfo('✅ GaitManager and MotionManager initialized')
        except Exception as e:
            rospy.logerr(f'❌ Failed to initialize managers: {e}')
            import traceback
            rospy.logerr(traceback.format_exc())
            raise
        
        # Initialize camera
        self.cap = self.initialize_camera()
        if self.cap is None:
            raise Exception('Failed to initialize camera')
        
        # Game state
        self.left_missing_frames = 0
        self.right_missing_frames = 0
        self.action_executed = False
        self.busy = False
        
        # Initialize robot position
        rospy.loginfo('🤖 Initializing robot to ready position...')
        try:
            time.sleep(0.3)
            self.gait_manager.disable()
            time.sleep(0.05)
            self.motion_manager.run_action('walk_ready')
            time.sleep(0.1)
            
            # Position camera to look down at the game board
            # Servo 23: Pan (left/right) - 500 is center
            # Servo 24: Tilt (up/down) - lower value = look down
            rospy.loginfo("📷 Positioning camera to look down...")
            self.motion_manager.set_servos_position(1000, [[23, 500], [24, 200]])
            time.sleep(1.0)
            rospy.loginfo("✅ Robot ready, camera looking down")
        except Exception as e:
            rospy.logwarn(f'⚠️  Could not initialize robot position: {e}')
    
    def initialize_camera(self):
        """Initialize camera with process cleanup"""
        print("📷 Initializing camera...")
        
        # Kill any processes using the camera
        try:
            subprocess.run(['pkill', '-f', 'usb_cam_node'], stderr=subprocess.DEVNULL)
            subprocess.run(['bash', '-c', 
                           'lsof /dev/video0 2>/dev/null | awk \'NR>1 {print $2}\' | xargs -r kill -9'], 
                          stderr=subprocess.DEVNULL)
            time.sleep(0.5)
        except:
            pass
        
        # Try V4L2 backend
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            print("✅ Camera initialized")
            return cap
        
        return None
    
    def get_color_density(self, frame, roi):
        """Detect colors in ROI"""
        x, y, w, h = roi
        roi_frame = frame[y:y+h, x:x+w]
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        
        detected_colors = []
        for color_name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            density = np.sum(mask > 0) / (w * h)
            if density > COLOR_THRESHOLD:
                detected_colors.append(color_name)
        
        return detected_colors
    
    def execute_action(self, action_name):
        """
        Execute robot action using MotionManager (like mimic_node.py)
        This is the proper way to call actions!
        """
        print(f"\n{'='*60}")
        print(f"🎬 EXECUTING ACTION: {action_name}")
        print(f"{'='*60}")
        
        self.busy = True
        success = False
        
        try:
            # Disable gait before action
            self.gait_manager.disable()
            time.sleep(0.02)
            
            # Execute action using MotionManager
            self.motion_manager.run_action(action_name)
            time.sleep(0.1)
            
            print(f"✅ ACTION COMPLETED: {action_name}")
            print(f"{'='*60}\n")
            success = True
        except Exception as e:
            rospy.logerr(f'❌ Action failed: {e}')
            import traceback
            traceback.print_exc()
        finally:
            self.busy = False
        
        return success
    
    def run(self):
        """Main game loop"""
        print("👁️  Monitoring ROIs...")
        print("📍 Remove LEFT square → lefthand_place_square")
        print("📍 Remove RIGHT square → righthand_place_square")
        print("Press 'q' to quit\n")
        
        try:
            while not self.action_executed and not rospy.is_shutdown():
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ Failed to read frame")
                    break
                
                # Detect colors in ROIs
                left_colors = self.get_color_density(frame, LEFT_ROI)
                right_colors = self.get_color_density(frame, RIGHT_ROI)
                
                left_has_square = len(left_colors) > 0
                right_has_square = len(right_colors) > 0
                
                # Draw ROIs
                for roi, has_square, label in [
                    (LEFT_ROI, left_has_square, "LEFT"),
                    (RIGHT_ROI, right_has_square, "RIGHT")
                ]:
                    x, y, w, h = roi
                    color = (0, 255, 0) if has_square else (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                    cv2.putText(frame, label, (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # Check LEFT square removal (1 second stability)
                if not left_has_square and not self.busy:
                    self.left_missing_frames += 1
                    if self.left_missing_frames >= STABILITY_FRAMES:
                        print(f"\n✅ LEFT SQUARE REMOVED (stable for {self.left_missing_frames} frames)")
                        cv2.imshow("Game", frame)
                        cv2.waitKey(500)
                        
                        # Execute left hand action
                        success = self.execute_action("lefthand_place_square")
                        if success:
                            self.action_executed = True
                            self.chosen_action = "lefthand_place_square"
                        else:
                            self.left_missing_frames = 0  # Reset on failure
                    else:
                        status = f"LEFT Stability: {self.left_missing_frames}/{STABILITY_FRAMES}"
                        cv2.putText(frame, status, (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    self.left_missing_frames = 0
                
                # Check RIGHT square removal (1 second stability)
                if not right_has_square and not self.action_executed and not self.busy:
                    self.right_missing_frames += 1
                    if self.right_missing_frames >= STABILITY_FRAMES:
                        print(f"\n✅ RIGHT SQUARE REMOVED (stable for {self.right_missing_frames} frames)")
                        cv2.imshow("Game", frame)
                        cv2.waitKey(500)
                        
                        # Execute right hand action
                        success = self.execute_action("righthand_place_square")
                        if success:
                            self.action_executed = True
                            self.chosen_action = "righthand_place_square"
                        else:
                            self.right_missing_frames = 0  # Reset on failure
                    else:
                        status = f"RIGHT Stability: {self.right_missing_frames}/{STABILITY_FRAMES}"
                        cv2.putText(frame, status, (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    self.right_missing_frames = 0
                
                # Show status
                if not self.action_executed:
                    cv2.putText(frame, "Waiting for square removal...", (10, frame.shape[0] - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Display frame
                cv2.imshow("Game", frame)
                
                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n👋 User quit")
                    break
        
        except KeyboardInterrupt:
            print("\n\n👋 Interrupted")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        
        if self.action_executed:
            print(f"\n🎯 Action '{self.chosen_action}' completed")
            print("\n✅ GAME FINISHED SUCCESSFULLY!\n")
        else:
            print("\n⚠️  Game ended without action execution\n")

if __name__ == "__main__":
    try:
        game = InteractiveGame()
        game.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
