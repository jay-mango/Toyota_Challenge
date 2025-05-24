from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# === Import required helper classes ===
from TMMC_Wrapper.Lidar import Lidar           # For accessing LIDAR distance readings
from TMMC_Wrapper.Control import Control       # For controlling robot movement
from TMMC_Wrapper.Robot import Robot           # Main interface to initialize everything
from TMMC_Wrapper.Camera import Camera         # (Used in Level 2+)
from TMMC_Wrapper.IMU import IMU               # (Used in Level 3+)
from TMMC_Wrapper.Logging import Logging       # (Optional: for debugging)

# === Configuration ===
challengeLevel = 1               # Set this to 1 to run Level 1 logic
is_SIM = False                    # True = simulation, False = real robot
Debug = False                    # True = print extra info

# === Initialize the robot and interfaces ===
robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)

control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

# === Start keyboard control in case it's needed for manual overrides (Levels 0–2) ===
if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


# === Helper Function: Check obstacle in direction ===
def is_obstacle_in_direction(lidar, direction, threshold=0.5, cone_width=15):
    """
    Checks for an obstacle in the given direction using LiDAR.

    Args:
        lidar (Lidar): Lidar object
        direction (str): "forward", "backward", "left", or "right"
        threshold (float): Distance (m) considered "too close"
        cone_width (int): Degrees to scan left/right of center

    Returns:
        bool: True if obstacle detected in that direction
    """
    direction_to_angle = {
        "forward": 0,
        "backward": 180,
        "left": 90,
        "right": 270
    }

    if direction not in direction_to_angle:
        return False  # Invalid or idle direction

    scan = lidar.checkScan()
    min_dist, _ = lidar.detect_obstacle_in_cone(scan, threshold, direction_to_angle[direction], cone_width)
    return min_dist != -1

try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

    # === Main Challenge Logic for Level 1 ===
    if challengeLevel == 1:
        movement_mode = None  # Tracks current direction of motion: "forward", "backward", etc.

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)

            # === Determine movement direction based on key press ===
            key = robot.last_key_pressed  # <-- You must expose this in Control.py (see below)

            if key == 'w':
                movement_mode = "forward"
            elif key == 's':
                movement_mode = "backward"
            elif key == 'a':
                movement_mode = "left"
            elif key == 'd':
                movement_mode = "right"
            else:
                movement_mode = None

            # === Only block movement if obstacle in same direction ===
            if movement_mode and is_obstacle_in_direction(lidar, movement_mode):
                print(f"\n🚨 Obstacle detected while moving {movement_mode} — stopping and backing up.")

                # Stop movement
                control.stop_keyboard_input()
                control.stop_keyboard_control()
                control.set_cmd_vel(0.0, 0.0, 0.5)

                # Back up safely (always backwards)
                if movement_mode == "forward":
                    control.set_cmd_vel(-0.1, 0.0, 1.5)
                elif movement_mode == "backward":
                    control.set_cmd_vel(0.1, 0.0, 1.5)
                elif movement_mode == "left":
                    control.set_cmd_vel(0.0, -0.4, 1.5)
                elif movement_mode == "right":
                    control.set_cmd_vel(0.0, 0.4, 1.5)

                # Resume keyboard control
                control.start_keyboard_input()
                control.start_keyboard_control()

                # Clear movement mode to wait for next key
                movement_mode = None


    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 2
            
    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            
except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()