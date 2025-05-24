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
    control.start_keyboard_control()       # (Optional: for debugging)

    rclpy.spin_once(robot, timeout_sec=0.1)


# === Helper Function: Checks if there's an obstacle ahead ===
def is_obstacle_ahead(lidar, threshold, cone_width):
    """
    Checks if an obstacle is detected in front of the robot using LIDAR.

    Args:
        lidar (Lidar): Lidar object
        threshold (float): Distance in meters to consider "too close"
        cone_width (int): Degrees to the left/right of center to scan

    Returns:
        bool: True if something is within the threshold distance
    """
    scan = lidar.checkScan()
    min_dist, angle = lidar.detect_obstacle_in_cone(scan, threshold, 0, cone_width)
    #print(lidar.detect_obstacle_in_cone(scan, threshold, 0, cone_width))
    return min_dist != -1

# Main Challenge Stuff
try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing
            
    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.01)
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall

            dist_to_wall = 0.40 # 15 cm from the front of robot to the wall
            cone_angle = 10      # 20 degrees from centre
            backing = False

            if is_obstacle_ahead(lidar, dist_to_wall, cone_angle):
                # logging.configure_logging(lidar)
                #control.stop_keyboard_control()
                # control.set_cmd_vel(0.0, 0.0, 0.5)   # Stop
                control.set_cmd_vel(-0.1, 0.0, 1.5)  # Back up
                # control.start_keyboard_control()

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
