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
challengeLevel = 2               # Set this to 1 or 2 to run the desired level
is_SIM = False                   # True = simulation, False = real robot
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
    return min_dist != -1

# === Main Challenge Logic ===
try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Level 0 = Manual keyboard control only

    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.01)

            dist_to_wall = 0.40   # 15 cm from front of robot
            cone_angle = 10       # ±10° cone for obstacle detection

            if is_obstacle_ahead(lidar, dist_to_wall, cone_angle):
                print("🚧 Obstacle detected. Backing up...")
                control.set_cmd_vel(-0.1, 0.0, 1.5)  # Back up

    if challengeLevel == 2:
        print("🚦 Level 2 – Stop Sign Detection Enabled")
        stop_sign_last_seen = 0
        cooldown = 5  # Seconds between stops to prevent spamming
        area_threshold = 5000  # Tune this number to match "close enough"
 
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)

            # Step 1: Get the latest image from the camera
            img = camera.rosImg_to_cv2()
            camera.checkImageRelease()

            # Step 2: Run stop sign detection
            stop_detected, x1, y1, x2, y2 = camera.ML_predict_stop_sign(img)

            # Step 3: If stop sign is detected and cooldown has passed
            if stop_detected and (time.time() - stop_sign_last_seen > cooldown):
                box_width = x2 - x1
                box_height = y2 - y1
                box_area = box_width * box_height
                print(f"Stop sign box area: {box_area}  (width: {box_width}, height: {box_height})")
                print(f"Stop sign detected. Bounding box area: {box_area}")

                # Step 3: Only stop if close enough AND cooldown passed
                if box_area > area_threshold and (time.time() - stop_sign_last_seen > cooldown):
                    print("Close to stop sign — stopping for 3 seconds")
                    control.set_cmd_vel(0.0, 0.0, 3.0)
                    stop_sign_last_seen = time.time()

            # Step 4: Slight delay to reduce CPU usage
            time.sleep(0.1)

    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()