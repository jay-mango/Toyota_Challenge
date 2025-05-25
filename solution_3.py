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
challengeLevel = 3               # Set this to 1 or 2 to run the desired level
is_SIM = False                   # True = simulation, False = real robot
Debug = False                    # True = print extra info
last_stop_sign_time = 0  # Global variable for stop sign cooldown

# === Initialize the robot and interfaces ===
robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)


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


# Create a global variable to store the last stop time
def stop_sign_detection(frame, camera, control,
                        cooldown=5, area_threshold=5000, pause_duration=3):
    """
    Detects stop signs using the provided image.
    Stops the robot if a stop sign is detected close enough and cooldown has passed.
    """
    global last_stop_sign_time

    camera.checkImageRelease()  # Optional: release internal buffer

    detected, x1, y1, x2, y2 = camera.ML_predict_stop_sign(frame)
    if not detected:
        return False

    box_area = (x2 - x1) * (y2 - y1)
    now = time.time()

    if box_area > area_threshold and (now - last_stop_sign_time > cooldown):
        print(f"🛑 Stop sign detected (area={box_area:.0f}) — pausing for {pause_duration}s")
        control.send_cmd_vel(0.0, 0.0)
        time.sleep(pause_duration)
        last_stop_sign_time = now
        control.rotate(15, -CCW)
        control.set_cmd_vel(0.2, 0.0, 1)
        control.rotate(15, CCW)
        return True

# === Start keyboard control in case it's needed for manual overrides (Levels 0–2) ===
if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)

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

        # ────────── TUNABLES ──────────
        CRUISE_FWD      = 0.3      # m/s
        KP_BEARING      = 0.015     # rad/s per degree
        CORNER_RANGE    = 0.6      # m
        COLLISION_RANGE = 0.3      # m
        CCW              = 1        # counter clockwise flag for Control.rotate()

        turning      = False
        last_turn_ts = time.time()
        stop_detected = False   # <<<<<<<<<<<

        print("🟣  Level 3 started — Tag-lock guidance on Loop B")
        while rclpy.ok():
            # 1. ROS spin / sensors
            rclpy.spin_once(robot, timeout_sec=0.0)
            frame = camera.rosImg_to_cv2()
            tags = camera.estimate_apriltag_pose(frame)
            scan = lidar.checkScan()

            # 2. LiDAR emergency
            #f_dist, _ = lidar.detect_obstacle_in_cone(scan, COLLISION_RANGE, 0, 20)
            #if f_dist != -1 and f_dist < COLLISION_RANGE:
                #control.send_cmd_vel(0.0, 0.0)
                #print("⚠️  Obstacle ahead — braking.")
                #continue

            # # 3. Stop-sign handler  ← **single function call**
            stop_sign_detection(frame, camera, control)

            # 4. April-Tag steering
            if tags and not turning:
                tag_id, rng, bearing_deg, _ = min(tags, key=lambda t: t[1])

                if rng < CORNER_RANGE and (time.time() - last_turn_ts) > 1.0:
                    print(f"↩️  Corner detected (tag {tag_id}, range={rng:.2f} m) — evaluating turn angle")
                    turning = True
                    last_turn_ts = time.time()

                    if tag_id == 3:
                        print("⤿ Diagonal corner — turning 45° left")
                        control.rotate(45, CCW)
                    elif tag_id == 5:
                        print("⤿ Diagonal corner — turning 135° left")
                        control.rotate(135, CCW)
                    else:
                        print(f"↪️ Right-angle or unknown tag {tag_id} — turning 90° left")
                        control.rotate(90, CCW)

                    turning = False
                    continue

                # Regular tag-lock
                ang_z = -KP_BEARING * bearing_deg   # minus = clockwise
                control.send_cmd_vel(CRUISE_FWD, ang_z)

            else:
                # Tag lost → creep forward
                control.send_cmd_vel(0.10, 0.0)

            time.sleep(0.04)    # ≈25 Hz loop

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