#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo
from hb_interfaces.msg import Pose2D, Poses2D
import yaml
import json  # <--- NEW
from std_msgs.msg import String # <--- NEW


# Arena layout: IDs and their real-world coordinates in meters (X, Y)
# Using the specified coordinate system: X-positive left, Y-positive down
ARENA_LAYOUT = {
    1: [0.0, 0.0],   # Top-left corner
    3: [2.4384, 0.0],   # Top-right corner
    7: [2.4384, 2.4384],   # Bottom-right corner
    5: [0.0, 2.4384]    # Bottom-left corner
}

class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')

        # ---------- LOAD REAL CAMERA CALIBRATION ----------
        calib_file = "/home/harry/calibration_data2/ost.yaml"  # <-- CHANGE PATH

        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        # Camera matrix (K)
        self.camera_matrix = np.array(
            calib_data['camera_matrix']['data']
        ).reshape(3, 3)

        # Distortion coefficients (D)
        self.dist_coeffs = np.array(
            calib_data['distortion_coefficients']['data']
        )

        self.get_logger().info("Loaded REAL camera calibration from YAML")

        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # ---------- PARAMETERS ----------
        # Set marker size in meters (assuming all markers are the same size)
        self.crates_marker_length = 0.05
        self.bots_marker_length = 0.05
        
        self.aruco_dict_name = 'DICT_4X4_50' # Choose ArUco dictionary
        # ---------- ARENA LIMITS (mm) ----------
        # 2.4384 meters = 2438.4 mm
        self.arena_min = 0.0
        self.arena_max = 2438.4
        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo, "/camera/camera_info", self.camera_info_callback, 1
        # )
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)

        self.camera_debug_pub = self.create_publisher(String, '/camera_debug', 10)
        # [NEW] Subscribe to Controller Debug Info
        self.debug_sub = self.create_subscription(String, '/perception_debug', self.debug_callback, 10)
        self.latest_debug_data = {} # Stores the latest JSON packet
        
        self.get_logger().info('PoseDetector initialized with AR Overlay')
        
        # Camera Parameters
        # self.camera_matrix = None
        # self.dist_coeffs = None

        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = []  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = []  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        
        # ---------- ARUCO SETUP ----------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.get_logger().info('PoseDetector initialized')

    # def camera_info_callback(self, msg):
    #     """
    #     Callback to receive and load camera intrinsic parameters.
    #     """
    #     if self.camera_matrix is None:
    #         # The 'k' field in the message is a flat list of 9 elements
    #         self.camera_matrix = np.array(msg.k).reshape(3, 3)
            
    #         # The 'd' field is a list of distortion coefficients
    #         self.dist_coeffs = np.array(msg.d)
            
    #         self.get_logger().info('Camera parameters loaded successfully.')
            
    #         # Destroy the subscription since we only need the info once
    #         self.destroy_subscription(self.camera_info_sub)

    def pixel_to_world(self, pixel_x, pixel_y):
        """
        Converts pixel coordinates to real-world coordinates using the pre-computed homography matrix.
        """
        # Step 1: Ensure H_matrix is computed
        if self.H_matrix is None:
            self.get_logger().error("Homography matrix is not computed!")
            return None, None
            
        # Step 2: Create pixel point in correct format for cv2.perspectiveTransform
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        
        # Step 3: Apply transformation
        world_point = cv2.perspectiveTransform(pixel_point, self.H_matrix)
        
        # Return world coordinates (x, y)
        world_x = world_point[0][0][0]
        world_y = world_point[0][0][1]

        # Convert to millimeters as per the requirement
        world_x_mm = world_x * 1000.0
        world_y_mm = world_y * 1000.0

        return world_x_mm, world_y_mm
    

    # [NEW] Callback to store controller data
    def debug_callback(self, msg):
        try:
            self.latest_debug_data = json.loads(msg.data)
        except: pass

    # [NEW] Helper to map World (mm) -> Pixel (x,y) for drawing zones
    def world_to_pixel(self, wx, wy):
        if self.H_matrix is None: return None, None
        try:
            # Inverse Homography to map back from World to Image
            H_inv = np.linalg.inv(self.H_matrix)
            # Input must be in meters to match your homography scale
            world_pt = np.array([[[wx/1000.0, wy/1000.0]]], dtype=np.float32)
            pixel_pt = cv2.perspectiveTransform(world_pt, H_inv)
            return int(pixel_pt[0][0][0]), int(pixel_pt[0][0][1])
        except:
            return None, None

    # [NEW] The main drawing function
    def draw_hud(self, img, ids, corners):
        if not self.latest_debug_data: return

        # A. Draw Drop Zones
        # Zone 0 (Top): 1020-1400, 1075-1355
        self.draw_zone_box(img, 0, 1020, 1400, 1075, 1355)
        # Zone 1 (Left): 675-920, 1920-2115
        self.draw_zone_box(img, 1, 675, 920, 1920, 2115)
        # Zone 2 (Right): 1470-1752, 1920-2115
        self.draw_zone_box(img, 2, 1470, 1752, 1920, 2115)

        # B. Draw Bot Status Tags
        bots_data = self.latest_debug_data.get('bots', {})
        
        if ids is not None:
            for i, aruco_id in enumerate(ids.flatten()):
                sid = str(aruco_id)
                if sid in bots_data: 
                    # Get pixel center of this bot
                    c = corners[i][0]
                    cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))

                    # Extract Info
                    info = bots_data[sid]
                    status = info['status']
                    vel = info['velocity']
                    
                    # Color Logic
                    color = (0, 255, 255) # Yellow (Default)
                    if "DROPPING" in status: color = (0, 255, 0) # Green
                    elif "STAGING" in status: color = (255, 100, 0) # Blue
                    
                    # Draw Text
                    label = f"ID:{aruco_id} {status}"
                    cv2.putText(img, label, (cx+20, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(img, f"V:{vel:.1f}", (cx+20, cy+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

                    # Draw Sharing Indicator (Purple Circle)
                    if info['sharing_zone']:
                        cv2.circle(img, (cx, cy), 40, (128, 0, 128), 2)

    # [NEW] Helper to draw a single zone rectangle
    # [UPDATED] Helper to draw a single zone rectangle with "Inside" visualization
    def draw_zone_box(self, img, zone_id, x_min, x_max, y_min, y_max):
        tl = self.world_to_pixel(x_min, y_min)
        tr = self.world_to_pixel(x_max, y_min)
        br = self.world_to_pixel(x_max, y_max)
        bl = self.world_to_pixel(x_min, y_max)

        if tl[0] is None: return

        # Check lock status from data
        locks = self.latest_debug_data.get('drop_zones', {}).get('locks', {})
        locked_by = locks.get(str(zone_id))
        
        # Determine Style
        if locked_by is not None: 
            color = (0, 0, 255) # Red for Locked
            text = f"LOCKED: {locked_by}"
            thickness = 3
        else:
            color = (0, 255, 0) # Green for Free
            text = "FREE"
            thickness = 2

        # 1. Draw the Zone Boundary
        pts = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [pts], True, color, thickness)
        
        # 2. If Locked, draw a large "X" inside the zone
        if locked_by is not None:
            cv2.line(img, tl, br, color, 2)
            cv2.line(img, tr, bl, color, 2)

        # 3. Calculate Center Point
        cx = int((tl[0] + br[0]) / 2)
        cy = int((tl[1] + br[1]) / 2)

        # 4. Center the Text
        font_scale = 0.6
        (text_w, text_h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)
        text_x = cx - (text_w // 2)
        text_y = cy + (text_h // 2)

        # Draw black background rectangle for text readability
        cv2.rectangle(img, 
                      (text_x - 5, text_y - text_h - 5), 
                      (text_x + text_w + 5, text_y + 5), 
                      (0, 0, 0), -1)
        
        # Draw the text overlay
        cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)

    def image_callback(self, msg):
        try:
            if self.camera_matrix is None or self.dist_coeffs is None: return

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
            gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = self.detector.detectMarkers(gray)
            
            # Use refined corners if available
            refined_corners = corners
            if ids is not None:
                # Subpixel refinement
                all_corners_flat = np.concatenate(corners, axis=1).reshape(-1, 2).astype(np.float32)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                refined_flat = cv2.cornerSubPix(gray, all_corners_flat, (5, 5), (-1, -1), criteria)
                
                # Reshape back
                start = 0
                refined_corners = []
                for mc in corners:
                    num = len(mc[0])
                    end = start + num
                    refined_corners.append(refined_flat[start:end].reshape(1, num, 2))
                    start = end
                
                cv2.aruco.drawDetectedMarkers(undistorted_image, refined_corners, ids)

                # --- HOMOGRAPHY ---
                self.pixel_matrix = []
                self.world_matrix = []
                for i, aruco_id in enumerate(ids.flatten()):
                    if aruco_id in ARENA_LAYOUT:
                        c = refined_corners[i][0]
                        if aruco_id == 1:   px, py, wx, wy = c[0][0], c[0][1], 0.0, 0.0
                        elif aruco_id == 3: px, py, wx, wy = c[1][0], c[1][1], 2.4384, 0.0
                        elif aruco_id == 7: px, py, wx, wy = c[2][0], c[2][1], 2.4384, 2.4384
                        elif aruco_id == 5: px, py, wx, wy = c[3][0], c[3][1], 0.0, 2.4384
                        self.pixel_matrix.append([px, py])
                        self.world_matrix.append([wx, wy])

                if len(self.pixel_matrix) >= 4:
                    pp = np.array(self.pixel_matrix, dtype=np.float32)
                    wp = np.array(self.world_matrix, dtype=np.float32)
                    self.H_matrix, _ = cv2.findHomography(pp, wp, cv2.RANSAC, 5.0)

                # --- POSE ---
                if self.H_matrix is not None:
                    bot_poses = {}
                    crate_poses = {}
                    
                    for i, aruco_id in enumerate(ids.flatten()):
                        if aruco_id not in ARENA_LAYOUT:
                            marker_corners = refined_corners[i][0]
                            cx, cy = np.mean(marker_corners[:, 0]), np.mean(marker_corners[:, 1])
                            x, y = self.pixel_to_world(cx, cy)

                            # Correction
                            cam_h, obj_h, center_mm = 3000, 51.0, 1219.2
                            if aruco_id in (0,2,4): obj_h = 111.25
                            scale = obj_h / cam_h
                            x -= scale * (x - center_mm)
                            y -= scale * (y - center_mm)

                            # Yaw
                            rvecs, _, _ = cv2.aruco.estimatePoseSingleMarkers(
                                np.array([marker_corners]), self.bots_marker_length, 
                                self.camera_matrix, self.dist_coeffs
                            )
                            rmat, _ = cv2.Rodrigues(rvecs[0][0])
                            yaw = math.degrees(math.atan2(rmat[1, 0], rmat[0, 0]))

                            # --- [FIX 1] CAST NUMPY INT TO PYTHON INT ---
                            dict_key = int(aruco_id) 

                            pose_data = {'id': dict_key, 'x': x, 'y': y, 'yaw': yaw}
                            
                            if aruco_id in (0,2,4):
                                bot_poses[dict_key] = pose_data
                            elif (self.arena_min <= x <= self.arena_max) and (self.arena_min <= y <= self.arena_max):
                                crate_poses[dict_key] = pose_data

                    # --- [FIX 2] UN-INDENT THESE LINES ---
                    # They must run ONCE after the loop, not 10 times inside the loop!
                    self.publish_crate_poses(list(crate_poses.values()))
                    self.publish_bot_poses(list(bot_poses.values()))
                    self.publish_camera_debug(bot_poses, crate_poses)

                    # Draw HUD
                    self.draw_hud(undistorted_image, ids, refined_corners)
            
            cv2.imshow('Detected Markers', undistorted_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


    def publish_crate_poses(self, poses):
        """
        Publishes the list of crate poses.
        """
        poses_msg = Poses2D()
        for pose_data in poses:
            crate_pose = Pose2D()
            crate_pose.id = pose_data['id']
            crate_pose.x = pose_data['x']
            crate_pose.y = pose_data['y']
            crate_pose.w = pose_data['yaw']
            poses_msg.poses.append(crate_pose)
        
        self.crate_poses_pub.publish(poses_msg)

    def publish_bot_poses(self, poses):
        """
        Publishes the list of bot poses.
        """
        poses_msg = Poses2D()
        for pose_data in poses:
            bot_pose = Pose2D()
            bot_pose.id = pose_data['id']
            bot_pose.x = pose_data['x']
            bot_pose.y = pose_data['y']
            bot_pose.w = pose_data['yaw']
            poses_msg.poses.append(bot_pose)
            
        self.bot_poses_pub.publish(poses_msg)
    # [ADD THIS NEW FUNCTION]
    def publish_camera_debug(self, bot_poses, crate_poses):
        """
        Publishes raw detection data for the visualization dashboard.
        """
        # Get current time
        now = self.get_clock().now().nanoseconds / 1e9
        
        packet = {
            "timestamp": now,
            "bots": bot_poses,      # Dictionary of {id: {x, y, yaw}}
            "crates": crate_poses   # Dictionary of {id: {x, y, yaw}}
        }
        
        msg = String()
        msg.data = json.dumps(packet)
        self.camera_debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pose_detector = PoseDetector()
    try:
        rclpy.spin(pose_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
