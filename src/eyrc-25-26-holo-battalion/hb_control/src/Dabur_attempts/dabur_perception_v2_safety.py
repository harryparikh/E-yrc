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
import json
from std_msgs.msg import String


# Arena layout: IDs and their real-world coordinates in meters (X, Y)
ARENA_LAYOUT = {
    1: [0.0, 0.0],      # Top-left corner
    3: [2.4384, 0.0],   # Top-right corner
    7: [2.4384, 2.4384],# Bottom-right corner
    5: [0.0, 2.4384]    # Bottom-left corner
}

class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')

        # ---------- LOAD REAL CAMERA CALIBRATION ----------
        calib_file = "/home/harry/calibration_data2/ost.yaml"

        with open(calib_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
        self.dist_coeffs = np.array(calib_data['distortion_coefficients']['data'])

        self.get_logger().info("Loaded REAL camera calibration from YAML")
        
        self.bridge = CvBridge()
        
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05
        self.bots_marker_length = 0.05
        self.aruco_dict_name = 'DICT_4X4_50'
        self.arena_min = 0.0
        self.arena_max = 2438.4
        
        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)

        self.camera_debug_pub = self.create_publisher(String, '/camera_debug', 10)
        self.debug_sub = self.create_subscription(String, '/perception_debug', self.debug_callback, 10)
        self.latest_debug_data = {} 
        
        # ---------- ARUCO SETUP ----------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Matrices
        self.pixel_matrix = []
        self.world_matrix = []
        self.H_matrix = None
        
        self.get_logger().info('PoseDetector initialized')

    def pixel_to_world(self, pixel_x, pixel_y):
        if self.H_matrix is None:
            return None, None
        pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(pixel_point, self.H_matrix)
        
        world_x_mm = world_point[0][0][0] * 1000.0
        world_y_mm = world_point[0][0][1] * 1000.0
        return world_x_mm, world_y_mm
    
    def debug_callback(self, msg):
        try:
            self.latest_debug_data = json.loads(msg.data)
        except: pass

    def world_to_pixel(self, wx, wy):
        if self.H_matrix is None: return None, None
        try:
            H_inv = np.linalg.inv(self.H_matrix)
            world_pt = np.array([[[wx/1000.0, wy/1000.0]]], dtype=np.float32)
            pixel_pt = cv2.perspectiveTransform(world_pt, H_inv)
            return int(pixel_pt[0][0][0]), int(pixel_pt[0][0][1])
        except:
            return None, None

    # [UPDATED] VISUALIZATION HELPER
    # -------------------------------------------------------------
    def draw_triangle_from_vertices(self, img, vertices_world):
        """
        Draws follower triangle EXACTLY as controller computed it.
        No geometry, no guessing, only projection.
        """
        if self.H_matrix is None or not vertices_world:
            return

        pts = []
        for wx, wy in vertices_world:
            px, py = self.world_to_pixel(wx, wy)
            if px is None or py is None:
                return
            pts.append([px, py])

        pts = np.array(pts, np.int32)

        # Filled transparent triangle
        overlay = img.copy()
        cv2.fillPoly(overlay, [pts], (0, 255, 255))
        cv2.addWeighted(overlay, 0.3, img, 0.7, 0, img)

        # Outline
        cv2.polylines(img, [pts], True, (0, 165, 255), 2)


    
    def draw_hud(self, img, ids, corners):
        if not self.latest_debug_data: return

        # A. Draw Drop Zones
        self.draw_zone_box(img, 0, 1020, 1400, 1075, 1355)
        self.draw_zone_box(img, 1, 675, 920, 1920, 2115)
        self.draw_zone_box(img, 2, 1470, 1752, 1920, 2115)

        # B. Get Bot Data
        bots_data = self.latest_debug_data.get('bots', {})
        
        if ids is not None:
            for i, aruco_id in enumerate(ids.flatten()):
                sid = str(aruco_id)
                if sid in bots_data:
                    # Extract Info FIRST
                    info = bots_data[sid]

                    # World center from controller debug (GROUND TRUTH)
                    wx, wy = info['pose'][0], info['pose'][1]

                    # Convert to pixel for drawing
                    cx, cy = self.world_to_pixel(wx, wy)
                    if cx is None or cy is None:
                        continue

                    status = info['status']
                    vel = info['velocity']
                    
                    # ---------------------------------------------------------
                    # 1. DRAW TRIANGLE (Using Vertices from Controller)
                    # ---------------------------------------------------------
                    vertices = info.get('triangle_vertices', [])
                    if vertices:
                        self.draw_triangle_from_vertices(img, vertices)

                    # ---------------------------------------------------------
                    # 2. DRAW SAFETY CIRCLE (NEW) 
                    # ---------------------------------------------------------
                    # Get the world radius (default 80mm if missing)
                    r_world_mm = info.get('radius', 80.0)
                    
                    # To draw it accurately in pixels, we project 
                    # the center + radius in world coords back to pixels
                    wx, wy = info['pose'][0], info['pose'][1]
                    if wx is not None:
                        # Project a point on the circle's edge (e.g., to the right)
                        px_edge, py_edge = self.world_to_pixel(wx + r_world_mm, wy)
                        
                        if px_edge is not None:
                            # Calculate pixel radius
                            pixel_r = int(math.hypot(px_edge - cx, py_edge - cy))
                            
                            # Draw the Safety Circle (Orange)
                            cv2.circle(img, (cx, cy), pixel_r, (0, 165, 255), 2)

                    # ---------------------------------------------------------
                    # 3. DRAW TEXT TAGS
                    # ---------------------------------------------------------
                    color = (0, 255, 255) 
                    if "DROPPING" in status: color = (0, 255, 0)
                    elif "STAGING" in status: color = (255, 100, 0)
                    elif "TARGETING" in status: color = (50, 200, 255)
                    
                    label_top = f"ID:{aruco_id} {status}"
                    label_bot = f"V:{vel:.1f}"

                    def draw_text_with_bg(image, text, x, y, font_scale, txt_color):
                        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
                        cv2.rectangle(image, (x, y - h - 4), (x + w, y + 4), (0, 0, 0), -1)
                        cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, txt_color, 1)

                    draw_text_with_bg(img, label_top, cx + 25, cy - 5, 0.5, color)
                    draw_text_with_bg(img, label_bot, cx + 25, cy + 15, 0.5, (255, 255, 255))

                    if info.get('sharing_zone', False):
                        cv2.circle(img, (cx, cy), 15, (128, 0, 128), -1)

    def draw_zone_box(self, img, zone_id, x_min, x_max, y_min, y_max):
        tl = self.world_to_pixel(x_min, y_min)
        tr = self.world_to_pixel(x_max, y_min)
        br = self.world_to_pixel(x_max, y_max)
        bl = self.world_to_pixel(x_min, y_max)

        if tl[0] is None: return

        locks = self.latest_debug_data.get('drop_zones', {}).get('locks', {})
        locked_by = locks.get(str(zone_id))
        
        if locked_by is not None: 
            color = (0, 0, 255) # Red
            text = f"LOCKED: {locked_by}"
            thickness = 3
        else:
            color = (0, 255, 0) # Green
            text = "FREE"
            thickness = 2

        pts = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [pts], True, color, thickness)
        
        if locked_by is not None:
            cv2.line(img, tl, br, color, 2)
            cv2.line(img, tr, bl, color, 2)

        cx = int((tl[0] + br[0]) / 2)
        cy = int((tl[1] + br[1]) / 2)
        font_scale = 0.6
        (text_w, text_h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 2)
        text_x = cx - (text_w // 2)
        text_y = cy + (text_h // 2)

        cv2.rectangle(img, (text_x - 5, text_y - text_h - 5), (text_x + text_w + 5, text_y + 5), (0, 0, 0), -1)
        cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 2)

    def image_callback(self, msg):
        try:
            if self.camera_matrix is None or self.dist_coeffs is None: return

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
            gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = self.detector.detectMarkers(gray)
            
            refined_corners = corners
            if ids is not None:
                # Subpixel refinement
                all_corners_flat = np.concatenate(corners, axis=1).reshape(-1, 2).astype(np.float32)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                refined_flat = cv2.cornerSubPix(gray, all_corners_flat, (5, 5), (-1, -1), criteria)
                
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

                            dict_key = int(aruco_id) 
                            pose_data = {'id': dict_key, 'x': x, 'y': y, 'yaw': yaw}
                            
                            if aruco_id in (0,2,4):
                                bot_poses[dict_key] = pose_data
                            elif (self.arena_min <= x <= self.arena_max) and (self.arena_min <= y <= self.arena_max):
                                crate_poses[dict_key] = pose_data

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
        poses_msg = Poses2D()
        for pose_data in poses:
            bot_pose = Pose2D()
            bot_pose.id = pose_data['id']
            bot_pose.x = pose_data['x']
            bot_pose.y = pose_data['y']
            bot_pose.w = pose_data['yaw']
            poses_msg.poses.append(bot_pose)
        self.bot_poses_pub.publish(poses_msg)

    def publish_camera_debug(self, bot_poses, crate_poses):
        now = self.get_clock().now().nanoseconds / 1e9
        packet = {
            "timestamp": now,
            "bots": bot_poses,
            "crates": crate_poses
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