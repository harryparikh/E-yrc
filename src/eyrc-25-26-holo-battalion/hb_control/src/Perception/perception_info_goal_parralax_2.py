#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
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
        calib_file = "/home/harry/calibration_data_techshila/ost.yaml"

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

    # [UPDATED] DRAW HUD
    # Handles: Drop/Pickup Zones, Safety Circles, Text Status, Live Coordinate Boxes
    # -------------------------------------------------------------
    # [UPDATED] DRAW HUD
    # [UPDATED] DRAW HUD
    def draw_hud(self, img, ids, corners, bot_poses, crate_poses):
        
        # --- A. DRAW ZONES (Static) ---
        self.draw_zone_box(img, 0, 1020, 1400, 1075, 1355, zone_type="drop")
        self.draw_zone_box(img, 1, 675, 920, 1920, 2115, zone_type="drop")
        self.draw_zone_box(img, 2, 1470, 1752, 1920, 2115, zone_type="drop")

        pickup_zones = [
            {'id': 0, 'x_min': 180, 'x_max': 390, 'y_min': 560, 'y_max': 1111},
            {'id': 1, 'x_min': 2060, 'x_max': 2266, 'y_min': 560, 'y_max': 1111},
            {'id': 2, 'x_min': 180, 'x_max': 390, 'y_min': 1350, 'y_max': 1900},
            {'id': 3, 'x_min': 2060, 'x_max': 2266, 'y_min': 1350, 'y_max': 1900}
        ]
        for pz in pickup_zones:
            self.draw_zone_box(img, pz['id'], pz['x_min'], pz['x_max'], 
                               pz['y_min'], pz['y_max'], zone_type="pickup")

        # --- B. PREPARE DATA ---
        controller_data = self.latest_debug_data.get('bots', {})
        drop_data = self.latest_debug_data.get('drop_positions', {})
        
        # Dictionary to store pixel centers of bots for line drawing later
        bot_pixel_centers = {} 

        if ids is not None:
            for i, aruco_id in enumerate(ids.flatten()):
                sid = str(aruco_id)
                iid = int(aruco_id)
                
                # Get Pixel Center
                c = corners[i][0]
                cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))

                # --- LOGIC FOR BOTS ---
                if iid in bot_poses: 
                    # Store center for distance lines
                    bot_pixel_centers[iid] = (cx, cy) 
                    
                    pose = bot_poses[iid]
                    # Current Position Text (Bottom Right)
                    coord_text = f"B{iid} X:{int(pose['x'])} Y:{int(pose['y'])} W:{int(pose['yaw'])}"
                    cv2.putText(img, coord_text, (cx - 55, cy + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                    if sid in controller_data:
                        info = controller_data[sid]
                        
                        # 1. Prepare Goal Data
                        goal_arr = info.get('goal', [])
                        goal_text = ""  # Default empty if no goal
                        
                        # 2. Draw Goal Line & Format Text
                        if len(goal_arr) >= 2:
                            gx_mm, gy_mm = goal_arr[0], goal_arr[1]
                            
                            dx = gx_mm - pose['x']
                            dy = gy_mm - pose['y']
                            
                            if len(goal_arr) >= 3:
                                g_w = goal_arr[2]
                                dyaw = g_w - pose['yaw']
                                # Normalize dyaw to [-180, 180] so it doesn't show 350 deg
                                dyaw = (dyaw + 180) % 360 - 180
                                goal_text = f" D:[{int(dx)},{int(dy)},{int(dyaw)}]" 
                            else:
                                goal_text = f" D:[{int(dx)},{int(dy)}]"
                            
                            pgx, pgy = self.world_to_pixel(gx_mm, gy_mm)
                            if pgx is not None:
                                cv2.line(img, (cx, cy), (pgx, pgy), (0, 255, 0), 1)
                                cv2.circle(img, (pgx, pgy), 6, (0, 255, 0), -1)
                                # Optional: Draw a tiny "G" at the actual goal pixel
                                cv2.putText(img, "G", (pgx + 5, pgy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                        
                        # 3. Draw Status + Goal Text (Top Right)
                        # Combines Status (e.g., "MOVING") with Goal (e.g., "G:[1200,500]")
                        status_label = f"ID:{aruco_id} {info.get('status', 'IDLE')}{goal_text}"
                        
                        cv2.putText(img, status_label, (cx + 25, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                # --- LOGIC FOR CRATES ---
                elif iid in crate_poses:
                    pose = crate_poses[iid]
                    cyan = (255, 255, 0)
                    cv2.rectangle(img, (cx-20, cy-20), (cx+20, cy+20), cyan, 2)
                    cv2.putText(img, f"CRATE {iid}", (cx-35, cy-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cyan, 2)
                    
                    coord_text = f"X:{int(pose['x'])} Y:{int(pose['y'])}"
                    cv2.putText(img, coord_text, (cx - 40, cy + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, cyan, 1)

                    if sid in drop_data:
                        tgt = drop_data[sid]
                        tx, ty = self.world_to_pixel(tgt['x'], tgt['y'])
                        if tx is not None:
                            cv2.line(img, (cx, cy), (tx, ty), cyan, 1)
                            cv2.circle(img, (tx, ty), 10, cyan, 2)
                            cv2.circle(img, (tx, ty), 3, cyan, -1)
                            cv2.putText(img, "DROP", (tx+15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cyan, 2)

        # --- C. DRAW DISTANCE LINES BETWEEN ALL BOTS ---
        # Get all detected bot IDs
        detected_bots = list(bot_pixel_centers.keys())
        
        # Iterate through every unique pair of bots
        for i in range(len(detected_bots)):
            for j in range(i + 1, len(detected_bots)):
                id1 = detected_bots[i]
                id2 = detected_bots[j]

                # 1. Get Pixel Centers (for drawing the line)
                pt1 = bot_pixel_centers[id1]
                pt2 = bot_pixel_centers[id2]

                # 2. Get World Coordinates (for calculating accurate distance)
                # Ensure we use the pose data which is perspective corrected
                p1_real = bot_poses[id1] 
                p2_real = bot_poses[id2]

                # 3. Calculate Euclidean Distance (in meters)
                dist_mm = math.sqrt( (p1_real['x'] - p2_real['x'])**2 + (p1_real['y'] - p2_real['y'])**2 )
                dist_m = dist_mm / 1000.0

                # 4. Draw Line (Magenta)
                cv2.line(img, pt1, pt2, (255, 0, 255), 2)

                # 5. Draw Text (Distance) at the midpoint
                mid_x = int((pt1[0] + pt2[0]) / 2)
                mid_y = int((pt1[1] + pt2[1]) / 2)
                
                # Create a small black background for text readability
                label = f"{dist_m:.2f}m"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(img, (mid_x - 5, mid_y - th - 5), (mid_x + tw + 5, mid_y + 5), (0,0,0), -1)
                
                cv2.putText(img, label, (mid_x, mid_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

    # [UPDATED] DRAW ZONE BOX (Handles both Drop and Pickup Locks)
    # -------------------------------------------------------------
    def draw_zone_box(self, img, zone_id, x_min, x_max, y_min, y_max, zone_type="drop"):
        tl = self.world_to_pixel(x_min, y_min)
        tr = self.world_to_pixel(x_max, y_min)
        br = self.world_to_pixel(x_max, y_max)
        bl = self.world_to_pixel(x_min, y_max)

        if tl[0] is None: return

        # --- DYNAMIC LOCK LOOKUP ---
        locked_by = None
        
        # Determine which lock list to check based on zone type
        if zone_type == "drop":
            locks = self.latest_debug_data.get('drop_zones', {}).get('locks', {})
            locked_by = locks.get(str(zone_id))
            label_prefix = "DROP"
        else: # pickup
            locks = self.latest_debug_data.get('drop_zones', {}).get('pickup_zone_locks', {})
            locked_by = locks.get(str(zone_id))
            label_prefix = "PICK"
        
        # --- COLOR & TEXT LOGIC ---
        if locked_by is not None: 
            color = (0, 0, 255) # Red
            text = f"{label_prefix} {zone_id}: LOCKED ({locked_by})"
            thickness = 2
        else:
            color = (0, 255, 0) # Green
            text = f"{label_prefix} {zone_id}: FREE"
            thickness = 1

        # Draw Box
        pts = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [pts], True, color, thickness)
        
        # Crosshatch if locked
        if locked_by is not None:
            cv2.line(img, tl, br, color, 1)
            cv2.line(img, tr, bl, color, 1)

        # Draw Label
        cx = int((tl[0] + br[0]) / 2)
        cy = int((tl[1] + br[1]) / 2)
        
        font_scale = 0.5
        (text_w, text_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
        text_x = cx - (text_w // 2)
        text_y = cy + (text_h // 2)

        # cv2.rectangle(img, (text_x - 5, text_y - text_h - 5), (text_x + text_w + 5, text_y + 5), (0, 0, 0), -1)
        cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 1)

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

                        rvecs, _, _ = cv2.aruco.estimatePoseSingleMarkers(
                            np.array([c]), self.bots_marker_length, 
                            self.camera_matrix, self.dist_coeffs
                        )
                        
                        if rvecs is not None:
                            rmat, _ = cv2.Rodrigues(rvecs[0][0])
                            yaw = math.degrees(math.atan2(rmat[1, 0], rmat[0, 0]))
                            # self.get_logger().info(f"Arena Marker ID: {aruco_id} | Yaw: {yaw:.2f}°")

                if len(self.pixel_matrix) >= 4:
                    pp = np.array(self.pixel_matrix, dtype=np.float32)
                    wp = np.array(self.world_matrix, dtype=np.float32)
                    self.H_matrix, _ = cv2.findHomography(pp, wp, cv2.RANSAC, 5.0)

                # --- POSE & PARALLAX CORRECTION ---
                if self.H_matrix is not None:
                    # 1. Get the optical center in pixels from intrinsic matrix
                    cam_cx_pixel = self.camera_matrix[0, 2]
                    cam_cy_pixel = self.camera_matrix[1, 2]
                    
                    # 2. Convert optical center to real-world coordinates using Homography
                    cam_world_x, cam_world_y = self.pixel_to_world(cam_cx_pixel, cam_cy_pixel)

                    bot_poses = {}
                    crate_poses = {}
                    
                    for i, aruco_id in enumerate(ids.flatten()):
                        if aruco_id not in ARENA_LAYOUT:
                            marker_corners = refined_corners[i][0]
                            cx, cy = np.mean(marker_corners[:, 0]), np.mean(marker_corners[:, 1])
                            x, y = self.pixel_to_world(cx, cy)

                            # --- UPDATED PARALLAX CORRECTION ---
                            cam_h = 2590.0
                            obj_h = 51.0
                            if aruco_id in (0,2,4): 
                                obj_h = 111.25
                            
                            scale = obj_h / cam_h
                            
                            # Ensure we have a valid camera world center before correcting
                            if cam_world_x is not None and cam_world_y is not None:
                                x -= scale * (x - cam_world_x)
                                y -= scale * (y - cam_world_y)

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

                    # Draw HUD with LIVE POSES
                    self.draw_hud(undistorted_image, ids, refined_corners, bot_poses, crate_poses)
            
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