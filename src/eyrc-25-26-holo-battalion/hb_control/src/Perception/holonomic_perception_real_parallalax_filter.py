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
        calib_file = "/home/harry/calibration_data_techshila/ost.yaml"  # <-- CHANGE PATH

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
        self.allowed_crate_ids = [32, 35, 14, 21, 12, 30]
        # Set marker size in meters (assuming all markers are the same size)
        self.crates_marker_length = 0.05
        self.bots_marker_length = 0.05
        
        self.aruco_dict_name = 'DICT_4X4_50' # Choose ArUco dictionary
        
        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo, "/camera/camera_info", self.camera_info_callback, 1
        # )
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
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

    def image_callback(self, msg):
        try:
            if self.camera_matrix is None or self.dist_coeffs is None:
                self.get_logger().warn("Camera parameters not loaded yet.")
                return

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
            gray = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected_img_points = self.detector.detectMarkers(gray)
            
            if ids is not None:
                # Flatten the corners array for sub-pixel refinement
                all_corners_flat = np.concatenate(corners, axis=1).reshape(-1, 2).astype(np.float32)
                
                # Criteria for corner sub-pixel refinement
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                
                # Refine all detected corners
                refined_corners_flat = cv2.cornerSubPix(gray, all_corners_flat, (5, 5), (-1, -1), criteria)
                
                # Now, reshape the refined corners back into the original list of arrays format
                # Each marker has 4 corners, so we can split based on that
                start_idx = 0
                refined_corners = []
                for marker_corners in corners:
                    num_corners = len(marker_corners[0]) # Get the number of corners for the current marker
                    end_idx = start_idx + num_corners
                    refined_marker_corners_flat = refined_corners_flat[start_idx:end_idx]
                    
                    # Reshape it to the expected (1, N, 2) format, where N=4
                    refined_marker_corners = refined_marker_corners_flat.reshape(1, num_corners, 2)
                    refined_corners.append(refined_marker_corners)
                    
                    start_idx = end_idx
                
                # Draw the newly refined corners
                cv2.aruco.drawDetectedMarkers(undistorted_image, refined_corners, ids)


                # Print the coordinates of all four corners for each detected marker
                for i, aruco_id in enumerate(ids.flatten()):
                    corner = refined_corners[i][0]  # Get the corners for the current marker
                    self.get_logger().info(f"Marker ID: {aruco_id}")
                    for j, c in enumerate(corner):
                        x, y = c[0], c[1]
                        wrx,wry=self.pixel_to_world(x,y)
                        if wrx is not None and wry is not None:
                            self.get_logger().info(f"  Corner {j+1}: ({wrx:.2f}, {wry:.2f})")
                        else:
                            self.get_logger().info(f"  Corner {j+1}: Homography not yet computed")

                # Use refined_corners for homography and pose estimation

                self.pixel_matrix = []
                self.world_matrix = []

                for i, aruco_id in enumerate(ids.flatten()):
                    if aruco_id in ARENA_LAYOUT:
                        corner = refined_corners[i][0]

                        if aruco_id == 1:  # Top-left
                            px, py = corner[0]  # top-left corner
                            wx, wy = 0.0, 0.0

                        elif aruco_id == 3:  # Top-right
                            px, py = corner[1]  # top-right corner
                            wx, wy = 2.4384, 0.0

                        elif aruco_id == 7:  # Bottom-right
                            px, py = corner[2]  # bottom-right corner
                            wx, wy = 2.4384, 2.4384

                        elif aruco_id == 5:  # Bottom-left
                            px, py = corner[3]  # bottom-left corner
                            wx, wy = 0.0, 2.4384

                        self.pixel_matrix.append([px, py])
                        self.world_matrix.append([wx, wy])

                # self.pixel_matrix = []
                # self.world_matrix = []
                # for i, aruco_id in enumerate(ids.flatten()):
                #     if aruco_id in ARENA_LAYOUT:
                #         corner = refined_corners[i][0]
                #         center_x = np.mean(corner[:, 0])
                #         center_y = np.mean(corner[:, 1])
                #         self.pixel_matrix.append([center_x, center_y])
                #         self.world_matrix.append(ARENA_LAYOUT[aruco_id])
                
                if len(self.pixel_matrix) >= 4:
                    pixel_pts = np.array(self.pixel_matrix, dtype=np.float32)
                    world_pts = np.array(self.world_matrix, dtype=np.float32)
                    
                    self.H_matrix, _ = cv2.findHomography(pixel_pts, world_pts, cv2.RANSAC, 5.0)

                # ... (the rest of your pose estimation code remains the same, but uses refined_corners)
                if self.H_matrix is not None:
                    
                    # --- NEW: Calculate True Optical Center ---
                    cam_cx_pixel = self.camera_matrix[0, 2]
                    cam_cy_pixel = self.camera_matrix[1, 2]
                    cam_world_x, cam_world_y = self.pixel_to_world(cam_cx_pixel, cam_cy_pixel)

                    bot_poses = {}
                    crate_poses = {}
                    
                    for i, aruco_id in enumerate(ids.flatten()):
                        if aruco_id not in ARENA_LAYOUT:
                            marker_corners = refined_corners[i][0]

                            # Center in pixel coords
                            center_x_pixel = np.mean(marker_corners[:, 0])
                            center_y_pixel = np.mean(marker_corners[:, 1])
                            
                            # Convert to world coords
                            x, y = self.pixel_to_world(center_x_pixel, center_y_pixel)

                            camera_height_mm = 2590.0   # camera height from floor in mm

                            if aruco_id in (0,2,4):
                                object_height_mm = 111.25     # height of bot markers above floor
                            else:
                                object_height_mm = 51.0

                            # --- UPDATED PARALLAX CORRECTION ---
                            # Only correct if we successfully found the camera's real-world center
                            if cam_world_x is not None and cam_world_y is not None:
                                scale = object_height_mm / camera_height_mm
                                x_corrected = x - scale * (x - cam_world_x)
                                y_corrected = y - scale * (y - cam_world_y)
                                x, y = x_corrected, y_corrected

                            # --- Compute yaw using estimatePoseSingleMarkers ---
                            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                                np.array([marker_corners]),      # needs to be wrapped in a list/array
                                self.bots_marker_length,
                                self.camera_matrix,
                                self.dist_coeffs
                            )

                            rvec = rvecs[0][0]
                            tvec = tvecs[0][0]

                            rmat, _ = cv2.Rodrigues(rvec)

                            yaw_rad = math.atan2(rmat[1, 0], rmat[0, 0])
                            yaw_deg = math.degrees(yaw_rad)
                            
                            pose_data = {
                                'id': int(aruco_id),
                                'x': x,
                                'y': y,
                                'yaw': yaw_deg
                            }
                            
                            text = f"id:{pose_data['id']} x:{pose_data['x']:.2f} y:{pose_data['y']:.2f} yaw:{pose_data['yaw']:.1f}"
                            cv2.putText(
                                undistorted_image,
                                text,
                                (int(center_x_pixel) - 50, int(center_y_pixel) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.4,
                                (0, 255, 0),
                                1,
                                cv2.LINE_AA
                            )
                            if aruco_id in (0, 2, 4):
                                bot_poses[int(aruco_id)] = pose_data
                            elif aruco_id in self.allowed_crate_ids:
                                crate_poses[int(aruco_id)] = pose_data
                            else:
                                continue

                    # Step 6: Publish poses
                    self.publish_crate_poses(list(crate_poses.values()))
                    self.publish_bot_poses(list(bot_poses.values()))
            
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
