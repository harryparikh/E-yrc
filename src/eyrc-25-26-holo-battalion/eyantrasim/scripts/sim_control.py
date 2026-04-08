#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose as BotPose
import ast
import math
import time


class BotController(Node):
    def __init__(self):
        super().__init__('bot_controller')

        # --- Service client (get targets) ---
        self.cli = self.create_client(Trigger, '/eyantrasim/get_coordinates')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /eyantrasim/get_coordinates service...')
        self.future = self.cli.call_async(Trigger.Request())
        self.future.add_done_callback(self.handle_service_response)

        # --- Publishers ---
        self.pub_glacio = self.create_publisher(Twist, '/eyantrasim/glacio/cmd_vel', 10)
        self.pub_crystal = self.create_publisher(Twist, '/eyantrasim/crystal/cmd_vel', 10)
        self.pub_frostbite = self.create_publisher(Twist, '/eyantrasim/frostbite/cmd_vel', 10)

        # --- Subscribers ---
        self.sub_glacio = self.create_subscription(BotPose, '/eyantrasim/glacio/pose', self.update_glacio_pose, 10)
        self.sub_crystal = self.create_subscription(BotPose, '/eyantrasim/crystal/pose', self.update_crystal_pose, 10)
        self.sub_frostbite = self.create_subscription(BotPose, '/eyantrasim/frostbite/pose', self.update_frostbite_pose, 10)

        # --- Internal storage ---
        self.targets_glacio = []
        self.targets_crystal = []
        self.targets_frostbite = []

        self.pose_glacio = None
        self.pose_crystal = None
        self.pose_frostbite = None

        self.idx_glacio = 0
        self.idx_crystal = 0
        self.idx_frostbite = 0

        # --- PD Controller parameters ---
        self.kp_ang = 1.2
        self.kd_ang = 0.4
        self.kp_lin = 0.08
        self.kd_lin = 0.02

        self.prev_err_ang = {'glacio': 0.0, 'crystal': 0.0, 'frostbite': 0.0}
        self.prev_err_lin = {'glacio': 0.0, 'crystal': 0.0, 'frostbite': 0.0}
        self.prev_time = time.time()

        # --- Control loop ---
        self.create_timer(0.1, self.control_loop)

    # ---------------- Pose Callbacks ----------------
    def update_glacio_pose(self, msg): self.pose_glacio = msg
    def update_crystal_pose(self, msg): self.pose_crystal = msg
    def update_frostbite_pose(self, msg): self.pose_frostbite = msg

    # ---------------- Handle Service Response ----------------
    def handle_service_response(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().error("Failed to get coordinates")
                return

            coords_list = ast.literal_eval(res.message)
            coords = []
            for (x, y) in coords_list:
                pose = BotPose()
                pose.x = float(x)
                pose.y = float(y)
                coords.append(pose)

            # Divide into 3 groups of 50
            self.targets_glacio = coords[0:50]
            self.targets_crystal = coords[50:100]
            self.targets_frostbite = coords[100:150]

            self.get_logger().info("Targets received successfully.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    # ---------------- PD-Controlled Motion ----------------
    def move_bot(self, bot_name, pub, pose, targets, idx):
        if pose is None or idx >= len(targets):
            return idx

        target = targets[idx]
        dx, dy = target.x - pose.x, target.y - pose.y
        dist = math.hypot(dx, dy)
        desired_theta = math.atan2(dy, dx)
        error_theta = (desired_theta - pose.theta + math.pi) % (2 * math.pi) - math.pi

        # --- Time step ---
        curr_time = time.time()
        dt = curr_time - self.prev_time
        if dt <= 0:
            dt = 0.1
        self.prev_time = curr_time

        # --- PD for angle ---
        prev_e_ang = self.prev_err_ang[bot_name.lower()]
        d_ang = (error_theta - prev_e_ang) / dt
        ang_vel = self.kp_ang * error_theta + self.kd_ang * d_ang

        # --- PD for distance ---
        prev_e_lin = self.prev_err_lin[bot_name.lower()]
        d_lin = (dist - prev_e_lin) / dt
        lin_vel = self.kp_lin * dist + self.kd_lin * d_lin

        # --- Store previous errors ---
        self.prev_err_ang[bot_name.lower()] = error_theta
        self.prev_err_lin[bot_name.lower()] = dist

        twist = Twist()
        angle_tolerance = 0.1     # radians
        distance_tolerance = 5.0  # units (adjust per scale)

        if dist > distance_tolerance:
            if abs(error_theta) > angle_tolerance:
                twist.linear.x = 0.0
                twist.angular.z = max(-1.0, min(1.0, ang_vel))
            else:
                twist.linear.x = max(0.0, min(2.0, lin_vel))
                twist.angular.z = max(-1.0, min(1.0, ang_vel * 0.5))
        else:
            self.get_logger().info(f"{bot_name} reached target {idx+1}/{len(targets)}")
            idx += 1

        pub.publish(twist)
        return idx

    # ---------------- Control Loop ----------------
    def control_loop(self):
        self.idx_glacio = self.move_bot("Glacio", self.pub_glacio, self.pose_glacio, self.targets_glacio, self.idx_glacio)
        self.idx_crystal = self.move_bot("Crystal", self.pub_crystal, self.pose_crystal, self.targets_crystal, self.idx_crystal)
        self.idx_frostbite = self.move_bot("Frostbite", self.pub_frostbite, self.pose_frostbite, self.targets_frostbite, self.idx_frostbite)


def main(args=None):
    rclpy.init(args=args)
    node = BotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
