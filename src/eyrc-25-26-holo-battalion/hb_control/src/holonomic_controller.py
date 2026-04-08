#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Poses2D, BotCmd, BotCmdArray
from linkattacher_msgs.srv import AttachLink, DetachLink
import numpy as np
import math
import time

# ---------------------- PID Controller Class --------------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return np.clip(output, -self.max_out, self.max_out)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicMoveToCrate(Node):
    def __init__(self):
        super().__init__('holonomic_move_to_crate')

        # ---------------- Robot Parameters ----------------
        self.robot_id = 0
        self.current_pose = None
        self.crate_pose = None
        self.last_time = self.get_clock().now()
        self.xy_tolerance = 5.0
        self.theta_tolerance_deg = 3.0
        self.theta_tolerance_center_deg = 15.0
        self.max_vel = 50.0
        self.goal_reached = False
        self.arm_placed = False
        self.box_attached = False
        self.arm_lifted = False
        self.tracked_crate_id = None
        self.move_after_attach = False
        self.final_goal = np.array([1219.0, 1180.0, 0.0])
        self.return_to_start = False
        self.return_goal = np.array([1218.0, 205.0, 0.0])

        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 4, 'ki': 0.001, 'kd': 0.001, 'max_out': self.max_vel},
            'y': {'kp': 5, 'ki': 0.001, 'kd': 0.0001, 'max_out': self.max_vel},
            'theta': {'kp': -9, 'ki': 0.001, 'kd': 0.0002, 'max_out': self.max_vel * 2}
        }

        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS Setup ----------------
        self.create_subscription(Poses2D, 'bot_pose', self.pose_cb, 10)
        self.create_subscription(Poses2D, 'crate_pose', self.crate_cb, 10)
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        self.timer = self.create_timer(0.03, self.control_cb)

        # Service client for attaching crate
        self.attach_client = self.create_client(AttachLink, '/attach_link')

        self.get_logger().info("Move-to-Crate controller initialized.")

        # Service client for detaching the box

        self.detach_client = self.create_client(DetachLink, '/detach_link')
        self.box_detached = False


    # ---------------- Callbacks ----------------
    def pose_cb(self, msg):
        for pose in msg.poses:
            if pose.id == self.robot_id:
                self.current_pose = np.array([pose.x, pose.y, pose.w])
                break

    def crate_cb(self, msg):
        if msg.poses:
            self.crate_pose = np.array([msg.poses[0].x, msg.poses[0].y, msg.poses[0].w])
            self.tracked_crate_id=msg.poses[0].id

            self.get_logger().info(f"position (x={self.crate_pose[0]:.1f}, y={self.crate_pose[1]:.1f}, θ={self.crate_pose[2]:.1f})")
            

    # ---------------- Control Loop ----------------
    def control_cb(self):
        if self.current_pose is None or self.crate_pose is None:
            return

        # --- Step 1: Move near the crate ---
        if not self.goal_reached:
            goal_x = self.crate_pose[0]
            goal_y = self.crate_pose[1] - 160.0
            goal_theta_deg = 0.0

            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = now

            x, y, theta_deg = self.current_pose
            theta_rad = math.radians(theta_deg)
            goal_theta_rad = math.radians(goal_theta_deg)

            error_x = goal_x - x
            error_y = goal_y - y
            error_theta = goal_theta_rad - theta_rad

            vx_world = self.pid_x.compute(error_x, dt)
            vy_world = self.pid_y.compute(error_y, dt)
            omega = self.pid_theta.compute(error_theta, dt)

            cos_t = math.cos(theta_rad)
            sin_t = math.sin(theta_rad)
            vx_robot = vx_world * cos_t + vy_world * sin_t
            vy_robot = -vx_world * sin_t + vy_world * cos_t

            alphas = [math.radians(30), math.radians(150), math.radians(270)]
            M = np.array([
                [math.cos(a + math.pi/2) for a in alphas],
                [math.sin(a + math.pi/2) for a in alphas],
                [1.0, 1.0, 1.0]
            ])
            v = np.array([vx_robot, vy_robot, omega])
            wheel_vel = np.linalg.pinv(M) @ v

            self.publish_wheel_velocities(wheel_vel.tolist(), base=0.0, elbow=0.0)

            dist = math.hypot(error_x, error_y)
            if dist < self.xy_tolerance:
                self.publish_wheel_velocities([0.0, 0.0, 0.0])
                self.goal_reached = True
                self.get_logger().info(f"Reached near crate at ({goal_x:.1f}, {goal_y:.1f})")
                self.get_logger().info("Preparing to place arm on the crate...")

        # --- Step 2: Place arm onto crate ---
        elif not self.arm_placed:
            base_angle = 90.0
            elbow_angle = 90.0

            self.publish_wheel_velocities([0.0, 0.0, 0.0], base=base_angle, elbow=elbow_angle)
            self.arm_placed = True
            time.sleep(10)
            self.get_logger().info(f"Arm placed on crate (base={base_angle}°, elbow={elbow_angle}°)")
            self.get_logger().info("Calling /attach_link service...")

        # --- Step 3: Attach the crate ---
        elif self.arm_placed and not self.box_attached:
            if not self.attach_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("/attach_link service not available.")
                return
            if self.tracked_crate_id%3==0:
                model2_name = f"crate_red_{self.tracked_crate_id}"
            elif self.tracked_crate_id%3==1:
                model2_name=f"crate_green_{self.tracked_crate_id}"
            elif self.tracked_crate_id%3==2:
                model2_name=f"crate_blue_{self.tracked_crate_id}"

            link2_name = f"box_link_{self.tracked_crate_id}"


            req = AttachLink.Request()
            req.data = f"""{{
                "model1_name": "hb_crystal",
                "link1_name": "arm_link_2",
                "model2_name": "{model2_name}",
                "link2_name": "{link2_name}"
            }}"""


            future = self.attach_client.call_async(req)
            future.add_done_callback(self.attach_done_callback)
            self.box_attached = True  # Prevent multiple calls

        # --- Step 4: Lift the arm after attaching ---
        elif self.box_attached and not self.arm_lifted:
            base_angle = 45.0
            elbow_angle = 45.0
            self.publish_wheel_velocities([0.0, 0.0, 0.0], base=base_angle, elbow=elbow_angle)
            self.arm_lifted = True
            self.move_after_attach = True
            self.get_logger().info("Arm lifted with crate attached!")

        # --- Step 5: Move to final goal after attaching the crate ---
        elif self.move_after_attach:
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = now

            x, y, theta_deg = self.current_pose
            theta_rad = math.radians(theta_deg)
            goal_x, goal_y, goal_theta_deg = self.final_goal
            goal_theta_rad = math.radians(goal_theta_deg)

            error_x = goal_x - x
            error_y = goal_y - y
            error_theta = goal_theta_rad - theta_rad

            vx_world = self.pid_x.compute(error_x, dt)
            vy_world = self.pid_y.compute(error_y, dt)
            omega = self.pid_theta.compute(error_theta, dt)

            cos_t = math.cos(theta_rad)
            sin_t = math.sin(theta_rad)
            vx_robot = vx_world * cos_t + vy_world * sin_t
            vy_robot = -vx_world * sin_t + vy_world * cos_t

            alphas = [math.radians(30), math.radians(150), math.radians(270)]
            M = np.array([
                [math.cos(a + math.pi/2) for a in alphas],
                [math.sin(a + math.pi/2) for a in alphas],
                [1.0, 1.0, 1.0]
            ])
            v = np.array([vx_robot, vy_robot, omega])
            wheel_vel = np.linalg.pinv(M) @ v

            self.publish_wheel_velocities(wheel_vel.tolist(), base=45.0, elbow=45.0)

            dist = math.hypot(error_x, error_y)
            if dist < (self.xy_tolerance+3) and abs(error_theta) < math.radians(self.theta_tolerance_center_deg):
                self.publish_wheel_velocities([0.0, 0.0, 0.0], base=45.0, elbow=45.0)
                self.move_after_attach = False
                self.get_logger().info(f"Reached final goal at ({goal_x:.1f}, {goal_y:.1f})")

                # Step 6: Place arm down before detaching crate
                if not self.box_detached:
                    base_angle = 85.0
                    elbow_angle = 85.0
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], base=base_angle, elbow=elbow_angle)
                    self.get_logger().info(f"Arm positioned for drop (base={base_angle}°, elbow={elbow_angle}°)")
                    time.sleep(2)  # Give time for the arm to move before detaching

                    # Detach the crate
                    if not self.detach_client.wait_for_service(timeout_sec=2.0):
                        self.get_logger().warn("/detach_link service not available.")
                        return

                    if self.tracked_crate_id % 3 == 0:
                        model2_name = f"crate_red_{self.tracked_crate_id}"
                    elif self.tracked_crate_id % 3 == 1:
                        model2_name = f"crate_green_{self.tracked_crate_id}"
                    elif self.tracked_crate_id % 3 == 2:
                        model2_name = f"crate_blue_{self.tracked_crate_id}"

                    link2_name = f"box_link_{self.tracked_crate_id}"

                    req = DetachLink.Request()
                    req.data = f"""{{
                        "model1_name": "hb_crystal",
                        "link1_name": "arm_link_2",
                        "model2_name": "{model2_name}",
                        "link2_name": "{link2_name}"
                    }}"""

                    future = self.detach_client.call_async(req)
                    future.add_done_callback(self.detach_done_callback)
                    self.box_detached = True
                    self.get_logger().info("Detaching crate after lowering arm...")
                    time.sleep(1)

                    lift_base = 45.0
                    lift_elbow = 45.0
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], base=lift_base, elbow=lift_elbow)
                    self.get_logger().info(f"Arm lifted back up (base={lift_base}°, elbow={lift_elbow}°)")
                    self.return_to_start = True
                    self.get_logger().info("Starting return to initial docking zone (1218, 205, 0)")
                # --- Step 7: Return to initial position after detaching ---
        elif self.return_to_start:
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            self.last_time = now

            x, y, theta_deg = self.current_pose
            theta_rad = math.radians(theta_deg)
            goal_x, goal_y, goal_theta_deg = self.return_goal
            goal_theta_rad = math.radians(goal_theta_deg)

            error_x = goal_x - x
            error_y = goal_y - y
            error_theta = goal_theta_rad - theta_rad

            vx_world = self.pid_x.compute(error_x, dt)
            vy_world = self.pid_y.compute(error_y, dt)
            omega = self.pid_theta.compute(error_theta, dt)

            cos_t = math.cos(theta_rad)
            sin_t = math.sin(theta_rad)
            vx_robot = vx_world * cos_t + vy_world * sin_t
            vy_robot = -vx_world * sin_t + vy_world * cos_t

            alphas = [math.radians(30), math.radians(150), math.radians(270)]
            M = np.array([
                [math.cos(a + math.pi/2) for a in alphas],
                [math.sin(a + math.pi/2) for a in alphas],
                [1.0, 1.0, 1.0]
            ])
            v = np.array([vx_robot, vy_robot, omega])
            wheel_vel = np.linalg.pinv(M) @ v

            self.publish_wheel_velocities(wheel_vel.tolist(), base=45.0, elbow=45.0)

            dist = math.hypot(error_x, error_y)
            if dist < (self.xy_tolerance + 3) and abs(error_theta) < math.radians(self.theta_tolerance_center_deg):
                self.publish_wheel_velocities([0.0, 0.0, 0.0], base=45.0, elbow=45.0)
                self.return_to_start = False
                self.get_logger().info(f"Returned to starting position ({goal_x:.1f}, {goal_y:.1f}, {goal_theta_deg:.1f})")






    # ---------------- Attach Callback ----------------
    def attach_done_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Box successfully attached to arm.")
            else:
                self.get_logger().warn("Failed to attach box.")
        except Exception as e:
            self.get_logger().error(f"Attach service call failed: {e}")


    def detach_done_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Crate successfully detached at center.")
            else:
                self.get_logger().warn("Failed to detach crate.")
        except Exception as e:
            self.get_logger().error(f"Detach service call failed: {e}")


    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel, base=0.0, elbow=0.0):
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = self.robot_id
        cmd.m1, cmd.m2, cmd.m3 = wheel_vel
        cmd.base = base
        cmd.elbow = elbow
        msg.cmds.append(cmd)
        self.publisher.publish(msg)


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HolonomicMoveToCrate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
