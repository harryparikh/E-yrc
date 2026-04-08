#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Poses2D, BotCmd, BotCmdArray
import numpy as np
import math
import time
import json
import paho.mqtt.client as mqtt
from std_srvs.srv import SetBool
from std_msgs.msg import Bool



# ---------------------- PID Controller --------------------------------------
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
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return np.clip(out, -self.max_out, self.max_out)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node -------------------------------------------
class HolonomicMoveToCrate(Node):
    def __init__(self):
        super().__init__('holonomic_move_to_crate')

        self.robot_id = 0
        self.current_pose = None
        self.crate_pose = None
        self.last_time = self.get_clock().now()

        self.xy_tolerance = 16.0
        self.max_vel = 50.0

        self.final_goal = np.array([1219.0, 1180.0, 0.0])
        self.return_goal = np.array([1218.0, 204.0, 0.0])

        # ---------------- State Flags ----------------
        self.goal_reached = False
        self.arm_placed = False
        self.magnet_in_hold = False
        self.arm_lifted = False
        self.move_after_attach = False
        self.arm_at_drop_pose = False
        self.box_detached = False
        self.drop_wait_done = False
        self.return_to_start = False
        self.return_reached = False

        # ---------------- Distance Logging ----------------
        self.last_dist_log_time = time.time()
        self.dist_log_interval = 0.5  # seconds

        # ---------------- Magnet ----------------
        self.mag_value = 0
        self.mag_timer_start = None
        self.drop_wait_start = None

        # ---------------- Arm motion ----------------
        self.base_angle = 45
        self.elbow_angle = 135
        self.arm_target_base = 45
        self.arm_target_elbow = 135
        self.arm_step = 1

        # ---------------- PID ----------------
        self.pid_x = PID(2.2, 0.007, 0, self.max_vel)
        self.pid_y = PID(2.5, 0.008, 0.01, self.max_vel)
        self.pid_theta = PID(40, 0.002, 0.1, self.max_vel * 2)

        # ---------------- MQTT ----------------
        self.mqtt_topic = "bot0/cmd"
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("10.151.99.6", 1883, 60)
        self.mqtt_client.loop_start()

        # ---------------- ROS ----------------
        self.create_subscription(Poses2D, 'bot_pose', self.pose_cb, 10)
        self.create_subscription(Poses2D, 'crate_pose', self.crate_cb, 10)
        self.publisher = self.create_publisher(BotCmdArray, '/bot_control', 10)
        self.timer = self.create_timer(0.03, self.control_cb)
        self.attach_pub = self.create_publisher(Bool, '/attach_state', 10)


        self.get_logger().info("Holonomic controller started")

 # ---------------- Attach Service ----------------
        self.attach_active = False

        self.attach_srv = self.create_service(
            SetBool,
            '/attach',
            self.attach_service_cb
        )



    # ---------------- Callbacks ----------------
    def pose_cb(self, msg):
        for p in msg.poses:
            if p.id == self.robot_id:
                self.current_pose = np.array([p.x, p.y, p.w])

    def crate_cb(self, msg):
        if msg.poses:
            p = msg.poses[0]
            self.crate_pose = np.array([p.x, p.y, p.w])

    # ---------------- Distance Logger ----------------
    def log_distance(self, gx, gy, label):
        now = time.time()
        if now - self.last_dist_log_time >= self.dist_log_interval:
            x, y, _ = self.current_pose
            dist = math.hypot(gx - x, gy - y)
            self.get_logger().info(
                f"Distance to {label}: {dist:.2f} | Goal: ({gx:.1f}, {gy:.1f})"
            )
            self.last_dist_log_time = now

    # ---------------- Smooth Arm Update ----------------
    def update_arm_smooth(self):
        moved = False

        if self.base_angle < self.arm_target_base:
            self.base_angle += self.arm_step
            moved = True
        elif self.base_angle > self.arm_target_base:
            self.base_angle -= self.arm_step
            moved = True

        if self.elbow_angle < self.arm_target_elbow:
            self.elbow_angle += self.arm_step
            moved = True
        elif self.elbow_angle > self.arm_target_elbow:
            self.elbow_angle -= self.arm_step
            moved = True

        return not moved

    # ---------------- Control Loop ----------------
    def control_cb(self):

        # ----- Sync attach boolean with magnet -----
        if self.mag_value > 0:
            self.attach_active = True
        else:
            self.attach_active = False

        attach_msg = Bool()
        attach_msg.data = self.attach_active
        self.attach_pub.publish(attach_msg)

        

        if self.current_pose is None or self.crate_pose is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0:
            return

        x, y, _ = self.current_pose

        # ----- Magnet PULL → HOLD -----
        if self.mag_value == 10 and self.mag_timer_start:
            if time.time() - self.mag_timer_start >= 5.0:
                self.mag_value = 9
                self.mag_timer_start = None
                self.magnet_in_hold = True
                self.get_logger().info("Magnet switched to HOLD mode")

        # ----- Step 1: Move near crate -----
        if not self.goal_reached:
            gx = self.crate_pose[0] - 30
            gy = self.crate_pose[1] - 120
            self.drive_to_target(gx, gy, 0, dt)
            self.log_distance(gx, gy, "crate approach point")

            if self.is_at_target(gx, gy):
                self.goal_reached = True
                self.get_logger().info(
                    f"Reached near crate at (x={gx:.1f}, y={gy:.1f})"
                )
                self.get_logger().info("Preparing to place arm on the crate...")

        # ----- Step 2: Arm DOWN + Magnet ON -----
        elif not self.arm_placed:
            if self.mag_value == 0:
                self.mag_value = 10
                self.mag_timer_start = time.time()

            self.arm_target_base = 15
            self.arm_target_elbow = 180
            arm_done = self.update_arm_smooth()
            self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)

            if arm_done:
                self.arm_placed = True
                self.get_logger().info(
                    f"Arm placed on crate (base={self.base_angle}°, elbow={self.elbow_angle}°)"
                )
                self.get_logger().info("Magnet pull-in started")

        # ----- Step 3: WAIT for magnet HOLD -----
        elif not self.magnet_in_hold:
            self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)

        # ----- Step 4: Lift arm -----
        elif not self.arm_lifted:
            self.arm_target_base = 45
            self.arm_target_elbow = 135
            arm_done = self.update_arm_smooth()
            self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)

            if arm_done:
                self.arm_lifted = True
                self.move_after_attach = True
                self.get_logger().info("Arm lifted with crate attached!")

        # ----- Step 5: Move to drop zone -----
        elif self.move_after_attach:
            gx, gy = self.final_goal[0], self.final_goal[1]
            self.drive_to_target(*self.final_goal, dt, self.base_angle, self.elbow_angle)
            self.log_distance(gx, gy, "drop zone")

            if math.hypot(gx - x, gy - y) < self.xy_tolerance:
                self.move_after_attach = False

        # ----- Step 6A: Arm to DROP pose -----
        elif not self.arm_at_drop_pose:
            self.arm_target_base = 15
            self.arm_target_elbow = 180
            arm_done = self.update_arm_smooth()
            self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)

            if arm_done:
                self.arm_at_drop_pose = True
                self.get_logger().info(
                    f"Arm positioned for drop (base={self.base_angle}°, elbow={self.elbow_angle}°)"
                )

        # ----- Step 6B: DROP crate -----
        elif not self.box_detached:
            self.mag_value = 0
            self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)
            self.box_detached = True
            self.drop_wait_start = time.time()
            self.get_logger().info("Crate released (magnet OFF)")

        # ----- Step 7: WAIT after drop -----
        elif not self.drop_wait_done:
            self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)
            if time.time() - self.drop_wait_start >= 5.0:
                self.drop_wait_done = True
                self.return_to_start = True

        # ----- Step 8: Return -----
        # elif self.return_to_start:
        #     gx, gy = self.return_goal[0], self.return_goal[1]
        #     self.arm_target_base = 45
        #     self.arm_target_elbow = 135
        #     self.update_arm_smooth()
        #     self.drive_to_target(*self.return_goal, dt, self.base_angle, self.elbow_angle)
        #     self.log_distance(gx, gy, "start position")

        # ----- Step 8: Return -----
        elif self.return_to_start and not self.return_reached:
            gx, gy = self.return_goal[0], self.return_goal[1]

            self.arm_target_base = 45
            self.arm_target_elbow = 135
            self.update_arm_smooth()

            self.drive_to_target(*self.return_goal, dt, self.base_angle, self.elbow_angle)
            self.log_distance(gx, gy, "start position")

            if self.is_at_target(gx, gy):
                self.return_reached = True
                self.return_to_start = False

                # RESET PIDs to avoid residual motion
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()

                # HARD STOP
                self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)

                self.get_logger().info("Returned home. Robot stopped.")

        # ----- FINAL STOP STATE -----
        elif self.return_reached:
            self.publish_cmd(0, 0, 0, self.base_angle, self.elbow_angle)
            return



    # ---------------- Motion ----------------
    # def drive_to_target(self, gx, gy, gtheta, dt, base=45, elbow=135):
    #     x, y, theta_deg = self.current_pose
    #     theta = math.radians(theta_deg)

    #     vx = self.pid_x.compute(gx - x, dt)
    #     vy = self.pid_y.compute(gy - y, dt)
    #     omega = self.pid_theta.compute(math.radians(gtheta) - theta, dt)

    #     cos_t, sin_t = math.cos(theta), math.sin(theta)
    #     vx_r = vx * cos_t + vy * sin_t
    #     vy_r = -vx * sin_t + vy * cos_t

    #     alphas = [math.radians(30), math.radians(150), math.radians(270)]
    #     M = np.array([
    #         [math.cos(a + math.pi/2) for a in alphas],
    #         [math.sin(a + math.pi/2) for a in alphas],
    #         [1, 1, 1]
    #     ])

    #     wheels = np.linalg.pinv(M) @ np.array([vx_r, vy_r, omega])
    #     self.publish_cmd(*wheels, base, elbow)

    def drive_to_target(self, gx, gy, gtheta, dt, base=45, elbow=135):
        x, y, theta_deg = self.current_pose
        theta = math.radians(theta_deg)

        # --- Distance to goal ---
        dist = math.hypot(gx - x, gy - y)

        # --- PID outputs ---
        vx = self.pid_x.compute(gx - x, dt)
        vy = self.pid_y.compute(gy - y, dt)
        omega = self.pid_theta.compute(math.radians(gtheta) - theta, dt)

        # --- Speed reduction near goal ---
        if dist < 100.0:
            speed_scale = 0.5   # 20% reduction
            vx *= speed_scale
            vy *= speed_scale
            omega *= speed_scale

        # --- World → Robot frame ---
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx * cos_t + vy * sin_t
        vy_r = -vx * sin_t + vy * cos_t

        # --- Holonomic wheel mapping ---
        alphas = [math.radians(30), math.radians(150), math.radians(270)]
        M = np.array([
            [math.cos(a + math.pi/2) for a in alphas],
            [math.sin(a + math.pi/2) for a in alphas],
            [1, 1, 1]
        ])

        wheels = np.linalg.pinv(M) @ np.array([vx_r, vy_r, omega])
        self.publish_cmd(*wheels, base, elbow)


    def is_at_target(self, gx, gy):
        x, y, _ = self.current_pose
        return math.hypot(gx - x, gy - y) < self.xy_tolerance

    # ---------------- Command Publish ----------------
    def publish_cmd(self, m1, m2, m3, base, elbow):
        def map_vel(v):
            v = max(min(v, 50), -50)
            return int(90 + (v / 50) * 90)

        payload = {
            "m1": map_vel(m2),
            "m2": map_vel(m1),
            "m3": map_vel(m3),
            "base": int(base),
            "elbow": int(elbow),
            "mag": int(self.mag_value)
        }

        self.mqtt_client.publish(self.mqtt_topic, json.dumps(payload))

        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = self.robot_id
        cmd.m1, cmd.m2, cmd.m3 = float(m1), float(m2), float(m3)
        cmd.base = float(base)
        cmd.elbow = float(elbow)
        msg.cmds.append(cmd)
        self.publisher.publish(msg)

    def attach_service_cb(self, request, response):
        """
        External attach/detach request.
        True  -> attach
        False -> detach
        """
        self.attach_active = request.data

        # Sync magnet with attach state
        if self.attach_active:
            if self.mag_value == 0:
                self.mag_value = 10
                self.mag_timer_start = time.time()
            response.message = "Attach enabled (magnet ON)"
        else:
            self.mag_value = 0
            response.message = "Attach disabled (magnet OFF)"

        response.success = True
        return response



def main(args=None):
    rclpy.init(args=args)
    node = HolonomicMoveToCrate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
