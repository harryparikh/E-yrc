#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Poses2D, BotCmd, BotCmdArray
import numpy as np
import math
import json
import paho.mqtt.client as mqtt

# ---------------------- PID Class -------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        # Integral Windup Clamp
        if abs(self.integral * self.ki) > self.max_out:
            self.integral = 0.0 
            
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return np.clip(out, -self.max_out, self.max_out)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

# ---------------------- Helper ----------------------------------------------
def normalize_angle(angle):
    """Wraps angle to [-pi, pi] for efficient turning."""
    return math.atan2(math.sin(angle), math.cos(angle))

# ---------------------- Main Node -------------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')

        # ---------------- Robot Params ----------------
        self.robot_id = 0  # <--- CHECK THIS ID
        self.current_pose = None
        self.goal_index = 0
        self.last_time = self.get_clock().now()

        # Tolerances
        self.xy_tolerance = 15.0     # Distance in pixels/units
        self.theta_tolerance_deg = 10.0
        self.max_vel = 50.0

        # ---------------- Goals (x, y, theta) ----------------
        self.goals = [
            (1219, 204, 0),
            # (800, 800, 0),
            # (1620, 1520, 0),
        ]

        # ---------------- PID Init ----------------
        # X and Y PIDs enabled
        # self.pid_x = PID(0.97, 0.007, 0, self.max_vel) 
        # self.pid_y = PID(1, 0.008, 0.01, self.max_vel)
        
        # # Theta PID
        # self.pid_theta = PID(40, 0.002, 0.1, self.max_vel * 2)

        self.pid_x = PID(1.3, 0.004, 0.01, self.max_vel)
        self.pid_y = PID(1.9, 0.001, 0.07, self.max_vel)
        self.pid_theta = PID(45, 0.006, 0.1, self.max_vel * 2)

        # ---------------- MQTT Setup ----------------
        self.mqtt_topic = "bot0/cmd"
        self.mqtt_client = mqtt.Client()
        self.mqtt_broker_ip = "10.95.243.6" 

        try:
            self.mqtt_client.connect(self.mqtt_broker_ip, 1883, 60)
            self.get_logger().info(f"✅ MQTT Connected to {self.mqtt_broker_ip}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT CONNECTION FAILED: {e}")

        self.mqtt_client.loop_start()

        # ---------------- ROS Setup ----------------
        self.create_subscription(Poses2D, 'bot_pose', self.pose_cb, 10)
        self.publisher = self.create_publisher(BotCmdArray, '/bot_control', 10)
        self.timer = self.create_timer(0.05, self.control_cb)

        self.get_logger().info("--- Full Holonomic PID Started ---")

    def pose_cb(self, msg):
        for p in msg.poses:
            if p.id == self.robot_id:
                self.current_pose = np.array([p.x, p.y, p.w])
                break

    def control_cb(self):
        # 1. Safety Check
        if self.current_pose is None:
            self.get_logger().warn_once(f"WAITING: No pose for Robot {self.robot_id}...")
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now

        # 2. Check if all goals done
        if self.goal_index >= len(self.goals):
            self.publish_cmd(0, 0, 0)
            return

        # 3. Calculate Errors
        x, y, theta_deg = self.current_pose
        theta = math.radians(theta_deg)

        gx, gy, gtheta_deg = self.goals[self.goal_index]
        gtheta = math.radians(gtheta_deg)

        ex = gx - x
        ey = gy - y
        etheta = normalize_angle(gtheta - theta)

        # 4. Compute PID Outputs (RESTORED X and Y)
        vx_world = self.pid_x.compute(ex, dt)
        vy_world = self.pid_y.compute(ey, dt)
        omega = self.pid_theta.compute(etheta, dt)

        # 5. Transform World Velocity to Robot Frame
        # Because it's holonomic, motors need commands relative to the robot's heading
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        
        vx_robot = vx_world * cos_t + vy_world * sin_t
        vy_robot = -vx_world * sin_t + vy_world * cos_t

        # 6. Inverse Kinematics (Omni Wheel Mixing)
        alphas = [math.radians(30), math.radians(150), math.radians(270)]
        M = np.array([
            [math.cos(a + math.pi/2) for a in alphas],
            [math.sin(a + math.pi/2) for a in alphas],
            [1, 1, 1]
        ])

        wheels = np.linalg.pinv(M) @ np.array([vx_robot, vy_robot, omega])
        self.publish_cmd(wheels[0], wheels[1], wheels[2])

        # 7. Check Completion (Using Distance + Angle)
        dist_error = math.hypot(ex+ey)
        angle_error_deg = abs(math.degrees(etheta))

        if dist_error < self.xy_tolerance and angle_error_deg < self.theta_tolerance_deg :
            self.get_logger().info(f"Goal {self.goal_index} Reached! (Dist: {dist_error:.1f}, Ang: {angle_error_deg:.1f})")
            self.goal_index += 1
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()

    def publish_cmd(self, m1, m2, m3):
        def map_vel(v):
            min_pwm = 10 
            if abs(v) < 0.5: return 90
            
            pwm_offset = (v / 50.0) * 90.0
            if v > 0: base = 90 + min_pwm
            else: base = 90 - min_pwm
                
            return int(max(min(base + pwm_offset, 180), 0))

        payload = {
            "m1": map_vel(m2), 
            "m2": map_vel(m1),
            "m3": map_vel(m3),
            "base": 45, "elbow": 135, "mag": 0
        }

        try:
            self.mqtt_client.publish(self.mqtt_topic, json.dumps(payload))
        except Exception:
            pass

        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = self.robot_id
        cmd.m1 = float(m1)
        cmd.m2 = float(m2)
        cmd.m3 = float(m3)
        msg.cmds.append(cmd)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HolonomicPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()