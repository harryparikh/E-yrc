#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hb_interfaces.msg import Poses2D, BotCmd, BotCmdArray
import numpy as np
import math
import time
import json
import paho.mqtt.client as mqtt
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

# ---------------------- PID Controller Class --------------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt, error_threshold=None):
        if dt <= 0: return 0.0
        
        if error_threshold is not None and abs(error) < error_threshold:
            self.integral = 0.0
        
        self.integral += error * dt

        if self.ki != 0:
            limit = self.max_out / self.ki
            self.integral = float(np.clip(self.integral, -limit, limit))
        else:
            self.integral = 0.0

        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        return float(np.clip(output, -self.max_out, self.max_out))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Per-bot State ---------------------------------------
class BotState:
    def __init__(self, bot_id, pid_params):
        self.id = bot_id
        self.pose = None
        self.step_index = 0
        self.state_start_time = 0
        self.waiting = False
        self.velocity = 0.0
        self.last_time = None

        self.pid_x = PID(**pid_params['x'])
        self.pid_y = PID(**pid_params['y'])
        self.pid_theta = PID(**pid_params['theta'])

    def update_pid_params(self, axis, kp, ki, kd):
        if axis == 'x':
            self.pid_x.kp, self.pid_x.ki, self.pid_x.kd = kp, ki, kd
            self.pid_x.reset()
        elif axis == 'y':
            self.pid_y.kp, self.pid_y.ki, self.pid_y.kd = kp, ki, kd
            self.pid_y.reset()
        elif axis == 'theta':
            self.pid_theta.kp, self.pid_theta.ki, self.pid_theta.kd = kp, ki, kd
            self.pid_theta.reset()

# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDTuner(Node):

    def __init__(self):
        super().__init__('holonomic_pid_tuner') 

        self.bot_ids = [0, 2, 4] 
        self.max_vel = 50.0  

        # ---------------- HARDCODED ZONES (From your other code) ----------------
        # Using the "Gate" coordinates as the target points for movement
        # Sequence: P1 -> P2 -> P4 -> P3 (Loop)
        self.waypoints = [
            (285.0, 480.0, 0.0),   # P1 Gate (Bottom Left)
            (2163.0, 480.0, 0.0),  # P2 Gate (Bottom Right)
            (2163.0, 1270.0, 0.0), # P4 Gate (Top Right)
            (285.0, 1270.0, 0.0)   # P3 Gate (Top Left)
        ]

        self.xy_tolerance = 15.0   
        self.theta_tolerance = 3.0 
        self.wait_time = 2.0       

        alphas = [math.radians(30), math.radians(150), math.radians(270)]
        M = np.array([
            [math.cos(a + math.pi/2) for a in alphas],
            [math.sin(a + math.pi/2) for a in alphas],
            [1, 1, 1]
        ])
        self.M_inv = np.linalg.pinv(M)

        self.pid_params_by_bot = {
            0: {'x': {'kp': 1.8, 'ki': 0.00001, 'kd': 0.01, 'max_out': self.max_vel},
                'y': {'kp': 1.9, 'ki': 0.0, 'kd': 0.01, 'max_out': self.max_vel},
                'theta': {'kp': 55, 'ki': 0.006, 'kd': 0.1, 'max_out': self.max_vel * 2}},
            2: {'x': {'kp': 1.7, 'ki': 0.0002, 'kd': 0.02, 'max_out': self.max_vel},
                'y': {'kp': 1.9, 'ki': 0.00001, 'kd': 0.03, 'max_out': self.max_vel},
                'theta': {'kp': 60, 'ki': 0.01, 'kd': 0.1, 'max_out': self.max_vel * 2}},
            4: {'x': {'kp': 1.45, 'ki': 0.0002, 'kd': 0.04, 'max_out': self.max_vel},
                'y': {'kp': 2.1, 'ki': 0.000006, 'kd': 0.05, 'max_out': self.max_vel},
                'theta': {'kp': 80, 'ki': 0.000, 'kd': 0.02, 'max_out': self.max_vel * 2}},
        }

        self.bots = {bid: BotState(bid, self.pid_params_by_bot[bid]) for bid in self.bot_ids}

        for bid in self.bot_ids:
            for axis in ['x', 'y', 'theta']:
                for p_type, val in self.pid_params_by_bot[bid][axis].items():
                    if p_type == 'max_out': continue
                    p_name = f'bot{bid}_{axis}_{p_type}'
                    self.declare_parameter(p_name, val)

        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.graph_pub = self.create_publisher(String, '/pid_graph_data', 10)
        self.cmd_pub = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        self.create_subscription(Poses2D, 'bot_pose', self.pose_cb, 10)
        
        self.timer = self.create_timer(0.03, self.control_cb)

        self.mqtt_client = mqtt.Client()
        try:
            self.mqtt_client.connect("10.198.166.6", 1883, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("MQTT Connected")
        except Exception as e:
            self.get_logger().error(f"MQTT Failed: {e}")

    def pose_cb(self, msg: Poses2D):
        now = self.get_clock().now()
        for pose in msg.poses:
            if pose.id in self.bots:
                bot = self.bots[pose.id]
                bot.pose = np.array([pose.x, pose.y, pose.w])
                bot.last_time = now

    def parameter_callback(self, params):
        for param in params:
            name = param.name
            try:
                parts = name.split('_') 
                if len(parts) != 3: continue
                
                bid = int(parts[0].replace('bot',''))
                axis = parts[1]
                p_type = parts[2]
                
                if bid in self.bots:
                    bot = self.bots[bid]
                    current_pid = getattr(bot, f'pid_{axis}')
                    
                    kp, ki, kd = current_pid.kp, current_pid.ki, current_pid.kd
                    
                    val = float(param.value)

                    if p_type == 'kp': kp = val
                    elif p_type == 'ki': ki = val
                    elif p_type == 'kd': kd = val
                    
                    bot.update_pid_params(axis, kp, ki, kd)
                    
                    self.get_logger().info(f"[PID UPDATE] Bot {bid} | Axis: {axis.upper()} | {p_type.upper()} -> {val:.4f}")

            except Exception as e:
                self.get_logger().warn(f"Failed to update param {name}: {e}")
        return SetParametersResult(successful=True)

    def control_cb(self):
        msg = BotCmdArray()
        
        for bid, bot in self.bots.items():
            if bot.pose is None:
                if bid == 0 and int(time.time()) % 5 == 0:
                    self.get_logger().warn(f"Bot {bid} has no pose data. Graph will be empty.", throttle_duration_sec=5.0)
                continue

            # ---------------- TARGET SELECTION ----------------
            # Use absolute waypoints from self.waypoints list
            target = self.waypoints[bot.step_index]
            
            target_x = target[0]
            target_y = target[1]
            target_theta = target[2]

            curr_x, curr_y, curr_theta = bot.pose
            
            error_x = target_x - curr_x
            error_y = target_y - curr_y
            
            error_theta = (math.radians(target_theta) - math.radians(curr_theta) + math.pi) % (2*math.pi) - math.pi

            dist_error = math.hypot(error_x, error_y)
            deg_error = math.degrees(abs(error_theta))

            is_at_target = (dist_error < self.xy_tolerance) and (deg_error < self.theta_tolerance)

            if is_at_target:
                if not bot.waiting:
                    bot.waiting = True
                    bot.state_start_time = time.time()
                    self.get_logger().info(f"Bot {bid}: Reached Zone {bot.step_index + 1}. Waiting...")
                
                if time.time() - bot.state_start_time > self.wait_time:
                    bot.waiting = False
                    # Cycle through 0 -> 1 -> 2 -> 3 -> 0
                    bot.step_index = (bot.step_index + 1) % len(self.waypoints)
                    self.get_logger().info(f"Bot {bid}: Moving to Zone {bot.step_index + 1}")
                    
                    bot.pid_x.reset()
                    bot.pid_y.reset()
                    bot.pid_theta.reset()

            dt = 0.03
            
            # If waiting at target, zero velocity
            if bot.waiting:
                vx, vy, omega = 0.0, 0.0, 0.0
            else:
                vx = bot.pid_x.compute(error_x, dt)
                vy = bot.pid_y.compute(error_y, dt)
                omega = bot.pid_theta.compute(error_theta, dt)

            theta_rad = math.radians(curr_theta)
            cos_t = math.cos(theta_rad)
            sin_t = math.sin(theta_rad)

            vx_r = vx * cos_t + vy * sin_t
            vy_r = -vx * sin_t + vy * cos_t

            wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])

            self.publish_cmd(bid, wheel_vel)

            # PUBLISH GRAPH DATA
            graph_msg = {
                "timestamp": time.time(),
                "bot_id": bid,
                "error_x": error_x,
                "error_y": error_y,
                "target_x": target_x,
                "current_x": curr_x
            }
            self.graph_pub.publish(String(data=json.dumps(graph_msg)))

    def publish_cmd(self, bot_id, wheel_vel):
        cmd = BotCmd()
        cmd.id = bot_id
        cmd.m1, cmd.m2, cmd.m3 = wheel_vel[0], wheel_vel[1], wheel_vel[2]
        cmd.base = 45.0
        cmd.elbow = 135.0
        
        msg = BotCmdArray()
        msg.cmds.append(cmd)
        self.cmd_pub.publish(msg)

        def map_vel(v):
            v = max(min(v, 50), -50)
            return int(90 + (v / 50.0) * 90)

        payload = {
            "m1": map_vel(cmd.m2),
            "m2": map_vel(cmd.m1),
            "m3": map_vel(cmd.m3),
            "base": 45,
            "elbow": 135,
            "mag": 0
        }
        topic = f"bot{bot_id}/cmd"
        self.mqtt_client.publish(topic, json.dumps(payload))

def main(args=None):
    rclpy.init(args=args)
    node = HolonomicPIDTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()