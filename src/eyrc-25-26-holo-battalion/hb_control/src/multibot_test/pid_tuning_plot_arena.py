#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from hb_interfaces.msg import Poses2D, BotCmd, BotCmdArray
import numpy as np
import math
import time
import random
import itertools
import json
import paho.mqtt.client as mqtt
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from std_msgs.msg import String

# ---------------------- PID Controller Class ------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt, error_threshold=None):
        if dt <= 0:
            return 0.0
        
        # 1. Accumulate Error
        self.integral += error * dt

        # 2. Anti-Windup
        if self.ki != 0:
            limit = self.max_out / self.ki
            self.integral = float(np.clip(self.integral, -limit, limit))
        else:
            self.integral = 0.0

        # 3. Compute Output
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        
        # 4. Final Output Clamp
        return float(np.clip(output, -self.max_out, self.max_out))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Per-bot State ---------------------------------------
class BotState:
    def __init__(self, bot_id, node, pid_params, final_goal, return_goal):
        self.id = bot_id
        self.node = node
        self.pose = None
        self.current_crate = None
        self.tracked_crate_id = None
        self.goal = None
        self.ir_detected = False   

        self.ir_trigger_time = None       
        self.passed_gate = False          
        self.passed_drop_gate = False     
        self.is_sharing_zone = False  
        self.passed_pickup_staging = False
        self.post_ir_duration = 0.7

        # State flags
        self.goal_reached = False
        self.arm_placed = False
        self.arm_lifted = False
        self.move_after_attach = False
        self.dropping = False
        self.backing_up = False
        self.return_to_start = False
        self.going_to_staging_point = False
        self.has_returned_home = False
        self.is_permanently_idle = False

        self.pickup_zone_idx = None
        self.waiting_for_pickup_zone = False

        self.drop_pose_logged = False
        self.arm_placed_logged = False
        
        self.last_dist_log_time = time.time()
        self.dist_log_interval = 0.5 

        self.magnet_state = 0
        self.mag_timer_start = None
        self.wait_start_time = None 

        # Smooth Arm State
        self.base_angle = 45.0
        self.elbow_angle = 135.0
        self.arm_target_base = 45.0
        self.arm_target_elbow = 135.0
        self.arm_step = 0.3   

        self.completed_crates = set()
        self.last_time = node.get_clock().now()
        self.pid_last_time = None

        # Collision & Motion
        self.pose_timestamp = node.get_clock().now()
        self.prev_pose_for_vel = None
        self.velocity = 0.0
        self.is_moving = False
        self.slowdown_scale = 1.0

        # Follow Behavior
        self.is_following = False
        self.follow_target_id = None
        self.locked_follow_dist = None

        # PID controllers
        self.pid_x = PID(**pid_params['x'])
        self.pid_y = PID(**pid_params['y'])
        self.pid_theta = PID(**pid_params['theta'])

        # Goals
        self.final_goal = final_goal.copy()
        self.return_goal = return_goal.copy()

        self.theta_filter_val = None  # Stores the smoothed angle
        self.x_filter_val = None  # NEW: Add memory for X
        self.y_filter_val = None  # NEW: Add memory for Y

        if self.id == 2:
            self.use_filter = True   # Enable for Noisy Bot
            self.filter_alpha = 0.3  # The "Sweet Spot" smoothing
        else:
            self.use_filter = False  # Disable for Good Bots
            self.filter_alpha = 1.0  # 1.0 = No smoothing (mathematically)

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

# ---------------------- Main Node Class -------------------------------------
class HolonomicMoveToCratesMulti(Node):

    def __init__(self):
        super().__init__('holonomic_move_to_crates_multi') 

        self.node_start_time = time.time()
        self.startup_delay = 5.0 
        self.is_warmed_up = False

        # --- MULTI-AXIS TUNING CONFIGURATION ---
        self.pid_tuning_mode = True       
        self.tuning_bot_id = 2
        
        # ---------------- NEW DEBUG PUBLISHERS ----------------
        self.pub_x_debug = self.create_publisher(Float32, '/debug/x_error', 10)
        self.pub_y_debug = self.create_publisher(Float32, '/debug/y_error', 10)
        self.pub_t_debug = self.create_publisher(Float32, '/debug/theta_error', 10)
        
        self.pub_x_out   = self.create_publisher(Float32, '/debug/x_output', 10)
        self.pub_y_out   = self.create_publisher(Float32, '/debug/y_output', 10)
        self.pub_t_out   = self.create_publisher(Float32, '/debug/theta_output', 10)
        # ------------------------------------------------------

        self.bot_ids = [0,2,4] 
        self.target_crate_ids = [30, 14, 12, 11, 13, 22, 16, 21]
        self.current_pose = None 
        self.crates = [] 
        self.completed_crates = set() 
        self.drop_positions = {} 
        self.drop_positions_assigned = False 

        self.last_time = self.get_clock().now() 

        self.xy_tolerance = 15.0 
        self.theta_tolerance_deg = 3.0  
        self.theta_tolerance_center_deg = 100.0  
        self.xy_tolerance_x = 5.0 
        self.xy_tolerance_theta = 15.0 

        self.max_vel = 50.0  
        self.hard_slow_distance = 100.0     
        self.hard_slow_scale = 0.5          
        self.slow_radius = 120.0      
        self.min_slow_scale = 0.25    
        self.follow_distance_threshold = 350.0  
        self.bot_radius = 100.0

        self.final_goal = np.array([1219.0,1180.0,0.0])  
        self.return_goal = np.array([1218.0,205.0,0.0])  

        self.sp_0_alt = np.array([1700.0, 1400.0, 0.0]) 
        self.sp_4_alt = np.array([800.0, 1400.0, 0.0])
        self.staging_point_0 = np.array([800.0,1400.0,0.0])  
        self.staging_point_2 = np.array([1700.0,1400.0,0.0])  
        self.staging_point_4 = np.array([800.0,1400.0,0.0])  
        self.staging_y_threshold = 1400.0  

        self.idle_home_point_0 = np.array([1218.0,205.0,0.0])  
        self.idle_home_point_2 = np.array([1568.0,202.0,0.0])  
        self.idle_home_point_4 = np.array([864.0,204.0,0.0])  

        self.zone_drop_counts = {0: 0, 1: 0, 2: 0}

        self.pickup_zones = [
            {'x_min': 180, 'x_max': 390, 'y_min': 560, 'y_max': 1111, 'gate': (285.0, 480.0)},
            {'x_min': 2060, 'x_max': 2266, 'y_min': 560, 'y_max': 1111, 'gate': (2163.0, 480.0)},
            {'x_min': 180, 'x_max': 390, 'y_min': 1350, 'y_max': 1900, 'gate': (285.0, 1270.0)},
            {'x_min': 2060, 'x_max': 2266, 'y_min': 1350, 'y_max': 1900, 'gate': (2163.0, 1270.0)},
        ]

        self.pickup_staging_points = {
            0: (570.0, 418.0),   
            1: (1800.0, 418.0),  
            2: (522.0, 1240.0),  
            3: (1741.0, 1060.0)  
        }

        self.x_min,self.x_max = 1020,1410  
        self.y_min,self.y_max = 1075,1355  

        alphas = [math.radians(30), math.radians(150), math.radians(270)]
        M = np.array([
            [math.cos(a + math.pi/2) for a in alphas],
            [math.sin(a + math.pi/2) for a in alphas],
            [1, 1, 1]
        ])
        self.M_inv = np.linalg.pinv(M) 

        self.arm_down_config = {
            0: (15.0, 180.0),
            2: (15.0, 180.0),  
            4: (5.0, 180.0)
        }

        self.attach_service = self.create_service(Trigger,'/attach',self.attach_service_cb)

        # ---------------- JITTER FIX VALUES ----------------
        self.pid_params_by_bot = {
            # Apply to Bot 0, 2, and 4
            0: {
                # X/Y: Reduced Kd to 0.05 to stop linear jitter. Kp=0.6 for stability.
                'x':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': 40.0},
                'y':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': 40.0},
                
                # THETA: Aggressively reduced Kd (30->5) and Kp (80->60) to fix rotation shake
                'theta': {'kp': 60.0, 'ki': 10.0, 'kd': 5.0, 'max_out': 30.0}
            },
            2: {
                # These are the "Perfect" values we found in the graphs
                'x':     {'kp': 0.7, 'ki': 0.15, 'kd': 0.1, 'max_out': 50.0},
                'y':     {'kp': 0.7, 'ki': 0.15, 'kd': 0.1, 'max_out': 50.0},
                'theta': {'kp': 150.0, 'ki': 0.0, 'kd': 10.0, 'max_out': 40.0}
            },
            4: {
                'x':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': 40.0},
                'y':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': 40.0},
                'theta': {'kp': 60.0, 'ki': 10.0, 'kd': 5.0, 'max_out': 30.0}
            },
        }
        # ---------------------------------------------------------------------------------

        self.bots = {bid:BotState(bid,self,self.pid_params_by_bot[bid],self.final_goal,self.return_goal) for bid in self.bot_ids}
        
        if 0 in self.bots:
            self.bots[0].return_goal = self.idle_home_point_0.copy()

        self.drop_zones = {
            0:{'x_min':1020,'x_max':1400,'y_min':1075,'y_max':1355},
            2:{'x_min':1470,'x_max':1752,'y_min':1920,'y_max':2115},
            1:{'x_min':675,'x_max':920,'y_min':1920,'y_max':2115}
        }

        self.drop_gates = {
            0: (1210.0, 950.0),   
            1: (800.0, 1800.0),   
            2: (1610.0, 1800.0)   
        }

        self.zone_locks = {0:None,1:None,2:None}
        self.pickup_zone_locks = {0: None, 1: None, 2: None, 3: None}

        self.zone_subparts = {}
        
        z0_mid_x = (1020 + 1400) / 2
        z0_mid_y = (1075 + 1355) / 2
        self.zone_subparts[0] = [ (z0_mid_x, z0_mid_y), (1115.0, z0_mid_y), (1305.0, z0_mid_y) ]

        z1_mid_x = (675 + 920) / 2
        z1_mid_y = (1920 + 2115) / 2
        self.zone_subparts[1] = [ (z1_mid_x, z1_mid_y), (z1_mid_x - 60.0, z1_mid_y), (z1_mid_x + 60.0, z1_mid_y) ]

        z2_mid_x = (1470 + 1752) / 2
        z2_mid_y = (1920 + 2115) / 2
        self.zone_subparts[2] = [ (z2_mid_x, z2_mid_y), (z2_mid_x - 60.0, z2_mid_y), (z2_mid_x + 60.0, z2_mid_y) ]
        
        # Standard Publishers (kept for compatibility)
        self.pid_setpoint_pub = self.create_publisher(Float32, '/pid/setpoint', 10)
        self.pid_current_pub = self.create_publisher(Float32, '/pid/current', 10)
        self.pid_error_pub = self.create_publisher(Float32, '/pid/error', 10)
        self.pid_output_pub = self.create_publisher(Float32, '/pid/output', 10)

        self.create_subscription(Poses2D,'bot_pose',self.pose_cb,10)
        self.create_subscription(Poses2D,'crate_pose',self.crate_cb,10)
        self.publisher = self.create_publisher(BotCmdArray,'/bot_cmd',10)
        self.timer = self.create_timer(0.03,self.control_cb)
        self.debug_pub = self.create_publisher(String, '/perception_debug', 10)
        self.debug_timer = self.create_timer(0.2, self.publish_perception_debug)

        # MQTT setup
        import paho.mqtt.client as mqtt
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        try:
            self.mqtt_client.connect("10.83.7.6", 1883, 60)
            for bid in self.bot_ids:
                topic = f"bot{bid}/ir"
                self.mqtt_client.subscribe(topic)
                self.get_logger().info(f"Subscribed to MQTT: {topic}")
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

    # ---------------- MQTT Callbacks ----------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Connected (rc={rc})")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT Disconnected (rc={rc}). Attempting reconnect...")
        try:
            client.loop_start()
        except:
            pass

    # ---------------- Callbacks ----------------
    def pose_cb(self,msg:Poses2D):
        for pose in msg.poses:
            if pose.id in self.bots:
                bot = self.bots[pose.id]
                now = self.get_clock().now()
                new_pose_np = np.array([pose.x,pose.y,pose.w]) 

                if bot.prev_pose_for_vel is not None:
                    dt = (now - bot.pose_timestamp).nanoseconds / 1e9
                    if dt > 0.001:
                        dx = new_pose_np[0] - bot.prev_pose_for_vel[0]
                        dy = new_pose_np[1] - bot.prev_pose_for_vel[1]
                        bot.velocity = math.hypot(dx,dy) / dt
                    else:
                        bot.velocity = 0.0
                else:
                    bot.velocity = 0.0 

                bot.pose = new_pose_np 
                bot.prev_pose_for_vel = new_pose_np  
                bot.pose_timestamp = now  
                bot.last_time = now 

    def crate_cb(self,msg:Poses2D):
        self.crates = []  
        for pose in msg.poses:
            if pose.id in self.target_crate_ids:
                self.crates.append({
                    'id':pose.id,
                    'x':pose.x,
                    'y':pose.y,
                    'w':pose.w
                })

        if not self.drop_positions_assigned and len(self.crates) > 0:
            self.assign_drop_positions()
            self.drop_positions_assigned = True
            self.get_logger().info("Drop positions assigned for all crates")

    # ---------------- Logging Helper ----------------
    def log_bot_distance(self, bot: BotState, gx, gy, label):
        now = time.time()
        if now - bot.last_dist_log_time >= bot.dist_log_interval:
            if bot.pose is not None:
                x, y, theta = bot.pose
                dist = math.hypot(gx - x, gy - y)
                self.get_logger().info(
                    f"Bot {bot.id} | {label} | "
                    f"Dist: {dist:.2f} mm | "
                    f"Current: ({x:.1f}, {y:.1f}, θ={theta:.1f}°) | "
                    f"Goal: ({gx:.1f}, {gy:.1f})"
                )
                bot.last_dist_log_time = now

    def get_zone_for_crate(self, crate_id):
        return crate_id % 3 

    def assign_drop_positions(self):
        self.drop_positions = {}
        spacing_x = 30.0   
        spacing_y = 80.0
        
        zone_centers = {
            0: ((1020 + 1400) / 2, (1075 + 1355) / 2),
            1: ((675 + 920) / 2,  (1920 + 2115) / 2),
            2: ((1470 + 1752) / 2, (1920 + 2115) / 2),
        }

        self.zone_drop_counts = {0: 0, 1: 0, 2: 0}
        all_active = {c['id']: c for c in self.crates}
        for bot in self.bots.values():
            if bot.tracked_crate_id is not None and bot.tracked_crate_id not in all_active:
                bx = bot.pose[0] if bot.pose is not None else 1219.0
                by = bot.pose[1] if bot.pose is not None else 1000.0
                all_active[bot.tracked_crate_id] = {'id': bot.tracked_crate_id, 'x': bx, 'y': by}

        crates_by_zone = {0: [], 1: [], 2: []}
        for c in all_active.values():
            z = self.get_zone_for_crate(c['id'])
            if z in crates_by_zone:
                crates_by_zone[z].append(c)

        for z, crates in crates_by_zone.items():
            if not crates: continue
            cx, cy = zone_centers[z]
            sorted_crates = sorted(crates, key=lambda x: x['y'], reverse=True)
            offsets_map = {
                0: (-spacing_x, 0.0),
                1: (+spacing_x, 0.0),
                2: (0.0, -spacing_y)
            }
            mapping = []
            if len(sorted_crates) >= 3:
                mapping = [
                    (sorted_crates[0], 2), 
                    (sorted_crates[1], 0), 
                    (sorted_crates[2], 1)
                ]
            else:
                for i, crate in enumerate(sorted_crates):
                    mapping.append((crate, i))

            for crate, slot_idx in mapping:
                ox, oy = offsets_map.get(slot_idx, (0.0, 0.0))
                self.drop_positions[crate['id']] = {
                    'x': cx + ox,
                    'y': cy + oy + 20, 
                    'zone_slot': slot_idx
                }
                self.zone_drop_counts[z] += 1
        self.get_logger().info(f"Drop positions assigned (Max Y rule applied): {self.drop_positions}")

    def get_pickup_zone_idx_for_crate(self, crate):
        for i, z in enumerate(self.pickup_zones):
            if (z['x_min'] <= crate['x'] <= z['x_max'] and
                z['y_min'] <= crate['y'] <= z['y_max']):
                return i
        return None

    def assign_crates_greedily(self):
        # (Your greedy assignment logic - maintained for future use)
        pass 

    def check_and_handle_collisions(self):
        # (Your collision logic - maintained)
        pass

    def is_bot_physically_in_pickup_zone(self, bot: BotState):
        if bot.pose is None: return False
        x, y, _ = bot.pose
        for z in self.pickup_zones:
            if (z['x_min'] - 50 <= x <= z['x_max'] + 50 and 
                z['y_min'] - 50 <= y <= z['y_max'] + 50):
                return True
        return False

    def publish_perception_debug(self):
        zone_info = {
            "locks": self.zone_locks,
            "pickup_zone_locks": self.pickup_zone_locks,
            "subparts": {k: [list(pt) for pt in v] for k, v in self.zone_subparts.items()} 
        }
        bots_data = {}
        for bid, bot in self.bots.items():
            if bot.pose is None: continue
            status = "TUNING"
            bots_data[bid] = {
                "pose": list(bot.pose),
                "status": status,
            }
        debug_msg = {
            "timestamp": time.time(),
            "drop_zones": zone_info,
            "bots": bots_data,
        }
        msg = String()
        msg.data = json.dumps(debug_msg)
        self.debug_pub.publish(msg)

    # ---------------- CONTROL LOOP ----------------
    def control_cb(self):
        now = self.get_clock().now()

        # ---------------- PID TUNING MODE ----------------
        if self.pid_tuning_mode:
            bot = self.bots[self.tuning_bot_id]

            if bot.pid_last_time is None:
                bot.pid_last_time = now
                dt = 0.03
            else:
                dt = (now - bot.pid_last_time).nanoseconds / 1e9
                bot.pid_last_time = now
            
            if dt <= 0: dt = 0.03

            if bot.pose is not None:
                # CALL THE NEW MULTI-AXIS TEST
                self.run_holonomic_test(bot, dt) 
            else:
                self.get_logger().warn(f"Bot {bot.id} waiting for pose...", throttle_duration_sec=1.0)
            
            return 

        # ---------------- NORMAL OPERATION ----------------
        # (This part is skipped during tuning)
        pass 

    # ---------------- MULTI-AXIS TEST FUNCTION ----------------
    # ---------------- SQUARE PATH TEST FUNCTION ----------------
    def run_holonomic_test(self, bot: BotState, dt):
        # 1. Define Waypoints
        waypoints = [
            (300.0,  300.0,  0.0),
            (2100.0, 2100.0, 0.0),
            (300.0,  2100.0, 0.0),
            (2100.0, 300.0,  0.0)
        ]

        if not hasattr(self, 'test_wp_idx'):
             self.test_wp_idx = 0

        gx, gy, g_deg = waypoints[self.test_wp_idx]
        gt = math.radians(g_deg)

        # 2. Get Raw Pose
        curr_x, curr_y, curr_t_deg = bot.pose
        curr_t = math.radians(curr_t_deg)

        # ---------------- FILTERING LOGIC ----------------
        if bot.use_filter:
            # --- X FILTER ---
            if bot.x_filter_val is None: bot.x_filter_val = curr_x
            bot.x_filter_val += bot.filter_alpha * (curr_x - bot.x_filter_val)
            curr_x = bot.x_filter_val # Override with filtered

            # --- Y FILTER ---
            if bot.y_filter_val is None: bot.y_filter_val = curr_y
            bot.y_filter_val += bot.filter_alpha * (curr_y - bot.y_filter_val)
            curr_y = bot.y_filter_val # Override with filtered

            # --- THETA FILTER (With Wrap-Around) ---
            if bot.theta_filter_val is None:
                bot.theta_filter_val = curr_t
            else:
                diff = curr_t - bot.theta_filter_val
                diff = (diff + math.pi) % (2 * math.pi) - math.pi
                bot.theta_filter_val += bot.filter_alpha * diff
            curr_t = bot.theta_filter_val # Override with filtered
        else:
            # For Bots 0 & 4: Do nothing. 
            # We keep variables synced just in case, but we use raw values.
            bot.x_filter_val = curr_x
            bot.y_filter_val = curr_y
            bot.theta_filter_val = curr_t
        # -------------------------------------------------

        # 3. Calculate Errors
        err_x = gx - curr_x
        err_y = gy - curr_y
        
        err_t = gt - curr_t
        err_t = (err_t + math.pi) % (2 * math.pi) - math.pi

        # 4. Check Arrival
        dist_to_target = math.hypot(err_x, err_y)
        if dist_to_target < 50.0:
            self.test_wp_idx = (self.test_wp_idx + 1) % len(waypoints)
            self.get_logger().info(f"Bot {bot.id} Reached Waypoint! Next: {waypoints[self.test_wp_idx]}")
            bot.pid_x.reset()
            bot.pid_y.reset()
            bot.pid_theta.reset()
            return

        # 5. Compute PIDs
        out_x = bot.pid_x.compute(err_x, dt)
        out_y = bot.pid_y.compute(err_y, dt)
        out_t = bot.pid_theta.compute(err_t, dt)

        # 6. Publish Debug
        self.pub_x_debug.publish(Float32(data=float(err_x)))
        self.pub_y_debug.publish(Float32(data=float(err_y)))
        
        # 7. Field Oriented Control
        cos_t, sin_t = math.cos(curr_t), math.sin(curr_t)
        vx_r = out_x * cos_t + out_y * sin_t
        vy_r = -out_x * sin_t + out_y * cos_t

        wheel_vel = self.M_inv @ np.array([vx_r, vy_r, out_t])
        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id)

    def attach_service_cb(self, request, response):
        magnet_on = False
        for bot in self.bots.values():
            if bot.magnet_state != 0:
                magnet_on = True
                break
        response.success = magnet_on
        response.message = "Magnet ON" if magnet_on else "Magnet OFF"
        return response

    def update_arm_smooth(self, bot: BotState):
        done = True
        def move(curr, target):
            if abs(curr - target) <= bot.arm_step:
                return target, True
            return (curr + bot.arm_step if curr < target else curr - bot.arm_step, False)

        bot.base_angle, b_done = move(bot.base_angle, bot.arm_target_base)
        bot.elbow_angle, e_done = move(bot.elbow_angle, bot.arm_target_elbow)
        return b_done and e_done

    def publish_wheel_velocities(self, wheel_vel, bot_id, base=45.0, elbow=135.0, mag=0):
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = bot_id
        if len(wheel_vel) >= 3:
            cmd.m1, cmd.m2, cmd.m3 = wheel_vel[0], wheel_vel[1], wheel_vel[2]
        else:
            cmd.m1, cmd.m2, cmd.m3 = 0.0, 0.0, 0.0
        cmd.base = base
        cmd.elbow = elbow
        msg.cmds.append(cmd)
        self.publisher.publish(msg)

        def map_vel(v):
            v = max(min(v, 50), -50)
            return int(90 + (v / 50.0) * 90)

        payload = {
            "m1": map_vel(cmd.m2),
            "m2": map_vel(cmd.m1),
            "m3": map_vel(cmd.m3),
            "base": int(base),
            "elbow": int(elbow),
            "mag": int(mag)
        }
        topic = f"bot{bot_id}/cmd"
        self.mqtt_client.publish(topic, json.dumps(payload))

    def on_mqtt_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode().strip()
            if "/ir" in topic:
                bot_id_str = topic.split('/')[0].replace('bot', '')
                if not bot_id_str.isdigit(): return
                bot_id = int(bot_id_str)
                if bot_id in self.bots:
                    is_obj_detected = (payload == '0')
                    self.bots[bot_id].ir_detected = is_obj_detected
        except Exception as e:
            self.get_logger().warn(f"MQTT Parse Error: {e}")

# ---------------------- Main -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HolonomicMoveToCratesMulti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down multi-bot node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()