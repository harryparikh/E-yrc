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
from std_msgs.msg import String

# ---------------------- PID Controller Class --------------------------------
# class PID:
#     def __init__(self, kp, ki, kd, max_out=1.0):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.max_out = max_out
#         self.integral = 0.0
#         self.prev_error = 0.0

#     def compute(self, error, dt):
#         if dt <= 0:
#             return 0.0
#         self.integral += error * dt
#         derivative = (error - self.prev_error) / dt
#         output = self.kp * error + self.ki * self.integral + self.kd * derivative
#         self.prev_error = error
#         return float(np.clip(output, -self.max_out, self.max_out))

#     def reset(self):
#         self.integral = 0.0
#         self.prev_error = 0.0

# ---------------------- PID Controller Class (Fixed) ------------------------
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
        
        # [Existing] Deadband reset (Keep this if you use it for "done" state)
        if error_threshold is not None and abs(error) < error_threshold:
            self.integral = 0.0
        
        # ---------------- MODIFIED SECTION ----------------
        # 1. Accumulate Error with a WINDOW check
        # Only start accumulating Integral if we are within 150mm of the target.
        # If we are far away, reset Integral to 0 to prevent "Windup" during travel.
        if abs(error) < 50.0:
            self.integral += error * dt
        else:
            self.integral = 0.0
        # --------------------------------------------------

        # 2. Anti-Windup: Clamp the Integral Term
        # (Keep your existing clamping logic below, it acts as a secondary safety)
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

        self.ir_trigger_time = None       # Timestamp when IR first went low
        self.passed_gate = False          # For Pickup Gates
        self.passed_drop_gate = False     # <--- NEW: For Drop Zone Gates
        self.is_sharing_zone = False  # <--- ADD THIS LINE HERE
        self.passed_pickup_staging = False
        self.post_ir_duration = 0.7

        # state flags
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

        # Logging flags for one-time messages
        self.drop_pose_logged = False
        self.arm_placed_logged = False
        
        # Distance logging
        self.last_dist_log_time = time.time()
        self.dist_log_interval = 0.5 

        # magnet state
        self.magnet_state = 0
        self.mag_timer_start = None
        self.wait_start_time = None  # Timer for post-place delay

        # ---------------- Smooth Arm State ----------------
        self.base_angle = 45.0
        self.elbow_angle = 135.0

        self.arm_target_base = 45.0
        self.arm_target_elbow = 135.0

        self.arm_step = 0.3   # degrees per control cycle (smoothness)

        self.completed_crates = set()
        self.last_time = node.get_clock().now()

        self.pid_last_time = None

        # [COLLISION AVOIDANCE] State variables
        self.pose_timestamp = node.get_clock().now()
        self.prev_pose_for_vel = None
        self.velocity = 0.0
        self.is_moving = False
        self.slowdown_scale = 1.0

        # [FOLLOW BEHAVIOR] New variables
        self.is_following = False
        self.follow_target_id = None
        self.locked_follow_dist = None
        self.follow_offset_distance = 245.

        # PID controllers
        self.pid_x = PID(**pid_params['x'])
        self.pid_y = PID(**pid_params['y'])
        self.pid_theta = PID(**pid_params['theta'])

        # Goals
        self.final_goal = final_goal.copy()
        self.return_goal = return_goal.copy()

        self.theta_filter_val = None  # Stores the smoothed angle
        self.x_filter_val = None      # Stores smoothed X
        self.y_filter_val = None      # Stores smoothed Y

        # Enable filter only for Bot 2 (The Noisy Bot)
        if self.id == 2:
            self.use_filter = True   
            self.filter_alpha = 0.3  # 0.6 = "Sweet Spot" smoothing
        else:
            self.use_filter = False  
            self.filter_alpha = 1.0  # 1.0 = No smoothing (Raw Data)

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

# ---------------------- Main Node Class -------------------------------------
class HolonomicMoveToCratesMulti(Node):

    def __init__(self):
        super().__init__('holonomic_move_to_crates_multi') 

        self.node_start_time = time.time()
        self.startup_delay = 5.0  # Seconds to wait for vision
        self.is_warmed_up = False

        # ---------------- Robot and world state ----------------
        self.bot_ids = [0,2,4]  # Controlled robot IDs
        self.target_crate_ids = [30, 14, 12, 11, 13, 22, 16, 21]
        self.current_pose = None  # Unused global pose placeholder
        self.crates = []  # List of detected crates
        self.completed_crates = set()  # Global tracker for all completed crates
        self.drop_positions = {}  # Assigned drop locations per crate
        self.drop_positions_assigned = False  # One-time assignment flag

        self.last_time = self.get_clock().now()  # Global timing reference

        # ---------------- Motion tolerances ----------------
        self.xy_tolerance = 15.0  # Position tolerance in mm
        self.theta_tolerance_deg = 3.0  
        self.theta_tolerance_center_deg = 100.0  
        self.xy_tolerance_x = 5.0   # mm
        self.xy_tolerance_theta = 15.0   #degree

        self.max_vel = 50.0  

        # ---------------- Hard slowdown parameters ----------------
        self.hard_slow_distance = 100.0     # mm
        self.hard_slow_scale = 0.5          # 50%


        # ---------------- Slowdown near goal ----------------
        self.slow_radius = 120.0      
        self.min_slow_scale = 0.25    

        # ---------------- Collision avoidance parameters ----------------
        self.follow_distance_threshold = 350.0  
        self.bot_radius = 100.0
        # self.safety_radius = 550.0  
        # self.critical_radius = 400.0  

        # ---------------- Global goals ----------------
        self.final_goal = np.array([1219.0,1180.0,0.0])  
        self.return_goal = np.array([1218.0,205.0,0.0])  

        # ---------------- Staging points (Consolidated) ----------------
        # self.staging_y_threshold = 1400.0  
        
        # # Standard Staging Points
        # self.sp_0_standard = np.array([600.0, 1219.0, 0.0])
        # self.sp_2_standard = np.array([1700.0, 1500.0, 0.0])
        # self.sp_4_standard = np.array([600.0, 1500.0, 0.0])
        
        # Alternative Staging Points (Right Side)
        self.sp_0_alt = np.array([1700.0, 1400.0, 0.0]) 
        self.sp_4_alt = np.array([800.0, 1400.0, 0.0])

        self.staging_point_0 = np.array([800.0,1400.0,0.0])  
        self.staging_point_2 = np.array([1700.0,1400.0,0.0])  
        self.staging_point_4 = np.array([800.0,1400.0,0.0])  
        self.staging_y_threshold = 1400.0  

        # ---------------- Idle home points ----------------
        self.idle_home_point_0 = np.array([1218.0,205.0,0.0])  
        self.idle_home_point_2 = np.array([1568.0,202.0,0.0])  
        self.idle_home_point_4 = np.array([864.0,204.0,0.0])  

        self.zone_drop_counts = {0: 0, 1: 0, 2: 0}


        # -----------------Crate Puickup Points----------------

        self.pickup_zones = [
            # P1: Entry from Y=560. Gate at Y=480.
            {'x_min': 180, 'x_max': 390, 'y_min': 560, 'y_max': 1111, 'gate': (285.0, 480.0)},
            
            # P2: Entry from Y=560. Gate at Y=480.
            {'x_min': 2060, 'x_max': 2266, 'y_min': 560, 'y_max': 1111, 'gate': (2163.0, 480.0)},
            
            # P3: Entry from Y=1350. Gate at Y=1270.
            {'x_min': 180, 'x_max': 390, 'y_min': 1350, 'y_max': 1900, 'gate': (285.0, 1270.0)},
            
            # P4: Entry from Y=1350. Gate at Y=1270.
            {'x_min': 2060, 'x_max': 2266, 'y_min': 1350, 'y_max': 1900, 'gate': (2163.0, 1270.0)},
        ]

        self.pickup_staging_points = {
            0: (570.0, 418.0),   # P1
            1: (1800.0, 418.0),  # P2
            2: (522.0, 1240.0),  # P3
            3: (1741.0, 1060.0)  # P4
        }

        # ---------------- Crate region limits ----------------
        self.x_min,self.x_max = 1020,1410  
        self.y_min,self.y_max = 1075,1355  

        # ---------------- Kinematics Pre-Calculation (CPU FIX) ----------------
        alphas = [math.radians(30), math.radians(150), math.radians(270)]
        M = np.array([
            [math.cos(a + math.pi/2) for a in alphas],
            [math.sin(a + math.pi/2) for a in alphas],
            [1, 1, 1]
        ])
        self.M_inv = np.linalg.pinv(M) # Calculate once here, use everywhere

        # ---------------- Per-bot ARM DOWN configuration ----------------
        self.arm_down_config = {
            0: (15.0, 180.0),
            2: (15.0, 180.0),  
            4: (5.0, 180.0)
        }

        self.attach_service = self.create_service(Trigger,'/attach',self.attach_service_cb)


        # ---------------- Per-bot PID parameters ----------------
        self.pid_params_by_bot = {
            # 0:{'x':{'kp':2.2,'ki':0.004,'kd':0.0,'max_out':self.max_vel},
            #    'y':{'kp':2.5,'ki':0.004,'kd':0.01,'max_out':self.max_vel},
            #    'theta':{'kp':40,'ki':0.002,'kd':0.1,'max_out':self.max_vel*2}},
            # 2:{'x':{'kp':2.2,'ki':0.008,'kd':0,'max_out':self.max_vel},
            #    'y':{'kp':2.7,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
            #    'theta':{'kp':50,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            # 4:{'x':{'kp':2.5,'ki':0.008,'kd':0,'max_out':self.max_vel},
            #    'y':{'kp':2.7,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
            #    'theta':{'kp':50,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},

            # 0:{'x':{'kp':1.4999999999999998,'ki':-0.0008100000000000001,'kd':0.009999999999999995,'max_out':self.max_vel},
            #    'y':{'kp':1.9,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
            #    'theta':{'kp':45,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            # # 0:{'x':{'kp':1.5,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
            # #    'y':{'kp':1.9,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
            # #    'theta':{'kp':45,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            # 2:{'x':{'kp':2.2,'ki':0.008,'kd':0,'max_out':self.max_vel},
            #    'y':{'kp':2.7,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
            #    'theta':{'kp':50,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            # 4:{'x':{'kp':1.3,'ki':0.0002,'kd':0.01,'max_out':self.max_vel},
            #    'y':{'kp':1.9,'ki':0.0001,'kd':0.03,'max_out':self.max_vel},
            #    'theta':{'kp':40,'ki':0.005,'kd':0.1,'max_out':self.max_vel*2}},


            # 0:{'x':{'kp':1.8,'ki':0.00001,'kd':0.01,'max_out':self.max_vel},
            #    'y':{'kp':1.9,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
            #    'theta':{'kp':65,'ki':0.0006,'kd':0.1,'max_out':self.max_vel*2}},
            # 2:{'x':{'kp':1.7,'ki':0.0002,'kd':0.02,'max_out':self.max_vel},
            #    'y':{'kp':1.9,'ki':0.00001,'kd':0.03,'max_out':self.max_vel},
            #    'theta':{'kp':70,'ki':0.0001,'kd':0.1,'max_out':self.max_vel*2}},
            # 4:{'x':{'kp':1.45,'ki':0.0002,'kd':0.04,'max_out':self.max_vel},
            #    'y':{'kp':2.1,'ki':0.000006,'kd':0.05,'max_out':self.max_vel},
            #    'theta':{'kp':80,'ki':0.000,'kd':0.02,'max_out':self.max_vel*2}},

            # 0:{'x':{'kp':1.8,'ki':0.00001,'kd':0.01,'max_out':self.max_vel},
            #    'y':{'kp':1.9,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
            #    'theta':{'kp':65,'ki':0.0006,'kd':0.1,'max_out':self.max_vel*2}},
            # 2:{'x':{'kp':1.7,'ki':0.0002,'kd':0.02,'max_out':self.max_vel},
            #    'y':{'kp':1.9,'ki':0.00001,'kd':0.03,'max_out':self.max_vel},
            #    'theta':{'kp':80,'ki':0.0001,'kd':0.1,'max_out':self.max_vel*2}},
            # 4:{'x':{'kp':1.45,'ki':0.0002,'kd':0.04,'max_out':self.max_vel},
            #    'y':{'kp':2.1,'ki':0.000006,'kd':0.05,'max_out':self.max_vel},
            #    'theta':{'kp':80,'ki':0.000,'kd':0.02,'max_out':self.max_vel*2}},

            0: {
                # X/Y: Reduced Kd to 0.05 to stop linear jitter. Kp=0.6 for stability.
                'x':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': self.max_vel},
                'y':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': self.max_vel},
                
                # THETA: Aggressively reduced Kd (30->5) and Kp (80->60) to fix rotation shake
                'theta': {'kp': 60.0, 'ki': 10.0, 'kd': 5.0, 'max_out': 30.0}
            },
            
            2: {
                # 'x':     {'kp': 0.8, 'ki': 0.08, 'kd': 0.5, 'max_out': self.max_vel},
                # 'y':     {'kp': 0.8, 'ki': 0.08, 'kd': 0.5, 'max_out': self.max_vel},
                # 'theta': {'kp': 90.0, 'ki': 1.5, 'kd': 10.0, 'max_out': 40.0}

                'x':     {'kp': 0.7, 'ki': 0.15, 'kd': 0.1, 'max_out': 50.0},
                'y':     {'kp': 0.7, 'ki': 0.15, 'kd': 0.1, 'max_out': 50.0},
                'theta': {'kp': 80.0, 'ki': 1.0, 'kd': 2.0, 'max_out': 40.0}
            },
            4: {
                'x':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': self.max_vel},
                'y':     {'kp': 0.6, 'ki': 0.0, 'kd': 0.05, 'max_out': self.max_vel},
                'theta': {'kp': 60.0, 'ki': 10.0, 'kd': 5.0, 'max_out': 30.0}
            },
        }

        # ---------------- Per-bot state machines ----------------
        self.bots = {bid:BotState(bid,self,self.pid_params_by_bot[bid],self.final_goal,self.return_goal) for bid in self.bot_ids}

        if 0 in self.bots:
            self.bots[0].return_goal = self.idle_home_point_0.copy()

        # ---------------- Drop zones ----------------
        self.drop_zones = {
            0:{'x_min':1020,'x_max':1400,'y_min':1075,'y_max':1355},
            2:{'x_min':1470,'x_max':1752,'y_min':1920,'y_max':2115},
            1:{'x_min':675,'x_max':920,'y_min':1920,'y_max':2115}
        }

        # [NEW] Drop Zone Gates (Entry Points)
        # Robots wait here if the zone is locked.
        # Zone 0 Gate: X=1210, Y=950 (125mm above zone start)
        # Zone 1 Gate: X=800, Y=1800 (120mm above zone start)
        # Zone 2 Gate: X=1610, Y=1800 (120mm above zone start)
        self.drop_gates = {
            0: (1210.0, 950.0),   # Gate for Top Zone
            1: (800.0, 1800.0),   # Gate for Left Zone
            2: (1610.0, 1800.0)   # Gate for Right Zone
        }

        self.zone_locks = {0:None,1:None,2:None}

        self.pickup_zone_locks = {0: None, 1: None, 2: None, 3: None}
        
        # NEW: Define 3 subparts for each zone [Full_Center, Left_Half, Right_Half]
        self.zone_subparts = {}
        
        # --- Zone 0 (Top) - ALREADY CORRECT ---
        z0_mid_x = (1020 + 1400) / 2
        z0_mid_y = (1075 + 1355) / 2
        self.zone_subparts[0] = [
            (z0_mid_x, z0_mid_y),            # 0: Full Center
            (1115.0, z0_mid_y),              # 1: Left Slot
            (1305.0, z0_mid_y)               # 2: Right Slot
        ]

        # --- Zone 1 (Left) - FIX: MAKE HORIZONTAL ---
        z1_mid_x = (675 + 920) / 2
        z1_mid_y = (1920 + 2115) / 2
        
        # Width is ~245mm. Let's space slots by ~100mm from center to fit inside.
        # Left Slot: Center X - 50
        # Right Slot: Center X + 50
        self.zone_subparts[1] = [
            (z1_mid_x, z1_mid_y),            # 0: Full Center
            (z1_mid_x - 60.0, z1_mid_y),     # 1: Left Slot (Varies X)
            (z1_mid_x + 60.0, z1_mid_y)      # 2: Right Slot (Varies X)
        ]

        # --- Zone 2 (Right) - FIX: MAKE HORIZONTAL ---
        z2_mid_x = (1470 + 1752) / 2
        z2_mid_y = (1920 + 2115) / 2

        # Width is ~282mm. 
        self.zone_subparts[2] = [
            (z2_mid_x, z2_mid_y),            # 0: Full Center
            (z2_mid_x - 60.0, z2_mid_y),     # 1: Left Slot (Varies X)
            (z2_mid_x + 60.0, z2_mid_y)      # 2: Right Slot (Varies X)
        ]
        
        # ---------------- ROS interfaces ----------------
        self.create_subscription(Poses2D,'bot_pose',self.pose_cb,10)
        self.create_subscription(Poses2D,'crate_pose',self.crate_cb,10)
        self.publisher = self.create_publisher(BotCmdArray,'/bot_cmd',10)
        self.timer = self.create_timer(0.03,self.control_cb)
        # Debug / Perception Publisher
        self.debug_pub = self.create_publisher(String, '/perception_debug', 10)
        self.debug_timer = self.create_timer(0.2, self.publish_perception_debug)

        # ---------------- MQTT setup (hardware control) ----------------
        import paho.mqtt.client as mqtt
        # ---------------- MQTT setup (Robust) ----------------
        self.mqtt_client = mqtt.Client()
        # Add these two lines to handle disconnects automatically:
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        try:
            self.mqtt_client.connect("10.198.166.6", 1883, 60)
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

    # ---------------- SMART Drop position assignment (Fixed with +20 Offset) ----------------
    def get_zone_for_crate(self, crate_id):
        return crate_id % 3 

    # ---------------- SMART Drop position assignment (Fixed) ----------------
    # ---------------- SMART Drop position assignment (Updated for Max Y -> Center) ----------------
    def assign_drop_positions(self):
        self.drop_positions = {}

        # Distance from center to side slots
        spacing_x = 30.0   
        spacing_y = 80.0
        
        # Zone centers
        zone_centers = {
            0: ((1020 + 1400) / 2, (1075 + 1355) / 2),
            1: ((675 + 920) / 2,  (1920 + 2115) / 2),
            2: ((1470 + 1752) / 2, (1920 + 2115) / 2),
        }

        self.zone_drop_counts = {0: 0, 1: 0, 2: 0}

        # 1. Consolidate all active crates (Detected + Currently Held)
        all_active = {c['id']: c for c in self.crates}
        for bot in self.bots.values():
            if bot.tracked_crate_id is not None and bot.tracked_crate_id not in all_active:
                bx = bot.pose[0] if bot.pose is not None else 1219.0
                by = bot.pose[1] if bot.pose is not None else 1000.0
                all_active[bot.tracked_crate_id] = {'id': bot.tracked_crate_id, 'x': bx, 'y': by}

        # 2. Group crates by Zone
        crates_by_zone = {0: [], 1: [], 2: []}
        for c in all_active.values():
            z = self.get_zone_for_crate(c['id'])
            # Ensure zone is valid (0-2)
            if z in crates_by_zone:
                crates_by_zone[z].append(c)

        # 3. Assign Slots per Zone
        for z, crates in crates_by_zone.items():
            if not crates: 
                continue

            cx, cy = zone_centers[z]
            
            # Sort crates by Y coordinate DESCENDING (Max Y first)
            sorted_crates = sorted(crates, key=lambda x: x['y'], reverse=True)
            
            # Define Slot Offsets
            # Slot 0: Left (-X)
            # Slot 1: Right (+X)
            # Slot 2: Center (-Y, Middle X)
            offsets_map = {
                0: (-spacing_x, 0.0),
                1: (+spacing_x, 0.0),
                2: (0.0, -spacing_y)
            }

            mapping = []

            # --- LOGIC UPDATE: If 3 crates, Max Y goes to Center (Slot 2) ---
            if len(sorted_crates) >= 3:
                # sorted_crates[0] is Max Y -> Assign to Slot 2 (Center)
                # sorted_crates[1] -> Assign to Slot 0 (Left)
                # sorted_crates[2] -> Assign to Slot 1 (Right)
                mapping = [
                    (sorted_crates[0], 2), 
                    (sorted_crates[1], 0), 
                    (sorted_crates[2], 1)
                ]
            else:
                # Fallback: If < 3 crates, fill Side Slots first (0, then 1)
                for i, crate in enumerate(sorted_crates):
                    mapping.append((crate, i))

            # Apply the Calculated Assignments
            for crate, slot_idx in mapping:
                ox, oy = offsets_map.get(slot_idx, (0.0, 0.0))
                
                self.drop_positions[crate['id']] = {
                    'x': cx + ox,
                    'y': cy + oy + 20, # Preserving your +20 offset
                    'zone_slot': slot_idx
                }
                self.zone_drop_counts[z] += 1

        self.get_logger().info(f"Drop positions assigned (Max Y rule applied): {self.drop_positions}")


    def get_pickup_zone_idx_for_crate(self, crate):
        """
        Returns pickup zone index (0–3) if crate lies in a pickup zone.
        Returns None if crate is not inside any pickup zone.
        """
        for i, z in enumerate(self.pickup_zones):
            if (z['x_min'] <= crate['x'] <= z['x_max'] and
                z['y_min'] <= crate['y'] <= z['y_max']):
                return i
        return None
    
    def get_filtered_pose(self, bot):
        # 1. Get Raw Data
        curr_x, curr_y, curr_t_deg = bot.pose
        curr_t = math.radians(curr_t_deg)

        # 2. Apply Filter if enabled (Alpha < 1.0)
        if bot.use_filter:
            # --- X FILTER ---
            if bot.x_filter_val is None: bot.x_filter_val = curr_x
            bot.x_filter_val += bot.filter_alpha * (curr_x - bot.x_filter_val)
            curr_x = bot.x_filter_val 

            # --- Y FILTER ---
            if bot.y_filter_val is None: bot.y_filter_val = curr_y
            bot.y_filter_val += bot.filter_alpha * (curr_y - bot.y_filter_val)
            curr_y = bot.y_filter_val 

            # --- THETA FILTER (With Wrap-Around) ---
            if bot.theta_filter_val is None:
                bot.theta_filter_val = curr_t
            else:
                diff = curr_t - bot.theta_filter_val
                diff = (diff + math.pi) % (2 * math.pi) - math.pi
                bot.theta_filter_val += bot.filter_alpha * diff
            curr_t = bot.theta_filter_val
        else:
            # Keep values synced for safety, even if not filtering
            bot.x_filter_val = curr_x
            bot.y_filter_val = curr_y
            bot.theta_filter_val = curr_t

        return curr_x, curr_y, curr_t


    # ---------------- Greedy assignment with "Lift-Triggered" Unblocking ----------------
    # ---------------- Greedy assignment with Priority for Bot 2 (P2) and Bot 4 (P1) ----------------
    # ---------------- Greedy assignment with "Lift-Triggered" Unblocking ----------------
    def assign_crates_greedily(self):
        all_completed = self.completed_crates

        crates_in_use = set()
        assigned_zones_this_cycle = set()
        
        # 1. Identify crates already being targeted or carried
        for b in self.bots.values():
            if b.current_crate is not None:
                crates_in_use.add(b.current_crate['id'])

                # --- FIXED LOGIC START ---
                # Only block the zone if the bot is still working on the pickup.
                # If b.arm_lifted is True, the bot has finished picking up and is leaving.
                # Therefore, we DO NOT add it to assigned_zones_this_cycle, allowing 
                # another bot to be assigned to this zone immediately.
                if not b.arm_lifted:
                    z = self.get_pickup_zone_idx_for_crate(b.current_crate)
                    if z is not None:
                        assigned_zones_this_cycle.add(z)
                # --- FIXED LOGIC END ---

            if b.tracked_crate_id is not None and b.magnet_state != 0:
                crates_in_use.add(b.tracked_crate_id)

        # 2. Create list of theoretically available crates
        unassigned_crates = [
            c for c in self.crates
            if c['id'] not in all_completed
            and c['id'] not in crates_in_use
        ]

        if not unassigned_crates:
            return 

        # 3. Get Idle Bots
        idle_bots = [
            b for b in self.bots.values()
            if b.current_crate is None
            and not b.has_returned_home
            and not b.is_permanently_idle
            and b.pose is not None
        ]

        if not idle_bots:
            return

        # Helper function to apply assignment
        def assign_bot_to_crate(bot, crate, label="Priority"):
            bot.current_crate = crate
            bot.tracked_crate_id = crate['id']
            bot.pickup_zone_idx = self.get_pickup_zone_idx_for_crate(crate)
            bot.waiting_for_pickup_zone = False
            bot.passed_gate = False
            bot.passed_pickup_staging = False

            # Reset Logic Flags
            bot.goal_reached = False
            bot.arm_placed = False
            bot.arm_lifted = False
            bot.move_after_attach = False
            bot.return_to_start = False
            bot.going_to_staging_point = False
            bot.magnet_state = 0
            bot.ir_trigger_time = None
            bot.arm_placed_logged = False
            bot.drop_pose_logged = False
            bot.wait_start_time = None
            bot.mag_timer_start = None
            bot.reset_pid()

            self.get_logger().info(
                f"[{label}] Crate {crate['id']} (Zone {bot.pickup_zone_idx}) → Bot {bot.id}"
            )

        # --- Priority 1: Assign P2 (Zone 1) to Bot 2 ---
        bot2 = next((b for b in idle_bots if b.id == 2), None)
        
        # Check if Zone 1 is free (not in assigned_zones_this_cycle)
        if bot2 and 1 not in assigned_zones_this_cycle:
            p2_crates = [c for c in unassigned_crates if self.get_pickup_zone_idx_for_crate(c) == 1]
            if p2_crates:
                p2_crates.sort(key=lambda c: c['y'])
                target_crate = p2_crates[0]
                
                assign_bot_to_crate(bot2, target_crate, label="P2->Bot2")
                
                idle_bots.remove(bot2)
                unassigned_crates.remove(target_crate)
                assigned_zones_this_cycle.add(1) # Mark Zone 1 taken for this cycle

        # --- Priority 2: Assign P1 (Zone 0) to Bot 4 ---
        bot4 = next((b for b in idle_bots if b.id == 4), None)
        
        # Check if Zone 0 is free
        if bot4 and 0 not in assigned_zones_this_cycle:
            p1_crates = [c for c in unassigned_crates if self.get_pickup_zone_idx_for_crate(c) == 0]
            if p1_crates:
                p1_crates.sort(key=lambda c: c['y'])
                target_crate = p1_crates[0]
                
                assign_bot_to_crate(bot4, target_crate, label="P1->Bot4")
                
                idle_bots.remove(bot4)
                unassigned_crates.remove(target_crate)
                assigned_zones_this_cycle.add(0) # Mark Zone 0 taken for this cycle

        # =========================================================================
        # STANDARD LOGIC (Remaining Bots & Crates)
        # =========================================================================

        # Group remaining crates by pickup zone
        zone_crates = {}
        for c in unassigned_crates:
            z = self.get_pickup_zone_idx_for_crate(c)
            if z is None: z = -1
            zone_crates.setdefault(z, []).append(c)

        # Process each pickup zone independently
        for zone_idx, crates in zone_crates.items():

            if zone_idx in assigned_zones_this_cycle:
                continue

            # Sort strategy
            # if zone_idx in [0, 1]:
            #     crates.sort(key=lambda c: c['y']) # Low Y first
            # else:
            #     crates.sort(key=lambda c: c['y'], reverse=True) # High Y first

            # [NEW CODE]
            # Sort strategy: ALWAYS Low Y first (for all zones 0, 1, 2, 3)
            crates.sort(key=lambda c: c['y'])

            for crate in crates:
                if not idle_bots:
                    return

                # Choose nearest idle bot
                best_bot = None
                best_dist = float('inf')

                for bot in idle_bots:
                    dist = math.hypot(crate['x'] - bot.pose[0], crate['y'] - bot.pose[1])
                    if dist < best_dist:
                        best_dist = dist
                        best_bot = bot

                if best_bot is not None:
                    assign_bot_to_crate(best_bot, crate, label="Standard")

                    idle_bots.remove(best_bot)
                    unassigned_crates.remove(crate)

                    # Block this zone for the rest of THIS specific cycle
                    # so we don't assign two bots to the same zone instantly.
                    assigned_zones_this_cycle.add(zone_idx)
                    break


            # Update Proximity Blocks for the remaining loop (so 2 idle bots don't grab neighbors)
            # nx, ny = nearest['x'], nearest['y']
            # for other_c in unassigned_crates:
            #     dist = math.hypot(other_c['x'] - nx, other_c['y'] - ny)
            #     if dist < 300.0:
            #         temporarily_blocked_ids.add(other_c['id'])

    # ---------------- Follow Behavior ----------------
    # ---------------- Follow Behavior (Fixed) ----------------
    # def check_and_handle_collisions(self):
    #     bots_to_check = [b for b in self.bots.values() if b.pose is not None]
    #     bots_that_should_follow = set()

    #     for bot_a, bot_b in itertools.combinations(bots_to_check, 2):
    #         if not bot_a.is_moving and not bot_b.is_moving:
    #             continue

    #         # 1. Calculate Distance
    #         dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])
            
    #         # 2. Determine Dynamic Threshold
    #         # Default safety distance
    #         current_threshold = self.follow_distance_threshold 

    #         # CHECK: Are they sharing a zone?
    #         if bot_a.is_sharing_zone and bot_b.is_sharing_zone:
    #             za = self.get_zone_for_crate(bot_a.tracked_crate_id) if bot_a.tracked_crate_id is not None else -1
    #             zb = self.get_zone_for_crate(bot_b.tracked_crate_id) if bot_b.tracked_crate_id is not None else -2
    #             if za == zb:
    #                 # If sharing, shrink the bubble to 160mm (Physical touch limit)
    #                 # instead of ignoring it completely. This prevents crossing crashes.
    #                 current_threshold = 160.0 

    #         # 3. Check against threshold
    #         if dist >= current_threshold:
    #             continue

    #         # 4. Collision Imminent - Decide Priority
    #         mag_a = getattr(bot_a, 'magnet_state', 0)
    #         mag_b = getattr(bot_b, 'magnet_state', 0)

    #         if mag_a != 0 and mag_b == 0:
    #             leader, follower = bot_a, bot_b
    #         elif mag_b != 0 and mag_a == 0:
    #             leader, follower = bot_b, bot_a
    #         else:
    #             dist_a = math.hypot(bot_a.pose[0]-bot_a.goal[0], bot_a.pose[1]-bot_a.goal[1]) if bot_a.goal is not None else float('inf')
    #             dist_b = math.hypot(bot_b.pose[0]-bot_b.goal[0], bot_b.pose[1]-bot_b.goal[1]) if bot_b.goal is not None else float('inf')
                
    #             if dist_a < dist_b:
    #                 leader, follower = bot_a, bot_b
    #             elif dist_b < dist_a:
    #                 leader, follower = bot_b, bot_a
    #             else:
    #                 leader, follower = (bot_a, bot_b) if bot_a.id > bot_b.id else (bot_b, bot_a)

    #         follower.is_following = True
    #         follower.follow_target_id = leader.id
    #         bots_that_should_follow.add(follower.id)

    #     for bot in bots_to_check:
    #         if bot.id not in bots_that_should_follow and bot.is_following:
    #             bot.is_following = False
    #             bot.follow_target_id = None


    # ---------------- Follow Behavior (Fixed: Distance Locking) ----------------

    def is_bot_physically_in_pickup_zone(self, bot: BotState):
        """Checks if the bot's current coordinates are inside ANY pickup zone."""
        if bot.pose is None: 
            return False
        
        x, y, _ = bot.pose
        
        # Check against all defined pickup zones
        for z in self.pickup_zones:
            # We add a small buffer (+50mm) to ensure we catch them 
            # even if they are just on the boundary
            if (z['x_min'] - 50 <= x <= z['x_max'] + 50 and 
                z['y_min'] - 50 <= y <= z['y_max'] + 50):
                return True
                
        return False
    # ---------------- Collision Avoidance (Fixed: Omnidirectional) ----------------
    def check_and_handle_collisions(self):
        bots_to_check = [b for b in self.bots.values() if b.pose is not None]
        bots_that_should_follow = set()

        for bot_a, bot_b in itertools.combinations(bots_to_check, 2):

            # 1. EXCLUSIONS: Only ignore completely parked/idle bots.
            if bot_a.has_returned_home or bot_a.is_permanently_idle:
                continue
            if bot_b.has_returned_home or bot_b.is_permanently_idle:
                continue

            # Active check: Are they trying to move?
            a_active = bot_a.is_moving or bot_a.return_to_start or bot_a.dropping or bot_a.backing_up
            b_active = bot_b.is_moving or bot_b.return_to_start or bot_b.dropping or bot_b.backing_up

            if not a_active and not b_active:
                continue

            # 2. Determine Dynamic Threshold
            # Base threshold
            current_threshold = self.follow_distance_threshold 
            
            # Expand threshold if magnets are active (carrying crates needs more space)
            if bot_a.magnet_state != 0 or bot_b.magnet_state != 0:
                current_threshold = 450.0

            # Shrink threshold ONLY if targeting the SAME zone (to allow packing)
            # [FIXED LOGIC] 1. Calculate the zones both bots are targeting
            za = self.get_zone_for_crate(bot_a.tracked_crate_id) if bot_a.tracked_crate_id is not None else -1
            zb = self.get_zone_for_crate(bot_b.tracked_crate_id) if bot_b.tracked_crate_id is not None else -2

            # 2. If they are targeting the SAME zone, reduce the safety distance
            if za == zb and za != -1:
                # Use a small threshold (160mm) so they can park side-by-side
                # without triggering collision avoidance.
                current_threshold = 300.0

            in_zone_a = self.is_bot_physically_in_pickup_zone(bot_a)
            in_zone_b = self.is_bot_physically_in_pickup_zone(bot_b)
            
            if in_zone_a or in_zone_b:
                current_threshold = 100.0

            # 3. Calculate Distance (Omnidirectional)
            dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])

            # 4. Check against threshold
            # FIX: Removed 'is_in_triangle' call. If they are close, we MUST react.
            if dist >= current_threshold:
                continue

            # ---------------- LOGIC KICKS IN HERE ----------------
            
            # Log the conflict
            # self.get_logger().warn(f"⚠ TRAFFIC: Bot {bot_a.id} & {bot_b.id} too close! (Dist: {dist:.0f}mm)")

            # 5. Collision Imminent - Decide Priority (Leader vs Follower)
            # mag_a = getattr(bot_a, 'magnet_state', 0)
            # mag_b = getattr(bot_b, 'magnet_state', 0)

            # 5. Collision Imminent - Decide Priority (Leader vs Follower)
            mag_a = getattr(bot_a, 'magnet_state', 0)
            mag_b = getattr(bot_b, 'magnet_state', 0)

            # Check if bots are inside the drop zone
            in_zone_a = self.is_in_drop_zone(bot_a)
            in_zone_b = self.is_in_drop_zone(bot_b)

            # --- PRIORITY RULES ---
            # Rule 1: "Let me out" -> Bot inside zone has priority over bot outside
            if in_zone_a and not in_zone_b:
                leader, follower = bot_a, bot_b
            elif in_zone_b and not in_zone_a:
                leader, follower = bot_b, bot_a
            
            # Rule 2: High Priority Status (Magnet / Backing Up)
            else:
                has_priority_a = (mag_a != 0) or bot_a.backing_up
                has_priority_b = (mag_b != 0) or bot_b.backing_up

                if has_priority_a and not has_priority_b:
                    leader, follower = bot_a, bot_b
                elif has_priority_b and not has_priority_a:
                    leader, follower = bot_b, bot_a
                else:
                    # Rule 3: Distance / ID Fallback
                    dist_a = math.hypot(bot_a.pose[0]-bot_a.goal[0], bot_a.pose[1]-bot_a.goal[1]) if bot_a.goal is not None else float('inf')
                    dist_b = math.hypot(bot_b.pose[0]-bot_b.goal[0], bot_b.pose[1]-bot_b.goal[1]) if bot_b.goal is not None else float('inf')
                    
                    if dist_a < dist_b:
                        leader, follower = bot_a, bot_b
                    elif dist_b < dist_a:
                        leader, follower = bot_b, bot_a
                    else:
                        leader, follower = (bot_a, bot_b) if bot_a.id > bot_b.id else (bot_b, bot_a)

            # Priority 1: Bot with Magnet (Carrying load) is Leader
            # if mag_a != 0 and mag_b == 0:
            #     leader, follower = bot_a, bot_b
            # elif mag_b != 0 and mag_a == 0:
            #     leader, follower = bot_b, bot_a
            # else:
            #     # Priority 2: Bot closer to its goal is Leader
            #     dist_a = math.hypot(bot_a.pose[0]-bot_a.goal[0], bot_a.pose[1]-bot_a.goal[1]) if bot_a.goal is not None else float('inf')
            #     dist_b = math.hypot(bot_b.pose[0]-bot_b.goal[0], bot_b.pose[1]-bot_b.goal[1]) if bot_b.goal is not None else float('inf')
                
            #     if dist_a < dist_b:
            #         leader, follower = bot_a, bot_b
            #     elif dist_b < dist_a:
            #         leader, follower = bot_b, bot_a
            #     else:
            #         # Tie-breaker: Higher ID is leader
            #         leader, follower = (bot_a, bot_b) if bot_a.id > bot_b.id else (bot_b, bot_a)

            has_priority_a = (mag_a != 0) or bot_a.backing_up
            has_priority_b = (mag_b != 0) or bot_b.backing_up

            # Priority 1: High Priority Status (Magnet or Backing Up) vs Low Priority
            if has_priority_a and not has_priority_b:
                leader, follower = bot_a, bot_b
            elif has_priority_b and not has_priority_a:
                leader, follower = bot_b, bot_a
            else:
                # Priority 2: If both have equal status (both busy or both empty), use distance to goal
                dist_a = math.hypot(bot_a.pose[0]-bot_a.goal[0], bot_a.pose[1]-bot_a.goal[1]) if bot_a.goal is not None else float('inf')
                dist_b = math.hypot(bot_b.pose[0]-bot_b.goal[0], bot_b.pose[1]-bot_b.goal[1]) if bot_b.goal is not None else float('inf')
                
                if dist_a < dist_b:
                    leader, follower = bot_a, bot_b
                elif dist_b < dist_a:
                    leader, follower = bot_b, bot_a
                else:
                    # Tie-breaker: Higher ID is leader
                    leader, follower = (bot_a, bot_b) if bot_a.id > bot_b.id else (bot_b, bot_a)

            # 6. Apply Follower Logic
            
            # LOCK DISTANCE: Prevents jitter. We lock the *current* distance 
            # so the follower stops exactly where it is relative to the leader.
            if not follower.is_following:
                # Clamp to avoid locking them inside each other (min 180mm)
                follower.locked_follow_dist = max(dist, 180.0)
                self.get_logger().warn(f"Bot {follower.id} LOCKED follow distance: {follower.locked_follow_dist:.1f}mm")

            follower.is_following = True
            follower.follow_target_id = leader.id

            # Apply speed scaling (Leader slows down slightly to be safe)
            leader.slowdown_scale = 0.75
            follower.slowdown_scale = 1.0

            bots_that_should_follow.add(follower.id)

        # 7. Cleanup: Reset bots that are no longer in collision range
        for bot in bots_to_check:
            if bot.id not in bots_that_should_follow and bot.is_following:
                bot.is_following = False
                bot.follow_target_id = None
                bot.locked_follow_dist = None 
                bot.slowdown_scale = 1.0

    # ---------------- Main timer callback ----------------
    # def control_cb(self):
    #     if len(self.crates) == 0:
    #         return

    #     now_wall = time.time()
    #     for bot in self.bots.values():
    #         bot.slowdown_scale = 1.0

    #     self.assign_crates_greedily()

    #     # ---------------- Idle and staging logic ----------------
    #     for bot in self.bots.values():
    #         if (bot.current_crate is None and not bot.return_to_start and
    #             not bot.move_after_attach and not bot.going_to_staging_point and
    #             not bot.has_returned_home and not bot.is_permanently_idle):

    #             if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
    #                 bot.going_to_staging_point = True
    #                 # Set staging goal based on ID
    #                 if bot.id == 0: staging_goal = self.staging_point_0
    #                 elif bot.id == 2: staging_goal = self.staging_point_2
    #                 elif bot.id == 4: staging_goal = self.staging_point_4
    #                 else: staging_goal = self.staging_point_0
    #                 bot.goal = staging_goal.copy()

    #             else:
    #                 bot.return_to_start = True
    #                 # Set return goal based on ID
    #                 if bot.id == 0: bot.return_goal = self.idle_home_point_0
    #                 elif bot.id == 2: bot.return_goal = self.idle_home_point_2
    #                 elif bot.id == 4: bot.return_goal = self.idle_home_point_4
    #                 else: bot.return_goal = self.idle_home_point_0
    #                 bot.goal = bot.return_goal.copy()

    #     self.check_and_handle_collisions()

    #     # ---------------- Per-bot state machine ----------------
    #     for bot in self.bots.values():
    #         if bot.pose is None:
    #             continue

    #         # Always update arm smooth
    #         self.update_arm_smooth(bot)

    #         # FOLLOW behavior
    #         if bot.is_following and bot.follow_target_id is not None:
    #             self.follow_bot(bot,bot.follow_target_id)
    #             continue

    #         # Handle staging
    #         if bot.going_to_staging_point:
    #             self.move_to_staging(bot)
    #             continue

    #         # Handle dropping
    #         if bot.dropping:
    #             self.drop_crate(bot)
    #             continue

    #         # Handle return home
    #         if bot.return_to_start:
    #             self.return_home(bot)
    #             continue

    #         # Skip if no crate
    #         if bot.current_crate is None:
    #             continue

    #         # ---------------- Magnet HOLD Transition Log ----------------
    #         if bot.magnet_state == 10 and bot.mag_timer_start is not None:
    #             if now_wall - bot.mag_timer_start >= 5.0:
    #                 bot.magnet_state = 9
    #                 bot.mag_timer_start = None
    #                 # LOG MATCHING REFERENCE
    #                 self.get_logger().info(f"Bot {bot.id} | Magnet switched to HOLD mode")

    #         # ---------------- Zone Locking ----------------
    #         if bot.goal_reached and not bot.arm_placed:
    #             zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
    #             locking_bot_id = self.zone_locks.get(zone_id)
                
    #             higher_priority_exists = False
    #             for other_bot in self.bots.values():
    #                 if (other_bot.id > bot.id and other_bot.current_crate is not None and
    #                     self.get_zone_for_crate(other_bot.tracked_crate_id) == zone_id):
    #                     higher_priority_exists = True
    #                     break

    #             if locking_bot_id is None:
    #                 if higher_priority_exists:
    #                     self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
    #                     continue
    #                 else:
    #                     self.zone_locks[zone_id] = bot.id
    #             elif locking_bot_id != bot.id:
    #                 self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
    #                 continue

    #         # ---------------- FSM Execution ----------------
    #         if not bot.goal_reached:
    #             self.move_near_crate(bot)

    #         elif not bot.arm_placed:
    #             # Arm DOWN + Magnet ON
    #             if bot.magnet_state == 0:
    #                 bot.magnet_state = 10
    #                 bot.mag_timer_start = now_wall

    #             base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
    #             bot.arm_target_base = base_down
    #             bot.arm_target_elbow = elbow_down

    #             arm_done = self.update_arm_smooth(bot)

    #             self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

    #             if arm_done and not bot.arm_placed_logged:
    #                 bot.arm_placed = True
    #                 bot.arm_placed_logged = True
    #                 # LOG MATCHING REFERENCE
    #                 self.get_logger().info(
    #                     f"Bot {bot.id} | Arm placed on crate (base={bot.base_angle:.1f}°, elbow={bot.elbow_angle:.1f}°)"
    #                 )
    #                 self.get_logger().info(f"Bot {bot.id} | Magnet pull-in started")

    #         elif bot.magnet_state != 9:
    #             self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
    #             continue

    #         elif not bot.arm_lifted:
    #             self.lift_arm_with_crate(bot)

    #         elif bot.move_after_attach:
    #             self.move_to_final_goal(bot)

    # ---------------- Main timer callback ----------------
    # ... previous methods (like on_mqtt_disconnect) ...

    def publish_perception_debug(self):
        # 1. Gather Zone Lock Data
        zone_info = {
            "locks": self.zone_locks,
            "pickup_zone_locks": self.pickup_zone_locks,
            "subparts": {k: [list(pt) for pt in v] for k, v in self.zone_subparts.items()} 
        }

        # 2. Gather Bot Data
        bots_data = {}
        for bid, bot in self.bots.items():
            if bot.pose is None: continue
            
            # Determine text status for readability
            status = "IDLE"
            if bot.return_to_start: status = "RETURNING_HOME"
            elif bot.going_to_staging_point: status = "STAGING"
            elif bot.dropping: status = "DROPPING"
            
            # --- PRIORITY STATUS OVERRIDES ---
            
            # 1. Waiting for Pickup Zone Lock
            elif bot.waiting_for_pickup_zone:
                status = f"WAITING_PICKUP_{bot.pickup_zone_idx}" 
            
            # 2. [NEW] Waiting for Drop Zone Lock (Gate Logic)
            elif bot.move_after_attach and not bot.passed_drop_gate:
                z = self.get_zone_for_crate(bot.tracked_crate_id) if bot.tracked_crate_id is not None else None
                if z is not None:
                    locker = self.zone_locks.get(z)
                    # If locked by someone else AND not sharing -> waiting
                    if locker is not None and locker != bot.id and not bot.is_sharing_zone:
                         # Use distance to confirm we are actually waiting there
                         gx, gy = self.drop_gates.get(z, (0,0))
                         d = math.hypot(gx - bot.pose[0], gy - bot.pose[1])
                         if d < 150.0:
                            status = f"WAITING_DROP_{z}" # Triggers Red Text

            # 3. Following another bot
            elif bot.is_following:
                status = f"FOLLOWING_{bot.follow_target_id}"
            
            # 4. Normal Targeting
            elif bot.current_crate: 
                status = f"TARGETING_{bot.tracked_crate_id}"

            # --- DYNAMIC RADIUS LOGIC ---
            # This matches your collision avoidance logic exactly
            # --- DYNAMIC RADIUS LOGIC ---
            # 1. NEW: Check Pickup Zone (Highest Priority -> Smallest Radius)
            if self.is_bot_physically_in_pickup_zone(bot):
                viz_radius = 100.0  # Small circle to allow entry/exit in tight zones

            # 2. Check Shared Drop Zone

            # 3. Check Magnet (Carrying Load)
            elif bot.magnet_state != 0:
                viz_radius = 450.0  # Large circle for carrying crates

            # 4. Default Safety Bubble
            else:
                viz_radius = self.follow_distance_threshold  # Default (350mm)  # Default (350mm)

            bots_data[bid] = {
                "pose": list(bot.pose),
                "goal": list(bot.goal) if bot.goal is not None else [],
                "velocity": bot.velocity,
                "magnet": bot.magnet_state,
                "status": status,
                "is_following": bot.is_following,
                "carrying_crate": bot.current_crate['id'] if bot.current_crate else None,
                "pickup_zone": bot.pickup_zone_idx,
                "waiting_for_pickup_zone": bot.waiting_for_pickup_zone,
                "radius": viz_radius  # Data for the Perception Script to draw the circle
            }

        # 3. Compile final packet
        debug_msg = {
            "timestamp": time.time(),
            "drop_zones": zone_info,
            "bots": bots_data,
            "drop_positions": self.drop_positions,
            "params": {
                "follow_dist": self.follow_distance_threshold
            }
        }
        # 4. Publish to ROS2 Topic
        msg = String()
        msg.data = json.dumps(debug_msg)
        self.debug_pub.publish(msg)

    def control_cb(self):
        # [REMOVED] The line below was causing the freeze:
        # if len(self.crates) == 0:
        #    return

        if not self.is_warmed_up:
            elapsed = time.time() - self.node_start_time
            if elapsed < self.startup_delay:
                # Log every 1 second so we know it's waiting
                self.get_logger().info(
                    f"Waiting for vision system... ({elapsed:.1f}/{self.startup_delay:.1f}s)", 
                    throttle_duration_sec=1.0
                )
                return # Exit callback here; do not assign tasks yet
            else:
                self.is_warmed_up = True
                self.get_logger().info("Vision warmup complete. Starting Logic!")

        # 1. Update timing and reset slowdown scales
        now_wall = time.time()
        for bot in self.bots.values():
            bot.slowdown_scale = 1.0

        # 2. Try to assign any visible crates to idle bots
        #    (This function safely handles empty crate lists internally)
        self.assign_crates_greedily()

        # ---------------- Idle and staging logic ----------------
        for bot in self.bots.values():
            # Only send idle bots home/staging if they aren't doing anything else
            if (bot.current_crate is None and not bot.return_to_start and
                not bot.move_after_attach and not bot.going_to_staging_point and
                not bot.has_returned_home and not bot.is_permanently_idle):

                if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
                    bot.going_to_staging_point = True
                    
                    # [SMARTER STAGING LOGIC]
                    if bot.id == 0:
                        # If on right side, go right staging
                        if bot.pose[0] > 1219.0: staging_goal = self.sp_0_alt.copy()
                        else: staging_goal = self.staging_point_0.copy()
                    
                    elif bot.id == 2:
                        staging_goal = self.staging_point_2.copy()
                    
                    elif bot.id == 4:
                        if bot.pose[0] > 1219.0: staging_goal = self.sp_4_alt.copy()
                        else: staging_goal = self.staging_point_4.copy()
                    else:
                        staging_goal = self.staging_point_0.copy()
                    
                    bot.goal = staging_goal
        # 3. Collision Avoidance
        self.check_and_handle_collisions()

        # ---------------- Per-bot state machine ----------------
        for bot in self.bots.values():
            if bot.pose is None:
                continue

            # Always update arm smooth
            self.update_arm_smooth(bot)

            # FOLLOW behavior
            if bot.is_following and bot.follow_target_id is not None:
                self.follow_bot(bot,bot.follow_target_id)
                continue

            if bot.backing_up:
                self.handle_backup(bot)
                continue

            # Handle staging
            if bot.going_to_staging_point:
                self.move_to_staging(bot)
                continue

            # Handle dropping
            if bot.dropping:
                self.drop_crate(bot)
                continue

            # Handle return home
            if bot.return_to_start:
                self.return_home(bot)
                continue

            # Skip if this specific bot has no crate assigned
            if bot.current_crate is None:
                continue

            # ---------------- Magnet HOLD Transition Log ----------------
            if bot.magnet_state == 10 and bot.mag_timer_start is not None:
                if now_wall - bot.mag_timer_start >= 5.0:
                    bot.magnet_state = 10
                    bot.mag_timer_start = None
                    self.get_logger().info(f"Bot {bot.id} | Magnet switched to HOLD mode")

            # ---------------- STRICT ZONE LOCKING ----------------
            # Check this when the bot has picked up the crate (goal_reached) 
            # but hasn't placed/dropped it yet.
            if bot.goal_reached and not bot.arm_placed:
                should_wait_for_center = False
                if bot.tracked_crate_id is not None:
                    d_pos = self.drop_positions.get(bot.tracked_crate_id)
                    # Check if this crate goes to Slot 2 (Center)
                    if d_pos and d_pos.get('zone_slot') == 2:
                        z_id = self.get_zone_for_crate(bot.tracked_crate_id)
                        
                        # Count how many crates are done in this zone
                        completed_count = 0
                        for c_id in self.completed_crates:
                            if self.get_zone_for_crate(c_id) == z_id:
                                completed_count += 1
                        
                        # If neighbors aren't ready, we must wait
                        if completed_count < 2:
                            should_wait_for_center = True
                            
                if should_wait_for_center:
                    # STOP HERE. Do not lock zone. Do not lower arm.
                    self.get_logger().info(
                        f"Bot {bot.id} | Waiting at Pickup. Center slot needs 2 placed, found {completed_count}.",
                        throttle_duration_sec=2.0
                    )
                    # Hold position, Magnet OFF (0) because we haven't picked up yet
                    self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)
                    continue # Skip the rest of the loop (prevents locking)
                zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
                locking_bot_id = self.zone_locks.get(zone_id)
                
                # Check if a higher ID bot also wants this zone (Priority Logic)
                higher_priority_exists = False
                for other_bot in self.bots.values():
                    if other_bot.id > bot.id and other_bot.current_crate is not None:
                        if self.get_zone_for_crate(other_bot.tracked_crate_id) == zone_id:
                            other_drop = self.drop_positions.get(other_bot.tracked_crate_id)
                            if other_drop and other_drop.get('zone_slot') == 2:
                                continue
                            higher_priority_exists = True
                            break

                # CASE 1: Zone is Free
                if locking_bot_id is None:
                    # If someone with higher priority also wants it, we wait (soft wait)
                    if higher_priority_exists:
                        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                        continue 
                    else:
                        # Lock it for myself
                        self.zone_locks[zone_id] = bot.id
                        self.get_logger().info(f"Bot {bot.id} LOCKED Drop Zone {zone_id}")

                # CASE 2: Zone is Locked by SOMEONE ELSE
                elif locking_bot_id != bot.id:
                    # WAIT HERE. Do not proceed to FSM.
                    # This prevents the bot from moving to 'place_arm' or 'move_to_final'
                    self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                    continue
            # ---------------- FSM Execution ----------------
            # if not bot.goal_reached:
            #     self.move_near_crate(bot)

            # elif not bot.arm_placed:
            #     # Arm DOWN + Magnet ON
            #     if bot.magnet_state == 0:
            #         bot.magnet_state = 10
            #         bot.mag_timer_start = now_wall

            #     base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
            #     bot.arm_target_elbow = elbow_down
            #     elbow_is_positioned = abs(bot.elbow_angle - elbow_down) < 2.0
            #     if not elbow_is_positioned:
            #         # If Elbow is still moving, keep Base UP (at travel angle 45.0)
            #         bot.arm_target_base = 45.0
            #     else:
            #         # Once Elbow is ready, allow Base to go DOWN
            #         bot.arm_target_base = base_down

            #     arm_done = self.update_arm_smooth(bot)

            #     self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

            #     if arm_done and not bot.arm_placed_logged:
            #         bot.arm_placed = True
            #         bot.arm_placed_logged = True
            #         bot.wait_start_time = time.time()
            #         self.get_logger().info(
            #             f"Bot {bot.id} | Arm placed on crate (base={bot.base_angle:.1f}°, elbow={bot.elbow_angle:.1f}°)"
            #         )
            #         self.get_logger().info(f"Bot {bot.id} | Magnet pull-in started")

            # elif bot.magnet_state != 10:
            #     self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
            #     continue

            # elif not bot.arm_lifted:

            #     if bot.wait_start_time is not None:
            #         if time.time() - bot.wait_start_time < 2.0:
            #             # Hold position and wait
            #             self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
            #             self.get_logger().info(
            #             f"Waiting to pick up the crate"
            #         )
            #             continue # Skip the rest of the loop
            #         else:
            #             bot.wait_start_time = None # Timer finished

            #     self.lift_arm_with_crate(bot)

            # elif bot.move_after_attach:
            #     self.move_to_final_goal(bot)

            # ---------------- FSM Execution ----------------
            if not bot.goal_reached:
                self.move_near_crate(bot)

            elif not bot.arm_placed:
                # 1. Arm DOWN + Magnet ON (State 10 = Pull)
                if bot.magnet_state == 0:
                    bot.magnet_state = 10
                    bot.mag_timer_start = now_wall

                base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
                
                # Logic: Keep Base UP (45) until Elbow is down (near 180)
                bot.arm_target_elbow = elbow_down
                elbow_is_positioned = abs(bot.elbow_angle - elbow_down) < 2.0
                
                if not elbow_is_positioned:
                    bot.arm_target_base = 45.0
                else:
                    bot.arm_target_base = base_down

                arm_done = self.update_arm_smooth(bot)

                # Keep robot still while arm moves
                self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

                if arm_done and not bot.arm_placed_logged:
                    bot.arm_placed = True
                    bot.arm_placed_logged = True
                    # START THE WAIT TIMER HERE
                    bot.wait_start_time = time.time() 
                    self.get_logger().info(f"Bot {bot.id} | Arm placed. Magnet ON. Waiting 2.0s...")

            # REMOVED THE BLOCKING "elif bot.magnet_state != 10" HERE

            elif not bot.arm_lifted:
                # 2. Wait Logic (The "Settle" Time)
                if bot.wait_start_time is not None:
                    if time.time() - bot.wait_start_time < 2.0:
                        # ACTIVE WAIT: Keep Magnet ON, Hold Position
                        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 10)
                        continue # STOP HERE, DO NOT LIFT YET
                    else:
                        bot.wait_start_time = None # Timer finished, proceed to lift

                # 3. Lift Logic
                self.lift_arm_with_crate(bot)

            elif bot.move_after_attach:
                self.move_to_final_goal(bot)

    def attach_service_cb(self, request, response):
        """
        /attach service
        Returns True if ANY bot currently has magnet ON
        Returns False if ALL magnets are OFF
        """

        magnet_on = False

        for bot in self.bots.values():
            # Magnet ON states in your code are 10 (pull) and 9 (hold)
            if bot.magnet_state != 0:
                magnet_on = True
                break

        response.success = magnet_on
        response.message = "Magnet ON" if magnet_on else "Magnet OFF"
        return response


    # ---------------- Helper: Triangle Inclusion Check (Updated for Circle) ----------------
    # ---------------- Helper: Triangle Inclusion Check (Arrowhead / Wedge) ----------------
    def is_in_triangle(self, observer_bot, target_bot, r):
        """
        Checks if target_bot's CIRCLE intersects a Safety Wedge (Arrowhead)
        extending from observer_bot.
        
        Shape: Base at x=0 (width=r), Tip at x=height.
        """
        if observer_bot.pose is None or target_bot.pose is None: return False

        cx, cy, c_theta_deg = observer_bot.pose
        tx, ty, _ = target_bot.pose

        # 1. Translate & Rotate into observer's local frame
        dx = tx - cx
        dy = ty - cy
        theta_rad = math.radians(c_theta_deg+90)
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)

        local_x = dx * cos_t + dy * sin_t
        local_y = -dx * sin_t + dy * cos_t

        # 2. Geometric Check: Circle (radius B) entering Wedge
        bubble = self.bot_radius
        sqrt3 = 1.73205
        h = (sqrt3 / 2) * r

        # Back face
        if local_x < (-h/3 - bubble):
            return False

        # Slanted sides (equilateral geometry)
        max_y = (h - (local_x + h/3) + bubble) / sqrt3
        if abs(local_y) > max_y:
            return False



        return True

    # ---------------- Follow Bot (Fixed: Tighter Deadband) ----------------
    def follow_bot(self, follower: BotState, leader_id: int):
        
        # 1. SAFETY CHECK (UNCHANGED)
        if (follower.has_returned_home or follower.is_permanently_idle):
            follower.is_following = False
            follower.follow_target_id = None
            self.publish_wheel_velocities([0.0,0.0,0.0], follower.id, follower.base_angle, follower.elbow_angle, follower.magnet_state)
            return

        if leader_id not in self.bots: return
        leader = self.bots[leader_id]
        if leader.pose is None or follower.pose is None: return

        follower.is_moving = True

        # Calculate vector and current distance to leader
        dx = leader.pose[0] - follower.pose[0]
        dy = leader.pose[1] - follower.pose[1]
        dist = math.hypot(dx, dy)

        # Retrieve the distance we locked
        target_dist = follower.locked_follow_dist if follower.locked_follow_dist else 350.0

        # ---------------- CHANGE: TIGHTER DEADBAND (50mm instead of 200mm) ----------------
        # If the error is greater than 50mm, we must move (either catch up or back away).
        if abs(dist - target_dist) < 50.0:
            
            self.get_logger().info(
                f"[FOLLOW] Bot {follower.id} STOPPED | "
                f"Leader {leader_id} Dist: {dist:.1f}mm (Target: {target_dist:.1f}) | "
                f"Cmd: [0, 0, 0] (Stable)", 
                throttle_duration_sec=0.5
            )
            
            self.publish_wheel_velocities([0.0, 0.0, 0.0], follower.id, follower.base_angle, follower.elbow_angle, follower.magnet_state)
            follower.is_moving = False
            return
        # ----------------------------------------------------------------------------------

        # Move to maintain distance
        ux, uy = dx/dist, dy/dist
        target_x = leader.pose[0] - ux * target_dist
        target_y = leader.pose[1] - uy * target_dist
        
        action = "BACKING UP" if dist < target_dist else "CATCHING UP"

        self.get_logger().info(
            f"[FOLLOW] Bot {follower.id} {action} | "
            f"Leader {leader_id} Dist: {dist:.1f}mm (Target: {target_dist:.1f}) | "
            f"Moving to: ({target_x:.1f}, {target_y:.1f})", 
            throttle_duration_sec=0.5
        )

        self.move_to_point(follower, np.array([target_x, target_y, 0.0]), "Maintain Lock")

    # ---------------- Smooth Arm Update ----------------
    def update_arm_smooth(self, bot: BotState):
        done = True

        def move(curr, target):
            if abs(curr - target) <= bot.arm_step:
                return target, True
            return (
                curr + bot.arm_step if curr < target else curr - bot.arm_step,
                False
            )

        bot.base_angle, b_done = move(bot.base_angle, bot.arm_target_base)
        bot.elbow_angle, e_done = move(bot.elbow_angle, bot.arm_target_elbow)

        return b_done and e_done
    
    # ---------------- OBSTACLE AVOIDANCE HELPER (UPDATED) ----------------
    def apply_crate_avoidance(self, bot, vx_pid, vy_pid):
        """
        Adjusts PID velocity by adding repulsive vectors.
        FIX: Now ignores crates that are in the same zone the bot is targeting.
        UPDATE: Disables avoidance if bot is within 300mm of its goal.
        """
        
        # ---------------- NEW CHECK: Distance to Goal ----------------
        if bot.pose is not None and bot.goal is not None:
            dist_to_goal = math.hypot(bot.goal[0] - bot.pose[0], bot.goal[1] - bot.pose[1])
            
            # If we are close to the goal (docking/dropping), disable avoidance
            # so the bot doesn't get pushed away from its target.
            if dist_to_goal < 300.0:
                return vx_pid, vy_pid
        # -------------------------------------------------------------

        avoid_radius = 300.0    
        repulsion_gain = 3.0    
        
        total_push_x = 0.0
        total_push_y = 0.0

        # 1. Identify which zone this bot is currently targeting
        target_zone = -1
        if bot.tracked_crate_id is not None:
            target_zone = self.get_zone_for_crate(bot.tracked_crate_id)

        # 2. Avoid CRATES
        for crate in self.crates:
            # A. Do not avoid the crate assigned to this bot!
            if bot.tracked_crate_id is not None and crate['id'] == bot.tracked_crate_id:
                continue

            # # B. Do not avoid crates that belong to my target zone.
            # if target_zone != -1:
            #     obstacle_zone = self.get_zone_for_crate(crate['id'])
            #     if obstacle_zone == target_zone:
            #         continue

            # Calculate Repulsion
            dx = bot.pose[0] - crate['x']
            dy = bot.pose[1] - crate['y']
            dist = math.hypot(dx, dy)

            if dist < avoid_radius and dist > 1.0:
                strength = (1.0 - (dist / avoid_radius)) * repulsion_gain
                push_x = (dx / dist) * strength * self.max_vel
                push_y = (dy / dist) * strength * self.max_vel
                total_push_x += push_x
                total_push_y += push_y

        # 3. Avoid HOME BOTS (Treat them as static obstacles)
        for other_bot in self.bots.values():
            if other_bot.id == bot.id: continue
            if other_bot.pose is None: continue

            # Only repel if the other bot is parked at home/idle
            if other_bot.has_returned_home or other_bot.is_permanently_idle:
                dx = bot.pose[0] - other_bot.pose[0]
                dy = bot.pose[1] - other_bot.pose[1]
                dist = math.hypot(dx, dy)

                if dist < avoid_radius and dist > 1.0:
                    strength = (1.0 - (dist / avoid_radius)) * repulsion_gain
                    push_x = (dx / dist) * strength * self.max_vel
                    push_y = (dy / dist) * strength * self.max_vel
                    
                    total_push_x += push_x
                    total_push_y += push_y

        final_vx = vx_pid + total_push_x
        final_vy = vy_pid + total_push_y

        return final_vx, final_vy
    
    def get_safe_highway_waypoint(self, bot_x, bot_y, zone_idx):
        """
        Determines if the bot is 'behind' the zone and needs to escape to the center highway (X=1200)
        before approaching the gate.
        """
        zone = self.pickup_zones[zone_idx]
        gate_x, gate_y = zone['gate']
        
        # Check if bot is in the "X Shadow" of the zone (plus padding)
        in_x_shadow = (zone['x_min'] - 100 <= bot_x <= zone['x_max'] + 100)
        if not in_x_shadow:
            return None # Safe to move directly to gate

        # Check if bot is on the "wrong side" vertically
        # For P1/P2 (Entry 560): Danger if Bot Y > 560 (e.g. at 1500)
        if zone['y_min'] == 560 and bot_y > zone['y_min']:
             return (1200.0, bot_y)

        # For P3/P4 (Entry 1350): Danger if Bot Y > 1350? 
        # Actually P3 gate is at 1270. If bot is at 500, it approaches from 500->1270. 
        # This approach hits the FRONT of P3. It is safe.
        # But if we assume zones block travel, we might want highway anyway.
        # Let's be safe: If stuck in X shadow, always go highway unless very close to gate Y.
        if abs(bot_y - gate_y) > 200:
             return (1200.0, bot_y)

        return None

    def is_bot_in_zone(self, bot_x, bot_y, zone_idx):
        z = self.pickup_zones[zone_idx]
        return (z['x_min']-50 <= bot_x <= z['x_max']+50 and 
                z['y_min']-50 <= bot_y <= z['y_max']+50)
    # ----------------------------------------------------------

    # def move_near_crate(self,bot:BotState):
    #     bot.is_moving = True
    #     crate = bot.current_crate
    #     if crate is None: return

    #     if bot.id==0:
    #         offset_x=30
    #         offset_y=120
    #     elif bot.id==2:
    #         offset_x=30
    #         offset_y=120
    #     elif bot.id==4:
    #         # offset_x=40
    #         # offset_y=120
    #         offset_x=26
    #         offset_y=128
        
    #     bot.arm_target_base = 45.0
    #     bot.arm_target_elbow = 135.0

    #     goal_x = crate['x'] - offset_x
    #     goal_y = crate['y'] - offset_y
    #     bot.goal = np.array([goal_x,goal_y,0.0])

    #     # Log distance similar to reference
    #     self.log_bot_distance(bot, goal_x, goal_y, "crate approach point")

    #     now = self.get_clock().now()
    #     dt = (now - bot.last_time).nanoseconds / 1e9 if bot.last_time else 0.03
    #     if dt <= 0: return

    #     x,y,theta_deg = bot.pose
    #     theta = math.radians(theta_deg)

    #     vx = bot.pid_x.compute(goal_x - x,dt)
    #     vy = bot.pid_y.compute(goal_y - y,dt)
        
    #     dist = math.hypot(goal_x - x, goal_y - y)
    #     if dist < self.slow_radius:
    #         bot.slowdown_scale = max(self.min_slow_scale, dist / self.slow_radius)

    #     goal_theta = 0.0
    #     error_theta = (goal_theta - theta + math.pi) % (2*math.pi) - math.pi
    #     omega = bot.pid_theta.compute(error_theta, dt)

    #     vx *= bot.slowdown_scale
    #     vy *= bot.slowdown_scale
    #     omega *= bot.slowdown_scale

    #     # OPTIMIZED KINEMATICS (Use self.M_inv)
    #     cos_t, sin_t = math.cos(theta), math.sin(theta)
    #     vx_r = vx * cos_t + vy * sin_t
    #     vy_r = -vx * sin_t + vy * cos_t
        
    #     # USE PRE-CALCULATED MATRIX
    #     wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])
        
    #     self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, 0)

    #     if math.hypot(goal_x-x,goal_y-y) < self.xy_tolerance:
    #         self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)
    #         bot.is_moving = False
    #         bot.goal_reached = True
    #         # LOG MATCHING REFERENCE
    #         self.get_logger().info(f"Bot {bot.id} | Reached near crate at (x={goal_x:.1f}, y={goal_y:.1f})")
    #         self.get_logger().info(f"Bot {bot.id} | Preparing to place arm on the crate...")

    # Replace the existing move_near_crate method with this:
    # def move_near_crate(self, bot: BotState):
    #     bot.is_moving = True
    #     crate = bot.current_crate
    #     if crate is None: return

    #     # ... [Keep your existing offset logic] ...
    #     if bot.id==0:
    #         offset_x=30; offset_y=120
    #     elif bot.id==2:
    #         offset_x=30; offset_y=120
    #     elif bot.id==4:
    #         offset_x=26; offset_y=128
        
    #     bot.arm_target_base = 45.0
    #     bot.arm_target_elbow = 135.0

    #     goal_x = crate['x'] - offset_x
    #     goal_y = crate['y'] - offset_y
    #     bot.goal = np.array([goal_x, goal_y, 0.0])

    #     # ... [Keep PID calculation setup] ...
    #     now = self.get_clock().now()
    #     dt = (now - bot.last_time).nanoseconds / 1e9 if bot.last_time else 0.03
    #     if dt <= 0: return

    #     x, y, theta_deg = bot.pose
        
    #     # Calculate Distance to visual target
    #     dist_error = math.hypot(goal_x - x, goal_y - y)

    #     # ---------------- NEW: STOP CONDITION ----------------
    #     # Stop if:
    #     # 1. We are within vision tolerance (self.xy_tolerance)
    #     # 2. OR The IR sensor detects the crate (bot.ir_detected is True)
        
    #     # We add a safety check (dist_error < 300) so the IR sensor doesn't 
    #     # trigger a stop if the robot is falsely triggered while far away.
    #     ir_triggered = bot.ir_detected and (dist_error < 300.0)

    #     if dist_error < self.xy_tolerance or ir_triggered:
    #         self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)
    #         bot.is_moving = False
    #         bot.goal_reached = True
            
    #         if ir_triggered:
    #             self.get_logger().info(f"Bot {bot.id} | STOPPED: IR Sensor Detected Crate!")
    #         else:
    #             self.get_logger().info(f"Bot {bot.id} | STOPPED: Vision Target Reached.")
                
    #         self.get_logger().info(f"Bot {bot.id} | Preparing to place arm...")
    #         return
    #     # -----------------------------------------------------

    #     # If not stopped, continue PID Control
    #     vx = bot.pid_x.compute(goal_x - x, dt)
    #     vy = bot.pid_y.compute(goal_y - y, dt)
        
    #     if dist_error < self.slow_radius:
    #         bot.slowdown_scale = max(self.min_slow_scale, dist_error / self.slow_radius)

    #     theta = math.radians(theta_deg)
    #     goal_theta = 0.0
    #     error_theta = (goal_theta - theta + math.pi) % (2*math.pi) - math.pi
    #     omega = bot.pid_theta.compute(error_theta, dt)

    #     vx *= bot.slowdown_scale
    #     vy *= bot.slowdown_scale
    #     omega *= bot.slowdown_scale

    #     cos_t, sin_t = math.cos(theta), math.sin(theta)
    #     vx_r = vx * cos_t + vy * sin_t
    #     vy_r = -vx * sin_t + vy * cos_t
        
    #     wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])
        
    #     self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, 0)

    # Replace the existing move_near_crate method with this:
    def move_near_crate(self, bot: BotState):
        bot.is_moving = True
        crate = bot.current_crate
        if crate is None: return

        # 1. SETUP TARGETS
        # ----------------
        if bot.id == 0:   ox, oy = 25, 120
        elif bot.id == 2: ox, oy = 20, 120
        elif bot.id == 4: ox, oy = 25, 125
        else:             ox, oy = 25, 120
        
        # Calculate the final crate position
        final_x = crate['x'] - ox
        final_y = crate['y'] - oy
        
        # Identify Target Zone
        target_zone_idx = -1
        for i, z in enumerate(self.pickup_zones):
            if (z['x_min'] <= crate['x'] <= z['x_max'] and 
                z['y_min'] <= crate['y'] <= z['y_max']):
                target_zone_idx = i
                break

        if bot.pickup_zone_idx is None:
            bot.pickup_zone_idx = target_zone_idx

        if bot.pickup_zone_idx is not None and not bot.passed_gate:
            locking_bot = self.pickup_zone_locks[bot.pickup_zone_idx]

            # Zone is free → acquire lock
            if locking_bot is None:
                self.pickup_zone_locks[bot.pickup_zone_idx] = bot.id
                bot.waiting_for_pickup_zone = False

            # Zone locked by another bot → WAIT OUTSIDE
            elif locking_bot != bot.id:
                bot.waiting_for_pickup_zone = True
                bot.is_moving = False

                # HARD STOP
                self.publish_wheel_velocities(
                    [0.0, 0.0, 0.0],
                    bot.id,
                    bot.base_angle,
                    bot.elbow_angle,
                    bot.magnet_state
                )

                bot.pid_last_time = None

                self.get_logger().warn(
                    f"Bot {bot.id} | Waiting for Pickup Zone {bot.pickup_zone_idx} "
                    f"(Locked by Bot {locking_bot})"
                )
                return

        # Default Target is the Crate
        target_x, target_y = final_x, final_y
        label = "Crate Approach"
        
        # 2. PATH ROUTING LOGIC
        if target_zone_idx != -1 and not bot.passed_gate:

            # --- STAGING LOGIC ---
            if not bot.passed_pickup_staging:
                # Force target to staging point
                s_pt = self.pickup_staging_points.get(target_zone_idx)
                if s_pt:
                    target_x, target_y = s_pt
                    label = f"Pickup Staging {target_zone_idx}"
                    
                    # Check arrival (Tolerance 100mm)
                    if math.hypot(target_x - bot.pose[0], target_y - bot.pose[1]) < 100.0:
                        bot.passed_pickup_staging = True
                        self.get_logger().info(f"Bot {bot.id} reached Pickup Staging {target_zone_idx}")

            # --- GATE & HIGHWAY LOGIC (Must be INSIDE this check) ---
            if bot.passed_pickup_staging:            
                gate_pt = self.pickup_zones[target_zone_idx]['gate']
            
                # INDENTED: Check A: Do we need to escape to the highway?
                highway_pt = self.get_safe_highway_waypoint(bot.pose[0], bot.pose[1], target_zone_idx)
                
                if highway_pt is not None:
                    # OVERRIDE: Go to Highway (X=1200)
                    target_x, target_y = highway_pt
                    label = "Highway Escape"
                    
                    if abs(bot.pose[0] - 1200.0) < 150.0:
                        pass 
                else:
                    # INDENTED: Check B: Go to Entry Gate
                    target_x, target_y = gate_pt  # <--- Safe now because we are inside the block
                    label = "Gate Approach"
                    
                    dist_gate = math.hypot(target_x - bot.pose[0], target_y - bot.pose[1])
                    inside_zone = self.is_bot_in_zone(bot.pose[0], bot.pose[1], target_zone_idx)
                    
                    if dist_gate < 80.0 or inside_zone:
                        bot.passed_gate = True
                        self.get_logger().info(f"Bot {bot.id} | Cleared Gate. Entering Zone.")

                # # if bot.pickup_zone_idx is not None:
                # #     if self.pickup_zone_locks.get(bot.pickup_zone_idx) == bot.id:
                # #         self.pickup_zone_locks[bot.pickup_zone_idx] = None
                #         self.get_logger().info(
                #             f"Bot {bot.id} | Released Pickup Zone {bot.pickup_zone_idx}"
                #         )

        # 3. SET GOAL & ARM
        # -----------------
        bot.goal = np.array([target_x, target_y, 0.0])
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        now = self.get_clock().now()
        
        # If this is the first loop, or we just came from a reset/wait, use default dt
        if bot.pid_last_time is None:
            dt = 0.03 
        else:
            dt = (now - bot.pid_last_time).nanoseconds / 1e9
        
        # Update the timer for the next loop
        bot.pid_last_time = now

        # Clamp dt to prevent derivative spikes on lag
        if dt <= 0.001: dt = 0.03
        if dt > 0.1:    dt = 0.1

        # ---------------- MODIFIED START ----------------
        # 1. Use the helper to get filtered values (Theta is in RADIANS)
        x, y, theta = self.get_filtered_pose(bot) 
        
        # 2. Convert to degrees immediately so your existing logic (theta_error_deg) works
        theta_deg = math.degrees(theta) 
        # ---------------- MODIFIED END ------------------
        dist_error = math.hypot(target_x - x, target_y - y)

        goal_theta = 0.0  # desired orientation
        theta_error_deg = abs(
            (goal_theta - theta_deg + 180.0) % 360.0 - 180.0
        )

        
        self.log_bot_distance(bot, target_x, target_y, label)

        # 4. STOP CONDITION (Only active if targeting Crate)
        # --------------------------------------------------
        is_final_leg = (target_zone_idx == -1) or bot.passed_gate

        x_ok = False

        if is_final_leg:
            # IR Detection Logic
            if bot.ir_detected and bot.ir_trigger_time is None:
                if dist_error < 50.0 and theta_error_deg < 15.0:
                    bot.ir_trigger_time = time.time()
                    bot.pid_y.reset()
                    self.get_logger().info(f"Bot {bot.id} | IR Triggered! Pushing...")

            ir_timeout_reached = False
            if bot.ir_trigger_time is not None:
                if time.time() - bot.ir_trigger_time >= bot.post_ir_duration:
                    ir_timeout_reached = True

            # if dist_error < self.xy_tolerance or ir_timeout_reached:
            #     self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)
            #     bot.is_moving = False
            #     bot.goal_reached = True
            #     self.get_logger().info(f"Bot {bot.id} | Reached Crate.")
            #     return

            dx_error = abs(target_x - x)
            dy_error = abs(target_y - y)

            radial_ok = dist_error < self.xy_tolerance
            x_ok = dx_error < self.xy_tolerance_x
            theta_ok = theta_error_deg < self.xy_tolerance_theta

            if x_ok and theta_ok and ((radial_ok ) or ir_timeout_reached):
                self.publish_wheel_velocities(
                    [0.0, 0.0, 0.0],
                    bot.id,
                    bot.base_angle,
                    bot.elbow_angle,
                    0
                )
                bot.is_moving = False
                bot.goal_reached = True

                self.get_logger().info(
                    f"Bot {bot.id} | Pickup aligned | "
                    f"dist={dist_error:.1f}, dx={dx_error:.1f}, dy={dy_error:.1f}"
                )
                return


        # 5. PID COMPUTATION
        # ------------------
        vx_pid = bot.pid_x.compute(target_x - x, dt)
        vy_pid = bot.pid_y.compute(target_y - y, dt)

        # --------------------------------------------------
# IR active: freeze Y until X is aligned
# --------------------------------------------------
        if bot.ir_trigger_time is not None and not x_ok:
            vy_pid = 0.0


        # Disable Obstacle Avoidance during "Highway Escape" 
        # (Otherwise nearby zones might push us back into the shadow)
        if label == "Highway Escape":
            vx, vy = vx_pid, vy_pid
        else:
            # Use your existing obstacle avoidance logic (ignore target crate)
            vx, vy = self.apply_crate_avoidance(bot, vx_pid, vy_pid)

        # Hard Slowdown Logic
        # --------- GOAL SLOWDOWN (FIXED TO 0.5) ---------
        if dist_error < self.slow_radius:
            bot.slowdown_scale = 0.5
        # ----------------------------------------------


        theta = math.radians(theta_deg)
        goal_theta = 0.0
        error_theta = (goal_theta - theta + math.pi) % (2*math.pi) - math.pi
        omega = bot.pid_theta.compute(error_theta, dt, error_threshold=0.087)

        vx *= bot.slowdown_scale
        vy *= bot.slowdown_scale
        omega *= bot.slowdown_scale

        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx * cos_t + vy * sin_t
        vy_r = -vx * sin_t + vy * cos_t
        
        wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])
        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, 0)
        # -----------------------------------------------------

        # # If not stopped, continue PID Control (Moving towards crate)
        # # vx = bot.pid_x.compute(goal_x - x, dt)
        # # vy = bot.pid_y.compute(goal_y - y, dt)
        
        # # if dist_error < self.slow_radius:
        # #     bot.slowdown_scale = max(self.min_slow_scale, dist_error / self.slow_radius)

        # # If not stopped, continue PID Control (Moving towards crate)
        # vx_pid = bot.pid_x.compute(goal_x - x, dt)
        # vy_pid = bot.pid_y.compute(goal_y - y, dt)

        # # [NEW] Apply Obstacle Avoidance
        # # This adds a "push" away from all crates EXCEPT the target one
        # vx, vy = self.apply_crate_avoidance(bot, vx_pid, vy_pid)
        
        # # if dist_error < self.slow_radius:
        # #     bot.slowdown_scale = max(self.min_slow_scale, dist_error / self.slow_radius)

        # # ---------------- HARD SLOWDOWN LOGIC ----------------
        # if dist_error < self.hard_slow_distance:
        #     bot.slowdown_scale = self.hard_slow_scale

        #     self.get_logger().info(
        #         f"Bot {bot.id} | HARD SLOWDOWN ACTIVE (dist={dist_error:.1f}mm, scale=0.5)",
        #         throttle_duration_sec=1.0
        #     )

        # elif dist_error < self.slow_radius:
        #     bot.slowdown_scale = max(self.min_slow_scale, dist_error / self.slow_radius)
        # # -----------------------------------------------------


        # theta = math.radians(theta_deg)
        # goal_theta = 0.0
        # error_theta = (goal_theta - theta + math.pi) % (2*math.pi) - math.pi
        # # omega = bot.pid_theta.compute(error_theta, dt)
        # omega = bot.pid_theta.compute(error_theta, dt, error_threshold=0.087)

        # vx *= bot.slowdown_scale
        # vy *= bot.slowdown_scale
        # omega *= bot.slowdown_scale

        # cos_t, sin_t = math.cos(theta), math.sin(theta)
        # vx_r = vx * cos_t + vy * sin_t
        # vy_r = -vx * sin_t + vy * cos_t
        
        # wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])
        
        # self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, 0)

    def handle_backup(self, bot: BotState):
        # Move to the backup goal
        self.log_bot_distance(bot, bot.goal[0], bot.goal[1], "backup clearance")

        # If backup position reached
        if self.move_to_point(bot, bot.goal, "backup"):
            self.get_logger().info(f"Bot {bot.id} | Backup complete. Safe to proceed.")
            
            bot.backing_up = False

            if bot.current_crate is not None:
                zone_id = self.get_zone_for_crate(bot.current_crate['id'])
                if self.zone_locks.get(zone_id) == bot.id:
                    self.zone_locks[zone_id] = None
                    self.get_logger().info(f"Bot {bot.id} | UNLOCKED Drop Zone {zone_id}")
            
            # --- CLEANUP STATE (Moved from drop_crate) ---
            # Now it is safe to release the bot for new tasks
            bot.current_crate = None
            bot.tracked_crate_id = None
            bot.arm_lifted = False
            bot.arm_placed = False
            
            # Reset arm targets to safe travel position
            bot.arm_target_base = 45.0
            bot.arm_target_elbow = 135.0

            # --- DECIDE NEXT MOVE (Staging vs Home) ---
            if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
                # Case 1: High up in the arena -> Go to Staging Point
                self.get_logger().info(f"Bot {bot.id} | High Y, going to STAGING.")
                bot.going_to_staging_point = True
                bot.return_to_start = False
                
                if bot.id == 0: staging_goal = self.staging_point_0
                elif bot.id == 2: staging_goal = self.staging_point_2
                elif bot.id == 4: staging_goal = self.staging_point_4
                else: staging_goal = self.staging_point_0
                
                # Check for ALT staging (Right side logic)
                if bot.id == 0 and bot.pose[0] > 1219.0: staging_goal = self.sp_0_alt.copy()
                if bot.id == 4 and bot.pose[0] > 1219.0: staging_goal = self.sp_4_alt.copy()
                
                bot.goal = staging_goal.copy()

            else:
                # Case 2: Low in the arena -> Go Home directly
                self.get_logger().info(f"Bot {bot.id} | Low Y, returning HOME.")
                bot.going_to_staging_point = False
                bot.return_to_start = True
                
                if bot.id == 0: bot.return_goal = self.idle_home_point_0.copy()
                elif bot.id == 2: bot.return_goal = self.idle_home_point_2.copy()
                elif bot.id == 4: bot.return_goal = self.idle_home_point_4.copy()
                
                bot.goal = bot.return_goal.copy()

            bot.reset_pid()
    
    def lift_arm_with_crate(self, bot: BotState):
        # --- NEW LOGIC: PRE-LIFT WAIT ---
        # Check if we are the "Center Slot" (Slot 2) which must go last.
        # If so, do not lift/leave until 2 other crates in this zone are done.
        if bot.tracked_crate_id is not None:
            drop_info = self.drop_positions.get(bot.tracked_crate_id)
            
            # If this is the center slot (index 2)
            if drop_info and drop_info.get('zone_slot') == 2:
                zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
                
                # Count how many are actually finished in this zone
                completed_count = 0
                for c_id in self.completed_crates:
                    if self.get_zone_for_crate(c_id) == zone_id:
                        completed_count += 1
                
                # If neighbors aren't done, WAIT HERE.
                if completed_count < 2:
                    self.get_logger().info(
                        f"Bot {bot.id} | Waiting to LIFT. Needs 2 placed, found {completed_count}.", 
                        throttle_duration_sec=2.0
                    )
                    
                    # 1. Keep Magnet ON (State 10) to hold crate
                    # 2. Keep Arm DOWN (Base 45/15, Elbow 180)
                    # 3. Stop Wheels
                    self.publish_wheel_velocities(
                        [0.0, 0.0, 0.0], 
                        bot.id, 
                        bot.base_angle, 
                        bot.elbow_angle, 
                        10 # Magnet ON
                    )
                    return # <--- CRITICAL: Do not proceed to lift logic yet
        # --------------------------------

        # Define the final lifted position
        final_base = 45.0
        final_elbow = 135.0

        # Get the current 'down' configuration
        base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
        
        # ---------------- SEQUENTIAL LIFT LOGIC ----------------
        # 1. Always target the Base to the final position immediately
        bot.arm_target_base = final_base

        # 2. Check if the Base has reached the target
        base_is_positioned = abs(bot.base_angle - final_base) < 2.0

        if not base_is_positioned:
            # If Base is still moving, Force Elbow to stay DOWN
            bot.arm_target_elbow = elbow_down
        else:
            # Once Base is ready, trigger the Elbow to move IN
            bot.arm_target_elbow = final_elbow
        # -------------------------------------------------------

        # Execute the smooth movement step
        arm_done = self.update_arm_smooth(bot)
        
        # Keep Magnet ON (State 10 = Pull/Hold)
        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 10)

        # Only mark as 'lifted' when BOTH have finished their movements
        if arm_done:
            bot.arm_lifted = True
            bot.move_after_attach = True
            bot.waiting_for_pickup_zone = False

            # Release the PICKUP zone lock (Bot 2 holds this while waiting above)
            if bot.pickup_zone_idx is not None:
                if self.pickup_zone_locks.get(bot.pickup_zone_idx) == bot.id:
                    self.pickup_zone_locks[bot.pickup_zone_idx] = None
                    self.get_logger().info(
                        f"Bot {bot.id} | Released Pickup Zone {bot.pickup_zone_idx} after pickup"
                    )

            bot.pickup_zone_idx = None
            
            self.get_logger().info(f"Bot {bot.id} | Arm lifted (Base -> Elbow) with crate attached!")

    # def move_to_final_goal(self,bot:BotState):
    #     drop = self.drop_positions.get(bot.tracked_crate_id)
    #     goal = np.array([drop['x'],drop['y']-190,0.0]) if drop else bot.final_goal
    #     bot.goal = goal.copy()

    #     # Log distance
    #     self.log_bot_distance(bot, goal[0], goal[1], "drop zone")

    #     if self.move_to_point(bot, goal, "drop zone"):
    #         bot.move_after_attach = False
    #         bot.dropping = True          
    #         bot.mag_timer_start = None   
    #         bot.drop_pose_logged = False # Reset flag for drop log

    
    def is_in_drop_zone(self, bot: BotState):
        if bot.pose is None: return False
        x, y, _ = bot.pose
        # Check all drop zones with padding
        for z in self.drop_zones.values():
            if (z['x_min'] - 50 <= x <= z['x_max'] + 50 and 
                z['y_min'] - 50 <= y <= z['y_max'] + 50):
                return True
        return False
    
    def move_to_final_goal(self, bot: BotState):
        # 1. Identify Target Zone & Drop Position
        crate_id = bot.tracked_crate_id
        if crate_id is None: return

        zone_id = self.get_zone_for_crate(crate_id)
        drop = self.drop_positions.get(crate_id)

        # 2. FAIL-SAFE: If missing, retry assignment
        if drop is None:
            self.get_logger().warn(f"Bot {bot.id} | Drop position missing. Retrying assignment...")
            self.assign_drop_positions()
            drop = self.drop_positions.get(crate_id)

        # 3. ULTIMATE FAIL-SAFE: If still missing, use default coordinates
        if drop is None:
            self.get_logger().error(f"Bot {bot.id} | CRITICAL: Drop lost. Using DEFAULT drop to unblock.")
            # Default to Right Zone (safe area)
            drop = {'x': 1600.0, 'y': 2000.0}
            zone_id = 2 # Force zone ID for default drop

        if drop is None: return

        # --- INSERT THIS BLOCK ---
        my_slot = drop.get('zone_slot', 99)

        # Logic: If I am the Center (Slot 2), wait for 2 completions.
        # Slots 0 and 1 skip this and go straight to the gate.
        if my_slot == 2:
            completed_in_zone = 0
            for cid in self.completed_crates:
                if self.get_zone_for_crate(cid) == zone_id:
                    completed_in_zone += 1
            
            if completed_in_zone < 2:
                # Wait here
                self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                bot.is_moving = False
                return

        # 4. GATE LOGIC (Stage 1)
        # ---------------------------------------------------------
        if not bot.passed_drop_gate:
            # A. Set Goal to the Gate coordinates
            # Default to center of map if gate key missing (safety)
            gate_x, gate_y = self.drop_gates.get(zone_id, (1219.0, 1200.0))
            bot.goal = np.array([gate_x, gate_y, 0.0])
            
            self.log_bot_distance(bot, gate_x, gate_y, f"Drop Gate {zone_id}")

            # B. Check Distance to Gate (60mm tolerance)
            dist_to_gate = math.hypot(gate_x - bot.pose[0], gate_y - bot.pose[1])
            
            if dist_to_gate < 60.0:
                # C. REACHED GATE -> CHECK LOCK STATUS
                locker = self.zone_locks.get(zone_id)
                
                # WAIT CONDITION:
                # Locked by someone else AND I am not in "Sharing" mode
                if locker is not None and locker != bot.id:
                    # STOP MOTORS
                    self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                    bot.is_moving = False
                    return # Exit here (Bot waits at Gate)

                # PROCEED CONDITION:
                # If zone is free, claim it
                if locker is None:
                    self.zone_locks[zone_id] = bot.id
                
                # Mark gate as passed
                bot.passed_drop_gate = True
                self.get_logger().info(f"Bot {bot.id} | Passed Drop Gate {zone_id}. Entering Zone.")
            
            else:
                # Not at gate yet -> Keep moving to gate
                self.move_to_point(bot, bot.goal, "Drop Gate Approach")
                return

        # 5. ENTERING ZONE (Stage 2: Post-Gate)
        # ---------------------------------------------------------
        # Goal is the actual drop slot (offset by -190mm Y for arm reach)
        final_goal = np.array([drop['x'], drop['y'] - 150.0, 0.0])
        bot.goal = final_goal.copy()

        self.log_bot_distance(bot, final_goal[0], final_goal[1], "Drop Slot")

        if self.move_to_point(bot, final_goal, "Drop Slot", xy_tol=5.0, theta_tol=15.0):
            bot.move_after_attach = False
            bot.dropping = True           
            bot.mag_timer_start = None    
            bot.drop_pose_logged = False

    def drop_crate(self, bot: BotState):
        base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))

        if bot.current_crate:
             cid = bot.current_crate['id']
             dpos = self.drop_positions.get(cid)
             # Index 2 corresponds to the 3rd crate (0, 1, 2)
             if dpos and dpos.get('zone_slot') == 2:
                  base_down = 0.0
                  elbow_down = 90.0
                  self.get_logger().info(f"Bot {bot.id} | Special 3rd Crate Drop -> Base:0, Elbow:90")
        
        # ---------------- SEQUENTIAL ARM LOGIC ----------------
        # Goal: Move Elbow first, THEN Move Base
        
        # 1. Always target the elbow down
        bot.arm_target_elbow = elbow_down

        # 2. Check if the elbow has reached the target (within 2 degrees tolerance)
        elbow_is_positioned = abs(bot.elbow_angle - elbow_down) < 2.0

        if not elbow_is_positioned:
            # If Elbow is moving, keep Base UP (Standard carry angle is 45.0)
            bot.arm_target_base = 45.0
        else:
            # If Elbow is ready, allow Base to go DOWN
            bot.arm_target_base = base_down
        # ------------------------------------------------------

        # Execute the smooth step based on the targets set above
        arm_done = self.update_arm_smooth(bot)

        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

        # Wait until arm is down (Both Base and Elbow reached final targets)
        if not arm_done:
            return

        # LOG MATCHING REFERENCE (Once)
        if not bot.drop_pose_logged:
            self.get_logger().info(f"Bot {bot.id} | Arm positioned for drop (base={bot.base_angle:.1f}°, elbow={bot.elbow_angle:.1f}°)")
            bot.drop_pose_logged = True

        # Start drop timer
        if bot.mag_timer_start is None:
            bot.mag_timer_start = time.time()
            return

        elapsed = time.time() - bot.mag_timer_start

        # Wait
        if elapsed < 1.5:
            return

        # Release
        bot.magnet_state = 0
        bot.mag_timer_start = None
        
        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)

        # Mark completed
        if bot.current_crate is not None:
            crate_id = bot.current_crate['id']
            bot.completed_crates.add(crate_id)
            self.completed_crates.add(crate_id)   # <--- ADD THIS LINE (Fixes the logic error)
            # zone_id = self.get_zone_for_crate(crate_id)
            # if self.zone_locks.get(zone_id) == bot.id:
            #     self.zone_locks[zone_id] = None

        # LOG MATCHING REFERENCE
        self.get_logger().info(f"Bot {bot.id} | Crate released (magnet OFF)")

        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        # Cleanup state
        # bot.current_crate = None
        # bot.tracked_crate_id = None
        # bot.dropping = False
        # bot.arm_lifted = False
        # bot.arm_placed = False
        
        # # Reset arm targets to safe travel position
        # bot.arm_target_base = 45.0
        # bot.arm_target_elbow = 135.0

        # # ---------------- FIX STARTS HERE ----------------
        # # Check current Y position to decide: Staging vs Return Home directly
        # if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
        #     # Case 1: High up in the arena -> Go to Staging Point
        #     self.get_logger().info(f"Bot {bot.id} | High Y ({bot.pose[1]:.1f}), going to STAGING.")
            
        #     bot.going_to_staging_point = True
        #     bot.return_to_start = False
            
        #     if bot.id == 0: staging_goal = self.staging_point_0
        #     elif bot.id == 2: staging_goal = self.staging_point_2
        #     elif bot.id == 4: staging_goal = self.staging_point_4
        #     else: staging_goal = self.staging_point_0
            
        #     bot.goal = staging_goal.copy()
            
        # else:
        #     # Case 2: Low in the arena -> Go Home directly
        #     self.get_logger().info(f"Bot {bot.id} | Low Y ({bot.pose[1] if bot.pose is not None else 0:.1f}), returning HOME.")
            
        #     bot.going_to_staging_point = False
        #     bot.return_to_start = True
            
        #     if bot.id == 0: bot.return_goal = self.idle_home_point_0.copy()
        #     elif bot.id == 2: bot.return_goal = self.idle_home_point_2.copy()
        #     elif bot.id == 4: bot.return_goal = self.idle_home_point_4.copy()
            
        #     bot.goal = bot.return_goal.copy()

        # # Reset PID for the new movement
        # bot.reset_pid()

        # ---------------- NEW BACKUP TRANSITION ----------------
        # 1. Stop the "Dropping" state
        bot.dropping = False
        
        # 2. Start the "Backing Up" state
        bot.backing_up = True
        
        # 3. Set the backup goal (100mm backwards in Y)
        # We assume the drop approach was from -Y direction, so we retreat further -Y to clear the crate.
        if bot.pose is not None:
            current_x, current_y, current_w = bot.pose
            bot.goal = np.array([current_x, current_y - 100.0, current_w])
            
        # 4. Reset PID to ensure smooth start for the new motion
        bot.reset_pid()
        
        self.get_logger().info(f"Bot {bot.id} | Backing up 100mm to clear drop zone...")

    def move_to_staging(self, bot: BotState):
        # Log dist
        self.log_bot_distance(bot, bot.goal[0], bot.goal[1], "staging point")
        
        # If reached staging point
        if self.move_to_point(bot, bot.goal, "staging"):
            self.get_logger().info(f"Bot {bot.id} reached Staging. Proceeding Home.")
            
            bot.going_to_staging_point = False
            bot.return_to_start = True
            
            # Set the Home Goal immediately so it doesn't wait for next cycle
            if bot.id == 0: bot.return_goal = self.idle_home_point_0
            elif bot.id == 2: bot.return_goal = self.idle_home_point_2
            elif bot.id == 4: bot.return_goal = self.idle_home_point_4
            else: bot.return_goal = self.idle_home_point_0
            
            bot.goal = bot.return_goal.copy()
            bot.reset_pid()

    def return_home(self,bot:BotState):
        # Log distance
        self.log_bot_distance(bot, bot.return_goal[0], bot.return_goal[1], "start position")

        if self.move_to_point(bot,bot.return_goal, "home"):
            bot.return_to_start = False
            bot.has_returned_home = True
            bot.is_permanently_idle = True
            bot.goal = bot.return_goal.copy()

            bot.is_following = False
            bot.follow_target_id = None
            # LOG MATCHING REFERENCE
            self.get_logger().info(f"Bot {bot.id} | Returned home. Robot stopped.")
            self.reset_bot_state(bot)

    def move_to_point(self, bot: BotState, goal, label="target", xy_tol=None, theta_tol=None):
        self.update_arm_smooth(bot)
        bot.is_moving = True
        now = self.get_clock().now()
        
        # Initialize pid_last_time if it's the first run of this state
        if bot.pid_last_time is None:
            dt = 0.03 # Default to nominal period for first run
        else:
            dt = (now - bot.pid_last_time).nanoseconds / 1e9
        
        # Update the PID timer to NOW
        bot.pid_last_time = now
        
        # Safety clamp for dt to prevent explosions on lag spikes
        if dt <= 0.001: dt = 0.03
        if dt > 0.1: dt = 0.1

        # ---------------- MODIFIED START ----------------
        # 1. Use the helper to get filtered values (Theta is returned in RADIANS)
        x, y, theta = self.get_filtered_pose(bot)
        
        # 2. Convert to degrees locally just in case it is needed for logging or consistency
        theta_deg = math.degrees(theta)
        # ---------------- MODIFIED END ------------------
        gx,gy,gtheta_deg = goal
        gtheta = math.radians(gtheta_deg)

        dist = math.hypot(gx - x, gy - y)
        # if dist < self.slow_radius:
        #     bot.slowdown_scale = max(self.min_slow_scale, dist / self.slow_radius)

        self.log_bot_distance(bot, gx, gy, label)

        # --------- GOAL SLOWDOWN (FIXED TO 0.5) ---------
        if dist < self.slow_radius:
            bot.slowdown_scale = 0.5
        # ----------------------------------------------


        elif dist < self.slow_radius:
            bot.slowdown_scale = max(self.min_slow_scale, dist / self.slow_radius)
        # -----------------------------------------------------

        
        # vx = bot.pid_x.compute(gx - x,dt)
        # vy = bot.pid_y.compute(gy - y,dt)
        # etheta = (gtheta - theta + math.pi)%(2*math.pi)-math.pi
        # omega = bot.pid_theta.compute(etheta,dt)

        vx_pid = bot.pid_x.compute(gx - x,dt)
        vy_pid = bot.pid_y.compute(gy - y,dt)
        
        # [NEW] Apply Obstacle Avoidance
        # Since we are moving to a point (not a crate), we avoid ALL crates here.
        # But we must temporarily ensure we don't treat the target point as a crate 
        # (though likely the target point is empty space).
        vx, vy = self.apply_crate_avoidance(bot, vx_pid, vy_pid)

        etheta = (gtheta - theta + math.pi)%(2*math.pi)-math.pi
        omega = bot.pid_theta.compute(etheta,dt)

        vx *= bot.slowdown_scale
        vy *= bot.slowdown_scale
        omega *= bot.slowdown_scale

        # --- NEW FAST CODE ---
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx*cos_t + vy*sin_t
        vy_r = -vx*sin_t + vy*cos_t
        
        # USE PRE-CALCULATED MATRIX
        wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])

        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

        # if math.hypot(gx - x, gy - y) < (self.xy_tolerance+3) and abs(etheta) < math.radians(self.theta_tolerance_center_deg):
        #     bot.is_moving = False
        #     self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
        #     return True
        # return False

        current_xy_tol = xy_tol if xy_tol is not None else (self.xy_tolerance + 3)
        current_theta_tol = theta_tol if theta_tol is not None else self.theta_tolerance_center_deg

        # Check against the selected tolerance
        if math.hypot(gx - x, gy - y) < current_xy_tol and abs(etheta) < math.radians(current_theta_tol):
            bot.is_moving = False
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
            return True
        return False

    def reset_bot_state(self, bot: BotState):
        bot.goal_reached = False
        bot.arm_placed = False
        bot.arm_lifted = False
        bot.goal = None
        bot.arm_placed_logged = False  # <--- CRITICAL FIX
        bot.drop_pose_logged = False   # <--- CRITICAL FIX
        bot.wait_start_time = None
        bot.move_after_attach = False
        bot.going_to_staging_point = False
        bot.is_moving = False
        bot.is_following = False
        bot.follow_target_id = None
        bot.mag_timer_start = None
        bot.magnet_state = 0
        bot.ir_trigger_time = None
        
        # --- GATE FLAGS RESET ---
        bot.passed_gate = False       # Reset Pickup Gate
        bot.passed_drop_gate = False  # <--- [NEW] Reset Drop Gate
        # ------------------------

        bot.passed_gate = False        # Reset Pickup Gate
        bot.passed_drop_gate = False   # Reset Drop Gate
        bot.passed_pickup_staging = False

        bot.base_angle = 45.0
        bot.elbow_angle = 135.0
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        # ---------------- PICKUP ZONE CLEANUP ----------------
        if bot.pickup_zone_idx is not None:
            # If this bot was holding a pickup zone lock, release it
            if self.pickup_zone_locks.get(bot.pickup_zone_idx) == bot.id:
                self.pickup_zone_locks[bot.pickup_zone_idx] = None

        bot.pickup_zone_idx = None
        bot.waiting_for_pickup_zone = False
        # ----------------------------------------------------

        bot.reset_pid()

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

        # Dynamic topic creation per bot
        topic = f"bot{bot_id}/cmd"
        self.mqtt_client.publish(topic, json.dumps(payload))

    # Add this method inside HolonomicMoveToCratesMulti class
    def on_mqtt_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode().strip()
            
            # Parse topic "bot0/ir" -> id = 0
            if "/ir" in topic:
                bot_id_str = topic.split('/')[0].replace('bot', '')
                if not bot_id_str.isdigit(): return
                
                bot_id = int(bot_id_str)
                
                if bot_id in self.bots:
                    # Logic: '0' means Detected (True), '1' means Clear (False)
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