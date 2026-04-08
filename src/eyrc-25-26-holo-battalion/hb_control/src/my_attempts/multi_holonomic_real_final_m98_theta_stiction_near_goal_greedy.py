#!/usr/bin/env python3

"""
* Team Id : eYrc#3027
* Author List : Harry Parikh, Dhruv Dabur
* Filename: multi_robot_controller.py
* Theme: Holo Battilion
* Functions: __init__, compute, reset, reset_pid, on_mqtt_connect, get_drop_priority_penalty, on_mqtt_disconnect, pose_cb, crate_cb, log_bot_distance, calculate_cost_matrix, assign_bot_to_crate, assign_crates_hungarian, get_zone_for_crate, assign_drop_positions, get_pickup_zone_idx_for_crate, normalize_angle, get_filtered_pose, assign_crates_greedily, assign_bot_to_crate(nested helper), is_bot_physically_in_pickup_zone, is_picking_up, check_and_handle_collisions, publish_pid_debug, publish_perception_debug, check_pickup_failure, control_cb, attach_service_cb, is_in_triangle, follow_bot, update_arm_smooth, apply_crate_avoidance, is_bot_in_zone, move_near_crate, is_in_drop_zone, move_to_final_goal, drop_crate, move_to_staging, return_home, move_to_point, reset_bot_state, publish_wheel_velocities, stop_all_bots, on_mqtt_message, main
* Global Variables: None
"""

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
from std_msgs.msg import String, Float64MultiArray 
from scipy.optimize import linear_sum_assignment

# ---------------------- PID Controller Class (Fixed) ------------------------
class PID:
    """
    * Function Name: __init__
    * Input: kp (float), ki (float), kd (float), max_out (float)
    * Output: None (Initializes object)
    * Logic: Sets the PID gain constants and initializes integral/error tracking variables.
    *
    * Example Call: pid_controller = PID(1.5, 0.01, 0.5, 50.0)
    """
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    """
    * Function Name: compute
    * Input: error (float), dt (float), integration_window (float, optional), error_threshold (float, optional), deadband (float)
    * Output: output (float) - The control signal clipped to max_out.
    * Logic: Calculates Proportional, Integral, and Derivative terms. 
    * Includes logic for:
    * 1. Deadband (returns 0 if error is small).
    * 2. Conditional Integration (anti-windup: only integrates if not saturated or if error helps desaturate).
    * 3. Integration Window (resets I-term if error is too large).
    *
    * Example Call: velocity = self.pid_x.compute(target_x - current_x, 0.03, integration_window=50.0)
    """

    def compute(self, error, dt, integration_window=None, error_threshold=None, deadband=0.0):
        if dt <= 0:
            return 0.0

        # 1. DEADBAND: If close enough, stop everything and reset I
        if abs(error) < deadband:
            self.integral = 0.0 
            return 0.0

        # 2. Compute P and D terms
        p_term = self.kp * error
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # 3. Compute what the output WOULD be without the new integral part
        current_output_guess = p_term + (self.ki * self.integral) + d_term

        # 4. CONDITIONAL INTEGRATION 
        # We only add to the integral if:
        # A. We are inside the window (if specified)
        # B. The output is NOT saturated, OR the error opposes the saturation
        
        can_integrate = True
        
        # Window Check
        if integration_window is not None and abs(error) > integration_window:
            self.integral = 0.0 # Reset if outside window
            can_integrate = False

        # Saturation Check
        if can_integrate:
            if current_output_guess > self.max_out and error > 0:
                # Output is maxed positive, and error wants to push it higher -> Don't Integrate
                can_integrate = False
            elif current_output_guess < -self.max_out and error < 0:
                # Output is maxed negative, and error wants to push it lower -> Don't Integrate
                can_integrate = False

        # Apply Integration
        if can_integrate:
            self.integral += error * dt

        # 5. Final Calculation
        i_term = self.ki * self.integral
        output = p_term + i_term + d_term
        self.prev_error = error
        
        return float(np.clip(output, -self.max_out, self.max_out))

    """
    * Function Name: reset
    * Input: None
    * Output: None
    * Logic: Resets the accumulated integral error and the previous error to zero to prevent jumps when restarting control.
    *
    * Example Call: self.pid_x.reset()
    """
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Per-bot State ---------------------------------------
class BotState:

    """
    * Function Name: __init__
    * Input: bot_id (int), node (Node), pid_params (dict), final_goal (np.array), return_goal (np.array)
    * Output: None (Initializes object)
    * Logic: Initializes all state flags (moving, lifting, dropping), timers, physical properties (pose, arm angles), and PID controllers for a specific robot.
    *
    * Example Call: bot_state = BotState(0, self, params, goal_A, goal_B)
    """
    def __init__(self, bot_id, node, pid_params, final_goal, return_goal):
        self.id = bot_id
        self.node = node
        self.pose = None
        self.current_crate = None
        self.tracked_crate_id = None
        self.goal = None
        self.ir_detected = False  

        self.ir_trigger_time = None # Timestamp when IR first went low
        self.passed_drop_gate = False    
        self.passed_drop_staging = False
        self.passed_crate_staging = False # Track if we reached the +/- 100mm point
        self.is_sharing_zone = False  
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

        self.drop_zone_released_early = False

        self.potential_collision_start_time = None
        self.dist_at_collision_start = None


        self.pickup_zone_idx = None
        self.waiting_for_pickup_zone = False

        # Logging flags for one-time messages
        self.drop_pose_logged = False
        self.arm_placed_logged = False

        self.alignment_start_time = None
        
        # Distance logging
        self.last_dist_log_time = time.time()
        self.dist_log_interval = 0.5 

        # magnet state
        self.magnet_state = 0
        self.mag_timer_start = None
        self.wait_start_time = None  

        # ---------------- Smooth Arm State ----------------
        self.base_angle = 45.0
        self.elbow_angle = 135.0

        self.arm_target_base = 45.0
        self.arm_target_elbow = 135.0

        self.arm_step = 0.3  
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

        # Enable filter 
        if self.id == 2:
            self.use_filter = True   
            self.filter_alpha = 0.3  
        elif self.id == 4:
            self.use_filter = True
            self.filter_alpha = 0.2
        elif self.id == 0:
            self.use_filter = True
            self.filter_alpha = 0.3
        else:
            self.use_filter = False  
            self.filter_alpha = 1.0 

    """
    * Function Name: reset_pid
    * Input: None
    * Output: None
    * Logic: Resets the internal state of the X, Y, and Theta PID controllers for this specific bot.
    *
    * Example Call: bot.reset_pid()
    """

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

# ---------------------- Main Node Class -------------------------------------
class HolonomicMoveToCratesMulti(Node):

    """
    * Function Name: __init__
    * Input: None
    * Output: None
    * Logic: Initializes the ROS 2 node for a multi-robot holonomic control system.
    * 1. Sets up system-wide constants (tolerances, velocities, safety radii).
    * 2. Pre-calculates the Inverse Kinematics Matrix (M_inv) for the 3-wheeled holonomic drive to save CPU cycles.
    * 3. Defines world geometry including pickup zones, drop zones, and "gates" for traffic control.
    * 4. Initializes independent BotState objects and PID controllers for each robot ID (0, 2, 4).
    * 5. Configures specific hardware offsets (Arm configurations) and MQTT/ROS communication interfaces.
    *
    * Example Call: node = HolonomicMoveToCratesMulti()
    """
    def __init__(self):
        super().__init__('holonomic_move_to_crates_multi') 

        self.node_start_time = time.time()
        self.startup_delay = 5.0  # Seconds to wait for vision
        self.is_warmed_up = False

        self.warmup_crate_buffer = {} # Dictionary to store lists of coordinates
        self.assigned_zones_this_cycle = set()

        # ---------------- Robot and world state ----------------
        self.bot_ids = [0,2,4]  # Controlled robot IDs
        # self.target_crate_ids = [30, 14, 12, 11, 13, 37, 16, 21, 39, 36, 35, 32]
        self.target_crate_ids = [32, 35, 14, 21, 12, 30]
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
        self.xy_tolerance_x = 2.0   # mm
        self.xy_tolerance_y = 4.0
        self.xy_tolerance_theta = 1.0   #degree

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

        self.collision_observations = {}

        # ---------------- Global goals ----------------
        self.final_goal = np.array([1219.0,1180.0,0.0])  
        self.return_goal = np.array([1218.0,205.0,0.0])  

        # ---------------- Staging points (Consolidated) ----------------
        
        # Alternative Staging Points (Right Side)
        self.sp_0_alt = np.array([1700.0, 1450.0, 0.0]) 
        self.sp_4_alt = np.array([800.0, 1450.0, 0.0])

        self.staging_point_0 = np.array([800.0,1450.0,0.0])  
        self.staging_point_2 = np.array([1700.0,1450.0,0.0])  
        self.staging_point_4 = np.array([800.0,1450.0,0.0])  
        self.staging_y_threshold = 1400.0  

        # ---------------- Idle home points ----------------
        self.idle_home_point_0 = np.array([1220.0,225.0,0.0])  
        self.idle_home_point_2 = np.array([1570.0,225.0,0.0])  
        self.idle_home_point_4 = np.array([870.0,225.0,0.0])  

        self.zone_drop_counts = {0: 0, 1: 0, 2: 0}

        self.pid_debug_pubs = {
            bid: self.create_publisher(Float64MultiArray, f'/bot{bid}/pid_debug', 10)
            for bid in self.bot_ids
        }


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
            0: (13.0, 180.0),
            2: (10.0, 180.0),  
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
                'x':     {'kp': 0.4, 'ki': 0.0, 'kd': 0.1, 'max_out': self.max_vel},
                'y':     {'kp': 0.8, 'ki': 0.05, 'kd': 0.02, 'max_out': self.max_vel},
                
                'theta': {'kp': 140.0, 'ki': 20.0, 'kd': 15.0, 'max_out': 40}
            },
            
            2: {

                'x':     {'kp': 0.7, 'ki': 0.15, 'kd': 0.1, 'max_out': 50.0},
                'y':     {'kp': 0.7, 'ki': 0.15, 'kd': 0.1, 'max_out': 50.0},
                'theta': {'kp': 150.0, 'ki': 2.5, 'kd': 2.0, 'max_out': 40.0}
                },
            4: {
                'x':     {'kp': 0.65, 'ki': 0.05, 'kd': 0.25, 'max_out': self.max_vel},
                'y':     {'kp': 0.65, 'ki': 0.05, 'kd': 0.25, 'max_out': self.max_vel},
                'theta': {'kp': 110.0, 'ki': 1.5, 'kd': 12.0, 'max_out': 45.0}
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

        
        self.drop_gates = {
            0: (1210.0, 950.0),   
            1: (800.0, 1800.0),   
            2: (1610.0, 1800.0)  
        }

        self.zone_locks = {0:None,1:None,2:None}

        self.pickup_zone_locks = {0: None, 1: None, 2: None, 3: None}
        
       
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
            self.mqtt_client.connect("192.168.0.183", 1883, 60)
            for bid in self.bot_ids:
                topic = f"bot{bid}/ir"
                self.mqtt_client.subscribe(topic)
                self.get_logger().info(f"Subscribed to MQTT: {topic}")
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

    # ---------------- MQTT Callbacks ----------------
    """
    * Function Name: on_mqtt_connect
    * Input: client (mqtt.Client), userdata (any), flags (dict), rc (int)
    * Output: None
    * Logic: Callback function triggered when the MQTT client receives a CONNACK response from the broker.
    * It logs the result code (rc) which indicates success (0) or various error states.
    *
    * Example Call: Triggered automatically by the MQTT client loop.
    """
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Connected (rc={rc})")

    """
    * Function Name: on_mqtt_disconnect
    * Input: client (mqtt.Client), userdata (any), rc (int)
    * Output: None
    * Logic: Callback triggered when the client loses connection to the broker. 
    * Logs the reason code (rc) and attempts to restart the background network loop 
    * to ensure automatic reconnection handling.
    *
    * Example Call: Triggered automatically by the MQTT client loop.
    """

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT Disconnected (rc={rc}). Attempting reconnect...")
        try:
            client.loop_start()
        except:
            pass

    # ---------------- Callbacks ----------------
    """
    * Function Name: pose_cb
    * Input: msg (Poses2D) - ROS message containing robot poses.
    * Output: None
    * Logic: Updates the global pose, calculates velocity based on position delta, and timestamps the data for every robot ID found in the message.
    *
    * Example Call: triggered automatically by ROS subscriber.
    """
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

    """
    * Function Name: crate_cb
    * Input: msg (Poses2D) - ROS message containing crate poses.
    * Output: None
    * Logic: Filters the incoming list of crate poses against the `target_crate_ids` list and updates `self.crates`.
    *
    * Example Call: triggered automatically by ROS subscriber.
    """

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

    # ---------------- Logging Helper ----------------

    """
    * Function Name: log_bot_distance
    * Input: bot (BotState), gx (float), gy (float), label (str), gtheta (float, optional)
    * Output: None
    * Logic: Helper to log telemetry data without spamming the console. 
    * 1. Checks if the `dist_log_interval` (0.5s) has passed.
    * 2. Calculates distance to the goal (gx, gy).
    * 3. Only logs if the robot is close (< 100mm) to the target to reduce noise.
    * 4. Updates the timestamp to throttle future logs.
    *
    * Example Call: self.log_bot_distance(bot, 1200.0, 500.0, "approach", gtheta=90.0)
    """
    def log_bot_distance(self, bot: BotState, gx, gy, label, gtheta=0.0):
        now = time.time()
        if now - bot.last_dist_log_time >= bot.dist_log_interval:
            if bot.pose is not None:
                x, y, theta = bot.pose
                dist = math.hypot(gx - x, gy - y)

                if dist < 100.0:
                    self.get_logger().info(
                        f"Bot {bot.id} | {label} | "
                        f"Dist: {dist:.2f} mm | "
                        f"Current: ({x:.1f}, {y:.1f}, θ={theta:.1f}°) | "
                        f"Goal: ({gx:.1f}, {gy:.1f}, θ={gtheta:.1f}°)"
                    )

                    bot.last_dist_log_time = now

    """
    * Function Name: calculate_cost_matrix
    * Input: idle_bots (list), available_crates (list), blocked_zones (set)
    * Output: cost_matrix (np.array) - NxM matrix of costs.
    * Logic: Calculates the "cost" for every bot-to-crate pairing based on:
    * 1. Euclidean distance.
    * 2. Zone blocking penalties (add high cost if zone is occupied).
    * 3. Specific bot priorities (e.g., Bot 2 prefers Zone 1).
    * 4. Drop slot logic (penalizes back-row crates if front-row isn't filled).
    *
    * Example Call: matrix = self.calculate_cost_matrix(bots, crates, blocked_zones)
    """                    

    def calculate_cost_matrix(self, idle_bots, available_crates, blocked_zones):
        """
        Builds a NxM matrix where:
        - Rows = Bots
        - Cols = Crates
        - Value = Cost (Lower is better)
        """
        n_bots = len(idle_bots)
        n_crates = len(available_crates)
        cost_matrix = np.zeros((n_bots, n_crates))

        
        drop_assignments = {}
        for c in available_crates:
            cid = c['id']
            
            if cid not in self.drop_positions:
                
                z = self.get_zone_for_crate(cid)
                
                pass
            drop_assignments[cid] = self.drop_positions.get(cid)

        for r, bot in enumerate(idle_bots):
            for c, crate in enumerate(available_crates):
                
                
                dist = math.hypot(crate['x'] - bot.pose[0], crate['y'] - bot.pose[1])
                cost = dist

               
                zone_idx = self.get_pickup_zone_idx_for_crate(crate)
                if zone_idx in blocked_zones:
                    cost += 100000.0 

                
                if bot.id == 2 and zone_idx == 1:
                    cost -= 2000.0
                elif bot.id == 4 and zone_idx == 0:
                    cost -= 2000.0

                cid = crate['id']
                dpos = self.drop_positions.get(cid)
                
                if dpos:
                    slot_idx = dpos.get('zone_slot')
                    target_zone = self.get_zone_for_crate(cid)
                    
                    
                    if slot_idx in [3, 4]:
                        # Count how many crates are ALREADY COMPLETED in this zone
                        completed_in_zone = 0
                        for completed_id in self.completed_crates:
                            if self.get_zone_for_crate(completed_id) == target_zone:
                                completed_in_zone += 1
                        
                        if completed_in_zone < 3:
                            
                            other_options_exist = False
                            for other_c in available_crates:
                                if other_c['id'] == cid: continue
                                
                                other_dpos = self.drop_positions.get(other_c['id'])
                                if other_dpos:
                                    other_slot = other_dpos.get('zone_slot')
                                    if other_slot in [0, 1, 2]:
                                        other_options_exist = True
                                        break
                            
                            if other_options_exist:
                                cost += 50000.0

                cost_matrix[r, c] = cost

        return cost_matrix

    """
    * Function Name: assign_bot_to_crate
    * Input: bot (BotState), crate (dict), label (str, optional)
    * Output: None
    * Logic: Resets all state flags for a specific robot and assigns it a new crate target.
    * 1. Updates the robot's current target crate ID and zone index.
    * 2. Clears all navigation flags (passed_gate, passed_staging).
    * 3. Clears all FSM flags (arm_lifted, dropping, backing_up).
    * 4. Resets PID controllers to prevent integral windup from previous tasks.
    *
    * Example Call: self.assign_bot_to_crate(bot, target_crate, label="Priority Assignment")
    """    

    def assign_bot_to_crate(self, bot, crate, label="Priority"):
        bot.current_crate = crate
        bot.tracked_crate_id = crate['id']
        bot.pickup_zone_idx = self.get_pickup_zone_idx_for_crate(crate)
        bot.waiting_for_pickup_zone = False
        bot.passed_gate = False
        bot.passed_drop_gate = False # Reset this too
        bot.passed_pickup_staging = False

        bot.passed_crate_staging = False

        # Reset Logic Flags
        bot.goal_reached = False
        bot.passed_drop_staging = False 
        bot.arm_placed = False
        bot.arm_lifted = False
        bot.move_after_attach = False
        bot.dropping = False
        bot.backing_up = False
        bot.return_to_start = False
        bot.going_to_staging_point = False
        bot.magnet_state = 0
        bot.ir_trigger_time = None
        bot.arm_placed_logged = False
        bot.drop_pose_logged = False
        bot.wait_start_time = None
        bot.mag_timer_start = None
        bot.drop_zone_released_early = False
        bot.alignment_start_time = None 

        bot.reset_pid()
        
        # Reset PID timer to ensure smooth start
        bot.pid_last_time = None 

        self.get_logger().info(
            f"[{label}] Crate {crate['id']} (Zone {bot.pickup_zone_idx}) -> Bot {bot.id}"
        )

    """
    * Function Name: assign_crates_hungarian
    * Input: None
    * Output: None
    * Logic: The main high-level logic for task distribution.
    * 1. Identifies idle bots and available crates.
    * 2. Loops iteratively to resolve zone conflicts.
    * 3. Uses `linear_sum_assignment` (Hungarian Algorithm) to find the optimal assignment.
    * 4. Assigns the winner to the crate and marks the zone as occupied for the current cycle.
    *
    * Example Call: self.assign_crates_hungarian()
    """
    def assign_crates_hungarian(self):
        # 1. RESET AND POPULATE OCCUPIED ZONES
        self.assigned_zones_this_cycle = set()

        # Check what busy bots are doing
        for b in self.bots.values():
            # If a bot has a crate and hasn't lifted it yet, it is occupying that pickup zone
            if b.current_crate is not None and not b.arm_lifted:
                z = self.get_pickup_zone_idx_for_crate(b.current_crate)
                if z is not None:
                    self.assigned_zones_this_cycle.add(z)
        
        # 2. Identify Participants (Idle Bots)
        idle_bots = [
            b for b in self.bots.values() 
            if b.current_crate is None 
            and not b.has_returned_home 
            and not b.is_permanently_idle
            and b.pose is not None ]

        crates_in_use = set()
        for b in self.bots.values():
            if b.current_crate: 
                crates_in_use.add(b.current_crate['id'])
            if b.tracked_crate_id and b.magnet_state != 0: 
                crates_in_use.add(b.tracked_crate_id)

        available_crates = [
            c for c in self.crates 
            if c['id'] not in self.completed_crates 
            and c['id'] not in crates_in_use
        ]
        

        if not idle_bots or not available_crates:
            return

        
        temp_blocked_zones = set(self.assigned_zones_this_cycle) 
        
        
        final_assignments = [] 

        max_iterations = len(idle_bots)
        
        current_idle_bots = idle_bots[:] # Working copy
        
        for iteration in range(max_iterations):
            if not current_idle_bots:
                break
            
            # Safety check: If we ran out of crates in a previous iteration
            if not available_crates:
                break

            # A. Build Matrix
            cost_matrix = self.calculate_cost_matrix(current_idle_bots, available_crates, temp_blocked_zones)

            # B. Solve Hungarian (Minimize Cost)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            # C. Check Assignments & Resolve Conflicts
            assignments_this_round = []
            
            # Group by zone to detect conflicts
            zone_requests = {} 

            for r, c in zip(row_ind, col_ind):
                bot = current_idle_bots[r]
                crate = available_crates[c]
                cost = cost_matrix[r, c]

                # Ignore impossible assignments (penalty > 50000)
                if cost > 50000:
                    continue

                z_idx = self.get_pickup_zone_idx_for_crate(crate)
                
                # Check conflict
                if z_idx in zone_requests:
                    # CONFLICT! Compare costs. Keep the lower cost assignment.
                    existing_bot, existing_cost, existing_crate = zone_requests[z_idx]
                    
                    if cost < existing_cost:
                        # New bot is better (lower cost). Replace existing.
                        zone_requests[z_idx] = (bot, cost, crate)
                        # The 'existing_bot' is kicked out and will retry next iteration
                    else:
                        # Existing bot is better. Keep it. Current bot fails this round.
                        pass 
                else:
                    # No conflict yet
                    zone_requests[z_idx] = (bot, cost, crate)

            # D. Commit the Winners
            bots_assigned_now = set()
            
            # We need to be careful removing from available_crates while iterating
            # So we collect crates to remove first
            crates_to_remove = []

            for z_idx, (bot, cost, crate) in zone_requests.items():
                # Actual Assignment Logic
                self.assign_bot_to_crate(bot, crate, label="Hungarian-Iter")
                
                # Block this zone for subsequent iterations/cycles
                temp_blocked_zones.add(z_idx)
                self.assigned_zones_this_cycle.add(z_idx)
                
                # Mark for removal
                crates_to_remove.append(crate)
                bots_assigned_now.add(bot.id)

            # Remove the assigned crates from the pool so next bot doesn't try to take them
            for c in crates_to_remove:
                if c in available_crates:
                    available_crates.remove(c)

            # E. Prepare for Next Iteration
            
            current_idle_bots = [b for b in current_idle_bots if b.id not in bots_assigned_now]
    # ---------------- SMART Drop position assignment (Fixed with +20 Offset) ----------------
    """
    * Function Name: get_zone_for_crate
    * Input: crate_id (int)
    * Output: zone_id (int) - The index of the drop zone (0, 1, or 2).
    * Logic: Uses modulo arithmetic to distribute crates evenly across the three drop zones.
    * Crate ID 0, 3, 6 -> Zone 0
    * Crate ID 1, 4, 7 -> Zone 1
    * Crate ID 2, 5, 8 -> Zone 2
    *
    * Example Call: zone = self.get_zone_for_crate(14)  # Returns 2
    """    
    def get_zone_for_crate(self, crate_id):
        return crate_id % 3 

    # ---------------- SMART Drop position assignment----------------

    """
    * Function Name: assign_drop_positions
    * Input: None
    * Output: None
    * Logic: Sorts all active crates by Y-coordinate and assigns them to specific drop slots (0-4) within their respective zones.
    * Implements a 5-crate layout logic (Front row: slots 0,1,2; Back row: slots 3,4).
    *
    * Example Call: self.assign_drop_positions()
    """    
    def assign_drop_positions(self):
        self.drop_positions = {}

        # Distance from center to side slots
        spacing_x = 35.0 
        spacing_y = 85.0
        
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
            if z in crates_by_zone:
                crates_by_zone[z].append(c)

        # 3. Assign Slots per Zone
        for z, crates in crates_by_zone.items():
            if not crates: 
                continue

            cx, cy = zone_centers[z]
            
            
            sorted_crates = sorted(crates, key=lambda x: x['y'], reverse=True)
            num_crates = len(sorted_crates)
            
            offsets_map = {}
            mapping = []


            # CASE 1: 0 to 3 Crates 
            if num_crates <= 3:
                # Slot 2 is Center/Back, 0 is Left, 1 is Right
                offsets_map = {
                    0: (-spacing_x, 0.0),
                    1: (+spacing_x, 0.0),
                    2: (0.0, -spacing_y)
                }

                if num_crates == 3:
                    # Max Y -> Slot 2, Next -> Slot 0, Next -> Slot 1
                    mapping = [
                        (sorted_crates[0], 2), 
                        (sorted_crates[1], 0), 
                        (sorted_crates[2], 1)
                    ]
                else:
                    for i in range(num_crates):
                        mapping.append((sorted_crates[i], i))

            # CASE 2: More than 3 Crates (Original 5-Crate Logic)
            else:
                # Original 5-slot layout
                offsets_map = {
                    0: (-2 * spacing_x, 0.0),      # Far Left
                    1: (0.0, 0.0),                 # Center
                    2: (2 * spacing_x, 0.0),       # Far Right
                    3: (-spacing_x, -spacing_y),   # Mid-Left-Back
                    4: (spacing_x, -spacing_y)     # Mid-Right-Back
                }

                if num_crates == 4:
                    # User requested specific order for 4 crates: 
                    # Skip Slot 4 (Mid-Right-Back)
                    slot_priority = [3, 2, 1, 0]
                else:
                    # Default for 5 crates (or fallback for >5)
                    # Fill all slots: 4, 3, 2, 1, 0
                    slot_priority = [4, 3, 2, 1, 0]

                mapping = []
                for i in range(num_crates):
                    if i < len(slot_priority):
                        mapping.append((sorted_crates[i], slot_priority[i]))
                
                

            # Apply the Calculated Assignments
            for crate, slot_idx in mapping:
                ox, oy = offsets_map.get(slot_idx, (0.0, 0.0))
                
                self.drop_positions[crate['id']] = {
                    'x': cx + ox,
                    'y': cy + oy + 20, 
                    'zone_slot': slot_idx
                }
                self.zone_drop_counts[z] += 1

        self.get_logger().info(f"Drop positions assigned (5-Crate Logic Active): {self.drop_positions}")

    """
    * Function Name: get_pickup_zone_idx_for_crate
    * Input: crate (dict) - Containing keys 'x' and 'y'.
    * Output: zone_index (int or None) - Returns 0, 1, 2, 3 if inside a zone, else None.
    * Logic: Iterates through the predefined `pickup_zones` list. checks if the crate's (x, y) coordinates fall within the [min, max] bounding box of any zone.
    *
    * Example Call: zone_idx = self.get_pickup_zone_idx_for_crate(crate_data)
    """

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
    
    """
    * Function Name: normalize_angle
    * Input: angle (float) - An angle in radians (can be outside -pi to pi range).
    * Output: angle (float) - The normalized angle in the range [-pi, pi].
    * Logic: Wraps any angle to the standard interval [-pi, pi] using modular arithmetic. 
    * This ensures that the robot always turns the shortest way (e.g., turning -10 degrees instead of +350 degrees).
    *
    * Example Call: norm_theta = self.normalize_angle(current_theta - target_theta)
    """    
    
    def normalize_angle(self, angle):
        """ Normalizes angle (radians) to range [-pi, pi] """
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    """
    * Function Name: get_filtered_pose
    * Input: bot (BotState)
    * Output: curr_x, curr_y, curr_t (floats)
    * Logic: Applies a low-pass filter (exponential moving average) to the robot's pose to smooth out vision noise. 
    * Handles angle wrap-around for Theta.
    *
    * Example Call: x, y, theta = self.get_filtered_pose(bot)
    """    
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
            t_alpha = 0.6 if bot.id == 4 else bot.filter_alpha
            
            if bot.theta_filter_val is None:
                bot.theta_filter_val = curr_t
            else:
                diff = curr_t - bot.theta_filter_val
                diff = (diff + math.pi) % (2 * math.pi) - math.pi
                bot.theta_filter_val += t_alpha * diff
            curr_t = bot.theta_filter_val
        else:
            # Keep values synced for safety, even if not filtering
            bot.x_filter_val = curr_x
            bot.y_filter_val = curr_y
            bot.theta_filter_val = curr_t

        return curr_x, curr_y, curr_t


    
    # ---------------- Greedy assignment with "Lift-Triggered" Unblocking ----------------

    """
    * Function Name: assign_crates_greedily
    * Input: None
    * Output: None
    * Logic: A fallback assignment strategy that prioritizes specific bot-to-zone pairings.
    * 1. Identifies crates that are not yet completed or targeted.
    * 2. Tracks "Busy Zones" to prevent multiple bots from entering the same pickup area simultaneously.
    * 3. Executes Priority Assignments: Forces Bot 2 to Zone 1 and Bot 4 to Zone 0 if available.
    * 4. Executes Standard Assignments: Matches remaining idle bots to the nearest available crate in free zones, prioritizing crates with lower Y coordinates.
    *
    * Example Call: self.assign_crates_greedily()
    """    
    def assign_crates_greedily(self):
        all_completed = self.completed_crates

        crates_in_use = set()
        assigned_zones_this_cycle = set()
        
        # 1. Identify crates already being targeted or carried
        for b in self.bots.values():
            if b.current_crate is not None:
                crates_in_use.add(b.current_crate['id'])

               
                if not b.arm_lifted:
                    z = self.get_pickup_zone_idx_for_crate(b.current_crate)
                    if z is not None:
                        assigned_zones_this_cycle.add(z)
                

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
        

        """
        * Function Name: get_drop_priority_penalty
        * Input: crate (dict)
        * Output: int (0 or 1)
        * Logic: Determines if a crate is physically blocked based on its spatial layout.
        * 1. Retrieves the assigned drop position (dpos) and zone slot index.
        * 2. Identifies the zone density (count_in_zone) to determine the layout type.
        * 3. 3-Crate Layout: Slot 2 is identified as back-row (blocked).
        * 4. 5-Crate Layout: Slots 3 and 4 are identified as back-row (blocked).
        * 5. Returns 1 (penalty) if the crate is dependent/blocked, 0 if it is clear for immediate drop.
        *
        * Example Call: penalty = self.get_drop_priority_penalty(target_crate)
        """
        def get_drop_priority_penalty(crate):
            """ Returns 1 if crate is back-row (must wait), 0 if front-row (ready to drop). """
            cid = crate['id']
            dpos = self.drop_positions.get(cid)
            if not dpos:
                return 0 # Default to high priority if missing
                
            slot_idx = dpos.get('zone_slot')
            zone_id = self.get_zone_for_crate(cid)
            count_in_zone = self.zone_drop_counts.get(zone_id, 0)
            
            # 3-crate layout: Slot 2 is the back-center (dependent)
            if count_in_zone <= 3 and slot_idx == 2:
                return 1
            # 5-crate layout: Slots 3 and 4 are the back-row (dependent)
            elif count_in_zone > 3 and slot_idx in [3, 4]:
                return 1
                
            return 0 # Front row crate (independent)

        """
        * Function Name: assign_bot_to_crate (Nested Helper)
        * Input: bot (BotState), crate (dict), label (str)
        * Output: None
        * Logic: A utility closure to finalize the assignment.
        * 1. Links the crate object to the bot.
        * 2. Resets all State Machine flags (lifted, placed, moving, etc.) to ensure a clean start.
        * 3. Resets PID controllers.
        * 4. Logs the assignment for debugging.
        *
        * Example Call: assign_bot_to_crate(bot2, target_crate, label="P2->Bot2")
        """        
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
            bot.drop_zone_released_early = False
            bot.alignment_start_time = None 

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
                p2_crates.sort(key=lambda c: (get_drop_priority_penalty(c), c['y']))
                target_crate = p2_crates[0]
                
                assign_bot_to_crate(bot2, target_crate, label="P2->Bot2")
                
                idle_bots.remove(bot2)
                unassigned_crates.remove(target_crate)
                assigned_zones_this_cycle.add(1) 

        # --- Priority 2: Assign P1 (Zone 0) to Bot 4 ---
        bot4 = next((b for b in idle_bots if b.id == 4), None)
        
        # Check if Zone 0 is free
        if bot4 and 0 not in assigned_zones_this_cycle:
            p1_crates = [c for c in unassigned_crates if self.get_pickup_zone_idx_for_crate(c) == 0]
            if p1_crates:
                p1_crates.sort(key=lambda c: (get_drop_priority_penalty(c), c['y']))
                target_crate = p1_crates[0]
                
                assign_bot_to_crate(bot4, target_crate, label="P1->Bot4")
                
                idle_bots.remove(bot4)
                unassigned_crates.remove(target_crate)
                assigned_zones_this_cycle.add(0) # Mark Zone 0 taken for this cycle

        

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

            
            crates.sort(key=lambda c: (get_drop_priority_penalty(c), c['y']))

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
    """
    * Function Name: is_bot_physically_in_pickup_zone
    * Input: bot (BotState) - The robot object containing current pose.
    * Output: bool - True if the robot is physically inside any pickup zone boundary.
    * Logic: Iterates through all defined pickup zones. Checks if the robot's (x, y) coordinates fall within the zone boundaries, applying a 50mm buffer to account for the robot's radius.
    *
    * Example Call: if self.is_bot_physically_in_pickup_zone(bot): ...
    """
    def is_bot_physically_in_pickup_zone(self, bot: BotState):
        """Checks if the bot's current coordinates are inside ANY pickup zone."""
        if bot.pose is None: 
            return False
        
        x, y, _ = bot.pose
        
        # Check against all defined pickup zones
        for z in self.pickup_zones:
            
            if (z['x_min'] - 50 <= x <= z['x_max'] + 50 and 
                z['y_min'] - 50 <= y <= z['y_max'] + 50):
                return True
                
        return False
    
    """
    * Function Name: is_picking_up
    * Input: bot (BotState)
    * Output: bool - True if the robot is in the critical "pickup" phase.
    * Logic: Checks a combination of flags: 
    * 1. Has a crate assigned.
    * 2. Has reached the visual goal.
    * 3. Has NOT yet lifted the arm.
    * This identifies the precise moment the robot is aligning or gripping, where collision avoidance should be disabled or strict.
    *
    * Example Call: if self.is_picking_up(bot): ...
    """    
    
    def is_picking_up(self, bot: BotState):
        """
        True ONLY when bot has reached the crate and is
        placing arm / magnet / lifting (pickup-critical phase)
        """
        return (
            bot.current_crate is not None and
            bot.goal_reached and
            not bot.arm_lifted
        )


    # ---------------- Collision Avoidance ----------------

    """
    * Function Name: check_and_handle_collisions
    * Input: None
    * Output: None
    * Logic: Checks pairwise distances between all active robots.
    * If distance < threshold, determines a "Leader" and a "Follower" based on priority (carrying crate > idle).
    * Forces the Follower to enter `is_following` mode and lock its distance.
    *
    * Example Call: self.check_and_handle_collisions()
    """    
    def check_and_handle_collisions(self):
        bots_to_check = [b for b in self.bots.values() if b.pose is not None]
        bots_that_should_follow = set()

        now_time = time.time()

        for bot_a, bot_b in itertools.combinations(bots_to_check, 2):

            pair_key = tuple(sorted((bot_a.id, bot_b.id)))

            dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])

            


            # 1. EXCLUSIONS: Only ignore completely parked/idle bots.
            if bot_a.has_returned_home or bot_a.is_permanently_idle:
                continue
            if bot_b.has_returned_home or bot_b.is_permanently_idle:
                continue

            # Active check: Are they trying to move?
            a_active = (bot_a.is_moving or bot_a.return_to_start or 
                        bot_a.dropping or bot_a.backing_up or 
                        bot_a.move_after_attach or bot_a.is_following)

            b_active = (bot_b.is_moving or bot_b.return_to_start or 
                        bot_b.dropping or bot_b.backing_up or 
                        bot_b.move_after_attach or bot_b.is_following)
            if not a_active and not b_active:
                continue

            # 2. Determine Dynamic Threshold
            # Base threshold
            current_threshold = self.follow_distance_threshold 
            
            # Expand threshold if magnets are active (carrying crates needs more space)
            if bot_a.magnet_state != 0 or bot_b.magnet_state != 0:
                current_threshold = 450.0

            # Shrink threshold ONLY if targeting the SAME zone (to allow packing)
            za = self.get_zone_for_crate(bot_a.tracked_crate_id) if bot_a.tracked_crate_id is not None else -1
            zb = self.get_zone_for_crate(bot_b.tracked_crate_id) if bot_b.tracked_crate_id is not None else -2

            # 2. If they are targeting the SAME zone, reduce the safety distance
            if za == zb and za != -1:
                # Use a small threshold (160mm) so they can park side-by-side
                # without triggering collision avoidance.
                current_threshold = 380.0

            in_zone_a = self.is_bot_physically_in_pickup_zone(bot_a)
            in_zone_b = self.is_bot_physically_in_pickup_zone(bot_b)
            
            if in_zone_a or in_zone_b:
                current_threshold = 380.0

            if bot_a.current_crate is not None and not bot_a.move_after_attach:
                dist_crate_a = math.hypot(bot_a.pose[0] - bot_a.current_crate['x'], bot_a.pose[1] - bot_a.current_crate['y'])
                if dist_crate_a < 200.0:
                    current_threshold = 200.0
            
            if bot_b.current_crate is not None and not bot_b.move_after_attach:
                dist_crate_b = math.hypot(bot_b.pose[0] - bot_b.current_crate['x'], bot_b.pose[1] - bot_b.current_crate['y'])
                if dist_crate_b < 200.0:
                    current_threshold = 200.0

            # 3. Calculate Distance (Omnidirectional)
            dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])

            
            if dist >= current_threshold:
                self.collision_observations.pop(pair_key, None)
                continue

            

            bots_already_locked = (bot_a.is_following and bot_a.follow_target_id == bot_b.id) or \
                              (bot_b.is_following and bot_b.follow_target_id == bot_a.id)

            if not bots_already_locked:
                if pair_key not in self.collision_observations:
                    # First time seeing them close: Record start state
                    self.collision_observations[pair_key] = {
                        'start_time': now_time,
                        'start_dist': dist
                    }
                    # Wait for next cycle to get trend data
                    continue 
                else:
                    obs = self.collision_observations[pair_key]
                    elapsed = now_time - obs['start_time']

                    # Wait for 0.2 seconds of history
                    if elapsed < 0.2:
                        continue 
                    
                    # Check trend: Are they moving apart?
                    if dist > obs['start_dist']:
                        # Distance INCREASING -> Reset and Do Not Engage
                        obs['start_time'] = now_time
                        obs['start_dist'] = dist
                        continue

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
                has_priority_a = (mag_a != 0) or bot_a.backing_up or bot_a.dropping
                has_priority_b = (mag_b != 0) or bot_b.backing_up or bot_b.dropping

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
            
            
            if not follower.is_following:
                follower.locked_follow_dist = current_threshold
                # self.get_logger().warn(f"Bot {follower.id} LOCKED follow distance: {follower.locked_follow_dist:.1f}mm")

            follower.is_following = True
            follower.follow_target_id = leader.id

            # self.get_logger().warn(f"COLLISION AVOIDANCE ACTIVE: Bot {follower.id} is STOPPING for Bot {leader.id}")

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

                if not bot.dropping and not bot.arm_placed:
                    bot.arm_target_base = 45.0
                    bot.arm_target_elbow = 135.0
                    self.get_logger().info(f"Bot {bot.id} | Follow ended. Arm resetting to 135.")

    
    """
    * Function Name: publish_pid_debug
    * Input: bot_id (int), tx, cx, ox (floats for X), ty, cy, oy (floats for Y), tt, ct, ot (floats for Theta)
    * Output: None
    * Logic: Publishes real-time PID data for visualization (e.g., in PlotJuggler).
    * 1. Checks if a publisher exists for the given bot.
    * 2. Normalizes angle values to [-180, 180] to prevent graphical "jumps" when crossing 0/360.
    * 3. Packs the Target, Current, and Output values for all three axes (X, Y, Theta) into a single array.
    *
    * Example Call: self.publish_pid_debug(0, 1000, 995, 10, 500, 502, -5, 90, 89, 0.5)
    """    
    def publish_pid_debug(self, bot_id, tx, cx, ox, ty, cy, oy, tt, ct, ot):
        """
        Publishes PID data for PlotJuggler.
        Params: Target, Current, Output (for X, Y, Theta)
        """
        if bot_id not in self.pid_debug_pubs:
            return

        msg = Float64MultiArray()
        
        # --- FIX: Normalize Theta values for clean plotting (-180 to 180) ---
        tt_norm = (tt + 180.0) % 360.0 - 180.0
        ct_norm = (ct + 180.0) % 360.0 - 180.0

        # Array Layout:
        # Index 0-2: X (Target, Current, Output)
        # Index 3-5: Y (Target, Current, Output)
        # Index 6-8: Theta (Target, Current, Output)
        msg.data = [
            float(tx), float(cx), float(ox),
            float(ty), float(cy), float(oy),
            float(tt_norm), float(ct_norm), float(ot)
        ]
        self.pid_debug_pubs[bot_id].publish(msg)

    """
    * Function Name: publish_perception_debug
    * Input: None
    * Output: None
    * Logic: Compiles a comprehensive JSON status packet for all robots and zones.
    * 1. Gathers lock status for pickup and drop zones.
    * 2. Iterates through all bots to determine their high-level "Status String" (e.g., WAITING_PICKUP, STAGING).
    * 3. Calculates dynamic visualization radii (e.g., shrink radius in tight zones).
    * 4. Publishes the JSON string to the `/perception_debug` topic for external visualization tools.
    *
    * Example Call: Triggered automatically by a timer (e.g., 5Hz).
    """
    
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
            
            # 2. Waiting for Drop Zone Lock (Gate Logic)
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
                            status = f"WAITING_DROP_{z}" # Triggers Red Text in visualizer

            # 3. Following another bot
            elif bot.is_following:
                status = f"FOLLOWING_{bot.follow_target_id}"
            
            # 4. Normal Targeting
            elif bot.current_crate: 
                status = f"TARGETING_{bot.tracked_crate_id}"

            # --- DYNAMIC RADIUS LOGIC ---
            if self.is_bot_physically_in_pickup_zone(bot):
                viz_radius = 100.0  # Small circle to allow entry/exit in tight zones
            elif bot.magnet_state != 0:
                viz_radius = 450.0  # Large circle for carrying crates
            else:
                viz_radius = self.follow_distance_threshold 

            
            goal_x = 0.0
            goal_y = 0.0
            if bot.goal is not None:
                goal_x = float(bot.goal[0])
                goal_y = float(bot.goal[1])

            bots_data[bid] = {
                "pose": list(bot.pose),
                "goal": list(bot.goal) if bot.goal is not None else [],
                "goal_x": goal_x, 
                "goal_y": goal_y,  
                "velocity": bot.velocity,
                "magnet": bot.magnet_state,
                "status": status,
                "is_following": bot.is_following,
                "carrying_crate": bot.current_crate['id'] if bot.current_crate else None,
                "pickup_zone": bot.pickup_zone_idx,
                "waiting_for_pickup_zone": bot.waiting_for_pickup_zone,
                "radius": viz_radius 
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


    # ---------------- PICKUP FAILURE LOGIC ----------------

    """
    * Function Name: check_pickup_failure
    * Input: bot (BotState) - The state object of the robot being checked.
    * Output: None
    * Logic: Validates if a crate is successfully attached during transport. 
    * 1. If vision detects the supposedly carried crate ID more than 400mm away, a failure is flagged.
    * 2. Triggers a full Recovery Sequence: updates crate coordinates, resets all FSM/navigation flags, turns off the magnet, and forces a re-approach.
    * * Example Call: self.check_pickup_failure(bot)
    """
    def check_pickup_failure(self, bot: BotState):
        """
        Checks if the robot has left the crate behind while moving to the drop zone.
        If the crate is visible > 400mm away, we assume failure, update coordinates, and retry.
        """
        # We only check for failure if we *think* we have the crate (move_after_attach is True)
        if not bot.move_after_attach:
            return

        # We need the ID of the crate we are supposedly carrying
        cid = bot.tracked_crate_id
        if cid is None:
            return

        
        live_crate = next((c for c in self.crates if c['id'] == cid), None)

        if live_crate:
            # 2. Calculate distance between Bot and the detected Crate
            dx = live_crate['x'] - bot.pose[0]
            dy = live_crate['y'] - bot.pose[1]
            dist = math.hypot(dx, dy)

            # 3. Threshold Check (400mm)
            if dist > 400.0:
                self.get_logger().error(
                    f"Bot {bot.id} | PICKUP FAILED! Crate {cid} detected {dist:.0f}mm away. Resetting to retry."
                )

            # --- RECOVERY SEQUENCE ---

                # A. Update the crate coordinates with the new live detection
                #    (Crucial because the bot likely pushed/displaced it)
                bot.current_crate = live_crate.copy()

                # B. Reset State Machine Flags to 'Start of Approach'
                bot.move_after_attach = False  # Stop going to drop zone

                bot.passed_crate_staging = False

                bot.arm_lifted = False         # Arm needs to go down
                bot.arm_placed = False         # Arm needs to be placed
                bot.goal_reached = False       # Will trigger move_near_crate logic

                bot.arm_placed_logged = False  # <--- THIS WAS MISSING
                bot.drop_pose_logged = False   # Reset this too for safety
                bot.wait_start_time = None     # Ensure timer is clear

                # C. Reset Hardware/Peripherals
                bot.magnet_state = 0           # Turn off magnet to reset cycle
                bot.mag_timer_start = None
                bot.ir_trigger_time = None
                bot.ir_detected = False

                # D. Reset Navigation Flags
                #    Force it to respect gates/staging again 
                bot.passed_gate = False
                bot.passed_pickup_staging = False
                bot.pickup_zone_idx = self.get_pickup_zone_idx_for_crate(live_crate) # Recalculate zone in case it moved
                
                # E. Reset PID and Force Stop momentarily
                bot.reset_pid()
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)

    """
    * Function Name: control_cb
    * Input: None
    * Output: None
    * Logic: The primary control loop for the multi-robot system, executed periodically by a ROS timer.
    * 1. Warmup Phase: Collects and averages vision data for several seconds to establish stable crate positions and initial assignments.
    * 2. Global State Updates: Resets speed scales, performs Hungarian crate reassignment, and handles idle bot staging/home logic.
    * 3. Safety: Executes the collision avoidance system to determine leader/follower relationships.
    * 4. State Machine Execution: Iterates through each robot to update servos, process follow behaviors, and advance the per-bot Finite State Machine (Approach -> Align -> Attach -> Lift -> Deliver).
    * 5. Synchronization: Implements slot-based waiting logic to ensure crates are dropped in an order that prevents physical blocking.
    *
    * Example Call: Automatically triggered by self.timer (0.03s interval).
    """

    def control_cb(self):
        # 1. Warmup Phase: Accumulate Data, Don't Move
        if not self.is_warmed_up:
            elapsed = time.time() - self.node_start_time
            
            # --- ACCUMULATE CRATE DATA ---
            for c in self.crates:
                cid = c['id']
                if cid not in self.warmup_crate_buffer:
                    self.warmup_crate_buffer[cid] = {'x':[], 'y':[], 'w':[]}
                self.warmup_crate_buffer[cid]['x'].append(c['x'])
                self.warmup_crate_buffer[cid]['y'].append(c['y'])
                self.warmup_crate_buffer[cid]['w'].append(c['w'])

            if elapsed < self.startup_delay:
                self.get_logger().info(f"Preparing... Gathering stable vision data ({elapsed:.1f}s)")
                return
            else:
                self.is_warmed_up = True
                
                # --- COMPUTE AVERAGED POSITIONS ---
                stable_crates = []
                for cid, data in self.warmup_crate_buffer.items():
                    if len(data['x']) > 5: 
                        avg_x = sum(data['x']) / len(data['x'])
                        avg_y = sum(data['y']) / len(data['y'])
                        avg_w = sum(data['w']) / len(data['w']) 
                        stable_crates.append({'id': cid, 'x': avg_x, 'y': avg_y, 'w': avg_w})
                
                if stable_crates:
                    self.get_logger().info(f"Warmup Complete. Optimization based on {len(stable_crates)} stable crates.")
                    temp_real_crates = self.crates
                    self.crates = stable_crates # Swap momentarily

                    if not self.drop_positions_assigned:
                        self.assign_drop_positions()
                        self.drop_positions_assigned = True
                        self.get_logger().info("Drop positions assigned based on WARMUP data.")

                    self.assign_crates_greedily() # <--- INITIAL ASSIGNMENT
                    self.crates = temp_real_crates # Swap back
                else:
                    self.get_logger().warn("Warmup ended but no crates seen!")

        # 2. Reset Slowdown (Normal Loop)
        for bot in self.bots.values():
            bot.slowdown_scale = 1.0

        # 3. RUN ASSIGNMENT (greedily)
        self.assign_crates_greedily()

        

        # 4. Idle and Staging Logic
        for bot in self.bots.values():
            # Only send idle bots home/staging if they aren't doing anything else
            if (bot.current_crate is None and not bot.return_to_start and
                not bot.move_after_attach and not bot.going_to_staging_point and 
                not bot.has_returned_home and not bot.is_permanently_idle):

                if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
                    bot.going_to_staging_point = True
                    
                    # [SMARTER STAGING LOGIC]
                    if bot.id == 0:
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

        # 5. Collision Avoidance
        self.check_and_handle_collisions()

        # 6. Execute Per-Bot State Machine
        for bot in self.bots.values():
            if bot.pose is None:
                continue

            # Always update arm smooth
            self.update_arm_smooth(bot)

            # FOLLOW behavior
            if bot.is_following and bot.follow_target_id is not None:
                self.follow_bot(bot, bot.follow_target_id)
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

            # ---------------- Magnet HOLD Transition ----------------
            now_wall = time.time()
            if bot.magnet_state == 10 and bot.mag_timer_start is not None:
                if now_wall - bot.mag_timer_start >= 5.0:
                    bot.magnet_state = 10
                    bot.mag_timer_start = None
                    self.get_logger().info(f"Bot {bot.id} | Magnet switched to HOLD mode")

            # ---------------- FSM Execution ----------------
            if not bot.goal_reached:
                self.move_near_crate(bot)

            elif not bot.arm_placed:
                
                should_wait = False

                if bot.tracked_crate_id is not None:
                    # 1. Get the assigned drop position for this crate
                    dpos = self.drop_positions.get(bot.tracked_crate_id)
                    
                    if dpos:
                        slot_idx = dpos.get('zone_slot')
                        
                        # 2. Identify which zone (0, 1, or 2) this belongs to
                        zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
                        
                        # 3. Count how many crates are ALREADY completed in this specific zone
                        completed_in_zone = 0
                        for cid in self.completed_crates:
                            if self.get_zone_for_crate(cid) == zone_id:
                                completed_in_zone += 1
                        
                        count_in_zone = self.zone_drop_counts.get(zone_id, 0)

                        # Case A: 3-Crate Setup (Triangle)
                        # Slot 2 is Back-Center. Needs Front Left (0) and Front Right (1) done.
                        if count_in_zone <= 3:
                            if slot_idx == 2:
                                if completed_in_zone < 2:
                                    should_wait = True
                                    self.get_logger().info(
                                        f"Bot {bot.id} | Back Center Wait (Slot 2): Waiting for Front Row (Done: {completed_in_zone}/2)...",
                                        throttle_duration_sec=2.0
                                    )

                        # Case B: 5-Crate Setup (Pentagon/Trapezoid)
                        # Slots 3 & 4 are Back Row. Need Front Row (0, 1, 2) done.
                        else:
                            if slot_idx in [3, 4]:
                                if completed_in_zone < 3:
                                    should_wait = True
                                    self.get_logger().info(
                                        f"Bot {bot.id} | Back Row Wait (Slot {slot_idx}): Waiting for Front Row (Done: {completed_in_zone}/3)...",
                                        throttle_duration_sec=2.0
                                    )

                if should_wait:
                    
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)
                    continue
                # ---------------------------------------------------------

                # Arm DOWN + Magnet ON (State 10 = Pull)
                if bot.magnet_state == 0:
                    bot.magnet_state = 10
                    bot.mag_timer_start = now_wall

                base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
                
                bot.arm_target_elbow = elbow_down
                elbow_is_positioned = abs(bot.elbow_angle - elbow_down) < 2.0
                
                if not elbow_is_positioned:
                    bot.arm_target_base = 45.0
                else:
                    bot.arm_target_base = base_down

                arm_done = self.update_arm_smooth(bot)
                self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

                if arm_done and not bot.arm_placed_logged:
                    bot.arm_placed = True
                    bot.arm_placed_logged = True
                    bot.wait_start_time = time.time() 
                    self.get_logger().info(f"Bot {bot.id} | Arm placed. Magnet ON. Waiting 2.0s...")

            elif not bot.arm_lifted:
                if bot.wait_start_time is not None:
                    if time.time() - bot.wait_start_time < 2.0:
                        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 10)
                        continue 
                    else:
                        bot.wait_start_time = None 

                # Lift Logic
                self.lift_arm_with_crate(bot)

            elif bot.move_after_attach:
                self.check_pickup_failure(bot)
                if bot.move_after_attach:
                    self.move_to_final_goal(bot)

    """
    * Function Name: attach_service_cb
    * Input: request (Trigger.Request), response (Trigger.Response)
    * Output: response (Trigger.Response) - success (bool), message (str)
    * Logic: Callback for the '/attach' ROS 2 service. 
    * 1. Iterates through the state of all bots managed by the node.
    * 2. Checks if any bot has its `magnet_state` active (non-zero).
    * 3. Returns a boolean success flag to the service caller indicating if at least one magnet is engaged.
    *
    * Example Call: ros2 service call /attach std_srvs/srv/Trigger {}
    """
    def attach_service_cb(self, request, response):
        magnet_on = False

        for bot in self.bots.values():
            # Magnet ON states in your code are 10 (pull) and 9 (hold)
            if bot.magnet_state != 0:
                magnet_on = True
                break

        response.success = magnet_on
        response.message = "Magnet ON" if magnet_on else "Magnet OFF"
        return response



    # ---------------- Helper: Triangle Inclusion Check (Arrowhead / Wedge) ----------------

    """
    * Function Name: is_in_triangle
    * Input: observer_bot (BotState), target_bot (BotState), r (float) - Radius/Range of the wedge.
    * Output: bool - True if the target bot's footprint intersects the safety wedge.
    * Logic: Implements a geometric "Safety Wedge" (equilateral triangle) projection.
    * 1. Transforms the target bot's coordinates into the observer bot's local coordinate frame.
    * 2. Accounts for the physical size (bot_radius) of the bots by creating a "Minkowski Sum" buffer.
    * 3. Checks if the target's center lies within the expanded equilateral wedge boundaries.
    *
    * Example Call: collision = self.is_in_triangle(bot_0, bot_2, 550.0)
    """
    def is_in_triangle(self, observer_bot, target_bot, r):
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
    """
    * Function Name: follow_bot
    * Input: follower (BotState), leader_id (int)
    * Output: None
    * Logic: Implements reactive station-keeping to prevent collisions.
    * 1. Validates that the follower is not already idle or home.
    * 2. Calculates the distance vector between the follower and the leader.
    * 3. If the distance is below the 'locked' safety threshold, it calculates a 
    * repulsive goal and forces the robot to move backward to maintain the gap.
    * 4. If the distance is safe, it forces the robot to wait (zero velocity) 
    * until the leader moves further away.
    *
    * Example Call: self.follow_bot(bot_2, 0)
    """
    def follow_bot(self, follower: BotState, leader_id: int):
        
        # 1. SAFETY CHECK (UNCHANGED)
        if (follower.has_returned_home or follower.is_permanently_idle):
            follower.is_following = False
            follower.follow_target_id = None
            self.publish_wheel_velocities([0.0,0.0,0.0], follower.id, follower.base_angle, follower.elbow_angle, follower.magnet_state)
            return
        
        follower.arm_target_base = 45.0
        follower.arm_target_elbow = 120.0

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

        # 1. SAFETY VIOLATION: We are too close -> BACK UP
        if dist < (target_dist - 10.0): # 10mm Deadband
             if dist > 0.1:
                 ux, uy = dx/dist, dy/dist
             else:
                 # If robots are exactly on top of each other (vision glitch), 
                 # pick an arbitrary direction (e.g., X axis) to force separation.
                 ux, uy = 1.0, 0.0
             # Calculate point away from leader
            #  ux, uy = dx/dist, dy/dist
             target_x = leader.pose[0] - ux * target_dist
             target_y = leader.pose[1] - uy * target_dist

             self.get_logger().info(
                 f"[COLLISION] Bot {follower.id} BACKING UP | Dist: {dist:.0f} < {target_dist:.0f}"
             )
             self.move_to_point(follower, np.array([target_x, target_y, 0.0]), "Backing Up")

        # 2. SAFE ZONE: We are at or beyond the safe distance -> JUST WAIT
        else:
             # Do NOT catch up. Just hold position.
             # The 'check_and_handle_collisions' loop will naturally set 
             # is_following = False once the leader moves far enough away.
             self.get_logger().info(
                 f"[COLLISION] Bot {follower.id} WAITING | Dist: {dist:.0f} (Safe)"
             )
             self.publish_wheel_velocities([0.0, 0.0, 0.0], follower.id, follower.base_angle, follower.elbow_angle, follower.magnet_state)
             follower.is_moving = False

    # ---------------- Smooth Arm Update ----------------
    """
    * Function Name: update_arm_smooth
    * Input: bot (BotState) - The robot object containing current and target arm angles.
    * Output: bool - Returns True if both the base and elbow servos have reached their target angles.
    * Logic: Implements a linear interpolation (LERP) style smoothing for servo movements.
    * 1. Defines a nested 'move' helper to increment/decrement angles by a fixed step.
    * 2. Updates the 'base_angle' and 'elbow_angle' independently.
    * 3. Prevents sudden jerky movements by limiting the change per control cycle to 'bot.arm_step'.
    *
    * Example Call: is_finished = self.update_arm_smooth(bot)
    """
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

    """
    * Function Name: apply_crate_avoidance
    * Input: bot (BotState), vx_pid (float), vy_pid (float)
    * Output: final_vx (float), final_vy (float)
    * Logic: Implements a Potential Field obstacle avoidance algorithm. 
    * 1. Checks proximity to the final goal; if within 300mm, avoidance is disabled to allow precision docking.
    * 2. Iterates through all visible crates and idle bots, calculating a repulsive force vector for any object within the 'avoid_radius'.
    * 3. Scales the repulsive force based on proximity (closer objects exert more "push").
    * 4. Sums these vectors and adds them to the original PID velocity to steer the bot around obstacles.
    *
    * Example Call: vx, vy = self.apply_crate_avoidance(bot, vx_pid, vy_pid)
    """
    def apply_crate_avoidance(self, bot, vx_pid, vy_pid):
        
        if bot.pose is not None and bot.goal is not None:
            dist_to_goal = math.hypot(bot.goal[0] - bot.pose[0], bot.goal[1] - bot.pose[1])
            
            # If we are close to the goal (docking/dropping), disable avoidance
            # so the bot doesn't get pushed away from its target.
            if dist_to_goal < 300.0:
                return vx_pid, vy_pid

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
    
    
    """
    * Function Name: is_bot_in_zone
    * Input: bot_x (float), bot_y (float), zone_idx (int)
    * Output: bool - True if the coordinates are within the specified pickup zone.
    * Logic: Checks if a robot's current (x, y) position is within the rectangular bounds 
    * of a specific pickup zone. A 50mm safety buffer is added to all sides of the zone 
    * boundary to account for the robot's physical footprint and ensure reliable detection.
    *
    * Example Call: if self.is_bot_in_zone(curr_x, curr_y, 0): ...
    """
    def is_bot_in_zone(self, bot_x, bot_y, zone_idx):
        z = self.pickup_zones[zone_idx]
        return (z['x_min']-50 <= bot_x <= z['x_max']+50 and 
                z['y_min']-50 <= bot_y <= z['y_max']+50)
    
    """
    * Function Name: move_near_crate
    * Input: bot (BotState) - The robot object containing current pose, ID, and assigned crate data.
    * Output: None
    * Logic: Manages the high-precision approach to a target crate for pickup.
    * 1. Target Geometry: Calculates the optimal approach point by checking 4 orthogonal candidates around the crate and snapping to the nearest 90-degree alignment.
    * 2. Zone Control: Checks if the target pickup zone is locked by another robot; waits outside if occupied.
    * 3. Two-Stage Navigation: Executes a "Staging" move to align with the correct approach lane before moving to the final pickup coordinates.
    * 4. Latency Compensation: Calculates "Phase Lead" based on current velocity to dynamically adjust stop tolerances.
    * 5. Stiction Fix: Boosts wheel power if the robot is stalled by motor friction near the target.
    * 6. Termination: Stops and marks 'goal_reached' when within dynamic tolerances and verified by IR sensor.
    *
    * Example Call: self.move_near_crate(bot)
    """
  
    def move_near_crate(self, bot: BotState):
        bot.is_moving = True
        crate = bot.current_crate
        if crate is None: return

        if bot.id == 0:   ox, oy = 8, 120
        elif bot.id == 2: ox, oy = 10, 120
        elif bot.id == 4: ox, oy = 19, 120
        else:             ox, oy = 25, 120
            
        cx, cy, cw_deg = crate['x'], crate['y'], crate['w']
        
        # Normalize angle to [0, 360)
        final_theta_rad = 0.0 # <--- Add this line
        norm_w = cw_deg % 360.0
        
        # Check if we are close to 0, 90, 180, 270, or 360 (+/- 5 deg)
        is_aligned = False
        aligned_angle_deg = 0.0

        for check_angle in [0.0, 90.0, 180.0, 270.0, 360.0]:
            if abs(norm_w - check_angle) <= 5.0:
                is_aligned = True
                aligned_angle_deg = check_angle
                break
        
        # --- STEP B: SELECT ROTATION ANGLE ---
        if is_aligned:
            # Snap to exact grid (0, 90, etc.) to match your "earlier" behavior
            use_angle_rad = math.radians(aligned_angle_deg)
        else:
            # Use exact angle for 45-degree crates
            use_angle_rad = math.radians(cw_deg)

        # --- STEP C: FIND NEAREST EDGE (4 Candidates) ---
        rx, ry, _ = bot.pose
        candidates = []
        
        # Base vector: approach from "Bottom" relative to crate face
        vec_x = -ox
        vec_y = -oy

        for i in range(4):
            # Rotate by 0, 90, 180, 270 relative to the CHOSEN angle
            angle = use_angle_rad + (i * math.pi / 2.0)

            # 2D Rotation
            rot_off_x = vec_x * math.cos(angle) - vec_y * math.sin(angle)
            rot_off_y = vec_x * math.sin(angle) + vec_y * math.cos(angle)

            candidates.append((cx + rot_off_x, cy + rot_off_y, angle))

        # Pick the point closest to the robot
        final_x, final_y, final_theta_rad = min(candidates, key=lambda p: math.hypot(p[0]-rx, p[1]-ry))

        final_theta_rad = self.normalize_angle(final_theta_rad)
        
        # Identify Target Zone
        target_zone_idx = -1
        for i, z in enumerate(self.pickup_zones):
            if (z['x_min'] <= crate['x'] <= z['x_max'] and 
                z['y_min'] <= crate['y'] <= z['y_max']):
                target_zone_idx = i
                break

        if bot.pickup_zone_idx is None:
            bot.pickup_zone_idx = target_zone_idx

        if bot.pickup_zone_idx is not None and bot.pickup_zone_idx != -1 and not bot.passed_gate:
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
            
            
        
        # Default Target is the Crate (Final Pickup Point)
        target_x, target_y = final_x, final_y
        label = "Crate Approach"

        # Check if we need to do the staging move first
        if not bot.passed_crate_staging:
            
            staging_x = target_x 

            # Zones P1 (0) and P3 (2) -> Stage at X + 200
            if bot.pickup_zone_idx in [0, 2]: 
                staging_x = crate['x'] + 200.0
                
            # Zones P2 (1) and P4 (3) -> Stage at X - 200
            elif bot.pickup_zone_idx in [1, 3]: 
                staging_x = crate['x'] - 200.0
            
            # Use final_y so we align with the correct approach lane
            staging_y = final_y 

            # Calculate distance to this staging point
            # Use raw pose for simple distance check or filtered if preferred
            curr_x = bot.pose[0]
            curr_y = bot.pose[1]
            dist_to_stage = math.hypot(staging_x - curr_x, staging_y - curr_y)

            # --- CHECK ARRIVAL AT STAGE POINT ---
            if dist_to_stage < 40.0: # 40mm Tolerance
                bot.passed_crate_staging = True
                bot.reset_pid() # Reset PID to prevent jerk when switching targets
                self.get_logger().info(f"Bot {bot.id} | Staging Complete. Moving to Final Pickup.")
            else:
                # OVERRIDE the target to the staging point
                target_x = staging_x
                target_y = staging_y
                label = "Crate Pre-Stage"



        # 3. SET GOAL & ARM
        # -----------------
        bot.goal = np.array([target_x, target_y, math.degrees(final_theta_rad)])
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

        # 1. Use the helper to get filtered values (Theta is in RADIANS)
        x, y, theta = self.get_filtered_pose(bot) 
        
        # 2. Convert to degrees immediately so your existing logic (theta_error_deg) works
        theta_deg = math.degrees(theta) 
        dist_error = math.hypot(target_x - x, target_y - y)

        goal_theta = math.degrees(final_theta_rad) 
        
        theta_error_deg = abs(
            (goal_theta - theta_deg + 180.0) % 360.0 - 180.0
        )

        log_theta = math.degrees(final_theta_rad) if label == "Crate Approach" else 0.0
        self.log_bot_distance(bot, target_x, target_y, label, gtheta=log_theta)

        x_ok = False

 
        # 1. Calculate Alignment Flags
        dx_error = abs(target_x - x)
        dy_error = abs(target_y - y)

        radial_ok = dist_error < self.xy_tolerance

        
        # 1. Calculate raw errors
        dx_error = abs(target_x - x)
        dy_error = abs(target_y - y)
        # (You already have theta_error_deg calculated above)


        comm_latency = 0.15 # 150ms prediction window

        # 3. Initialize Velocity Estimates
        current_vx_mm_s = 0.0
        current_vy_mm_s = 0.0
        current_omega_deg_s = 0.0

        if dt > 0:
            # Estimate X Velocity: |Current Error - Previous Error| / dt
            # We use abs() on prev_error to ensure we are comparing magnitudes
            prev_error_x = abs(bot.pid_x.prev_error)
            current_vx_mm_s = abs(dx_error - prev_error_x) / dt

            # Estimate Y Velocity
            prev_error_y = abs(bot.pid_y.prev_error)
            current_vy_mm_s = abs(dy_error - prev_error_y) / dt

            # Estimate Angular Velocity (You already had this)
            prev_error_deg = math.degrees(bot.pid_theta.prev_error)
            current_omega_deg_s = abs(theta_error_deg - abs(prev_error_deg)) / dt

        # 4. Calculate Dynamic Buffers (Distance traveled during latency)
        phase_lead_x = current_vx_mm_s * comm_latency
        phase_lead_y = current_vy_mm_s * comm_latency
        phase_lead_theta = current_omega_deg_s * comm_latency

        # 5. Apply Dynamic Tolerances
        # Base tolerance + buffer
        raw_x = self.xy_tolerance_x + phase_lead_x
        raw_y = self.xy_tolerance_y + phase_lead_y
        
        raw_theta_tol = self.xy_tolerance_theta + phase_lead_theta
        dynamic_theta_tol = min(raw_theta_tol, 5.0)

        dynamic_x_tol = min(raw_x, 5.0)
        dynamic_y_tol = min(raw_y, 5.0)

# 6. Update the 'OK' Flags using the new dynamic tolerances
        x_ok = dx_error < dynamic_x_tol
        y_ok = dy_error < dynamic_y_tol
        theta_ok = theta_error_deg < dynamic_theta_tol

        # 2. STOP LOGIC WITH DWELL TIME:
        if y_ok and x_ok and theta_ok and radial_ok :
            # Start the timer if it hasn't been started yet
            if bot.alignment_start_time is None:
                bot.alignment_start_time = time.time()
                self.get_logger().info(f"Bot {bot.id} | In tolerance. Waiting 0.5s to stabilize...")
            
            # Stop the wheels while waiting
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)
            bot.is_moving = False
            
            # Check if 0.5 seconds have passed
            if time.time() - bot.alignment_start_time >= 0.5:
                bot.goal_reached = True
                bot.alignment_start_time = None # Reset for next time
                self.get_logger().info(
                    f"Bot {bot.id} | Pickup aligned successfully | "
                    f"dist={dist_error:.1f}, dx={dx_error:.1f}, dy={dy_error:.1f}, dtheta={theta_error_deg:.1f}"
                )
            return
        else:
            # If the robot drifts out of tolerance during the wait, reset the timer
            if bot.alignment_start_time is not None:
                bot.alignment_start_time = None
                self.get_logger().info(f"Bot {bot.id} | Drifted out of tolerance. Re-aligning...")


        # 5. PID COMPUTATION
        # ------------------
        vx_pid = bot.pid_x.compute(target_x - x, dt, integration_window=50.0, deadband=2.0)
        vy_pid = bot.pid_y.compute(target_y - y, dt, integration_window=50.0, deadband=2.0)

        
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
        if dist_error < self.slow_radius:
            bot.slowdown_scale = 0.5

        if bot.ir_detected and dist_error < 80:
             bot.slowdown_scale = 0.3


        theta = math.radians(theta_deg)
        # goal_theta = 0.0
        goal_theta = final_theta_rad
        # if label == "Crate Approach":
        #     goal_theta = final_theta_rad
        # else:
        #     goal_theta = 0.0

        error_theta = (goal_theta - theta + math.pi) % (2*math.pi) - math.pi
        omega = bot.pid_theta.compute(error_theta, dt, math.radians(20.0))

        self.publish_pid_debug(
            bot.id,
            target_x, x, vx_pid,       # X Axis
            target_y, y, vy_pid,       # Y Axis
            math.degrees(goal_theta), theta_deg, omega      # Theta Axis (Target is 0.0)
        )

        vx *= bot.slowdown_scale
        vy *= bot.slowdown_scale
        omega *= bot.slowdown_scale

        min_stiction_cw = 15.0   # Minimum positive power (CW)
        min_stiction_ccw = 0.5   # Minimum negative power (CCW)
        soft_theta_tol = math.radians(3.0) # Tolerance to stop jittering

        # If within 20mm, ignore PID magnitude and use FIXED stiction speeds
        if dist_error < 20.0 and abs(error_theta) > soft_theta_tol:
            
            if omega > 0:
                # Force exact CW stiction speed
                omega = min_stiction_cw
            elif omega < 0:
                # Force exact CCW stiction speed
                omega = -min_stiction_ccw
            
            # Reset integral to prevent windup during this clamped movement
            bot.pid_theta.integral = 0.0
            
            # Debug log (optional)
            # self.get_logger().info(f"Bot {bot.id} | Soft Align: Force Omega={omega}")

        min_stiction_vel = 4.0   # Minimum PWM needed to turn wheels
        stiction_tol = 2.5        # mm (Don't boost if within 8mm)

        min_stiction_omega = 13.0 if bot.id == 4 else 13.0  # Minimum rad/s to overcome motor stiction
        theta_tol_deg = 3.0 if bot.id == 4 else 3.0       # Don't boost if within 2 degrees of target
        theta_tol = math.radians(theta_tol_deg)

        stiction_active_axes = []

        # Boost X
        if abs(target_x - x) > stiction_tol:
            if abs(vx) < min_stiction_vel:
                # Keep sign, force magnitude
                vx = math.copysign(min_stiction_vel, vx)
                bot.pid_x.integral = 0.0
                stiction_active_axes.append("X")

        # Boost Y
        if abs(target_y - y) > stiction_tol:
            if abs(vy) < min_stiction_vel:
                vy = math.copysign(min_stiction_vel, vy)
                bot.pid_y.integral = 0.0
                stiction_active_axes.append("Y")

        min_stiction_cw = 19.0   # Minimum positive power
        min_stiction_ccw = 1.0   # Minimum negative power (magnitude)

        if abs(error_theta) > theta_tol:
            
            # Case 1: PID wants to move CW (Positive Output)
            if omega > 0:
                if omega < min_stiction_cw:
                    omega = min_stiction_cw
                    bot.pid_theta.integral = 0.0
                    stiction_active_axes.append("Theta_CW(+)")

            # Case 2: PID wants to move CCW (Negative Output)
            elif omega < 0:
                # Check if the negative value is "weaker" than -1 (e.g., -0.5)
                if omega > -min_stiction_ccw:
                    omega = -min_stiction_ccw
                    bot.pid_theta.integral = 0.0
                    stiction_active_axes.append("Theta_CCW(-)")

        if stiction_active_axes:
            self.get_logger().info(
                f"Bot {bot.id} | STICTION FIX APPLIED: {stiction_active_axes}",
                # throttle_duration_sec=1.0
            )

        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx * cos_t + vy * sin_t
        vy_r = -vx * sin_t + vy * cos_t
        
        wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])
        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, 0)
        

    def handle_backup(self, bot: BotState):
        # Move to the backup goal
        self.log_bot_distance(bot, bot.goal[0], bot.goal[1], "backup clearance", gtheta=bot.goal[2])

        # If backup position reached
        if self.move_to_point(bot, bot.goal, "backup"):
            self.get_logger().info(f"Bot {bot.id} | Backup complete. Safe to proceed.")
            
            bot.backing_up = False

            if bot.current_crate is not None:
                zone_id = self.get_zone_for_crate(bot.current_crate['id'])
                if self.zone_locks.get(zone_id) == bot.id:
                    self.zone_locks[zone_id] = None
                    self.get_logger().info(f"Bot {bot.id} | UNLOCKED Drop Zone {zone_id}")
            
            bot.current_crate = None
            bot.tracked_crate_id = None
            bot.arm_lifted = False
            bot.arm_placed = False

            bot.drop_zone_released_early = False

            
            # Reset arm targets to safe travel position
            bot.arm_target_base = 45.0
            bot.arm_target_elbow = 135.0

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
        # Define the final lifted position
        final_base = 45.0
        final_elbow = 135.0

        # Get the current 'down' configuration
        base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
        
        
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

    """
    * Function Name: is_in_drop_zone
    * Input: bot (BotState) - The robot object containing the current pose data.
    * Output: bool - True if the robot is physically located within any drop zone boundary.
    * Logic: Iterates through the `drop_zones` dictionary. 
    * 1. Extracts the current (x, y) coordinates of the robot.
    * 2. Checks if these coordinates fall within the rectangular bounds of any defined drop zone.
    * 3. Applies a 50mm safety padding to the boundaries to ensure the robot is fully accounted for.
    *
    * Example Call: if self.is_in_drop_zone(bot): ...
    """
    def is_in_drop_zone(self, bot: BotState):
        if bot.pose is None: return False
        x, y, _ = bot.pose
        # Check all drop zones with padding
        for z in self.drop_zones.values():
            if (z['x_min'] - 50 <= x <= z['x_max'] + 50 and 
                z['y_min'] - 50 <= y <= z['y_max'] + 50):
                return True
        return False
    
    """
    * Function Name: move_to_final_goal
    * Input: bot (BotState) - The state object of the robot currently carrying a crate.
    * Output: None
    * Logic: Manages the complex navigation and synchronization logic for dropping a crate.
    * 1. Target Retrieval: Identifies the assigned drop zone and specific slot coordinates.
    * 2. Staging Phase: Navigates to a point 250mm away from the slot to ensure a straight approach.
    * 3. Slot Synchronization: Implements a "wait-your-turn" logic based on slot priority to prevent 
    * physical collisions in multi-crate layouts (e.g., Back row waits for Front row).
    * 4. Adaptive Arm Positioning: Tucks the arm in for tight back-row slots or extends for front-row slots.
    * 5. Precision Docking: Executes final approach with high-tolerance PID control. 
    * 6. State Transition: Switches the bot to the 'dropping' state once precisely aligned.
    *
    * Example Call: self.move_to_final_goal(bot)
    """
    
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

        if drop is None: return

        if not bot.passed_drop_staging:
            # Same Y as final approach (y - 150), but backed off X by 100
            staging_goal = np.array([drop['x'], drop['y'] - 250.0, 0.0])
            bot.goal = staging_goal.copy()

            # Move there. Once close (tolerance 40mm), mark done.
            if self.move_to_point(bot, staging_goal, label="Drop Staging", xy_tol=40.0):
                bot.passed_drop_staging = True
                bot.reset_pid() # Reset PID so we don't overshoot the next move
            
            return # Stop here. Next cycle will handle the final move.

        # 3. Slot Priority Logic (5-Crate Setup)
        my_slot = drop.get('zone_slot', 99)
        zone_id = self.get_zone_for_crate(crate_id) # Ensure this is defined
        count_in_zone = self.zone_drop_counts.get(zone_id, 0)

        if my_slot in [3, 4]:
            # Tucked in position for back row crates
            bot.arm_target_base = 45.0
            bot.arm_target_elbow = 110.0
        else:
            # Standard carry position for front row (0, 1, 2)
            bot.arm_target_base = 45.0
            bot.arm_target_elbow = 135.0
        
       
        
        required_completed = 0
        should_wait = False

        if count_in_zone <= 3:
            # If we only have 3 crates, Slot 2 must wait for 2 crates (0 and 1)
            if my_slot == 2:
                required_completed = 2
                should_wait = True
        else:
            # If we have >3 crates, Slots 3 and 4 must wait for 3 crates (0, 1, 2)
            if my_slot in [3, 4]:
                required_completed = 3
                should_wait = True
            
        if should_wait:
            completed_in_zone = 0
            for cid in self.completed_crates:
                if self.get_zone_for_crate(cid) == zone_id:
                    completed_in_zone += 1
            
            # If the zone isn't ready for this slot yet, HOLD POSITION.
            if completed_in_zone < required_completed:
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                bot.is_moving = False
                
                # Log occasionally (every ~2 seconds) to avoid spam
                if int(time.time()) % 2 == 0:
                     self.get_logger().info(f"Bot {bot.id} | Waiting for front row. (Done: {completed_in_zone}/{required_completed})")
                return

        # 4. Define the final drop target
        # Applying the -150.0 Y offset for the arm reach
        final_goal = np.array([drop['x'], drop['y'] - 150.0, 0.0])
        bot.goal = final_goal.copy()

        # 5. EXECUTE MOVEMENT WITH HIGH-PRECISION TOLERANCE
        # We pass the specific x, y, and theta tolerances to move_to_point
        reached = self.move_to_point(
            bot, 
            final_goal, 
            label="Drop Slot Alignment", 
            xy_tol=self.xy_tolerance,         # Radial tolerance (15.0)
            theta_tol=5 # Yaw tolerance (5.0 degrees)
        )

        if reached:
            # Additional check for independent X and Y axis precision
            x, y, _ = self.get_filtered_pose(bot)
            dx_error = abs(final_goal[0] - x)
            dy_error = abs(final_goal[1] - y)

            if dx_error < self.xy_tolerance_x and dy_error < self.xy_tolerance_y:
                bot.move_after_attach = False
                bot.dropping = True           
                bot.mag_timer_start = None    
                bot.drop_pose_logged = False
                self.get_logger().info(f"Bot {bot.id} | Precisely Aligned at Drop: dx={dx_error:.1f}, dy={dy_error:.1f}")

    """
    * Function Name: drop_crate
    * Input: bot (BotState) - The state object of the robot performing the drop.
    * Output: None
    * Logic: Executes the physical sequence to release a crate into a drop slot.
    * 1. Dynamic Configuration: Adjusts target arm angles based on the specific slot layout (3-crate vs 5-crate mode).
    * 2. Sequential Movement: Lowers the elbow servo first, then the base servo to ensure the crate clears the slot boundary.
    * 3. Resource Management: Implements an "Early Unlock" of the drop zone as soon as the arm starts lowering to allow other bots to queue.
    * 4. Actuation: Disengages the electromagnet after a 1.5-second settle time to ensure a clean release.
    * 5. State Transition: Marks the crate as completed and triggers a 100mm backward move to safely clear the drop zone.
    *
    * Example Call: self.drop_crate(bot)
    """
    def drop_crate(self, bot: BotState):

        if bot.magnet_state != 0: 
            bot.magnet_state = 10

        base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))

        if bot.current_crate:
             cid = bot.current_crate['id']
             dpos = self.drop_positions.get(cid)
             # Index 2 corresponds to the 3rd crate (0, 1, 2)
             if dpos:
                 slot_idx = dpos.get('zone_slot')
                 
                 zone_id = self.get_zone_for_crate(cid)
                 count_in_zone = self.zone_drop_counts.get(zone_id, 0)

                 # ---------------- UPDATED LOGIC START ----------------
                 # CASE 1: 3 Crates (or fewer) in Zone
                 # In this layout, Slot 2 is the "Back Center" slot.
                 if count_in_zone <= 3:
                     if slot_idx == 2:
                         base_down = 0.0
                         elbow_down = 90.0
                         self.get_logger().info(f"Bot {bot.id} | Special Drop (3-Crate Mode, Slot {slot_idx}) -> Base:0, Elbow:90")

                 # CASE 2: 5 Crates (or >3) in Zone
                 # In this layout, Slots 3 & 4 are the "Back Row" slots.
                 else:
                     if slot_idx in [3, 4]:
                          base_down = 0.0
                          elbow_down = 90.0
                          self.get_logger().info(f"Bot {bot.id} | Special Drop (5-Crate Mode, Slot {slot_idx}) -> Base:0, Elbow:90")
        
        # Goal: Move Elbow first, THEN Move Base
        
        # 1. Always target the elbow down
        bot.arm_target_elbow = elbow_down

        
        if not bot.drop_zone_released_early:
            zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
            if self.zone_locks.get(zone_id) == bot.id:
                self.zone_locks[zone_id] = None
                bot.drop_zone_released_early = True
                self.get_logger().info(
                    f"Bot {bot.id} | EARLY UNLOCK Drop Zone {zone_id} (Arm lowering started)"
                )
        


        # 2. Check if the elbow has reached the target (within 2 degrees tolerance)
        elbow_is_positioned = abs(bot.elbow_angle - elbow_down) < 2.0

        if not elbow_is_positioned:
            # If Elbow is moving, keep Base UP (Standard carry angle is 45.0)
            bot.arm_target_base = 45.0
        else:
            # If Elbow is ready, allow Base to go DOWN
            bot.arm_target_base = base_down
        

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
            self.completed_crates.add(crate_id)   
            

        # LOG MATCHING REFERENCE
        self.get_logger().info(f"Bot {bot.id} | Crate released (magnet OFF)")

        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        
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

    """
    * Function Name: move_to_staging
    * Input: bot (BotState) - The robot state object currently executing the staging maneuver.
    * Output: None
    * Logic: Manages the transition from the active arena area to an idle home state via an intermediate waypoint.
    * 1. Logs telemetry data regarding the distance to the staging point.
    * 2. Executes a movement command to the staging goal.
    * 3. Upon arrival, switches the bot's state from 'staging' to 'returning_home'.
    * 4. Dynamically assigns the correct 'Home Goal' based on the unique Robot ID and resets PID to ensure a smooth transition.
    *
    * Example Call: self.move_to_staging(bot)
    """

    def move_to_staging(self, bot: BotState):
        # Log dist
        self.log_bot_distance(bot, bot.goal[0], bot.goal[1], "staging point", gtheta=bot.goal[2])
        
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

    """
    * Function Name: return_home
    * Input: bot (BotState) - The state object of the robot currently returning to base.
    * Output: None
    * Logic: Finalizes the robot's operation by navigating it to its designated home position.
    * 1. Logs the distance to the home goal for telemetry.
    * 2. Executes the final navigation leg using move_to_point.
    * 3. Once reached, sets permanent idle flags to disable future task assignments.
    * 4. Resets the internal bot state and disables collision-following behavior.
    *
    * Example Call: self.return_home(bot)
    """

    def return_home(self,bot:BotState):
        # Log distance
        self.log_bot_distance(bot, bot.return_goal[0], bot.return_goal[1], "start position", gtheta=bot.return_goal[2])

        if self.move_to_point(bot,bot.return_goal, "home"):
            bot.return_to_start = False
            bot.has_returned_home = True
            unassigned_crates = [c for c in self.crates if c['id'] not in self.completed_crates]
            if len(unassigned_crates) == 0:
                bot.is_permanently_idle = True
            else:
                bot.is_permanently_idle = False
            bot.goal = bot.return_goal.copy()

            bot.is_following = False
            bot.follow_target_id = None
            # LOG MATCHING REFERENCE
            self.get_logger().info(f"Bot {bot.id} | Returned home. Robot stopped.")
            self.reset_bot_state(bot)

    """
    * Function Name: move_to_point
    * Input: bot (BotState), goal (np.array), label (str), xy_tol (float, optional), theta_tol (float, optional)
    * Output: bool - Returns True if the robot has reached the target point within the specified tolerances.
    * Logic: The core holonomic navigation engine.
    * 1. Timing: Calculates 'dt' with safety clamping for stable PID differentiation.
    * 2. Perception: Retrieves filtered pose data and calculates Euclidean/angular errors.
    * 3. Velocity Calculation: Computes independent PID signals for X, Y, and Theta, then applies obstacle avoidance.
    * 4. Scaling: Applies speed reduction based on distance to goal or specific state labels (e.g., staging).
    * 5. Stiction Fix: Injects a minimum PWM floor if the robot is stalled due to mechanical friction.
    * 6. Kinematics: Transforms global velocities into the robot's local frame and maps them to wheel speeds using an inverse kinematics matrix.
    *
    * Example Call: reached = self.move_to_point(bot, home_pose, label="home", xy_tol=10.0)
    """

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

        # 1. Use the helper to get filtered values (Theta is returned in RADIANS)
        x, y, theta = self.get_filtered_pose(bot)
        
        # 2. Convert to degrees locally just in case it is needed for logging or consistency
        theta_deg = math.degrees(theta)
        gx,gy,gtheta_deg = goal
        gtheta = math.radians(gtheta_deg)

        dist = math.hypot(gx - x, gy - y)
        # if dist < self.slow_radius:
        #     bot.slowdown_scale = max(self.min_slow_scale, dist / self.slow_radius)

        self.log_bot_distance(bot, gx, gy, label, gtheta=gtheta_deg)

        # If going to staging, maintain full speed (1.0)
        if label == "staging":
            bot.slowdown_scale = 1.0
            
        # Otherwise, apply standard slowdown logic
        elif dist < self.slow_radius:
            bot.slowdown_scale = 0.5

        vx_pid = bot.pid_x.compute(gx - x, dt, integration_window=50.0, deadband=2.0)
        vy_pid = bot.pid_y.compute(gy - y,dt, integration_window=50.0, deadband=2.0)
        
        
        vx, vy = self.apply_crate_avoidance(bot, vx_pid, vy_pid)

        etheta = (gtheta - theta + math.pi)%(2*math.pi)-math.pi
        theta_deadband_rad = math.radians(0.5)
        omega = bot.pid_theta.compute(etheta, dt, integration_window=0.5, deadband=theta_deadband_rad)

        self.publish_pid_debug(
            bot.id,
            gx, x, vx_pid,             # X Axis
            gy, y, vy_pid,             # Y Axis
            gtheta_deg, theta_deg, omega # Theta Axis
        )

        vx *= bot.slowdown_scale
        vy *= bot.slowdown_scale
        omega *= bot.slowdown_scale

        
        # 1. Update constants to match move_near_crate
        min_stiction_vel = 4.0    
        stiction_tol = 2.5        

        # 2. Add Rotation Constants
        min_stiction_cw = 15.0   # Minimum positive power (CW)
        min_stiction_ccw = 0.5   # Minimum negative power (CCW)
        theta_tol_deg = 3.0 if bot.id == 4 else 3.0
        stiction_theta_tol = math.radians(theta_tol_deg)

        # Boost X
        if abs(gx - x) > stiction_tol:
            if abs(vx) < min_stiction_vel:
                vx = math.copysign(min_stiction_vel, vx)
                bot.pid_x.integral = 0.0

        # Boost Y
        if abs(gy - y) > stiction_tol:
            if abs(vy) < min_stiction_vel:
                vy = math.copysign(min_stiction_vel, vy)
                bot.pid_y.integral = 0.0

        # 3. Add Omega Boost (This was missing entirely in move_to_point)
        if abs(etheta) > stiction_theta_tol:
            if omega > 0:  # CW direction
                if omega < min_stiction_cw:
                    omega = min_stiction_cw
                    bot.pid_theta.integral = 0.0
            elif omega < 0:  # CCW direction
                if omega > -min_stiction_ccw:
                    omega = -min_stiction_ccw
                    bot.pid_theta.integral = 0.0
        

        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx*cos_t + vy*sin_t
        vy_r = -vx*sin_t + vy*cos_t
        
        # USE PRE-CALCULATED MATRIX
        wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])

        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

        

        current_xy_tol = xy_tol if xy_tol is not None else (self.xy_tolerance + 3)
        current_theta_tol = theta_tol if theta_tol is not None else self.theta_tolerance_center_deg

        # Check against the selected tolerance
        if math.hypot(gx - x, gy - y) < current_xy_tol and abs(etheta) < math.radians(current_theta_tol):
            bot.is_moving = False
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
            return True
        return False

    """
    * Function Name: reset_bot_state
    * Input: bot (BotState) - The state object of the robot to be reset.
    * Output: None
    * Logic: Performs a comprehensive cleanup of a robot's state after completing a task or returning home.
    * 1. Clears all navigation, transport, and collision flags.
    * 2. Resets arm servos to the neutral travel position (45, 135).
    * 3. Releases any physical zone locks the bot was holding.
    * 4. Resets PID integrators to prevent erratic movement in the next task.
    *
    * Example Call: self.reset_bot_state(bot)
    """
    def reset_bot_state(self, bot: BotState):
        bot.goal_reached = False
        bot.arm_placed = False
        bot.arm_lifted = False
        bot.goal = None
        bot.arm_placed_logged = False  
        bot.drop_pose_logged = False   
        bot.wait_start_time = None
        bot.move_after_attach = False
        bot.going_to_staging_point = False
        bot.is_moving = False
        bot.is_following = False
        bot.follow_target_id = None
        bot.mag_timer_start = None
        bot.alignment_start_time = None         
        bot.magnet_state = 0
        bot.ir_trigger_time = None
        
        
        
        bot.passed_drop_gate = False  
        # ------------------------

        bot.passed_gate = False        # Reset Pickup Gate
        bot.passed_drop_gate = False   # Reset Drop Gate
        

        bot.base_angle = 45.0
        bot.elbow_angle = 135.0
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        if bot.pickup_zone_idx is not None:
            # If this bot was holding a pickup zone lock, release it
            if self.pickup_zone_locks.get(bot.pickup_zone_idx) == bot.id:
                self.pickup_zone_locks[bot.pickup_zone_idx] = None

        bot.pickup_zone_idx = None
        bot.waiting_for_pickup_zone = False
        

        bot.reset_pid()

    """
    * Function Name: publish_wheel_velocities
    * Input: wheel_vel (list), bot_id (int), base (float), elbow (float), mag (int)
    * Output: None
    * Logic: Bridges the high-level ROS 2 logic with the physical hardware.
    * 1. Publishes a ROS BotCmd message for internal logging and simulation.
    * 2. Maps physical velocities (-50 to 50) to servo-style PWM values (0 to 180, centered at 90).
    * 3. Constructs a JSON payload and transmits it via MQTT to the specific ESP32 controlling the robot.
    *
    * Example Call: self.publish_wheel_velocities([10.0, -10.0, 5.0], 0, 45.0, 180.0, 10)
    """

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
            "base": int(round(base)),
            "elbow": int(elbow),
            "mag": int(mag)
        }

        # Dynamic topic creation per bot
        topic = f"bot{bot_id}/cmd"
        self.mqtt_client.publish(topic, json.dumps(payload))

    """
    * Function Name: stop_all_bots
    * Input: None
    * Output: None
    * Logic: A safety shutdown routine. Directly sends a "neutral" MQTT payload to every robot ID, forcing motors to stop, arms to neutral, and magnets OFF. 
    * Uses a 0.5s sleep to ensure network packets are flushed before the program exits.
    *
    * Example Call: self.stop_all_bots()
    """
    def stop_all_bots(self):
        self.get_logger().info("Sending STOP (Idle) command to all bots before shutdown...")
        
        
        idle_payload = {
            "m1": 90,
            "m2": 90,
            "m3": 90,
            "base": 45,
            "elbow": 135,
            "mag": 0
        }
        
        
        for bot_id in self.bot_ids:
            topic = f"bot{bot_id}/cmd"
            try:
                self.mqtt_client.publish(topic, json.dumps(idle_payload))
            except Exception as e:
                print(f"Failed to send MQTT stop to Bot {bot_id}: {e}")

        
        time.sleep(0.5)

    """
    * Function Name: on_mqtt_message
    * Input: client, userdata, msg (MQTT message object)
    * Output: None
    * Logic: Asynchronous listener for incoming sensor data from the robots.
    * 1. Parses the topic string to identify the source robot ID.
    * 2. Decodes the payload; specifically handles "botX/ir" topics.
    * 3. Updates the `ir_detected` boolean based on the digital signal ('0' = detection).
    *
    * Example Call: Triggered automatically by the MQTT background thread.
    """
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
"""
* Function Name: main
* Input: args (list) - Command line arguments passed to the script.
* Output: None
* Logic: The entry point for the ROS 2 node.
* 1. Initializes the rclpy communication layer.
* 2. Instantiates the 'HolonomicMoveToCratesMulti' class.
* 3. Enters the 'spin' loop, which keeps the node alive to process timer callbacks, service requests, and topics.
* 4. Exception Handling: Catches a KeyboardInterrupt (Ctrl+C) to trigger a safety shutdown of all physical robots via MQTT.
* 5. Cleanup: Destroys the node and shuts down the ROS 2 context to free resources.
*
* Example Call: python3 multi_robot_controller.py
"""
def main(args=None):
    rclpy.init(args=args)
    node = HolonomicMoveToCratesMulti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down multi-bot node.")
        node.stop_all_bots()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()