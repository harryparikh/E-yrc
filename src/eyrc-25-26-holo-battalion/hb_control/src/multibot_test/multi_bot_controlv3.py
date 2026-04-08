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
from std_msgs.msg import String

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
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
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

        # state flags
        self.goal_reached = False
        self.arm_placed = False
        self.arm_lifted = False
        self.move_after_attach = False
        self.dropping = False
        self.return_to_start = False
        self.going_to_staging_point = False
        self.has_returned_home = False
        self.is_permanently_idle = False
        self.is_sharing_zone = False  # NEW FLAG

        # Logging flags for one-time messages
        self.drop_pose_logged = False
        self.arm_placed_logged = False
        
        # Distance logging
        self.last_dist_log_time = time.time()
        self.dist_log_interval = 0.5 

        # magnet state
        self.magnet_state = 0
        self.mag_timer_start = None

        # ---------------- Smooth Arm State ----------------
        self.base_angle = 45.0
        self.elbow_angle = 135.0

        self.arm_target_base = 45.0
        self.arm_target_elbow = 135.0

        self.arm_step = 0.3   # degrees per control cycle (smoothness)

        self.completed_crates = set()
        self.last_time = node.get_clock().now()

        # [COLLISION AVOIDANCE] State variables
        self.pose_timestamp = node.get_clock().now()
        self.prev_pose_for_vel = None
        self.velocity = 0.0
        self.is_moving = False
        self.slowdown_scale = 1.0

        # [FOLLOW BEHAVIOR] New variables
        self.is_following = False
        self.follow_target_id = None
        self.follow_offset_distance = 245.

        # PID controllers
        self.pid_x = PID(**pid_params['x'])
        self.pid_y = PID(**pid_params['y'])
        self.pid_theta = PID(**pid_params['theta'])

        # Goals
        self.final_goal = final_goal.copy()
        self.return_goal = return_goal.copy()

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

# ---------------------- Main Node Class -------------------------------------
class HolonomicMoveToCratesMulti(Node):

    def __init__(self):
        super().__init__('holonomic_move_to_crates_multi') 

        # ---------------- Robot and world state ----------------
        self.bot_ids = [0,2,4]  # Controlled robot IDs
        self.target_crate_ids = [30, 14, 12, 11, 13, 22]
        self.current_pose = None  # Unused global pose placeholder
        self.crates = []  # List of detected crates
        self.drop_positions = {}  # Assigned drop locations per crate
        self.drop_positions_assigned = False  # One-time assignment flag

        self.last_time = self.get_clock().now()  # Global timing reference

        # ---------------- Motion tolerances ----------------
        self.xy_tolerance = 15.0  # Position tolerance in mm
        self.theta_tolerance_deg = 3.0  
        self.theta_tolerance_center_deg = 100.0  
        self.max_vel = 50.0  

        # ---------------- Slowdown near goal ----------------
        self.slow_radius = 120.0      
        self.min_slow_scale = 0.25    

        # ---------------- Collision avoidance parameters ----------------
        self.follow_distance_threshold = 250.0  
        self.safety_radius = 550.0  
        self.critical_radius = 375.0  

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
        self.sp_0_alt = np.array([1700.0, 1219.0, 0.0]) 
        self.sp_4_alt = np.array([800.0, 1200.0, 0.0])

        self.staging_point_0 = np.array([800.0,1219.0,0.0])  
        self.staging_point_2 = np.array([1700.0,1200.0,0.0])  
        self.staging_point_4 = np.array([800.0,1200.0,0.0])  
        self.staging_y_threshold = 1400.0  

        # ---------------- Idle home points ----------------
        self.idle_home_point_0 = np.array([1218.0,205.0,0.0])  
        self.idle_home_point_2 = np.array([1568.0,202.0,0.0])  
        self.idle_home_point_4 = np.array([864.0,204.0,0.0])  

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
            0:{'x':{'kp':1.5,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
               'y':{'kp':1.9,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
               'theta':{'kp':45,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            2:{'x':{'kp':2.2,'ki':0.008,'kd':0,'max_out':self.max_vel},
               'y':{'kp':2.7,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
               'theta':{'kp':50,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            4:{'x':{'kp':1.3,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
               'y':{'kp':1.9,'ki':0.0001,'kd':0.03,'max_out':self.max_vel},
               'theta':{'kp':40,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
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

        self.zone_locks = {0:None,1:None,2:None}
        
        # [PASTE THIS INTO CODE B's __init__ METHOD]
        
        # NEW: Define 3 subparts for each zone [Full_Center, Half_1, Half_2]
        self.zone_subparts = {}
        
        # --- Zone 0 (Top) ---
        z0_mid_x = (1020 + 1400) / 2
        z0_y = (1075 + 1355) / 2
        self.zone_subparts[0] = [
            (z0_mid_x, z0_y),           # 0: Full Center
            (1115.0, z0_y),             # 1: Left Half Center
            (1305.0, z0_y)              # 2: Right Half Center
        ]

        # --- Zone 1 (Left) ---
        z1_x = (675 + 920) / 2
        z1_mid_y = (1920 + 2115) / 2
        self.zone_subparts[1] = [
            (z1_x, z1_mid_y),           # 0: Full Center
            (z1_x, 1968.0),             # 1: Bottom Half
            (z1_x, 2066.0)              # 2: Top Half
        ]

        # --- Zone 2 (Right) ---
        z2_x = (1470 + 1752) / 2
        z2_mid_y = (1920 + 2115) / 2
        self.zone_subparts[2] = [
            (z2_x, z2_mid_y),           # 0: Full Center
            (z2_x, 1968.0),             # 1: Bottom Half
            (z2_x, 2066.0)              # 2: Top Half
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
            self.mqtt_client.connect("10.95.243.6", 1883, 60)
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
        """Logs distance similar to the reference code"""
        now = time.time()
        if now - bot.last_dist_log_time >= bot.dist_log_interval:
            if bot.pose is not None:
                x, y, _ = bot.pose
                dist = math.hypot(gx - x, gy - y)
                self.get_logger().info(
                    f"Bot {bot.id} | Distance to {label}: {dist:.2f} | Goal: ({gx:.1f}, {gy:.1f})"
                )
                bot.last_dist_log_time = now

    # ---------------- Drop position assignment ----------------
    def get_zone_for_crate(self,crate_id):
        return crate_id % 3 

    def assign_drop_positions(self):
        self.drop_positions = {}  
        self.zone_last_positions = {}  

        for crate in self.crates:
            pos = self.generate_grid_position(crate['id'])
            self.drop_positions[crate['id']] = {
                'x':pos[0],
                'y':pos[1] + 20 
            }

    def generate_grid_position(self,crate_id):
        zone_id = self.get_zone_for_crate(crate_id)  
        zone = self.drop_zones[zone_id]

        center_x = (zone['x_min'] + zone['x_max']) / 2
        center_y = (zone['y_min'] + zone['y_max']) / 2
        spacing_x = 160
        spacing_y = 120

        if zone_id not in self.zone_last_positions:
            self.zone_last_positions[zone_id] = 0
        count = self.zone_last_positions[zone_id]

        offsets = [
            (-spacing_x/2, spacing_y/2),   
            ( spacing_x/2, spacing_y/2),   
            (-spacing_x/2,-spacing_y/2),   
            ( spacing_x/2,-spacing_y/2),   
        ]

        block_index = count // 4
        crate_index = count % 4
        base_x_shift = block_index * (2*spacing_x + 50) 
        offset_x,offset_y = offsets[crate_index]
        new_x = center_x + base_x_shift + offset_x
        new_y = center_y + offset_y
        self.zone_last_positions[zone_id] += 1
        return (new_x,new_y)

    # ---------------- Greedy assignment ----------------
    def assign_crates_greedily(self):
        all_completed = set() 
        for b in self.bots.values():
            all_completed |= b.completed_crates

        crates_in_use = set()  
        for b in self.bots.values():
            if b.current_crate is not None:
                crates_in_use.add(b.current_crate['id'])
            if b.tracked_crate_id is not None and  b.magnet_state != 0:
                crates_in_use.add(b.tracked_crate_id)

        unassigned_crates = [
            c for c in self.crates
            if c['id'] not in all_completed
            and c['id'] not in crates_in_use
        ]

        if not unassigned_crates:
            return 

        for bot in reversed(list(self.bots.values())):
            if bot.has_returned_home or bot.is_permanently_idle:
                continue
            if bot.magnet_state != 0:
                continue
            if bot.current_crate is None and bot.pose is not None:
                nearest = min(
                    unassigned_crates,
                    key=lambda c: math.hypot(c['x'] - bot.pose[0],c['y'] - bot.pose[1]),
                    default=None
                )
                if nearest is None:
                    continue

                bot.current_crate = nearest
                bot.tracked_crate_id = nearest['id']
                bot.goal_reached = False
                bot.arm_placed = False
                bot.arm_lifted = False
                bot.move_after_attach = False
                bot.drop_pose_logged = False     # Reset logging flag
                bot.arm_placed_logged = False    # Reset logging flag
                bot.magnet_state = 0  
                bot.mag_timer_start = None
                bot.is_sharing_zone = False # Reset sharing flag
                bot.reset_pid() 

                self.get_logger().info(f"Assign|Bot={bot.id}|Crate={nearest['id']}")
                try:
                    unassigned_crates.remove(nearest)
                except ValueError:
                    pass

    # ---------------- Follow Behavior ----------------
    # ---------------- Follow Behavior (Fixed) ----------------
    def check_and_handle_collisions(self):
        bots_to_check = [b for b in self.bots.values() if b.pose is not None]
        bots_that_should_follow = set()

        for bot_a, bot_b in itertools.combinations(bots_to_check, 2):
            if not bot_a.is_moving and not bot_b.is_moving:
                continue

            # 1. Calculate Distance
            dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])
            
            # 2. Determine Dynamic Threshold
            # Default safety distance
            current_threshold = self.follow_distance_threshold 

            # CHECK: Are they sharing a zone?
            if bot_a.is_sharing_zone and bot_b.is_sharing_zone:
                za = self.get_zone_for_crate(bot_a.tracked_crate_id) if bot_a.tracked_crate_id is not None else -1
                zb = self.get_zone_for_crate(bot_b.tracked_crate_id) if bot_b.tracked_crate_id is not None else -2
                if za == zb:
                    # If sharing, shrink the bubble to 160mm (Physical touch limit)
                    # instead of ignoring it completely. This prevents crossing crashes.
                    current_threshold = 160.0 

            # 3. Check against threshold
            if dist >= current_threshold:
                continue

            # 4. Collision Imminent - Decide Priority
            mag_a = getattr(bot_a, 'magnet_state', 0)
            mag_b = getattr(bot_b, 'magnet_state', 0)

            if mag_a != 0 and mag_b == 0:
                leader, follower = bot_a, bot_b
            elif mag_b != 0 and mag_a == 0:
                leader, follower = bot_b, bot_a
            else:
                dist_a = math.hypot(bot_a.pose[0]-bot_a.goal[0], bot_a.pose[1]-bot_a.goal[1]) if bot_a.goal is not None else float('inf')
                dist_b = math.hypot(bot_b.pose[0]-bot_b.goal[0], bot_b.pose[1]-bot_b.goal[1]) if bot_b.goal is not None else float('inf')
                
                if dist_a < dist_b:
                    leader, follower = bot_a, bot_b
                elif dist_b < dist_a:
                    leader, follower = bot_b, bot_a
                else:
                    leader, follower = (bot_a, bot_b) if bot_a.id > bot_b.id else (bot_b, bot_a)

            follower.is_following = True
            follower.follow_target_id = leader.id
            bots_that_should_follow.add(follower.id)

        for bot in bots_to_check:
            if bot.id not in bots_that_should_follow and bot.is_following:
                bot.is_following = False
                bot.follow_target_id = None

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

    # [PASTE THE FUNCTION HERE]
    def publish_perception_debug(self):
        # 1. Gather Zone Lock Data
        zone_info = {
            "locks": self.zone_locks,
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
            elif bot.current_crate: status = f"TARGETING_{bot.tracked_crate_id}"

            bots_data[bid] = {
                "pose": list(bot.pose),
                "goal": list(bot.goal) if bot.goal is not None else [],
                "velocity": bot.velocity,
                "magnet": bot.magnet_state,
                "status": status,
                "sharing_zone": bot.is_sharing_zone,
                "carrying_crate": bot.current_crate['id'] if bot.current_crate else None
            }

        # 3. Compile final packet
        debug_msg = {
            "timestamp": time.time(),
            "drop_zones": zone_info,
            "bots": bots_data
        }

        # 4. Publish
        msg = String()
        msg.data = json.dumps(debug_msg)
        self.debug_pub.publish(msg)


    def control_cb(self):
        # [REMOVED] The line below was causing the freeze:
        # if len(self.crates) == 0:
        #    return

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

            # ---------------- SIMULTANEOUS ZONE & LOCKING LOGIC ----------------
            if bot.current_crate is not None:
                current_zone = self.get_zone_for_crate(bot.tracked_crate_id)
                
                # 1. Find all bots targeting this zone
                bots_in_zone = sorted(
                    [b for b in self.bots.values() if b.current_crate and self.get_zone_for_crate(b.tracked_crate_id) == current_zone],
                    key=lambda b: b.id
                )

                # 2. If multiple bots are in the zone, enable SHARING mode
                # ---------------- SIMULTANEOUS ZONE & LOCKING LOGIC ----------------
            if bot.current_crate is not None:
                current_zone = self.get_zone_for_crate(bot.tracked_crate_id)
                
                # 1. Find all bots targeting this zone
                bots_in_zone = [b for b in self.bots.values() if b.current_crate and self.get_zone_for_crate(b.tracked_crate_id) == current_zone]

                # 2. If multiple bots are in the zone, enable SHARING mode
                if len(bots_in_zone) > 1:
                    bot.is_sharing_zone = True
                    try:
                        # [SMART SORTING]
                        # If Zone 0 (Top/Horizontal): Sort by X coordinate. Leftmost bot gets Left slot.
                        # If Zone 1/2 (Side/Vertical): Sort by Y coordinate. Bottom bot gets Bottom slot.
                        if current_zone == 0:
                            bots_in_zone.sort(key=lambda b: b.pose[0] if b.pose is not None else 0)
                        else:
                            bots_in_zone.sort(key=lambda b: b.pose[1] if b.pose is not None else 0)

                        my_index = bots_in_zone.index(bot)
                        
                        # Index 0 gets Subpart 1, Index 1 gets Subpart 2
                        subpart_idx = 1 if my_index == 0 else 2
                        
                        if current_zone in self.zone_subparts:
                            if subpart_idx >= len(self.zone_subparts[current_zone]):
                                subpart_idx = 1 
                            
                            target_coords = self.zone_subparts[current_zone][subpart_idx]
                            
                            # OVERRIDE Drop Position
                            self.drop_positions[bot.tracked_crate_id]['x'] = target_coords[0]
                            self.drop_positions[bot.tracked_crate_id]['y'] = target_coords[1]
                            
                            # Update immediate goal if currently moving to drop
                            if bot.move_after_attach:
                                # Update the PID goal immediately so it doesn't wait for next cycle
                                bot.goal = np.array([target_coords[0], target_coords[1] - 190, 0.0])
                    except: pass
                else:
                    bot.is_sharing_zone = False
            # 3. Apply Locking ONLY if NOT sharing a zone
            # (If sharing, we skip this block entirely so both bots move)
            if bot.goal_reached and not bot.arm_placed and not bot.is_sharing_zone:
                zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
                locking_bot_id = self.zone_locks.get(zone_id)
                
                higher_priority_exists = False
                for other_bot in self.bots.values():
                    if (other_bot.id > bot.id and other_bot.current_crate is not None and
                        self.get_zone_for_crate(other_bot.tracked_crate_id) == zone_id):
                        higher_priority_exists = True
                        break

                if locking_bot_id is None:
                    if higher_priority_exists:
                        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                        continue
                    else:
                        self.zone_locks[zone_id] = bot.id
                elif locking_bot_id != bot.id:
                    self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                    continue
            # ---------------- FSM Execution ----------------
            if not bot.goal_reached:
                self.move_near_crate(bot)

            elif not bot.arm_placed:
                # Arm DOWN + Magnet ON
                if bot.magnet_state == 0:
                    bot.magnet_state = 10
                    bot.mag_timer_start = now_wall

                base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
                bot.arm_target_base = base_down
                bot.arm_target_elbow = elbow_down

                arm_done = self.update_arm_smooth(bot)

                self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

                if arm_done and not bot.arm_placed_logged:
                    bot.arm_placed = True
                    bot.arm_placed_logged = True
                    self.get_logger().info(
                        f"Bot {bot.id} | Arm placed on crate (base={bot.base_angle:.1f}°, elbow={bot.elbow_angle:.1f}°)"
                    )
                    self.get_logger().info(f"Bot {bot.id} | Magnet pull-in started")

            elif bot.magnet_state != 10:
                self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
                continue

            elif not bot.arm_lifted:
                self.lift_arm_with_crate(bot)

            elif bot.move_after_attach:
                self.move_to_final_goal(bot)

    # ---------------- Follow Bot ----------------
    def follow_bot(self,follower:BotState,leader_id:int):
        if leader_id not in self.bots: return
        leader = self.bots[leader_id]
        if leader.pose is None or follower.pose is None: return

        follower.is_moving = True
        dx = leader.pose[0] - follower.pose[0]
        dy = leader.pose[1] - follower.pose[1]
        dist = math.hypot(dx,dy)

        if abs(dist - follower.follow_offset_distance) < 20.0 or dist < 1.0:
            self.publish_wheel_velocities([0.0,0.0,0.0], follower.id, follower.base_angle, follower.elbow_angle, follower.magnet_state)
            follower.is_moving = False
            return

        ux, uy = dx/dist, dy/dist
        target_x = leader.pose[0] - ux * follower.follow_offset_distance
        target_y = leader.pose[1] - uy * follower.follow_offset_distance
        self.move_to_point(follower, np.array([target_x,target_y,0.0]), "Leader Follow")

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

    def move_near_crate(self,bot:BotState):
        bot.is_moving = True
        crate = bot.current_crate
        if crate is None: return

        if bot.id==0:
            offset_x=30
            offset_y=120
        elif bot.id==2:
            offset_x=30
            offset_y=120
        elif bot.id==4:
            # offset_x=40
            # offset_y=120
            offset_x=26
            offset_y=128
        
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        goal_x = crate['x'] - offset_x
        goal_y = crate['y'] - offset_y
        bot.goal = np.array([goal_x,goal_y,0.0])

        # Log distance similar to reference
        self.log_bot_distance(bot, goal_x, goal_y, "crate approach point")

        now = self.get_clock().now()
        dt = (now - bot.last_time).nanoseconds / 1e9 if bot.last_time else 0.03
        if dt <= 0: return

        x,y,theta_deg = bot.pose
        theta = math.radians(theta_deg)

        vx = bot.pid_x.compute(goal_x - x,dt)
        vy = bot.pid_y.compute(goal_y - y,dt)
        
        dist = math.hypot(goal_x - x, goal_y - y)
        if dist < self.slow_radius:
            bot.slowdown_scale = max(self.min_slow_scale, dist / self.slow_radius)

        goal_theta = 0.0
        error_theta = (goal_theta - theta + math.pi) % (2*math.pi) - math.pi
        omega = bot.pid_theta.compute(error_theta, dt)

        vx *= bot.slowdown_scale
        vy *= bot.slowdown_scale
        omega *= bot.slowdown_scale

        # OPTIMIZED KINEMATICS (Use self.M_inv)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx * cos_t + vy * sin_t
        vy_r = -vx * sin_t + vy * cos_t
        
        # USE PRE-CALCULATED MATRIX
        wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])
        
        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, 0)

        if math.hypot(goal_x-x,goal_y-y) < self.xy_tolerance:
            self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 0)
            bot.is_moving = False
            bot.goal_reached = True
            # LOG MATCHING REFERENCE
            self.get_logger().info(f"Bot {bot.id} | Reached near crate at (x={goal_x:.1f}, y={goal_y:.1f})")
            self.get_logger().info(f"Bot {bot.id} | Preparing to place arm on the crate...")

    def lift_arm_with_crate(self, bot: BotState):
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0
        arm_done = self.update_arm_smooth(bot)
        
        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 10)

        if arm_done:
            bot.arm_lifted = True
            bot.move_after_attach = True
            # LOG MATCHING REFERENCE
            self.get_logger().info(f"Bot {bot.id} | Arm lifted with crate attached!")

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
    def move_to_final_goal(self, bot: BotState):
        # Try to get the drop position from the dictionary
        drop = self.drop_positions.get(bot.tracked_crate_id)

        # If it returns None (meaning we never calculated it at startup), do it now!
        if drop is None:
            self.get_logger().warn(f"Fixing missing drop pos for Crate {bot.tracked_crate_id}")
            
            # 1. Calculate the grid position using your existing helper
            pos = self.generate_grid_position(bot.tracked_crate_id)
            
            # 2. Save it to the dictionary so it persists
            self.drop_positions[bot.tracked_crate_id] = {
                'x': pos[0],
                'y': pos[1] + 20 
            }
            # 3. Update the 'drop' variable
            drop = self.drop_positions[bot.tracked_crate_id]
        # ------------------------------------

        # Now 'drop' is guaranteed to exist.
        # Logic: 14 % 3 = 2 -> Zone 2 (Right Side)
        goal = np.array([drop['x'], drop['y'] - 190, 0.0])
        bot.goal = goal.copy()

        # Log distance
        self.log_bot_distance(bot, goal[0], goal[1], "drop zone")

        if self.move_to_point(bot, goal, "drop zone"):
            bot.move_after_attach = False
            bot.dropping = True          
            bot.mag_timer_start = None   
            bot.drop_pose_logged = False

    def drop_crate(self, bot: BotState):
        base_down, elbow_down = self.arm_down_config.get(bot.id, (15.0, 180.0))
        bot.arm_target_base = base_down
        bot.arm_target_elbow = elbow_down

        arm_done = self.update_arm_smooth(bot)

        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)

        # Wait until arm is down
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
            zone_id = self.get_zone_for_crate(crate_id)
            if self.zone_locks.get(zone_id) == bot.id:
                self.zone_locks[zone_id] = None

        # LOG MATCHING REFERENCE
        self.get_logger().info(f"Bot {bot.id} | Crate released (magnet OFF)")

        # Cleanup state
        bot.current_crate = None
        bot.tracked_crate_id = None
        bot.dropping = False
        bot.arm_lifted = False
        bot.arm_placed = False
        
        # Reset arm targets
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        # ---------------- FIX STARTS HERE ----------------
        # Check current Y position to decide: Staging vs Return Home directly
        # Note: We use bot.pose[1] (current Y) to decide.
        
        if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
            # Case 1: High up in the arena -> Go to Staging Point
            self.get_logger().info(f"Bot {bot.id} | High Y ({bot.pose[1]:.1f}), going to STAGING.")
            
            bot.going_to_staging_point = True
            bot.return_to_start = False
            
            if bot.id == 0: staging_goal = self.staging_point_0
            elif bot.id == 2: staging_goal = self.staging_point_2
            elif bot.id == 4: staging_goal = self.staging_point_4
            else: staging_goal = self.staging_point_0
            
            bot.goal = staging_goal.copy()
            
        else:
            # Case 2: Low in the arena -> Go Home directly
            self.get_logger().info(f"Bot {bot.id} | Low Y ({bot.pose[1] if bot.pose is not None else 0:.1f}), returning HOME.")
            
            bot.going_to_staging_point = False
            bot.return_to_start = True
            
            if bot.id == 0: bot.return_goal = self.idle_home_point_0.copy()
            elif bot.id == 2: bot.return_goal = self.idle_home_point_2.copy()
            elif bot.id == 4: bot.return_goal = self.idle_home_point_4.copy()
            
            bot.goal = bot.return_goal.copy()

        # Reset PID for the new movement
        bot.reset_pid() 
        # ---------------- FIX ENDS HERE ----------------

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
            # LOG MATCHING REFERENCE
            self.get_logger().info(f"Bot {bot.id} | Returned home. Robot stopped.")
            self.reset_bot_state(bot)

    def move_to_point(self,bot:BotState,goal, label="target"):
        self.update_arm_smooth(bot)
        bot.is_moving = True
        now = self.get_clock().now()
        dt = (now - bot.last_time).nanoseconds / 1e9 if bot.last_time else 0.03
        if dt <= 0: return False

        x,y,theta_deg = bot.pose
        theta = math.radians(theta_deg)
        gx,gy,gtheta_deg = goal
        gtheta = math.radians(gtheta_deg)

        dist = math.hypot(gx - x, gy - y)
        if dist < self.slow_radius:
            bot.slowdown_scale = max(self.min_slow_scale, dist / self.slow_radius)
        
        vx = bot.pid_x.compute(gx - x,dt)
        vy = bot.pid_y.compute(gy - y,dt)
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

        if math.hypot(gx - x, gy - y) < (self.xy_tolerance+3) and abs(etheta) < math.radians(self.theta_tolerance_center_deg):
            bot.is_moving = False
            self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, bot.magnet_state)
            return True
        return False

    def reset_bot_state(self,bot:BotState):
        bot.goal_reached = False
        bot.arm_placed = False
        bot.arm_lifted = False
        bot.goal = None
        bot.move_after_attach = False
        bot.going_to_staging_point = False
        bot.is_moving = False
        bot.is_following = False
        bot.follow_target_id = None
        bot.is_sharing_zone = False
        bot.mag_timer_start = None
        bot.magnet_state = 0
        bot.base_angle = 45.0
        bot.elbow_angle = 135.0
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0
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
