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

        self.arm_step = 0.3  # degrees per control cycle (smoothness)

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
        self.critical_radius = 300.0  

        # ---------------- Global goals ----------------
        self.final_goal = np.array([1219.0,1180.0,0.0])  
        self.return_goal = np.array([1218.0,205.0,0.0])  

        # ---------------- Staging points ----------------
        self.staging_point_0 = np.array([600.0,1219.0,0.0])  
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

        # ---------------- Per-bot ARM DOWN configuration ----------------
        self.arm_down_config = {
            0: (10.0, 180.0),
            2: (15.0, 180.0),  
            4: (5.0, 180.0)
        }

        # ---------------- Per-bot PID parameters ----------------
        self.pid_params_by_bot = {
            0:{'x':{'kp':2.0,'ki':0.004,'kd':0.0,'max_out':self.max_vel},
               'y':{'kp':2.5,'ki':0.004,'kd':0.01,'max_out':self.max_vel},
               'theta':{'kp':40,'ki':0.002,'kd':0.1,'max_out':self.max_vel*2}},
            2:{'x':{'kp':2.2,'ki':0.008,'kd':0,'max_out':self.max_vel},
               'y':{'kp':2.7,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
               'theta':{'kp':50,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            4:{'x':{'kp':2.5,'ki':0.008,'kd':0,'max_out':self.max_vel},
               'y':{'kp':2.7,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
               'theta':{'kp':50,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
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
        
        # ---------------- ROS interfaces ----------------
        self.create_subscription(Poses2D,'bot_pose',self.pose_cb,10)
        self.create_subscription(Poses2D,'crate_pose',self.crate_cb,10)
        self.publisher = self.create_publisher(BotCmdArray,'/bot_cmd',10)
        self.timer = self.create_timer(0.03,self.control_cb)

        # ---------------- MQTT setup (hardware control) ----------------
        import paho.mqtt.client as mqtt
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("10.95.243.6",1883,60)
        self.mqtt_client.loop_start()

        self.get_logger().info("Multi-crate controller initialized. Waiting for crates...")

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
                bot.reset_pid() 

                self.get_logger().info(f"Assign|Bot={bot.id}|Crate={nearest['id']}")
                try:
                    unassigned_crates.remove(nearest)
                except ValueError:
                    pass

    # ---------------- Follow Behavior ----------------
    def check_and_handle_collisions(self):
        bots_to_check = [b for b in self.bots.values() if b.pose is not None]
        bots_that_should_follow = set()

        for bot_a,bot_b in itertools.combinations(bots_to_check,2):
            if not bot_a.is_moving and not bot_b.is_moving:
                continue

            dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])
            if dist >= self.follow_distance_threshold:
                continue

            mag_a = getattr(bot_a,'magnet_state',0)
            mag_b = getattr(bot_b,'magnet_state',0)

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
    # def control_cb(self):
    #     # [REMOVED] The line below was causing the freeze:
    #     # if len(self.crates) == 0:
    #     #    return

    #     # 1. Update timing and reset slowdown scales
    #     now_wall = time.time()
    #     for bot in self.bots.values():
    #         bot.slowdown_scale = 1.0

    #     # 2. Try to assign any visible crates to idle bots
    #     #    (This function safely handles empty crate lists internally)
    #     self.assign_crates_greedily()

    #     # ---------------- Idle and staging logic ----------------
    #     for bot in self.bots.values():
    #         # Only send idle bots home/staging if they aren't doing anything else
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

    #     # 3. Collision Avoidance
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

    #         # Skip if this specific bot has no crate assigned
    #         if bot.current_crate is None:
    #             continue

    #         # ---------------- Magnet HOLD Transition Log ----------------
    #         if bot.magnet_state == 10 and bot.mag_timer_start is not None:
    #             if now_wall - bot.mag_timer_start >= 5.0:
    #                 bot.magnet_state = 9
    #                 bot.mag_timer_start = None
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
    def control_cb(self):
        # 1. Update timing and reset slowdown scales
        now_wall = time.time()
        for bot in self.bots.values():
            bot.slowdown_scale = 1.0

        # 2. Try to assign any visible crates to idle bots
        self.assign_crates_greedily()

        # ---------------- Idle and staging logic ----------------
        for bot in self.bots.values():
            # Only send idle bots home/staging if they aren't doing anything else
            if (bot.current_crate is None and not bot.return_to_start and
                not bot.move_after_attach and not bot.going_to_staging_point and
                not bot.has_returned_home and not bot.is_permanently_idle):

                # Logic: If high up in the arena (Y > 1400), go to staging first.
                # Otherwise, go straight home.
                if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
                    bot.going_to_staging_point = True
                    if bot.id == 0: staging_goal = self.staging_point_0
                    elif bot.id == 2: staging_goal = self.staging_point_2
                    elif bot.id == 4: staging_goal = self.staging_point_4
                    else: staging_goal = self.staging_point_0
                    bot.goal = staging_goal.copy()
                    bot.reset_pid() # Good practice to reset here too

                else:
                    bot.return_to_start = True
                    # Set return goal based on ID
                    if bot.id == 0: bot.return_goal = self.idle_home_point_0
                    elif bot.id == 2: bot.return_goal = self.idle_home_point_2
                    elif bot.id == 4: bot.return_goal = self.idle_home_point_4
                    else: bot.return_goal = self.idle_home_point_0
                    
                    bot.goal = bot.return_goal.copy()
                    
                    # --- FIX START: Reset PID and Log ---
                    bot.reset_pid() 
                    self.get_logger().info(f"Bot {bot.id} | Drop complete. Returning to Start: ({bot.goal[0]}, {bot.goal[1]})")
                    # --- FIX END ---

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
                    bot.magnet_state = 9
                    bot.mag_timer_start = None
                    self.get_logger().info(f"Bot {bot.id} | Magnet switched to HOLD mode")

            # ---------------- Zone Locking ----------------
            if bot.goal_reached and not bot.arm_placed:
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

            elif bot.magnet_state != 9:
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
    # def update_arm_smooth(self, bot: BotState):
    #     moved = False
    #     if bot.base_angle < bot.arm_target_base:
    #         bot.base_angle += bot.arm_step
    #         moved = True
    #     elif bot.base_angle > bot.arm_target_base:
    #         bot.base_angle -= bot.arm_step
    #         moved = True

    #     if bot.elbow_angle < bot.arm_target_elbow:
    #         bot.elbow_angle += bot.arm_step
    #         moved = True
    #     elif bot.elbow_angle > bot.arm_target_elbow:
    #         bot.elbow_angle -= bot.arm_step
    #         moved = True
    #     return not moved

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
        
        bot.arm_target_base = 45.0
        bot.arm_target_elbow = 135.0

        goal_x = crate['x'] - 30
        goal_y = crate['y'] - 120.0
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

        # Holonomic Calc
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx*cos_t + vy*sin_t
        vy_r = -vx*sin_t + vy*cos_t
        alphas = [math.radians(30),math.radians(150),math.radians(270)]
        M = np.array([[math.cos(a+math.pi/2) for a in alphas],[math.sin(a+math.pi/2) for a in alphas],[1,1,1]])
        wheel_vel = np.linalg.pinv(M) @ np.array([vx_r,vy_r,omega])
        
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
        
        self.publish_wheel_velocities([0.0,0.0,0.0], bot.id, bot.base_angle, bot.elbow_angle, 9)

        if arm_done:
            bot.arm_lifted = True
            bot.move_after_attach = True
            # LOG MATCHING REFERENCE
            self.get_logger().info(f"Bot {bot.id} | Arm lifted with crate attached!")

    def move_to_final_goal(self,bot:BotState):
        drop = self.drop_positions.get(bot.tracked_crate_id)
        goal = np.array([drop['x'],drop['y']-190,0.0]) if drop else bot.final_goal
        bot.goal = goal.copy()

        # Log distance
        self.log_bot_distance(bot, goal[0], goal[1], "drop zone")

        if self.move_to_point(bot, goal, "drop zone"):
            bot.move_after_attach = False
            bot.dropping = True          
            bot.mag_timer_start = None   
            bot.drop_pose_logged = False # Reset flag for drop log

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

        bot.return_to_start = True
        if bot.id == 0: 
            bot.return_goal = self.idle_home_point_0.copy()
        elif bot.id == 2: 
            bot.return_goal = self.idle_home_point_2.copy()
        elif bot.id == 4: 
            bot.return_goal = self.idle_home_point_4.copy()
        
        bot.goal = bot.return_goal.copy()
        bot.reset_pid() 
        self.get_logger().info(f"Bot {bot.id} | Transitioning to Return Home at {bot.goal}")

    # def move_to_staging(self,bot:BotState):
    #     # Log dist
    #     self.log_bot_distance(bot, bot.goal[0], bot.goal[1], "staging point")
    #     if self.move_to_point(bot,bot.goal, "staging"):
    #         bot.going_to_staging_point = False
    #         bot.return_to_start = True
    #         # Return goal logic handled in main loop
    #         bot.reset_pid()

    def move_to_staging(self, bot: BotState):
        # Log dist
        self.log_bot_distance(bot, bot.goal[0], bot.goal[1], "staging point")
        if self.move_to_point(bot, bot.goal, "staging"):
            bot.going_to_staging_point = False
            bot.return_to_start = True
            
            # --- FIX: Assign the specific return goal here ---
            if bot.id == 0: bot.return_goal = self.idle_home_point_0
            elif bot.id == 2: bot.return_goal = self.idle_home_point_2
            elif bot.id == 4: bot.return_goal = self.idle_home_point_4
            else: bot.return_goal = self.idle_home_point_0
            
            bot.goal = bot.return_goal.copy() # Set the immediate goal
            # -------------------------------------------------
            
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

        cos_t, sin_t = math.cos(theta), math.sin(theta)
        vx_r = vx*cos_t + vy*sin_t
        vy_r = -vx*sin_t + vy*cos_t
        alphas = [math.radians(30),math.radians(150),math.radians(270)]
        M = np.array([[math.cos(a+math.pi/2) for a in alphas],[math.sin(a+math.pi/2) for a in alphas],[1,1,1]])
        wheel_vel = np.linalg.pinv(M) @ np.array([vx_r,vy_r,omega])

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