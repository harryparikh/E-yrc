#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from hb_interfaces.msg import Poses2D, BotCmd, BotCmdArray
from linkattacher_msgs.srv import AttachLink, DetachLink
import numpy as np
import math
import time
import random
import itertools

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
        self.box_attached = False
        self.arm_lifted = False
        self.move_after_attach = False
        self.box_detached = False
        self.return_to_start = False
        self.going_to_staging_point = False
        self.has_returned_home = False 
        self.is_permanently_idle = False

        self.completed_crates = set()
        self.last_time = node.get_clock().now()

        # [COLLISION AVOIDANCE] State variables
        self.pose_timestamp = node.get_clock().now()
        self.prev_pose_for_vel = None
        self.velocity = 0.0
        self.is_moving = False
        self.slowdown_scale = 1.0
        self.priority_cooldown_end_time = None
        
        # [FOLLOW BEHAVIOR] New variables
        self.is_following = False
        self.follow_target_id = None
        self.follow_offset_distance = 245.0  # Stay this far behind the leader

        # PID controllers
        self.pid_x = PID(**pid_params['x'])
        self.pid_y = PID(**pid_params['y'])
        self.pid_theta = PID(**pid_params['theta'])

        self.orig_pid_limits = {
            'x': pid_params['x']['max_out'],
            'y': pid_params['y']['max_out'],
            'theta': pid_params['theta']['max_out']
        }

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

        # Robot Parameters
        self.bot_ids = [0, 2, 4]
        self.current_pose = None
        self.crates = []
        self.drop_positions = {}
        self.drop_positions_assigned = False

        self.last_time = self.get_clock().now()

        self.xy_tolerance = 5.0
        self.theta_tolerance_deg = 3.0
        self.theta_tolerance_center_deg = 15.0
        self.max_vel = 50.0
        
        # [MODIFIED] Collision avoidance parameters
        self.follow_distance_threshold = 250.0  # Distance to trigger follow behavior
        self.safety_radius = 550.0
        self.critical_radius = 300.0

        self.final_goal = np.array([1219.0, 1180.0, 0.0])
        self.return_goal = np.array([1218.0, 205.0, 0.0])

        # MODIFIED: Staging points for ALL bots
        self.staging_point_0 = np.array([600.0, 1219.0, 0.0])
        self.staging_point_2 = np.array([1700.0, 1200.0, 0.0])
        self.staging_point_4 = np.array([800.0, 1200.0, 0.0])
        
        # Staging threshold
        self.staging_y_threshold = 1400.0
        
        # Idle home points
        self.idle_home_point_0 = np.array([1218.0, 205.0, 0.0])
        self.idle_home_point_2 = np.array([1568.0, 202.0, 0.0])
        self.idle_home_point_4 = np.array([864.0, 204.0, 0.0])

        # Crate region
        self.x_min, self.x_max = 1020, 1410
        self.y_min, self.y_max = 1075, 1355

        # Crate geometry
        self.crate_size = 60.0
        self.safe_gap = 10.0
        self.min_dist = self.crate_size + self.safe_gap

        # PID parameters
        self.pid_params = {
            'x': {'kp': 4, 'ki': 0.0001, 'kd': 0.001, 'max_out': self.max_vel},
            'y': {'kp': 5, 'ki': 0.0001, 'kd': 0.0001, 'max_out': self.max_vel},
            'theta': {'kp': -9, 'ki': 0.001, 'kd': 0.0002, 'max_out': self.max_vel * 2}
        }

        # Per-bot states
        self.bots = {bid: BotState(bid, self, self.pid_params, self.final_goal, self.return_goal) for bid in self.bot_ids}
        if 0 in self.bots:
            self.bots[0].return_goal = self.idle_home_point_0.copy()

        # Drop zones
        self.drop_zones = {
            0: {'x_min': 1020, 'x_max': 1400, 'y_min': 1075, 'y_max': 1355},
            2: {'x_min': 1470, 'x_max': 1752, 'y_min': 1920, 'y_max': 2115},
            1: {'x_min': 675,  'x_max': 920,  'y_min': 1920, 'y_max': 2115}
        }

        # Zone locks
        self.zone_locks = {0: None, 1: None, 2: None}
        self.get_logger().info("Zone locks initialized.")

        # ROS Setup
        self.create_subscription(Poses2D, 'bot_pose', self.pose_cb, 10)
        self.create_subscription(Poses2D, 'crate_pose', self.crate_cb, 10)
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        self.timer = self.create_timer(0.03, self.control_cb)
        self.logging_timer = self.create_timer(1.0, self.log_bot_distances)

        # Service clients
        self.attach_client = self.create_client(AttachLink, '/attach_link')
        self.detach_client = self.create_client(DetachLink, '/detach_link')

        self.assigned_positions = []

        self.get_logger().info("Multi-crate controller initialized for bots: " + ", ".join(map(str, self.bot_ids)))

        # Initialize arms
        rclpy.spin_once(self, timeout_sec=0.5)
        for _ in range(3):
            for bid in self.bot_ids:
                self.publish_wheel_velocities([0.0, 0.0, 0.0], bid, base=45.0, elbow=45.0)
                time.sleep(1)

    # ---------------- Callbacks ----------------
    def pose_cb(self, msg: Poses2D):
        for pose in msg.poses:
            if pose.id in self.bots:
                bot = self.bots[pose.id]
                now = self.get_clock().now()
                new_pose_np = np.array([pose.x, pose.y, pose.w])
                
                if bot.prev_pose_for_vel is not None:
                    dt = (now - bot.pose_timestamp).nanoseconds / 1e9
                    if dt > 0.001:
                        dx = new_pose_np[0] - bot.prev_pose_for_vel[0]
                        dy = new_pose_np[1] - bot.prev_pose_for_vel[1]
                        bot.velocity = math.hypot(dx, dy) / dt
                    else:
                        bot.velocity = 0.0
                
                bot.pose = new_pose_np
                bot.prev_pose_for_vel = new_pose_np
                bot.pose_timestamp = now
                bot.last_time = now

    def crate_cb(self, msg: Poses2D):
        self.crates = []
        for pose in msg.poses:
            self.crates.append({'id': pose.id, 'x': pose.x, 'y': pose.y, 'w': pose.w})
        if not self.drop_positions_assigned:
            self.assign_drop_positions()
            self.drop_positions_assigned = True

    def log_bot_distances(self):
        bots_with_pose = [b for b in self.bots.values() if b.pose is not None]
        for bot_a, bot_b in itertools.combinations(bots_with_pose, 2):
            try:
                dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])
                self.get_logger().info(f"Distance Bot {bot_a.id} <-> Bot {bot_b.id}: {dist:.2f}")
            except Exception as e:
                self.get_logger().error(f"Error calculating distance: {e}")

    # ---------------- Drop position assignment ----------------
    def get_zone_for_crate(self, crate_id):
        return crate_id % 3

    def assign_drop_positions(self):
        self.drop_positions = {}
        self.zone_last_positions = {}  # to track how many crates per zone

        for crate in self.crates:
            pos = self.generate_grid_position(crate['id'])
            self.drop_positions[crate['id']] = {'x': pos[0], 'y': pos[1] + 20}

        self.get_logger().info("Drop positions assigned in 2x2 grid pattern:")
        for cid, pos in self.drop_positions.items():
            self.get_logger().info(f"    Crate ID {cid}: ({pos['x']:.1f}, {pos['y']:.1f})")

    def generate_grid_position(self, crate_id):
        zone_id = self.get_zone_for_crate(crate_id)
        zone = self.drop_zones[zone_id]

        # Zone center
        center_x = (zone['x_min'] + zone['x_max']) / 2
        center_y = (zone['y_min'] + zone['y_max']) / 2

        # Distance between crates
        spacing_x = 160
        spacing_y = 120

        # Track how many crates already placed in this zone
        if zone_id not in self.zone_last_positions:
            self.zone_last_positions[zone_id] = 0
        count = self.zone_last_positions[zone_id]

        # Calculate relative offsets for 2x2 pattern
        offsets = [
            ( -spacing_x/2,  spacing_y/2),  # Crate 1 (top-left)
            (  spacing_x/2,  spacing_y/2),  # Crate 2 (top-right)
            ( -spacing_x/2, -spacing_y/2),  # Crate 3 (bottom-left)
            (  spacing_x/2, -spacing_y/2),  # Crate 4 (bottom-right)
        ]

        # If more than 4 crates, continue in next 2x2 block to the right
        block_index = count // 4
        crate_index = count % 4

        base_x_shift = block_index * (2 * spacing_x + 50)  # 50 gap between blocks
        offset_x, offset_y = offsets[crate_index]

        new_x = center_x + base_x_shift + offset_x
        new_y = center_y + offset_y

        # Update count for zone
        self.zone_last_positions[zone_id] += 1

        return (new_x, new_y)

    # ---------------- Greedy assignment ----------------
    def assign_crates_greedily(self):
        all_completed = set()
        for b in self.bots.values():
            all_completed |= b.completed_crates

        crates_in_use = set()
        for b in self.bots.values():
            if b.current_crate is not None:
                crates_in_use.add(b.current_crate['id'])
            if b.tracked_crate_id is not None and b.box_detached and b.current_crate is not None:
                crates_in_use.add(b.tracked_crate_id)

        unassigned_crates = [c for c in self.crates 
                            if c['id'] not in all_completed 
                            and c['id'] not in crates_in_use]

        if not unassigned_crates:
            return

        for bot in reversed(list(self.bots.values())):
            if bot.box_detached and bot.current_crate is not None:
                continue
            
            if bot.has_returned_home or bot.is_permanently_idle:
                continue
            
            if bot.current_crate is None and bot.pose is not None:
                nearest = min(unassigned_crates, key=lambda c: math.hypot(c['x'] - bot.pose[0], c['y'] - bot.pose[1]), default=None)
                if nearest:
                    bot.current_crate = nearest
                    bot.tracked_crate_id = nearest['id']
                    bot.goal_reached = False
                    bot.arm_placed = False
                    bot.box_attached = False
                    bot.arm_lifted = False
                    bot.move_after_attach = False
                    bot.box_detached = False
                    bot.reset_pid()
                    zone_id = self.get_zone_for_crate(nearest['id'])
                    self.get_logger().info(f"Bot {bot.id} assigned to crate {nearest['id']} (Zone {zone_id})")
                    try:
                        unassigned_crates.remove(nearest)
                    except ValueError:
                        pass

    # ---------------- [FIXED FOLLOW BEHAVIOR COLLISION AVOIDANCE] ----------------
    def check_and_handle_collisions(self):
        """
        If distance < 250, the bot farther from goal follows the bot closer to goal.
        """
        bots_to_check = [b for b in self.bots.values() if b.pose is not None]
        now = self.get_clock().now()
        
        # Track which bots should be following this iteration
        bots_that_should_follow = set()

        # Check bot-vs-bot collisions
        for bot_a, bot_b in itertools.combinations(bots_to_check, 2):
            
            # Only check if at least one bot is actively moving
            if not bot_a.is_moving and not bot_b.is_moving:
                continue
                
            dist = math.hypot(bot_a.pose[0] - bot_b.pose[0], bot_a.pose[1] - bot_b.pose[1])
            
            # Check if within follow distance threshold (250 units)
            if dist < self.follow_distance_threshold:
                
                # Determine which bot is closer to their goal
                dist_a_to_goal = float('inf')
                if bot_a.goal is not None and bot_a.pose is not None:
                    dist_a_to_goal = math.hypot(bot_a.pose[0] - bot_a.goal[0], bot_a.pose[1] - bot_a.goal[1])

                dist_b_to_goal = float('inf')
                if bot_b.goal is not None and bot_b.pose is not None:
                    dist_b_to_goal = math.hypot(bot_b.pose[0] - bot_b.goal[0], bot_b.pose[1] - bot_b.goal[1])

                # Determine leader (closer to goal) and follower (farther from goal)
                if dist_a_to_goal < dist_b_to_goal:
                    leader = bot_a
                    follower = bot_b
                elif dist_b_to_goal < dist_a_to_goal:
                    leader = bot_b
                    follower = bot_a
                else:
                    # If equal distance, use ID as tiebreaker (higher ID leads)
                    if bot_a.id > bot_b.id:
                        leader = bot_a
                        follower = bot_b
                    else:
                        leader = bot_b
                        follower = bot_a
                
                # Set follower to follow the leader
                follower.is_following = True
                follower.follow_target_id = leader.id
                bots_that_should_follow.add(follower.id)
                
                self.get_logger().info(
                    f"FOLLOW MODE: Bot {follower.id} (dist to goal: {dist_b_to_goal if follower == bot_b else dist_a_to_goal:.1f}) "
                    f"following Bot {leader.id} (dist to goal: {dist_a_to_goal if leader == bot_a else dist_b_to_goal:.1f}) "
                    f"at distance {dist:.1f}"
                )
        
        # Clear follow state for bots that should no longer be following
        for bot in bots_to_check:
            if bot.id not in bots_that_should_follow and bot.is_following:
                bot.is_following = False
                bot.follow_target_id = None
                self.get_logger().info(f"Bot {bot.id} stopped following (distance threshold exceeded)")

    # ---------------- Main timer callback ----------------
    def control_cb(self):
        if len(self.crates) == 0:
            return
            
        # Reset slowdown scales
        for bot in self.bots.values():
            bot.slowdown_scale = 1.0

        # Assign crates
        self.assign_crates_greedily()

        # MODIFIED: Check for idle bots - ALL BOTS NOW USE STAGING LOGIC
        for bot in self.bots.values():
            if (bot.current_crate is None and 
                not bot.return_to_start and 
                not bot.move_after_attach and 
                not bot.going_to_staging_point and 
                not bot.has_returned_home and
                not bot.is_permanently_idle):

                # Check if bot needs staging based on Y position
                if bot.pose is not None and bot.pose[1] > self.staging_y_threshold:
                    bot.going_to_staging_point = True
                    
                    # Assign appropriate staging point based on bot ID and X position
                    if bot.id == 0:
                        # Bot 0: Check X position for staging point selection
                        if bot.pose[0] > 1219.0:
                            staging_goal = np.array([1700.0, 1219.0, 0.0])
                            self.get_logger().info(
                                f"Bot {bot.id} idle at y={bot.pose[1]:.1f}, x={bot.pose[0]:.1f} (> 1219), moving to staging point [1700, 1200, 0]")
                        else:
                            staging_goal = self.staging_point_0.copy()
                            self.get_logger().info(
                                f"Bot {bot.id} idle at y={bot.pose[1]:.1f}, x={bot.pose[0]:.1f} (<= 1219), moving to staging point [600, 1219, 0]")
                    
                    elif bot.id == 2:
                        # Bot 2: Check X position for staging point selection
                        if bot.pose[0] > 1219.0:
                            staging_goal = np.array([1700.0, 1500.0, 0.0])
                            self.get_logger().info(
                                f"Bot {bot.id} idle at y={bot.pose[1]:.1f}, x={bot.pose[0]:.1f} (> 1219), moving to staging point [1700, 1200, 0]")
                        else:
                            staging_goal = np.array([1700.0, 1500.0, 0.0])
                            self.get_logger().info(
                                f"Bot {bot.id} idle at y={bot.pose[1]:.1f}, x={bot.pose[0]:.1f} (<= 1219), moving to staging point [600, 1219, 0]")
                    
                    elif bot.id == 4:
                        # Bot 4: Check X position for staging point selection
                        if bot.pose[0] > 1219.0:
                            staging_goal = np.array([600.0, 1500.0, 0.0])
                            self.get_logger().info(
                                f"Bot {bot.id} idle at y={bot.pose[1]:.1f}, x={bot.pose[0]:.1f} (> 1219), moving to staging point [1700, 1200, 0]")
                        else:
                            staging_goal = np.array([600.0, 1500.0, 0.0])
                            self.get_logger().info(
                                f"Bot {bot.id} idle at y={bot.pose[1]:.1f}, x={bot.pose[0]:.1f} (<= 1219), moving to staging point [800, 1200, 0]")
                    else:
                        # Fallback for any other bot
                        staging_goal = np.array([600.0, 1219.0, 0.0])
                        self.get_logger().info(f"Bot {bot.id} idle, moving to default staging point")
                    
                    bot.goal = staging_goal
                
                else:
                    # Bot is below threshold, go home directly (skip staging)
                    bot.return_to_start = True
                    
                    # Set appropriate home point based on bot ID
                    if bot.id == 0:
                        bot.return_goal = self.idle_home_point_0
                        bot.goal = self.idle_home_point_0.copy()
                    elif bot.id == 2:
                        bot.return_goal = self.idle_home_point_2
                        bot.goal = self.idle_home_point_2.copy()
                    elif bot.id == 4:
                        bot.return_goal = self.idle_home_point_4
                        bot.goal = self.idle_home_point_4.copy()
                    else:
                        bot.return_goal = self.idle_home_point_0
                        bot.goal = self.idle_home_point_0.copy()
                    
                    if bot.pose is not None:
                        self.get_logger().info(f"Bot {bot.id} is idle at y={bot.pose[1]:.1f} (<= {self.staging_y_threshold}), returning home directly")
                    else:
                        self.get_logger().info(f"Bot {bot.id} is idle (no pose), returning home directly")

        # Check for collisions and set follow behavior
        self.check_and_handle_collisions()

        # Iterate through each bot's state machine
        for bot in self.bots.values():
            if bot.pose is None:
                continue

            # Handle follow behavior FIRST (highest priority)
            if bot.is_following and bot.follow_target_id is not None:
                self.get_logger().info(f"Bot {bot.id} executing FOLLOW behavior for Bot {bot.follow_target_id}")
                self.follow_bot(bot, bot.follow_target_id)
                continue

            # Skip bots that are dropping
            if bot.box_detached and bot.current_crate is not None:
                self.get_logger().debug(f"Bot {bot.id} is dropping crate {bot.tracked_crate_id}, skipping")
                continue
            
            # Handle staging point move for ALL BOTS
            if bot.going_to_staging_point:
                self.move_to_staging(bot)
                continue

            # Handle return home
            if bot.return_to_start:
                self.return_home(bot)
                continue

            # If no active crate, skip
            if bot.current_crate is None:
                continue
                
            # Zone locking checkpoint
            if bot.goal_reached and not bot.arm_placed:
                zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
                locking_bot_id = self.zone_locks.get(zone_id)
                
                higher_priority_bot_exists = False
                higher_priority_bot_id = -1
                for other_bot in self.bots.values():
                    if other_bot.id > bot.id and other_bot.current_crate is not None:
                        other_zone_id = self.get_zone_for_crate(other_bot.tracked_crate_id)
                        if other_zone_id == zone_id:
                            higher_priority_bot_exists = True
                            higher_priority_bot_id = other_bot.id
                            break
                            
                if locking_bot_id is None:
                    if higher_priority_bot_exists:
                        self.get_logger().info(f"Bot {bot.id} WAITING at crate. Higher-priority Bot {higher_priority_bot_id} needs zone {zone_id}.")
                        self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id)
                        continue
                    else:
                        self.zone_locks[zone_id] = bot.id
                        self.get_logger().info(f"Bot {bot.id} has LOCKED zone {zone_id}.")
                
                elif locking_bot_id == bot.id:
                    pass
                else:
                    self.get_logger().info(f"Bot {bot.id} WAITING at crate. Zone {zone_id} locked by Bot {locking_bot_id}.")
                    self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id)
                    continue

            # Run state machine
            if not bot.goal_reached:
                self.move_near_crate(bot)
            elif not bot.arm_placed:
                self.place_arm_on_crate(bot)
            elif not bot.box_attached:
                self.attach_crate(bot)
            elif not bot.arm_lifted:
                self.lift_arm_with_crate(bot)
            elif bot.move_after_attach:
                self.move_to_final_goal(bot)

    # ---------------- Follow Behavior Function ----------------
    def follow_bot(self, follower: BotState, leader_id: int):
        """
        Makes the follower bot follow the leader bot at a safe distance.
        """
        if leader_id not in self.bots:
            self.get_logger().warn(f"Bot {follower.id}: Leader {leader_id} not found!")
            return
        
        leader = self.bots[leader_id]
        if leader.pose is None or follower.pose is None:
            self.get_logger().warn(f"Bot {follower.id}: Missing pose data (leader or follower)")
            return
        
        follower.is_moving = True
        
        # Calculate direction from follower to leader
        dx = leader.pose[0] - follower.pose[0]
        dy = leader.pose[1] - follower.pose[1]
        dist_to_leader = math.hypot(dx, dy)
        
        self.get_logger().info(f"Bot {follower.id} following Bot {leader.id}: distance = {dist_to_leader:.1f}")
        
        # If already at good distance (within tolerance), just stop
        tolerance = 20.0
        if abs(dist_to_leader - follower.follow_offset_distance) < tolerance:
            self.get_logger().info(f"Bot {follower.id}: At good follow distance, stopping")
            self.publish_wheel_velocities([0.0, 0.0, 0.0], follower.id)
            follower.is_moving = False
            return
        
        # Calculate the desired follow position behind the leader
        # We want to be at follow_offset_distance away from the leader
        if dist_to_leader < 1.0:
            # Too close, can't calculate direction
            self.get_logger().info(f"Bot {follower.id}: Too close to leader, stopping")
            self.publish_wheel_velocities([0.0, 0.0, 0.0], follower.id)
            follower.is_moving = False
            return
        
        # Unit direction vector from follower to leader
        dir_x = dx / dist_to_leader
        dir_y = dy / dist_to_leader
        
        # Target position: stay at follow_offset_distance behind leader
        # Position ourselves along the line between us and the leader
        target_x = leader.pose[0] - dir_x * follower.follow_offset_distance
        target_y = leader.pose[1] - dir_y * follower.follow_offset_distance
        
        self.get_logger().info(
            f"Bot {follower.id}: Current=({follower.pose[0]:.1f}, {follower.pose[1]:.1f}), "
            f"Leader=({leader.pose[0]:.1f}, {leader.pose[1]:.1f}), "
            f"Target=({target_x:.1f}, {target_y:.1f})"
        )
        
        # Create temporary follow goal
        follow_goal = np.array([target_x, target_y, 0.0])
        
        # Use the standard movement function
        self.move_to_point(follower, follow_goal, base=45.0, elbow=45.0)

    # ---------------- Per-bot actions ----------------
    def move_near_crate(self, bot: BotState):
        bot.is_moving = True
        crate_pose = bot.current_crate
        if crate_pose is None:
            return
        goal_x = crate_pose['x']
        goal_y = crate_pose['y'] - 130.0
        goal_theta_deg = 0.0

        bot.goal = np.array([goal_x, goal_y, goal_theta_deg])

        now = self.get_clock().now()
        dt = (now - bot.last_time).nanoseconds / 1e9 if bot.last_time is not None else 0.03
        if dt <= 0:
            return

        x, y, theta_deg = bot.pose
        theta_rad = math.radians(theta_deg)
        goal_theta_rad = math.radians(goal_theta_deg)

        error_x = goal_x - x
        error_y = goal_y - y
        error_theta = goal_theta_rad - theta_rad

        vx_world = bot.pid_x.compute(error_x, dt)
        vy_world = bot.pid_y.compute(error_y, dt)
        omega = bot.pid_theta.compute(error_theta, dt)
        
        vx_world *= bot.slowdown_scale
        vy_world *= bot.slowdown_scale
        omega *= bot.slowdown_scale

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

        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id)
        dist = math.hypot(error_x, error_y)

        if dist < self.xy_tolerance:
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id)
            bot.is_moving = False
            bot.goal_reached = True
            self.get_logger().info(f"Bot {bot.id} reached near crate ID {bot.tracked_crate_id}")

    def place_arm_on_crate(self, bot: BotState):
        base_angle = 90.0
        elbow_angle = 90.0
        self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, base=base_angle, elbow=elbow_angle)
        bot.arm_placed = True
        time.sleep(3)
        self.get_logger().info(f"Bot {bot.id} arm placed on crate {bot.tracked_crate_id}.")

    def get_model1_name(self, bot_id: int):
        if bot_id == 0:
            return "hb_crystal"
        elif bot_id == 4:
            return "hb_glacio"
        elif bot_id == 2:
            return "hb_frostbite"
        else:
            return "hb_crystal"
        
    def attach_crate(self, bot: BotState):
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/attach_link service unavailable.")
            return

        color_mod = self.get_zone_for_crate(bot.tracked_crate_id)
        
        if color_mod == 0:
            model2_name = f"crate_red_{bot.tracked_crate_id}"
        elif color_mod == 1:
            model2_name = f"crate_green_{bot.tracked_crate_id}"
        else:
            model2_name = f"crate_blue_{bot.tracked_crate_id}"

        link2_name = f"box_link_{bot.tracked_crate_id}"
        req = AttachLink.Request()
        model1_name = self.get_model1_name(bot.id)

        self.get_logger().info(
            f"[ATTACH] Bot {bot.id} → model1='{model1_name}', model2='{model2_name}'"
        )

        req.data = f"""{{
            "model1_name": "{model1_name}",
            "link1_name": "arm_link_2",
            "model2_name": "{model2_name}",
            "link2_name": "{link2_name}"
        }}"""
        future = self.attach_client.call_async(req)
        future.add_done_callback(lambda fut, b=bot: self.attach_done_callback(fut, b))
        bot.box_attached = True
        self.get_logger().info(f"Bot {bot.id} requested attach for crate {bot.tracked_crate_id}.")

    def lift_arm_with_crate(self, bot: BotState):
        base_angle = 45.0
        elbow_angle = 45.0
        self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, base=base_angle, elbow=elbow_angle)
        bot.arm_lifted = True
        bot.move_after_attach = True
        self.get_logger().info(f"Bot {bot.id} arm lifted with crate attached!")

    def move_to_final_goal(self, bot: BotState):
        drop_pos = self.drop_positions.get(bot.tracked_crate_id, None)
        if drop_pos is None:
            goal = bot.final_goal
        else:
            goal = np.array([drop_pos['x'], (drop_pos['y'] - 190), 0.0])

        bot.goal = goal.copy()
        if self.move_to_point(bot, goal, base=45.0, elbow=45.0):
            bot.move_after_attach = False
            self.get_logger().info(f"Bot {bot.id} reached final goal. Dropping crate {bot.tracked_crate_id}...")
            self.drop_crate(bot)

    # MODIFIED: Move to staging for ALL BOTS
    def move_to_staging(self, bot: BotState):
        """
        Moves ALL bots to their intermediate staging point.
        """
        if self.move_to_point(bot, bot.goal, base=45.0, elbow=45.0):
            self.get_logger().info(f"Bot {bot.id} reached staging point. Now returning home.")
            bot.going_to_staging_point = False
            bot.return_to_start = True
            
            # Set appropriate home point based on bot ID
            if bot.id == 0:
                bot.return_goal = self.idle_home_point_0
                bot.goal = self.idle_home_point_0.copy()
            elif bot.id == 2:
                bot.return_goal = self.idle_home_point_2
                bot.goal = self.idle_home_point_2.copy()
            elif bot.id == 4:
                bot.return_goal = self.idle_home_point_4
                bot.goal = self.idle_home_point_4.copy()
            else:
                bot.return_goal = self.idle_home_point_0
                bot.goal = self.idle_home_point_0.copy()
            
            bot.reset_pid()

    def return_home(self, bot: BotState):
        if self.move_to_point(bot, bot.return_goal, base=45.0, elbow=45.0):
            bot.return_to_start = False
            bot.has_returned_home = True
            bot.is_permanently_idle = True
            bot.goal = bot.return_goal.copy()
            self.get_logger().info(f"Bot {bot.id} returned to start position and is now permanently idle.")
            self.reset_bot_state(bot)

    # ---------------- Helper Movement Function ----------------
    def move_to_point(self, bot: BotState, goal, base=0.0, elbow=0.0):
        bot.is_moving = True
        now = self.get_clock().now()
        dt = (now - bot.last_time).nanoseconds / 1e9 if bot.last_time is not None else 0.03
        if dt <= 0:
            return False

        x, y, theta_deg = bot.pose
        theta_rad = math.radians(theta_deg)
        goal_x, goal_y, goal_theta_deg = goal
        goal_theta_rad = math.radians(goal_theta_deg)

        error_x = goal_x - x
        error_y = goal_y - y
        error_theta = (goal_theta_rad - theta_rad + math.pi) % (2 * math.pi) - math.pi

        vx_world = bot.pid_x.compute(error_x, dt)
        vy_world = bot.pid_y.compute(error_y, dt)
        omega = bot.pid_theta.compute(error_theta, dt)

        vx_world *= bot.slowdown_scale
        vy_world *= bot.slowdown_scale
        omega *= bot.slowdown_scale

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

        self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, base=base, elbow=elbow)

        dist = math.hypot(error_x, error_y)
        if dist < (self.xy_tolerance + 3) and abs(error_theta) < math.radians(self.theta_tolerance_center_deg):
            bot.is_moving = False
            self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, base=base, elbow=elbow)
            return True
        return False

    # ---------------- Drop Sequence ----------------
    def drop_crate(self, bot: BotState):
        self.get_logger().info(f"Bot {bot.id} DROPPING crate {bot.tracked_crate_id} - LOCKING from assignment")
        
        base_angle = 85.0
        elbow_angle = 85.0
        self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, base=base_angle, elbow=elbow_angle)
        time.sleep(3)

        if not self.detach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/detach_link service unavailable.")
            return

        color_mod = self.get_zone_for_crate(bot.tracked_crate_id)

        if color_mod == 0:
            model2_name = f"crate_red_{bot.tracked_crate_id}"
        elif color_mod == 1:
            model2_name = f"crate_green_{bot.tracked_crate_id}"
        else:
            model2_name = f"crate_blue_{bot.tracked_crate_id}"

        link2_name = f"box_link_{bot.tracked_crate_id}"
        req = DetachLink.Request()
        model1_name = self.get_model1_name(bot.id)

        req.data = f"""{{
            "model1_name": "{model1_name}",
            "link1_name": "arm_link_2",
            "model2_name": "{model2_name}",
            "link2_name": "{link2_name}"
        }}"""

        future = self.detach_client.call_async(req)
        future.add_done_callback(lambda fut, b=bot: self.detach_done_callback(fut, b))
        bot.box_detached = True

        time.sleep(0.8)
        lift_base, lift_elbow = 45.0, 45.0
        self.publish_wheel_velocities([0.0, 0.0, 0.0], bot.id, base=lift_base, elbow=lift_elbow)

        drop_pos = self.drop_positions.get(bot.tracked_crate_id, None)
        if drop_pos:
            self.get_logger().info(
                f"Bot {bot.id}: Crate ID {bot.tracked_crate_id} placed at "
                f"({drop_pos['x']:.1f}, {drop_pos['y']:.1f}) successfully."
            )
        else:
            self.get_logger().info(f"Bot {bot.id}: Crate ID {bot.tracked_crate_id} placed successfully.")

    # ---------------- Service callbacks ----------------
    def attach_done_callback(self, future, bot: BotState):
        try:
            res = future.result()
            if hasattr(res, 'success') and res.success:
                self.get_logger().info(f"Bot {bot.id}: Box attached successfully.")
            else:
                self.get_logger().warn(f"Bot {bot.id}: Attach failed.")
        except Exception as e:
            self.get_logger().error(f"Bot {bot.id}: Attach failed: {e}")

    def detach_done_callback(self, future, bot: BotState):
        try:
            res = future.result()
            if hasattr(res, 'success') and res.success:
                self.get_logger().info(f"Bot {bot.id}: Crate detached successfully.")
                
                # Unlock the zone
                try:
                    zone_id = self.get_zone_for_crate(bot.tracked_crate_id)
                    if self.zone_locks.get(zone_id) == bot.id:
                        self.zone_locks[zone_id] = None
                        self.get_logger().info(f"Bot {bot.id} has UNLOCKED zone {zone_id}.")
                    elif self.zone_locks.get(zone_id) is not None:
                        self.get_logger().warn(f"Bot {bot.id} finished in zone {zone_id}, but lock was held by {self.zone_locks.get(zone_id)}!")
                except Exception as e:
                    self.get_logger().error(f"Error during zone unlock: {e}")

                # Mark as completed BEFORE clearing current_crate
                if bot.current_crate is not None:
                    bot.completed_crates.add(bot.current_crate['id'])
                    self.get_logger().info(f"Bot {bot.id}: Marked crate {bot.current_crate['id']} as COMPLETED")
                
                # Clear current_crate
                bot.current_crate = None
                bot.tracked_crate_id = None

                # Reset flags
                bot.goal_reached = False
                bot.arm_placed = False
                bot.box_attached = False
                bot.arm_lifted = False
                bot.has_returned_home = False
                bot.move_after_attach = False
                bot.box_detached = False
                bot.reset_pid()
                
                # Priority cooldown
                bot.priority_cooldown_end_time = self.get_clock().now() + Duration(seconds=1)
                self.get_logger().info(f"Bot {bot.id} has priority cooldown")

            else:
                self.get_logger().warn(f"Bot {bot.id}: Detach failed.")
        except Exception as e:
            self.get_logger().error(f"Bot {bot.id}: Detach failed: {e}")

    # ---------------- Utilities ----------------
    def reset_bot_state(self, bot: BotState):
        """
        General reset, used when returning home.
        Does NOT reset has_returned_home or is_permanently_idle flags.
        """
        bot.goal_reached = False
        bot.arm_placed = False
        bot.box_attached = False
        bot.arm_lifted = False
        bot.goal = None
        bot.move_after_attach = False
        bot.box_detached = False
        bot.going_to_staging_point = False
        bot.is_moving = False
        bot.is_following = False
        bot.follow_target_id = None
        bot.reset_pid()

    def publish_wheel_velocities(self, wheel_vel, bot_id, base=45.0, elbow=45.0):
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = bot_id
        if len(wheel_vel) >= 3:
            cmd.m1, cmd.m2, cmd.m3 = wheel_vel[0], wheel_vel[1], wheel_vel[2]
        else:
            cmd.m1 = wheel_vel[0] if len(wheel_vel) > 0 else 0.0
            cmd.m2 = wheel_vel[1] if len(wheel_vel) > 1 else 0.0
            cmd.m3 = 0.0
        cmd.base = base
        cmd.elbow = elbow
        msg.cmds.append(cmd)
        self.publisher.publish(msg)


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