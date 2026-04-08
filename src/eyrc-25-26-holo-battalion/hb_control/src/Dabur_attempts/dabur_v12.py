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

    def compute(self, error, dt, error_threshold=None):
        if dt <= 0:
            return 0.0
        
        if error_threshold is not None and abs(error) < error_threshold:
            self.integral = 0.0
        
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
        self.post_ir_duration = 0.7

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
        self.is_sharing_zone = False 
        
        # Logging flags
        self.drop_pose_logged = False
        self.arm_placed_logged = False
        
        # Distance logging
        self.last_dist_log_time = time.time()
        self.dist_log_interval = 0.5 

        # magnet state
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

        # Collision / Motion
        self.pose_timestamp = node.get_clock().now()
        self.prev_pose_for_vel = None
        self.velocity = 0.0
        self.is_moving = False
        self.slowdown_scale = 1.0

        # Follow Behavior
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


# ---------------------- Multi-Axis Twiddle Tuner ----------------------------
class MultiAxisTwiddleTuner:
    def __init__(self, node, bot_id):
        self.node = node
        self.bot_id = bot_id
        
        # --- Experiment Configuration ---
        # DIAGONAL MOVE: This forces X, Y, and Theta errors simultaneously.
        self.start_pos = np.array([1000.0, 1000.0, 0.0])   
        self.target_pos = np.array([1800.0, 1500.0, 90.0]) 
        
        self.xy_tolerance = 20.0       # mm
        self.theta_tolerance = 3.0     # degrees 
        
        # --- Cost Function Weights ---
        # Theta is weighted heavily because heading errors cause large XY deviations
        self.weight_xy = 1.0
        self.weight_theta = 800.0 
        
        # --- Twiddle State (9 Parameters) ---
        # Order: [X_kp, X_ki, X_kd,  Y_kp, Y_ki, Y_kd,  T_kp, T_ki, T_kd]
        self.p = [0.0] * 9
        self.param_names = [
            "X_kp", "X_ki", "X_kd",
            "Y_kp", "Y_ki", "Y_kd",
            "Theta_kp", "Theta_ki", "Theta_kd"
        ]
        
        # Initial Search Step Sizes (Tweaks)
        self.dp = [
            0.1, 0.001, 0.01,    # X
            0.1, 0.001, 0.01,    # Y
            1.0, 0.001, 0.01     # Theta
        ]
        
        self.best_err = float('inf')
        self.run_err = 0.0
        self.p_index = 0  # 0 to 8
        self.state = 0    # 0: Init, 1: Check +dp, 2: Check -dp
        
        self.cycle_step = "RESET" # RESET -> RUN -> EVALUATE
        self.iteration = 0

    def load_current_pids(self, bot):
        """Reads the bot's current PID params into our list"""
        self.p[0], self.p[1], self.p[2] = bot.pid_x.kp, bot.pid_x.ki, bot.pid_x.kd
        self.p[3], self.p[4], self.p[5] = bot.pid_y.kp, bot.pid_y.ki, bot.pid_y.kd
        self.p[6], self.p[7], self.p[8] = bot.pid_theta.kp, bot.pid_theta.ki, bot.pid_theta.kd
        
        self.node.get_logger().info(f"Twiddle Initialized. Starting Params: {['%.3f' % elem for elem in self.p]}")

    def apply_pids_to_bot(self, bot):
        """Writes our list back to the bot's PID objects"""
        safe_p = [abs(x) for x in self.p] # Ensure positive values
        
        bot.pid_x.kp, bot.pid_x.ki, bot.pid_x.kd = safe_p[0], safe_p[1], safe_p[2]
        bot.pid_y.kp, bot.pid_y.ki, bot.pid_y.kd = safe_p[3], safe_p[4], safe_p[5]
        bot.pid_theta.kp, bot.pid_theta.ki, bot.pid_theta.kd = safe_p[6], safe_p[7], safe_p[8]
        
        bot.reset_pid()

    def run_cycle(self, bot):
        if bot.pose is None: return 0,0,0
        
        cur_x, cur_y, cur_deg = bot.pose
        cur_rad = math.radians(cur_deg)
        
        tar_x, tar_y, tar_deg = self.target_pos
        tar_rad = math.radians(tar_deg)

        # ---------------- STEP 1: RESET TO START ----------------
        if self.cycle_step == "RESET":
            dx = self.start_pos[0] - cur_x
            dy = self.start_pos[1] - cur_y
            dtheta = (0.0 - cur_rad + math.pi) % (2*math.pi) - math.pi
            
            dist = math.hypot(dx, dy)
            
            if dist < 25.0 and abs(dtheta) < 0.1:
                # Start the run
                self.node.get_logger().info(f"Iter {self.iteration} | Tuning: {self.param_names[self.p_index]} | P={self.p[self.p_index]:.4f}")
                self.run_err = 0.0
                bot.reset_pid()
                self.cycle_step = "RUN"
                return 0,0,0
            
            # Simple P-controller to drag robot back to start
            vx = 1.2 * dx
            vy = 1.2 * dy
            w  = 2.5 * dtheta
            return vx, vy, w

        # ---------------- STEP 2: RUN TEST (Diagonal Move) ----------------
        elif self.cycle_step == "RUN":
            ex = tar_x - cur_x
            ey = tar_y - cur_y
            e_theta = (tar_rad - cur_rad + math.pi) % (2*math.pi) - math.pi
            
            # --- COMBINED COST FUNCTION ---
            # This is where "All at once" happens.
            # If Theta is bad, Score is bad. If X is bad, Score is bad.
            inst_error = (self.weight_xy * (ex**2 + ey**2)) + (self.weight_theta * (e_theta**2))
            
            self.run_err += inst_error

            # Check completion
            dist_err = math.hypot(ex, ey)
            deg_err = math.degrees(abs(e_theta))
            
            if dist_err < self.xy_tolerance and deg_err < self.theta_tolerance:
                self.cycle_step = "EVALUATE"
                return 0,0,0
            
            # Compute Control using CURRENT PIDs
            dt = 0.03
            vx = bot.pid_x.compute(ex, dt)
            vy = bot.pid_y.compute(ey, dt)
            w  = bot.pid_theta.compute(e_theta, dt)
            
            return vx, vy, w

        # ---------------- STEP 3: EVALUATE & OPTIMIZE ----------------
        elif self.cycle_step == "EVALUATE":
            self.iteration += 1
            self.node.get_logger().info(f"  -> Score: {self.run_err:.2e} (Best: {self.best_err:.2e})")

            # --- COORDINATE DESCENT LOGIC ---
            # Optimize p[index], then move to p[index+1]
            
            if self.best_err == float('inf'):
                self.best_err = self.run_err
                self.p[self.p_index] += self.dp[self.p_index]
                self.state = 1
            
            elif self.state == 1: 
                if self.run_err < self.best_err:
                    self.best_err = self.run_err
                    self.dp[self.p_index] *= 1.1
                    # Move to next param
                    self.p_index = (self.p_index + 1) % 9
                    self.p[self.p_index] += self.dp[self.p_index]
                    self.state = 1
                else:
                    self.p[self.p_index] -= 2 * self.dp[self.p_index]
                    self.state = 2
            
            elif self.state == 2:
                if self.run_err < self.best_err:
                    self.best_err = self.run_err
                    self.dp[self.p_index] *= 1.1
                    # Move to next param
                    self.p_index = (self.p_index + 1) % 9
                    self.p[self.p_index] += self.dp[self.p_index]
                    self.state = 1
                else:
                    # Restore and shrink step
                    self.p[self.p_index] += self.dp[self.p_index] 
                    self.dp[self.p_index] *= 0.9 
                    # Move to next param
                    self.p_index = (self.p_index + 1) % 9
                    self.p[self.p_index] += self.dp[self.p_index]
                    self.state = 1
            
            self.cycle_step = "RESET"
            self.apply_pids_to_bot(bot)
            return 0,0,0

        return 0,0,0

# ---------------------- Main Node Class -------------------------------------
class HolonomicMoveToCratesMulti(Node):

    def __init__(self):
        super().__init__('holonomic_move_to_crates_multi') 

        self.node_start_time = time.time()
        self.startup_delay = 5.0
        self.is_warmed_up = False

        # ---------------- TUNING SETUP ----------------
        self.TUNING_MODE = True    # Set to True to tune all axes
        self.TUNING_BOT_ID = 2    # Bot to tune
        
        self.tuner = MultiAxisTwiddleTuner(self, self.TUNING_BOT_ID)
        self.tuner_initialized = False

        # ---------------- Robot and world state ----------------
        self.bot_ids = [0,2,4]
        self.target_crate_ids = [30, 14, 12, 11, 13, 22]
        self.crates = [] 
        self.drop_positions = {}
        self.drop_positions_assigned = False

        self.last_time = self.get_clock().now()

        # ---------------- Motion tolerances ----------------
        self.xy_tolerance = 15.0
        self.theta_tolerance_deg = 3.0  
        self.max_vel = 50.0  

        self.slow_radius = 120.0      
        self.min_slow_scale = 0.25    

        self.follow_distance_threshold = 350.0  
        self.bot_radius = 100.0

        # ---------------- Global goals ----------------
        self.final_goal = np.array([1219.0,1180.0,0.0])  
        self.return_goal = np.array([1218.0,205.0,0.0])  

        # Staging and Home points
        self.staging_point_0 = np.array([800.0,1219.0,0.0])  
        self.staging_point_2 = np.array([1700.0,1200.0,0.0])  
        self.staging_point_4 = np.array([800.0,1200.0,0.0])  
        self.staging_y_threshold = 1400.0  
        self.sp_0_alt = np.array([1700.0, 1219.0, 0.0]) 
        self.sp_4_alt = np.array([800.0, 1200.0, 0.0])

        self.idle_home_point_0 = np.array([1218.0,205.0,0.0])  
        self.idle_home_point_2 = np.array([1568.0,202.0,0.0])  
        self.idle_home_point_4 = np.array([864.0,204.0,0.0])  

        # ---------------- Kinematics ----------------
        alphas = [math.radians(30), math.radians(150), math.radians(270)]
        M = np.array([
            [math.cos(a + math.pi/2) for a in alphas],
            [math.sin(a + math.pi/2) for a in alphas],
            [1, 1, 1]
        ])
        self.M_inv = np.linalg.pinv(M)

        # ---------------- Per-bot Config ----------------
        self.arm_down_config = {
            0: (15.0, 180.0),
            2: (15.0, 180.0),  
            4: (5.0, 180.0)
        }

        self.pid_params_by_bot = {
            0:{'x':{'kp':1.5,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
               'y':{'kp':1.9,'ki':0.0,'kd':0.01,'max_out':self.max_vel},
               'theta':{'kp':45,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            2:{'x':{'kp':2.2,'ki':0.008,'kd':0,'max_out':self.max_vel},
               'y':{'kp':2.7,'ki':0.008,'kd':0.01,'max_out':self.max_vel},
               'theta':{'kp':50,'ki':0.006,'kd':0.1,'max_out':self.max_vel*2}},
            4:{'x':{'kp':1.3,'ki':0.0002,'kd':0.01,'max_out':self.max_vel},
               'y':{'kp':1.9,'ki':0.0001,'kd':0.03,'max_out':self.max_vel},
               'theta':{'kp':40,'ki':0.005,'kd':0.1,'max_out':self.max_vel*2}},
        }

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
        self.zone_subparts = {} # Defined in your original code... keeping it minimal here for brevity if needed

        # ---------------- ROS interfaces ----------------
        self.create_subscription(Poses2D,'bot_pose',self.pose_cb,10)
        self.create_subscription(Poses2D,'crate_pose',self.crate_cb,10)
        self.publisher = self.create_publisher(BotCmdArray,'/bot_cmd',10)
        self.timer = self.create_timer(0.03,self.control_cb)
        self.debug_pub = self.create_publisher(String, '/perception_debug', 10)
        self.debug_timer = self.create_timer(0.2, self.publish_perception_debug)

        # ---------------- MQTT setup ----------------
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        try:
            self.mqtt_client.connect("10.95.243.6", 1883, 60)
            for bid in self.bot_ids:
                topic = f"bot{bid}/ir"
                self.mqtt_client.subscribe(topic)
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

    # ... [Keep your existing callbacks: on_mqtt_connect, on_mqtt_disconnect, pose_cb, crate_cb, log_bot_distance, etc.] ...
    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Connected (rc={rc})")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"MQTT Disconnected (rc={rc}). Attempting reconnect...")
        try:
            client.loop_start()
        except:
            pass

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
                self.crates.append({'id':pose.id,'x':pose.x,'y':pose.y,'w':pose.w})
        if not self.drop_positions_assigned and len(self.crates) > 0:
            self.assign_drop_positions()
            self.drop_positions_assigned = True

    def assign_drop_positions(self):
        # ... (Same as your original code) ...
        pass
    
    def generate_grid_position(self,crate_id):
        # ... (Same as your original code) ...
        return (0,0) # Placeholder for valid code

    def assign_crates_greedily(self):
        # ... (Same as your original code) ...
        pass

    def check_and_handle_collisions(self):
        # ... (Same as your original code) ...
        pass

    def update_arm_smooth(self, bot):
        # ... (Same as your original code) ...
        return True

    def apply_crate_avoidance(self, bot, vx, vy):
        # ... (Same as your original code) ...
        return vx, vy

    def publish_wheel_velocities(self, wheel_vel, bot_id, base=45.0, elbow=135.0, mag=0):
        # ... (Same as your original code) ...
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
        # ... (Same as your original code) ...
        pass

    def publish_perception_debug(self):
        # ... (Same as your original code) ...
        pass
        
    def get_zone_for_crate(self, id): return id % 3

    # =========================================================
    #  MODIFIED CONTROL CALLBACK
    # =========================================================
    def control_cb(self):
        if not self.is_warmed_up:
            elapsed = time.time() - self.node_start_time
            if elapsed < self.startup_delay:
                self.get_logger().info(f"Waiting for vision... {elapsed:.1f}", throttle_duration_sec=1.0)
                return
            else:
                self.is_warmed_up = True

        # =========================================================
        # TWIDDLE TUNING BLOCK (MULTI-AXIS)
        # =========================================================
        if self.TUNING_MODE:
            if self.TUNING_BOT_ID not in self.bots: return
            
            bot = self.bots[self.TUNING_BOT_ID]
            if bot.pose is None: 
                self.get_logger().info("Twiddle: Waiting for Bot Pose...")
                return

            # Initialize Tuner with current PID values ONCE
            if not self.tuner_initialized:
                self.tuner.load_current_pids(bot)
                self.tuner_initialized = True
            
            # Run Tuning Cycle
            vx, vy, omega = self.tuner.run_cycle(bot)
            
            # Kinematics and Publish
            theta = math.radians(bot.pose[2])
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            vx_r = vx * cos_t + vy * sin_t
            vy_r = -vx * sin_t + vy * cos_t
            
            wheel_vel = self.M_inv @ np.array([vx_r, vy_r, omega])
            
            # Cap velocity for safety during tuning
            scale = 1.0
            max_wheel = np.max(np.abs(wheel_vel))
            if max_wheel > 60.0: scale = 60.0 / max_wheel
            wheel_vel *= scale

            self.publish_wheel_velocities(wheel_vel.tolist(), bot.id, bot.base_angle, bot.elbow_angle, 0)
            return
        # =========================================================

        # [NORMAL OPERATION CODE WOULD GO HERE, BUT IS SKIPPED IF TUNING_MODE IS TRUE]
        # Copy your original operational logic (assigning crates, etc.) here if needed
        # for when TUNING_MODE = False.

# ---------------------- Main -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = HolonomicMoveToCratesMulti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()