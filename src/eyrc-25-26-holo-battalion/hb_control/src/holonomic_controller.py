#!/usr/bin/env python3

'''
This Python file runs a ROS 2 node of name holonomic_pid_controller which holds the position of a holonomic robot
and drives it through a series of predefined goals using PID controllers on [x, y, θ].

This node publishes and subscribes to the following topics:

        PUBLICATIONS                               SUBSCRIPTIONS
        /forward_velocity_controller/commands      /bot_pose

Instead of defining separate variables for each PID axis, lists/dictionaries are used.
For example: pid_params['x'], pid_params['y'], pid_params['theta'], etc.

Code modularity and clarity are maintained to make tuning and extension easier.
'''

# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from hb_interfaces.msg import Poses2D
import numpy as np
import math


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
#-----------------------------PID Compute Steps--------------------------------------------------------------
        # 1. Accumulate the error over time for the Integral term
        self.integral += error*dt
        # 2. Compute the change in error for the Derivative term
        derivative = (error - self.prev_error) / dt
        # 3. Calculate the PID output:
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # 4. Store the current error for use in the next iteration
        self.prev_error = error
        # 5. Limit (clip) the output between [-max_out, +max_out] to avoid unsafe velocities
        return np.clip(output, -self.max_out, self.max_out)
#------------------------------------------------------------------------------------------------------------ 
    
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node

        # ---------------- Robot Parameters ----------------
        # 1. Robot ID(s)
        self.robot_id=9
        # 2. Current pose of the robot:
        self.current_pose = None
        #    - Updated from the /bot_pose topic in the callback function.
        #    - Stores [x, y, θ] information for the active robot.
        # 3. Goal tracking index
        self.goal_index = 0
        # 4. Timing information:
        self.last_time = self.get_clock().now()
        #    - Used to calculate the time difference (dt) between control loop iterations.
        # 5. Threshold for goal completion:
        self.xy_tolerance = 2.0  # meters
        self.theta_tolerance_deg = 2.0 # degrees
        self.max_vel = 3.0 # m/s
        #    - Defines the acceptable error tolerance for x, y, and θ.
        #    - Example: if error < 5 units → goal considered reached.

        # ---------------- Goal Definitions ----------------

        # List of waypoints [(x, y, yaw_deg)]
        self.goals = [
            (820, 920, 0),
            (820, 1520, 0),
            (1620, 1520, 0),
            (1620, 920, 0),
            (820, 920, 0),
        ]

        #----------------DO NOT CHNAGE----------------------

        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 0.2, 'ki': 0.00001, 'kd': 0.00001, 'max_out': self.max_vel},
            'y': {'kp': 0.2, 'ki': 0.00001, 'kd': 0.00001, 'max_out': self.max_vel},
            'theta': {'kp': 0.1, 'ki': 0.00002, 'kd': 0.00002, 'max_out': self.max_vel * 2}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        
        # Write a subscriber for /bot_pose

        self.subscription = self.create_subscription(
            Poses2D, 'bot_pose', self.pose_cb, 10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')


    # ---------------- Subscriber Callback ----------------
    def pose_cb(self, msg):
        """
        Callback function for /bot_pose topic.
        This function is executed each time a message is received.

        Steps:
        1. Iterate through all poses in the incoming message.
        2.  Update self.current_pose with this robot’s pose.
        """

        for pose in msg.poses:
            if pose.id == self.robot_id:
                self.current_pose = np.array([pose.x, pose.y, pose.w])
                break

    # ---------------- Control Loop ----------------
    def control_cb(self):

        """
        Control loop callback executed periodically by the ROS 2 timer.

        Main Steps:
        1. Check if the current pose is available; if not, exit.
        2. Compute the time difference (dt) since the last control cycle.
        3. Get the current robot pose (x, y, θ).
        4. If all goals are completed → stop the robot.
        5. Select the current goal (x, y, θ) from the goals list.
        6. Compute errors in x, y, and θ between current pose and goal.
        7. Use PID controllers to calculate required body velocities [vx, vy, ω].
        8. Convert body velocities into individual wheel velocities.
        9. Limit (clip) wheel velocities within safe bounds.
        10. Publish the wheel velocities to the motor controller.
        11. Check if the goal is reached:
              - If yes → update goal index, reset PIDs, and move to the next goal.
        """

        if self.current_pose is None:
            return

        # Time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Current robot pose
        x, y, theta_deg = self.current_pose
        theta_rad=math.radians(theta_deg)

        # If all goals are reached → stop

        if self.goal_index >= len(self.goals):
            self.publish_wheel_velocities([0.0, 0.0, 0.0])
            self.get_logger().info('All goals reached!')
            return

        # Current target goal

        goal_x, goal_y, goal_theta_deg = self.goals[self.goal_index]
        goal_theta_rad=math.radians(goal_theta_deg)

        # Errors

        error_x=goal_x-x
        error_y=goal_y-y
        error_theta=goal_theta_rad-theta_rad

        # PID outputs

        vx_world = self.pid_x.compute(error_x, dt)
        vy_world = self.pid_y.compute(error_y, dt)
        omega = self.pid_theta.compute(error_theta, dt)

        # Convert to wheel velocities (custom equations)

        cos_theta=math.cos(theta_rad)
        sin_theta=math.sin(theta_rad)
        vx_robot=vx_world*cos_theta+vy_world*sin_theta
        vy_robot=-vx_world*sin_theta+vy_world*cos_theta


        alphas = [math.radians(30), math.radians(150), math.radians(270)]

        M = np.array([
            [math.cos(alpha + math.pi/2) for alpha in alphas],
            [math.sin(alpha + math.pi/2) for alpha in alphas],
            [1.0, 1.0, 1.0]
        ])

        v_robot = np.array([vx_robot, vy_robot, omega])

        wheel_vel = np.linalg.pinv(M) @ v_robot

        self.publish_wheel_velocities(wheel_vel.tolist())        

        # Goal check

        distance_to_goal = math.sqrt(error_x**2 + error_y**2)
        theta_error_deg = abs(math.degrees(error_theta))

        if distance_to_goal < self.xy_tolerance and theta_error_deg < self.theta_tolerance_deg:
            self.get_logger().info(f'Goal {self.goal_index} reached: ({goal_x}, {goal_y})')
            self.goal_index += 1
            # Reset PIDs to prevent integral windup on the next goal
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()


    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = Float64MultiArray()
        msg.data = np.array(wheel_vel).tolist()
        self.publisher.publish(msg)


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
