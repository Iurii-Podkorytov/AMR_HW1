#!/usr/bin/env python3

from .generic_controller import GenericController
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import numpy as np
import do_mpc
from casadi import *
from .utils import *

class MPController(GenericController):
    def __init__(self):
        super().__init__('model_predictive_controller')
        self.get_logger().info("Model Predictive Controller started")
        
        self.waypoints = np.array([(0, 0), (3, 0), (6, 4), (3, 4), (3, 1), (0, 3)])

        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.03
        self.max_linear_vel = 0.5
        self.max_angular_vel = 1

        # Initialize MPC
        self.model = do_mpc.model.Model('continuous')
        self.setup_model()
        self.mpc = do_mpc.controller.MPC(self.model)

        # MPC parameters
        setup_mpc = {
            'n_horizon': 30,
            't_step': 0.1,
            'n_robust': 0,
            'store_full_solution': True,
        }
        self.mpc.set_param(**setup_mpc)
        p_template = self.mpc.get_p_template(n_combinations=1) 

        # Cost function weights
        Q_x = 50  # Position x weight
        Q_y = 50  # Position y weight
        R_v = 1  # Linear velocity control effort
        R_omega = 7  # Angular velocity control effort

        # Get model variables
        x = self.model.x['x']
        y = self.model.x['y']
        pos_des_x = self.model.p['pos_des_x']
        pos_des_y = self.model.p['pos_des_y']

        # Define the cost function
        lterm = Q_x * (x - pos_des_x)**2 + Q_y * (y - pos_des_y)**2
        self.mpc.set_objective(lterm=lterm, mterm=DM(0))
        self.mpc.set_rterm(v=R_v, omega=R_omega)

        # Input constraints
        self.mpc.bounds['lower', '_u', 'v'] = -self.max_linear_vel
        self.mpc.bounds['upper', '_u', 'v'] = self.max_linear_vel
        self.mpc.bounds['lower', '_u', 'omega'] = -self.max_angular_vel
        self.mpc.bounds['upper', '_u', 'omega'] = self.max_angular_vel

        self.mpc.set_p_fun(self.p_fun)
        # Setup MPC
        self.mpc.setup()

        self.mpc.x0 = np.zeros((3, 1))
        self.mpc.set_initial_guess()

    # Add this method to handle parameters
    def p_fun(self, t_now):
        # Populate template with current waypoint
        p_template = self.mpc.get_p_template(n_combinations=1)
        target = self.waypoints[self.current_waypoint_idx]
        p_template['_p', 0] = np.array([target[0], target[1]])
        return p_template

    def setup_model(self):
        # State variables (x, y, theta)
        self.model.set_variable(var_type='_x', var_name='x') 
        self.model.set_variable(var_type='_x', var_name='y')
        self.model.set_variable(var_type='_x', var_name='theta')

        # Control inputs (v, omega)
        self.model.set_variable(var_type='_u', var_name='v')
        self.model.set_variable(var_type='_u', var_name='omega')

        # Parameters (desired x and y)
        self.model.set_variable(var_type='_p', var_name='pos_des_x')
        self.model.set_variable(var_type='_p', var_name='pos_des_y')

        # Differential equations
        x_dot = self.model.u['v'] * cos(self.model.x['theta'])
        y_dot = self.model.u['v'] * sin(self.model.x['theta'])
        theta_dot = self.model.u['omega']

        self.model.set_rhs('x', x_dot)
        self.model.set_rhs('y', y_dot)
        self.model.set_rhs('theta', theta_dot)

        self.model.setup()

    def compute_cmd_vel(self):
        twist = Twist()

        # Check if all waypoints are reached
        if self.current_waypoint_idx >= len(self.waypoints):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist

        # Get current state from odometry
        current_x, current_y = get_position_from_odom(self.odom_data)
        current_theta = get_yaw_from_odom(self.odom_data)
        x0 = np.array([[current_x], [current_y], [current_theta]])

        # Current target waypoint
        target = self.waypoints[self.current_waypoint_idx]
        des_x, des_y = target[0], target[1]

        try:
            # Compute control action
            u0 = self.mpc.make_step(x0)
            v = u0[0][0]
            omega = u0[1][0]
        except Exception as e:
            self.get_logger().error(f"MPC failed: {e}")
            v = 0.0
            omega = 0.0

        # Apply velocity limits
        v = np.clip(v, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(omega, -self.max_angular_vel, self.max_angular_vel)

        # Publish the twist message
        twist.linear.x = float(v)
        twist.angular.z = float(omega)

        # Check if current waypoint is reached
        distance = np.hypot(current_x - des_x, current_y - des_y)
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx - 1}")

        return twist

def main(args=None):
    rclpy.init(args=args)
    controller = MPController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()