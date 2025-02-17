#!/usr/bin/env python3

import rclpy
from .generic_controller import GenericController
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import numpy as np
from .utils import *

class ProportionalController(GenericController):
    def __init__(self):
        super().__init__('proportional_controller')
        self.get_logger().info("Proportional controller started")
        
        self.waypoints = np.array([(0, 0), (3, 0), (6, 4), (3, 4), (3, 1), (0, 3)])
        self.current_waypoint_idx = 0
        self.linear_tolerance = 0.03
        self.angular_tolerance = 0.01
        self.angular_gain = 2
        self.linear_gain = 0.7


    def compute_cmd_vel(self):
        twist = Twist()

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("Goal reached!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist

        robot_pos = get_position_from_odom(self.odom_data)
        robot_yaw = get_yaw_from_odom(self.odom_data)

        target_vector = self.waypoints[self.current_waypoint_idx] - robot_pos
        distance_error = np.linalg.norm(target_vector)

        target_angle = np.arctan2(target_vector[1], target_vector[0])
        angle_error = target_angle - robot_yaw
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalize to [-π, π]

        twist.linear.x = min(self.linear_gain * distance_error, self.max_linear_vel)
        twist.angular.z = min(self.angular_gain * angle_error, self.max_angular_vel)

        if distance_error <= self.linear_tolerance:
            self.current_waypoint_idx += 1

        self.get_logger().info(
            f"Driving toward waypoint {self.current_waypoint_idx}. "
            f"Distance: {distance_error:.2f}, Angle Error: {angle_error:.2f}"
        )

        return twist

def main(args=None):
    rclpy.init(args=args)
    
    controller = ProportionalController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()