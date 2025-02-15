#!/usr/bin/env python3

import rclpy
from .generic_controller import GenericController
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import numpy as np
from .utils import *

class StanleyController(GenericController):
    def __init__(self):
        super().__init__('stanley_controller')
        self.get_logger().info("Stanley controller started")
        
        self.waypoints = np.array([(0, 0), (3, 0), (6, 4), (3, 4), (3, 1), (0, 3)])
        self.current_waypoint_idx = 1
        self.cross_track_gain = 1
        self.softening_constant = 0.1
        self.tolerance = 5e-2
        self.max_linear_vel = 0.5

    def cross_track_error(self, robot_pos):
        segment = self.waypoints[self.current_waypoint_idx-1:self.current_waypoint_idx+1]
        if len(segment) < 2:
            return 0.0 

        s = segment[1] - segment[0] 
        v = robot_pos - segment[0]
        perp = np.array([-s[1], s[0]])  # Rotate s by 90 degrees
        cross_track_error = (v @ perp) / np.linalg.norm(perp)
        return -cross_track_error

    def heading_error(self, robot_yaw):
        segment = self.waypoints[self.current_waypoint_idx-1:self.current_waypoint_idx+1]
        s = segment[1] - segment[0]
        target_angle = np.arctan2(s[1], s[0])
        angle_error = target_angle - robot_yaw
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))  # Normalize to [-π, π]
        return angle_error
    
    def compute_cmd_vel(self):
        twist = Twist()

        robot_pos = get_position_from_odom(self.odom_data)
        robot_yaw = get_yaw_from_odom(self.odom_data)

        distance_error = np.linalg.norm(robot_pos - self.waypoints[self.current_waypoint_idx])
        if distance_error <= self.tolerance:
            self.current_waypoint_idx += 1

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("Goal reached!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist

        cross_track_error = self.cross_track_error(robot_pos)
        heading_error = self.heading_error(robot_yaw)

        angular_vel = heading_error + np.arctan2(self.cross_track_gain * cross_track_error, self.max_linear_vel)
        twist.angular.z = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        twist.linear.x = max(min(self.max_linear_vel*distance_error, self.max_linear_vel), 0.2) 

        self.get_logger().info(
            f"Cross-Track Error: {cross_track_error:.2f}, "
            f"Heading Error: {heading_error:.2f}, "
            f"Angular Vel: {angular_vel:.2f}, "
            f"Waypoint: {self.waypoints[self.current_waypoint_idx]}"
            f"Distance: {np.linalg.norm(robot_pos - self.waypoints[self.current_waypoint_idx]):.2f}"
        )

        return twist

def main(args=None):
    rclpy.init(args=args)
    
    controller = StanleyController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()