#!/usr/bin/env python3

import rclpy
from .generic_controller import GenericController
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import numpy as np
from .utils import *

class PurePursuitController(GenericController):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info("Pure pursuit controller started")
        
        self.waypoints = np.array([(0, 0), (3, 0), (6, 4), (3, 4), (3, 1), (0, 3)])
        self.look_ahead_distance = 1
        self.angular_gain = 2.0

    
    def segment_circle_intersect(self, center, p1, p2):
        """
        Find the furthest intersection between line segment and a circle
        with radius look_ahead_distance centered at robot origin
        """
        
        A = (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

        if A == 0:  # Line segment is a point
            return None

        B = (p1[0]-p2[0]) * (p2[0]-center[0]) + (p1[1]-p2[1]) * (p2[1]-center[1])
        C = (p2[0]-center[0])**2 + (p2[1]-center[1])**2 - self.look_ahead_distance**2
        
        discriminant = B**2 - A * C
        if discriminant < 0:  # No intersection
            return None

        t1 = (-B + np.sqrt(discriminant)) / A
        t2 = (-B - np.sqrt(discriminant)) / A

        M = np.vstack((p1, p2))
        if 0<=t2<=1: return np.array([t2, (1-t2)]) @ M
        if 0<=t1<=1: return np.array([t1, (1-t1)]) @ M
        return None

    def get_target(self, robot_pos, robot_yaw):
        for i in range(len(self.waypoints)-1):
            prev = self.waypoints[i]
            next = self.waypoints[i+1]
            
            target = self.segment_circle_intersect(robot_pos, prev, next)
            if target is None:
                continue 

            robot_heading = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
            if (target-robot_pos) @ robot_heading.T > 0: # if target is ahead
                self.get_logger().info(f"robot: {robot_pos}, prev: {prev}, next: {next}, t: {target}")
                return target
        
        # If no target is found, return the last waypoint
        return self.waypoints[-1]

    def compute_cmd_vel(self):
        twist = Twist()
        robot_pos = get_position_from_odom(self.odom_data)
        robot_yaw = get_yaw_from_odom(self.odom_data)
        target = self.get_target(robot_pos, robot_yaw)

        target_vector = target - robot_pos
        target_angle = np.arctan2(target_vector[1], target_vector[0])

        angle_error = target_angle - robot_yaw
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error)) # normalize to [-pi, pi]
        

        if np.allclose(self.waypoints[-1], target):
            dist = np.linalg.norm(self.waypoints[-1]-robot_pos)
            if dist  <= 1e-2:
                twist.linear.x = 0.0     
                twist.angular.z = 0.0
                self.get_logger().info("Goal reached!")
            else:
                twist.linear.x = min(self.max_linear_vel * dist, self.max_linear_vel)
                twist.angular.z = self.angular_gain * angle_error
            self.get_logger().info(f"error={dist}")

        else:
            twist.linear.x = self.max_linear_vel
            twist.angular.z = self.angular_gain * angle_error

        return twist

def main(args=None):
    rclpy.init(args=args)
    
    controller = PurePursuitController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()