#!/usr/bin/env python3

from .generic_controller import GenericController
import rclpy
from geometry_msgs.msg import Twist
import termios
import tty
import sys

class KeyboardController(GenericController):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.get_logger().info("Robot teleop node started. Use WASD keys to move. Press 'q' to stop.")

    def get_key(self):
        """Reads a single key press."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def compute_cmd_vel(self):
        twist = Twist()
        speed = self.max_linear_vel
        turn = self.max_angular_vel

        match self.get_key():
            case 'w':
                twist.linear.x = speed
                twist.angular.z = 0.0
            case 's':
                twist.linear.x = -speed
                twist.angular.z = 0.0
            case 'a':
                twist.linear.x = 0.0
                twist.angular.z = turn
            case 'd':
                twist.linear.x = 0.0
                twist.angular.z = -turn
            case 'q':
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            case '':  # CTRL+C
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(f"Stoppint the robot and exiting...")
                return twist, True 

        self.get_logger().info(f"Velocity: linear={twist.linear.x}, angular={twist.angular.z}")
        return twist, False

    def control_loop(self):
        twist, exit_program = self.compute_cmd_vel()
        self.cmd_vel_publisher.publish(twist)
        if exit_program:
            raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    
    controller = KeyboardController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()