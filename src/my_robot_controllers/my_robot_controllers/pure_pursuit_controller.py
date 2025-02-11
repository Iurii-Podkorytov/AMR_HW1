#!/usr/bin/env python3

import rclpy
from .generic_controller import GenericController 

class PurePursuitController(GenericController):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info("Pure pursuit controller started")
        # self.declare_parameter('waypoints', 
                            #    [[3, 0], [6, 4], [3, 4], [3, 1], [0, 3]])
        # self.waypoints = self.get_parameter('waypoints').get_parameter_value().value
        self.waypoints = [[3, 0], [6, 4], [3, 4], [3, 1], [0, 3]]
        self.look_ahead_distance = 1

    def compute_cmd_vel(self):
        pass

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