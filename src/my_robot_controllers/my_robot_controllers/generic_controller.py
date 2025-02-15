from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class GenericController(Node, ABC):
    def __init__(self, node_name: str):
        """
        Base class for controller nodes
        """
        super().__init__(node_name)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10 
        )
        self.odom_data = None
        self.subscriptions  # Prevent unused variable warning
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10 
        )

        self.max_linear_vel = 1.0
        self.max_angular_vel = 1.0


    def odom_callback(self, msg: Odometry):
        """
        Callback function for the /odom subscription.
        """
        self.odom_data = msg
        self.control_loop()

    @abstractmethod
    def compute_cmd_vel(self) -> Twist:
        """
        Abstract method to compute the command velocity based on the odometry data.
        """
        pass

    def control_loop(self):
        """
        Execute the control loop. This method is called whenever new odometry data is received.
        """
        if self.odom_data is not None:
            cmd_vel = self.compute_cmd_vel()
            self.cmd_vel_publisher.publish(cmd_vel)