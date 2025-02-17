import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
import numpy as np
import matplotlib.pyplot as plt
from .utils import *

class OdomPlotterNode(Node):
    def __init__(self):
        super().__init__('odom_plotter_node')
        self.get_logger().info('Odom Plotter Node has been started.')
        self.declare_parameter("plot_title", 'Model Predictive Controller')
        self.plot_title = self.get_parameter('plot_title').value
        self.get_logger().info(self.plot_title)

        # Initialize subscriber
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Data storage
        self.x_data = []
        self.y_data = []
        self.time_data = []
        self.yaw_data = []
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # Start time in seconds

        # Waypoints
        self.waypoints = np.array([(0, 0), (3, 0), (6, 4), (3, 4), (3, 1), (0, 3)])
        self.last_waypoint = self.waypoints[-1]
        self.reached_last_waypoint = False
        self.distance_threshold = 0.1

        # Setup plots
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 8))
        self.fig.suptitle(self.plot_title, fontsize=16)  # Add main title to the figure
        self.axs = self.axs.flatten()

        # Titles for subplots
        self.axs[0].set_title("X vs Y (Trajectory)")
        self.axs[1].set_title("X vs Time")
        self.axs[2].set_title("Y vs Time")
        self.axs[3].set_title("Heading (Yaw) vs Time")

    def odom_callback(self, msg):
        """Callback function for odometry messages"""
        if self.reached_last_waypoint:
            return  # Stop processing if the last waypoint is reached

        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        x, y = get_position_from_odom(msg)
        yaw = get_yaw_from_odom(msg)

        # Append data
        self.x_data.append(x)
        self.y_data.append(y)
        self.time_data.append(current_time)
        self.yaw_data.append(np.degrees(yaw))  # Convert yaw to degrees for better visualization

        # Log received data
        self.get_logger().info(f"Received odom: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        # Check if the robot is close to the last waypoint
        distance_to_last_waypoint = np.linalg.norm([x - self.last_waypoint[0], y - self.last_waypoint[1]])
        if distance_to_last_waypoint < self.distance_threshold:
            self.get_logger().info("Reached the last waypoint! Saving plot...")
            self.reached_last_waypoint = True
            self.save_plot()

    def save_plot(self):
        """Save the final plots to files"""
        # Plot XY trajectory
        self.axs[0].plot(self.x_data, self.y_data, 'b-', label="Robot Trajectory")
        self.axs[0].plot(self.waypoints[:, 0], self.waypoints[:, 1], 'k--o', label="Reference Trajectory")
        self.axs[0].legend()
        self.axs[0].set_xlabel("X (m)")
        self.axs[0].set_ylabel("Y (m)")

        # Plot X vs Time
        self.axs[1].plot(self.time_data, self.x_data, 'r-', label="X")
        self.axs[1].legend()
        self.axs[1].set_xlabel("Time (s)")
        self.axs[1].set_ylabel("X (m)")

        # Plot Y vs Time
        self.axs[2].plot(self.time_data, self.y_data, 'g-', label="Y")
        self.axs[2].legend()
        self.axs[2].set_xlabel("Time (s)")
        self.axs[2].set_ylabel("Y (m)")

        # Plot Yaw vs Time
        self.axs[3].plot(self.time_data, self.yaw_data, 'm-', label="Yaw")
        self.axs[3].legend()
        self.axs[3].set_xlabel("Time (s)")
        self.axs[3].set_ylabel("Yaw (degrees)")

        # Adjust layout and save the figure
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(f"{self.plot_title}.png")
        self.get_logger().info(f"Plot saved as '{self.plot_title}.png'")

        # Close the figure
        plt.close(self.fig)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.reached_last_waypoint:
            node.get_logger().info("Interrupted before reaching the last waypoint. Saving current plot...")
            node.save_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()