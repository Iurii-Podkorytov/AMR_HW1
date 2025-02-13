from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import csv
import os

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        
        # Publisher for nav_msgs/Path
        self.publisher_ = self.create_publisher(Path, 'global_path', 10)
        
        # Timer to publish the path at regular intervals
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_path)

        # Load path data from file
        self.path_data = self.load_path_from_file('paths/path.csv')

    def load_path_from_file(self, filename):
        """
        Load path data from a CSV file.
        Each row should contain x, y coordinates.
        """
        path_data = []
        package_share_dir = get_package_share_directory('my_robot_bringup')
        file_path = os.path.join(package_share_dir, filename)
        
        try:
            with open(file_path, mode='r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    path_data.append({
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'z': 0.0
                    })
            self.get_logger().info(f'Loaded {len(path_data)} waypoints from {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to load path data: {e}')
        
        return path_data

    def publish_path(self):
        """
        Publish the loaded path as a nav_msgs/Path message.
        """
        if not self.path_data:
            self.get_logger().warn('No path data to publish.')
            return
        
        # Create a Path message
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Populate the poses in the path
        for waypoint in self.path_data:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint['x']
            pose.pose.position.y = waypoint['y']
            pose.pose.position.z = waypoint['z']
            pose.pose.orientation.w = 1.0  # Default orientation (no rotation)
            path_msg.poses.append(pose)

        # Publish the path
        self.publisher_.publish(path_msg)
        self.get_logger().info('Published global path.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()