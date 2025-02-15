import numpy as np
def get_position_from_odom(odom_data):
    """Get the current position of the robot from odometry"""
    return np.array([
        odom_data.pose.pose.position.x,
        odom_data.pose.pose.position.y
    ])

def get_yaw_from_odom(odom_data):
    """Get the robot's heading angle (yaw) from odometry"""
    # Extract yaw from quaternion
    orientation = odom_data.pose.pose.orientation
    _, _, yaw = _euler_from_quaternion(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    return yaw

def _euler_from_quaternion(x, y, z, w):
    """Convert a quaternion to Euler angles (roll, pitch, yaw)"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw
