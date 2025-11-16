#!/usr/bin/env python3
"""
Odometry Node for Mecanum Drive Robot
Publishes odometry from wheel encoders for SLAM and navigation
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Int32MultiArray
import tf2_ros
import math
from .encoder_reader import EncoderReader
from .mecanum_odometry import MecanumOdometry


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convert Euler angles to quaternion

    Args:
        roll, pitch, yaw: Euler angles in radians

    Returns:
        Quaternion message
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


class OdometryNode(Node):
    """
    ROS2 Node for mecanum drive wheel odometry

    Subscribes: None (reads GPIO directly)
    Publishes:
        - /odom (nav_msgs/Odometry)
        - /encoder_ticks (std_msgs/Int32MultiArray) - debugging
        - TF: odom → base_link
    """

    def __init__(self):
        super().__init__('odometry_node')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.05)  # meters
        self.declare_parameter('wheel_base', 0.25)    # meters
        self.declare_parameter('track_width', 0.30)   # meters
        self.declare_parameter('encoder_ppr', 2)      # pulses per revolution
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('publish_tf', False)   # Set to True to publish odom->base_link TF
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Odometry covariance (high uncertainty due to 2 PPR encoders)
        self.declare_parameter('pose_covariance_diagonal',
                              [0.1, 0.1, 0.0, 0.0, 0.0, 0.1])  # x, y, z, roll, pitch, yaw
        self.declare_parameter('twist_covariance_diagonal',
                              [0.05, 0.05, 0.0, 0.0, 0.0, 0.05])  # vx, vy, vz, vroll, vpitch, vyaw

        # Get parameters
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_base = self.get_parameter('wheel_base').value
        track_width = self.get_parameter('track_width').value
        encoder_ppr = self.get_parameter('encoder_ppr').value
        gear_ratio = self.get_parameter('gear_ratio').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        pose_cov_diag = self.get_parameter('pose_covariance_diagonal').value
        twist_cov_diag = self.get_parameter('twist_covariance_diagonal').value

        # Initialize encoder reader
        self.get_logger().info('Initializing encoder reader...')
        try:
            self.encoder_reader = EncoderReader()
            self.get_logger().info('✅ Encoder reader initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize encoders: {e}')
            raise

        # Initialize odometry calculator
        self.odometry = MecanumOdometry(
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
            track_width=track_width,
            encoder_ppr=encoder_ppr,
            gear_ratio=gear_ratio
        )

        self.get_logger().info(f'Robot parameters:')
        self.get_logger().info(f'  Wheel radius: {wheel_radius}m')
        self.get_logger().info(f'  Wheel base: {wheel_base}m')
        self.get_logger().info(f'  Track width: {track_width}m')
        self.get_logger().info(f'  Encoder PPR: {encoder_ppr}')
        self.get_logger().info(f'  Gear ratio: {gear_ratio}')

        # Create covariance matrices
        self.pose_covariance = [0.0] * 36
        self.twist_covariance = [0.0] * 36
        for i, val in enumerate(pose_cov_diag):
            self.pose_covariance[i * 7] = val  # Diagonal elements
        for i, val in enumerate(twist_cov_diag):
            self.twist_covariance[i * 7] = val

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.ticks_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for odometry updates
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.update_odometry)
        self.last_time = self.get_clock().now()

        self.get_logger().info(f'✅ Odometry node started (publish rate: {publish_rate} Hz)')

    def update_odometry(self):
        """
        Timer callback to update and publish odometry
        """
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        # Read encoder ticks and reset
        ticks = self.encoder_reader.get_ticks_and_reset()

        # Publish raw ticks for debugging
        ticks_msg = Int32MultiArray()
        ticks_msg.data = ticks
        self.ticks_pub.publish(ticks_msg)

        # Compute odometry
        x, y, theta, vx, vy, vtheta = self.odometry.compute_odometry(ticks, dt)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set pose
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, theta)
        odom_msg.pose.covariance = self.pose_covariance

        # Set twist
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = vtheta
        odom_msg.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom_msg)

        # Publish TF transform (only if enabled - disabled for pure scan-matching SLAM)
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = euler_to_quaternion(0.0, 0.0, theta)

            self.tf_broadcaster.sendTransform(tf_msg)

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info('Shutting down odometry node...')
        if hasattr(self, 'encoder_reader'):
            self.encoder_reader.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = OdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
