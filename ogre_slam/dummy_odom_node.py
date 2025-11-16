#!/usr/bin/env python3
"""
Dummy Odometry Node for Scan-Matching SLAM
Publishes zero-velocity odometry to satisfy slam_toolbox requirements
when no wheel encoders are available.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DummyOdomNode(Node):
    def __init__(self):
        super().__init__('dummy_odom_node')

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer - publish at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_odom)

        self.get_logger().info('Dummy Odometry Node started')
        self.get_logger().info('Publishing zero-velocity odometry for scan-matching SLAM')

    def publish_odom(self):
        now = self.get_clock().now()

        # Create odometry message (all zeros - no movement from odometry)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position and orientation (identity)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Velocity (zero)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Very high covariance = very uncertain
        odom.pose.covariance[0] = 1000.0   # x
        odom.pose.covariance[7] = 1000.0   # y
        odom.pose.covariance[35] = 1000.0  # yaw

        odom.twist.covariance[0] = 1000.0   # vx
        odom.twist.covariance[7] = 1000.0   # vy
        odom.twist.covariance[35] = 1000.0  # vyaw

        # Publish
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DummyOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
