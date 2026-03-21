#!/usr/bin/env python3
"""
odom_filter.py — Odometry noise filter for rf2o laser odometry

Filters rf2o phantom motion by comparing pose CHANGE between cycles
rather than trusting rf2o's velocity output (which is unreliable at
low speeds). Only updates the published pose when the robot has moved
more than the configured distance or angle threshold.

Subscribes:  /odom_rf2o      (nav_msgs/Odometry) — raw rf2o output
Publishes:   /odom_filtered  (nav_msgs/Odometry) — cleaned odometry
Broadcasts:  odom -> base_footprint TF

Setup:
  - rf2o must have publish_tf: false
  - rover_params.yaml must have pub_odom_tf: false

Tuning:
  - linear_threshold:  meters of pose change to allow update (try 0.03-0.05)
  - angular_threshold: radians of pose change to allow update (try 0.02-0.05)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class OdomFilter(Node):

    def __init__(self):
        super().__init__('odom_filter')

        # -- Parameters --------------------------------------------------------
        # These are POSE CHANGE thresholds, not velocity thresholds.
        # The pose must change by at least this much between cycles to be accepted.
        self.declare_parameter('linear_threshold',  0.03)   # meters
        self.declare_parameter('angular_threshold', 0.03)   # radians
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('base_frame',        'base_footprint')

        self.linear_threshold  = self.get_parameter('linear_threshold').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.odom_frame        = self.get_parameter('odom_frame').value
        self.base_frame        = self.get_parameter('base_frame').value

        # Covariances
        self.POSE_COV_MOVING = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-2,
        ]
        self.POSE_COV_STOPPED = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-9, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9,
        ]
        self.TWIST_COV_MOVING = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-2,
        ]
        self.TWIST_COV_STOPPED = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-9, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9,
        ]

        self.get_logger().info('OdomFilter started')
        self.get_logger().info(f'  Linear threshold:  {self.linear_threshold} m (pose change)')
        self.get_logger().info(f'  Angular threshold: {self.angular_threshold} rad (pose change)')
        self.get_logger().info(f'  TF: {self.odom_frame} -> {self.base_frame}')

        # -- State -------------------------------------------------------------
        # Last ACCEPTED pose
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # Last pose received from rf2o (used to compute delta)
        self.last_x   = None
        self.last_y   = None
        self.last_yaw = None

        # -- TF Broadcaster ----------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # -- Publisher ---------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, '/odom_filtered', 10)

        # -- Subscriber --------------------------------------------------------
        self.create_subscription(
            Odometry,
            '/odom_rf2o',
            self._odom_callback,
            10
        )

    def _odom_callback(self, msg: Odometry):
        new_x   = msg.pose.pose.position.x
        new_y   = msg.pose.pose.position.y
        new_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

        # On first message, initialize without publishing
        if self.last_x is None:
            self.last_x   = new_x
            self.last_y   = new_y
            self.last_yaw = new_yaw
            self.x   = new_x
            self.y   = new_y
            self.yaw = new_yaw
            self._publish(moving=False)
            return

        # Compute pose change since last accepted pose
        dx   = new_x - self.x
        dy   = new_y - self.y
        dyaw = self._angle_diff(new_yaw, self.yaw)

        linear_change  = math.sqrt(dx * dx + dy * dy)
        angular_change = abs(dyaw)

        moving = (linear_change  >= self.linear_threshold or
                  angular_change >= self.angular_threshold)

        if moving:
            self.x   = new_x
            self.y   = new_y
            self.yaw = new_yaw

        self.last_x   = new_x
        self.last_y   = new_y
        self.last_yaw = new_yaw

        self._publish(moving=moving)

    def _publish(self, moving: bool):
        now = self.get_clock().now().to_msg()

        # -- Publish filtered odometry -----------------------------------------
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        odom.twist.twist.linear.x  = 0.0
        odom.twist.twist.angular.z = 0.0

        if moving:
            odom.pose.covariance  = self.POSE_COV_MOVING
            odom.twist.covariance = self.TWIST_COV_MOVING
        else:
            odom.pose.covariance  = self.POSE_COV_STOPPED
            odom.twist.covariance = self.TWIST_COV_STOPPED

        self.odom_pub.publish(odom)

        # -- Broadcast TF: odom -> base_footprint ------------------------------
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)

        self.tf_broadcaster.sendTransform(tf)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """Shortest angle difference between two angles in radians."""
        diff = a - b
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()