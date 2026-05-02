#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class WheelRpmOdometry(Node):
    def __init__(self):
        super().__init__('wheel_rpm_odometry')

        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('wheel_base', 0.465)
        self.declare_parameter('rpm_topic', '/wheel_rpm_feedback')
        self.declare_parameter('odom_topic', '/odom/raw')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.rpm_topic = str(self.get_parameter('rpm_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        rpm_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        self.sub = self.create_subscription(
            Vector3,
            self.rpm_topic,
            self.rpm_callback,
            rpm_qos
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        self.get_logger().info(
            f'wheel_rpm_odometry started: '
            f'wheel_radius={self.wheel_radius}, '
            f'wheel_base={self.wheel_base}, '
            f'odom_topic={self.odom_topic}, '
            f'publish_tf={self.publish_tf}'
        )

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        half = yaw * 0.5
        z = math.sin(half)
        w = math.cos(half)
        return z, w

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def rpm_callback(self, msg: Vector3):
        # Invert both wheels so forward robot motion becomes +x odometry
        left_rpm = float(msg.x)
        right_rpm = float(msg.y)

        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            self.get_logger().warn(f'Skipping odom update due to abnormal dt={dt:.3f}s')
            return

        left_w = left_rpm * 2.0 * math.pi / 60.0
        right_w = right_rpm * 2.0 * math.pi / 60.0

        v_left = left_w * self.wheel_radius
        v_right = right_w * self.wheel_radius

        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / self.wheel_base

        d_left = v_left * dt
        d_right = v_right * dt

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_base

        self.x += d_center * math.cos(self.theta + 0.5 * d_theta)
        self.y += d_center * math.sin(self.theta + 0.5 * d_theta)
        self.theta = self.normalize_angle(self.theta + d_theta)

        stamp = now.to_msg()
        qz, qw = self.yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        odom.pose.covariance[0] = 1e-3
        odom.pose.covariance[7] = 1e-3
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e-2

        odom.twist.covariance[0] = 1e-2
        odom.twist.covariance[7] = 1e-2
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 1e-1

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = WheelRpmOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()