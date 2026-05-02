#!/usr/bin/env python3

import sys
import select
import termios
import tty
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


MSG = """
==================== MANUAL MAPPER ====================

Move:
        w
   a    s    d

w : forward
s : backward
a : rotate left
d : rotate right

q : forward + left
e : forward + right
z : backward + left
c : backward + right

x or SPACE : stop immediately

Speed:
r : increase linear speed
f : decrease linear speed
t : increase angular speed
g : decrease angular speed

Other:
p : print current robot pose
CTRL-C : quit

=======================================================
"""


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ManualMapper(Node):
    def __init__(self):
        super().__init__('manual_mapper')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 20.0)

        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.80)

        self.declare_parameter('linear_speed_min', 0.05)
        self.declare_parameter('linear_speed_max', 1.00)
        self.declare_parameter('angular_speed_min', 0.10)
        self.declare_parameter('angular_speed_max', 2.50)

        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.10)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.linear_speed_min = float(self.get_parameter('linear_speed_min').value)
        self.linear_speed_max = float(self.get_parameter('linear_speed_max').value)
        self.angular_speed_min = float(self.get_parameter('angular_speed_min').value)
        self.angular_speed_max = float(self.get_parameter('angular_speed_max').value)

        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        self.target_linear = 0.0
        self.target_angular = 0.0

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_cmd)

        self.get_logger().info('manual_mapper started')
        self.get_logger().info(
            f'cmd_vel_topic={self.cmd_vel_topic}, odom_topic={self.odom_topic}'
        )
        self.print_speed()

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.publish_cmd()

    def print_speed(self):
        self.get_logger().info(
            f'linear_speed={self.linear_speed:.2f} m/s | '
            f'angular_speed={self.angular_speed:.2f} rad/s'
        )

    def print_pose(self):
        if not self.have_odom:
            self.get_logger().warn('No odometry received yet.')
            return

        self.get_logger().info(
            f'robot pose | x={self.current_x:.3f} m | '
            f'y={self.current_y:.3f} m | '
            f'yaw={math.degrees(self.current_yaw):.1f} deg'
        )

    def handle_key(self, key: str):
        if key == 'w':
            self.target_linear = self.linear_speed
            self.target_angular = 0.0
        elif key == 's':
            self.target_linear = -self.linear_speed
            self.target_angular = 0.0
        elif key == 'a':
            self.target_linear = 0.0
            self.target_angular = self.angular_speed
        elif key == 'd':
            self.target_linear = 0.0
            self.target_angular = -self.angular_speed
        elif key == 'q':
            self.target_linear = self.linear_speed
            self.target_angular = self.angular_speed
        elif key == 'e':
            self.target_linear = self.linear_speed
            self.target_angular = -self.angular_speed
        elif key == 'z':
            self.target_linear = -self.linear_speed
            self.target_angular = self.angular_speed
        elif key == 'c':
            self.target_linear = -self.linear_speed
            self.target_angular = -self.angular_speed
        elif key in ['x', ' ']:
            self.stop_robot()
        elif key == 'r':
            self.linear_speed = clamp(
                self.linear_speed + self.linear_step,
                self.linear_speed_min,
                self.linear_speed_max
            )
            self.print_speed()
        elif key == 'f':
            self.linear_speed = clamp(
                self.linear_speed - self.linear_step,
                self.linear_speed_min,
                self.linear_speed_max
            )
            self.print_speed()
        elif key == 't':
            self.angular_speed = clamp(
                self.angular_speed + self.angular_step,
                self.angular_speed_min,
                self.angular_speed_max
            )
            self.print_speed()
        elif key == 'g':
            self.angular_speed = clamp(
                self.angular_speed - self.angular_step,
                self.angular_speed_min,
                self.angular_speed_max
            )
            self.print_speed()
        elif key == 'p':
            self.print_pose()
        else:
            self.stop_robot()


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = ManualMapper()

    print(MSG)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(settings)

            if key == '\x03':
                break

            if key:
                node.handle_key(key)

    except Exception as exc:
        node.get_logger().error(f'Exception: {exc}')

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()