#!/usr/bin/env python3

import math
import threading
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class AutoMapper(Node):
    SEEK_WALL = 'SEEK_WALL'
    TRACKING_WALL = 'TRACKING_WALL'
    EXPLORE = 'EXPLORE'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('auto_mapper')

        # ---------------- Topics ----------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('mode_topic', '/auto_mapper/mode')
        self.declare_parameter('explore_done_topic', '/auto_mapper/explore_done')

        # ---------------- Robot geometry ----------------
        self.declare_parameter('base_length', 0.500)
        self.declare_parameter('base_width', 0.400)
        self.declare_parameter('lidar_x', 0.200)
        self.declare_parameter('lidar_y', 0.000)

        # ---------------- Motion limits ----------------
        self.declare_parameter('v_max', 0.35)
        self.declare_parameter('a_max', 0.4)
        self.declare_parameter('a_d', 1.42)
        self.declare_parameter('w_max', 0.9)
        self.declare_parameter('w_accel', 2.4)

        # ---------------- Tracking / safety ----------------
        self.declare_parameter('left_target_clearance', 0.40)
        self.declare_parameter('seek_wall_clearance', 0.50)
        self.declare_parameter('front_stop_clearance', 0.22)
        self.declare_parameter('front_turn_clearance', 0.50)
        self.declare_parameter('front_slow_clearance', 0.95)
        self.declare_parameter('wall_lost_clearance', 0.78)
        self.declare_parameter('min_valid_scan', 0.05)

        # ---------------- SEEK_WALL tuning ----------------
        self.declare_parameter('seek_detect_wall_dist', 1.40)
        self.declare_parameter('seek_forward_speed', 0.24)
        self.declare_parameter('seek_turn_rate', 0.28)
        self.declare_parameter('seek_right_turn_time', 1.45)

        # ---------------- TRACKING_WALL tuning ----------------
        self.declare_parameter('k_dist', 2.0)
        self.declare_parameter('k_align', 1.3)
        self.declare_parameter('k_front', 3.2)
        self.declare_parameter('search_left_rate', 0.38)
        self.declare_parameter('turn_in_place_rate', 0.75)

        # ---------------- Recovery ----------------
        self.declare_parameter('stuck_window_sec', 4.0)
        self.declare_parameter('stuck_distance_thresh', 0.05)
        self.declare_parameter('recovery_reverse_sec', 0.30)
        self.declare_parameter('recovery_rotate_sec', 1.10)
        self.declare_parameter('recovery_cooldown_sec', 3.0)
        self.declare_parameter('stuck_min_cmd_v', 0.12)
        self.declare_parameter('stuck_max_cmd_w', 0.35)

        # ---------------- Explore handoff ----------------
        self.declare_parameter('bootstrap_min_time_sec', 20.0)
        self.declare_parameter('bootstrap_min_travel_m', 4.0)
        self.declare_parameter('bootstrap_frontier_handoff', True)

        # ---------------- Debug ----------------
        self.declare_parameter('debug_print_sec', 1.0)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('enable_keyboard_debug', False)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.explore_done_topic = self.get_parameter('explore_done_topic').value

        self.base_length = float(self.get_parameter('base_length').value)
        self.base_width = float(self.get_parameter('base_width').value)
        self.lidar_x = float(self.get_parameter('lidar_x').value)
        self.lidar_y = float(self.get_parameter('lidar_y').value)

        self.v_max = float(self.get_parameter('v_max').value)
        self.a_max = float(self.get_parameter('a_max').value)
        self.a_d = float(self.get_parameter('a_d').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.w_accel = float(self.get_parameter('w_accel').value)

        self.left_target_clearance = float(self.get_parameter('left_target_clearance').value)
        self.seek_wall_clearance = float(self.get_parameter('seek_wall_clearance').value)
        self.front_stop_clearance = float(self.get_parameter('front_stop_clearance').value)
        self.front_turn_clearance = float(self.get_parameter('front_turn_clearance').value)
        self.front_slow_clearance = float(self.get_parameter('front_slow_clearance').value)
        self.wall_lost_clearance = float(self.get_parameter('wall_lost_clearance').value)
        self.min_valid_scan = float(self.get_parameter('min_valid_scan').value)

        self.seek_detect_wall_dist = float(self.get_parameter('seek_detect_wall_dist').value)
        self.seek_forward_speed = float(self.get_parameter('seek_forward_speed').value)
        self.seek_turn_rate = float(self.get_parameter('seek_turn_rate').value)
        self.seek_right_turn_time = float(self.get_parameter('seek_right_turn_time').value)

        self.k_dist = float(self.get_parameter('k_dist').value)
        self.k_align = float(self.get_parameter('k_align').value)
        self.k_front = float(self.get_parameter('k_front').value)
        self.search_left_rate = float(self.get_parameter('search_left_rate').value)
        self.turn_in_place_rate = float(self.get_parameter('turn_in_place_rate').value)

        self.stuck_window_sec = float(self.get_parameter('stuck_window_sec').value)
        self.stuck_distance_thresh = float(self.get_parameter('stuck_distance_thresh').value)
        self.recovery_reverse_sec = float(self.get_parameter('recovery_reverse_sec').value)
        self.recovery_rotate_sec = float(self.get_parameter('recovery_rotate_sec').value)
        self.recovery_cooldown_sec = float(self.get_parameter('recovery_cooldown_sec').value)
        self.stuck_min_cmd_v = float(self.get_parameter('stuck_min_cmd_v').value)
        self.stuck_max_cmd_w = float(self.get_parameter('stuck_max_cmd_w').value)

        self.bootstrap_min_time_sec = float(self.get_parameter('bootstrap_min_time_sec').value)
        self.bootstrap_min_travel_m = float(self.get_parameter('bootstrap_min_travel_m').value)
        self.bootstrap_frontier_handoff = bool(self.get_parameter('bootstrap_frontier_handoff').value)

        self.debug_print_sec = float(self.get_parameter('debug_print_sec').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.enable_keyboard_debug = bool(self.get_parameter('enable_keyboard_debug').value)

        # Distances from lidar to body limits
        self.body_front = max(0.0, self.base_length * 0.5 - self.lidar_x)
        self.body_rear = max(0.0, self.base_length * 0.5 + self.lidar_x)
        self.body_left = max(0.0, self.base_width * 0.5 - self.lidar_y)
        self.body_right = max(0.0, self.base_width * 0.5 + self.lidar_y)

        # ---------------- State ----------------
        self.scan_msg = None
        self.odom_msg = None
        self.mapping_done = False
        self.explore_done = False

        self.mode = self.SEEK_WALL
        self.seek_turn_until = 0.0

        self.pose_history = deque()
        self.recovery_until = 0.0
        self.recovery_reverse_until = 0.0
        self.recovery_dir = 1.0
        self.last_recovery_time = -1e9

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_control_time = self.now_sec()
        self.last_debug_time = 0.0

        self.bootstrap_start_time = self.now_sec()
        self.bootstrap_distance = 0.0
        self.bootstrap_prev_xy = None

        self.shutdown_requested = False
        self.keyboard_thread = None

        # ---------------- ROS I/O ----------------
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)
        self.create_subscription(Bool, self.explore_done_topic, self.explore_done_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)

        self.timer = self.create_timer(self.control_period, self.control_loop)

        if self.enable_keyboard_debug:
            self.keyboard_thread = threading.Thread(target=self.keyboard_debug_loop, daemon=True)
            self.keyboard_thread.start()
            self.get_logger().info('Keyboard debug enabled: tw | ex | done')

        self.publish_mode()
        self.get_logger().info('AutoMapper started: SEEK_WALL -> TRACKING_WALL -> EXPLORE -> DONE')

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def scan_cb(self, msg):
        self.scan_msg = msg

    def odom_cb(self, msg):
        self.odom_msg = msg
        t = self.now_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose_history.append((t, x, y))

        if self.bootstrap_prev_xy is None:
            self.bootstrap_prev_xy = (x, y)
        else:
            dx = x - self.bootstrap_prev_xy[0]
            dy = y - self.bootstrap_prev_xy[1]
            self.bootstrap_distance += math.hypot(dx, dy)
            self.bootstrap_prev_xy = (x, y)

        while self.pose_history and (t - self.pose_history[0][0]) > self.stuck_window_sec:
            self.pose_history.popleft()

    def explore_done_cb(self, msg):
        if bool(msg.data):
            self.explore_done = True
            self.finish_mapping('frontier explorer done')

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def set_mode(self, new_mode):
        if self.mode != new_mode:
            self.mode = new_mode
            self.publish_mode()
            self.get_logger().info(f'Switch mode -> {self.mode}')

    def keyboard_debug_loop(self):
        while (not self.shutdown_requested) and rclpy.ok():
            try:
                cmd = input().strip().lower()
            except EOFError:
                break
            except Exception:
                if self.shutdown_requested:
                    break
                continue

            if cmd == 'tw':
                self.set_mode(self.TRACKING_WALL)
                self.seek_turn_until = 0.0
            elif cmd == 'ex':
                self.handoff_to_explore('debug command')
            elif cmd == 'done':
                self.finish_mapping('debug command')

    def body_distance_along_ray(self, angle_rad):
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        candidates = []

        if c > 1e-9:
            candidates.append(self.body_front / c)
        elif c < -1e-9:
            candidates.append(self.body_rear / (-c))

        if s > 1e-9:
            candidates.append(self.body_left / s)
        elif s < -1e-9:
            candidates.append(self.body_right / (-s))

        positive = [v for v in candidates if v > 0.0]
        return min(positive) if positive else 0.0

    def get_sector_clearances(self, center_deg, width_deg):
        if self.scan_msg is None:
            return []

        center = math.radians(center_deg)
        half = math.radians(width_deg * 0.5)
        vals = []

        for i, r in enumerate(self.scan_msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < self.min_valid_scan:
                continue

            angle = self.scan_msg.angle_min + i * self.scan_msg.angle_increment
            diff = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(diff) > half:
                continue

            body_d = self.body_distance_along_ray(angle)
            clearance = float(r) - body_d
            if clearance > 0.0:
                vals.append(clearance)

        vals.sort()
        return vals

    def sector_percentile(self, center_deg, width_deg, q=0.30, default=float('inf')):
        vals = self.get_sector_clearances(center_deg, width_deg)
        if not vals:
            return default
        idx = int(clamp(q, 0.0, 1.0) * (len(vals) - 1))
        return vals[idx]

    def is_stuck(self, front):
        if self.odom_msg is None or len(self.pose_history) < 2:
            return False

        now = self.now_sec()
        if (now - self.last_recovery_time) < self.recovery_cooldown_sec:
            return False
        if abs(self.cmd_v) < self.stuck_min_cmd_v:
            return False
        if abs(self.cmd_w) > self.stuck_max_cmd_w:
            return False
        if front < self.front_turn_clearance:
            return False

        x0, y0 = self.pose_history[0][1], self.pose_history[0][2]
        x1, y1 = self.pose_history[-1][1], self.pose_history[-1][2]
        dist = math.hypot(x1 - x0, y1 - y0)
        return dist < self.stuck_distance_thresh

    def start_recovery(self, front_clearance):
        now = self.now_sec()
        self.last_recovery_time = now
        self.recovery_dir *= -1.0
        reverse_time = self.recovery_reverse_sec if front_clearance < 0.25 else 0.0
        self.recovery_reverse_until = now + reverse_time
        self.recovery_until = now + reverse_time + self.recovery_rotate_sec
        self.get_logger().warn(f'Recovery triggered, rotate_dir={self.recovery_dir:+.0f}')

    def finish_mapping(self, reason):
        if self.mapping_done:
            return
        self.mapping_done = True
        self.set_mode(self.DONE)
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.publish_cmd(0.0, 0.0)
        self.get_logger().info(f'Auto-mapping stopped: {reason}')
        self.get_logger().info('Save map with: ros2 run nav2_map_server map_saver_cli -f src/amr_navigation/config/map')

    def handoff_to_explore(self, reason):
        if self.mapping_done:
            return
        if not self.bootstrap_frontier_handoff:
            return
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.publish_cmd(0.0, 0.0)
        self.set_mode(self.EXPLORE)
        self.get_logger().info(f'Handoff to EXPLORE: {reason}')

    def should_handoff_to_explore(self):
        elapsed = self.now_sec() - self.bootstrap_start_time
        if elapsed < self.bootstrap_min_time_sec:
            return False
        if self.bootstrap_distance < self.bootstrap_min_travel_m:
            return False
        return True

    def apply_velocity_limits(self, v_target, w_target, dt):
        v_target = clamp(v_target, -self.v_max, self.v_max)
        w_target = clamp(w_target, -self.w_max, self.w_max)

        if v_target >= self.cmd_v:
            dv_max = self.a_max * dt
        else:
            dv_max = self.a_d * dt

        self.cmd_v += clamp(v_target - self.cmd_v, -dv_max, dv_max)

        dw_max = self.w_accel * dt
        self.cmd_w += clamp(w_target - self.cmd_w, -dw_max, dw_max)
        self.cmd_w = clamp(self.cmd_w, -self.w_max, self.w_max)

        return self.cmd_v, self.cmd_w

    def maybe_debug(self, mode_name, front, front_left, front_right, left_mid, right_mid, v_t, w_t):
        now = self.now_sec()
        if (now - self.last_debug_time) < self.debug_print_sec:
            return
        self.last_debug_time = now
        self.get_logger().info(
            f'mode={mode_name} F={front:.2f} FL={front_left:.2f} FR={front_right:.2f} '
            f'L={left_mid:.2f} R={right_mid:.2f} | target v={v_t:.2f} w={w_t:.2f} '
            f'| cmd v={self.cmd_v:.2f} w={self.cmd_w:.2f}'
        )

    def control_seek_wall(self, front, front_left, front_right, left_mid, right_mid):
        now = self.now_sec()

        if now < self.seek_turn_until:
            return 0.0, -self.turn_in_place_rate

        wall_seen = (
            front < self.seek_detect_wall_dist or
            front_left < self.seek_detect_wall_dist or
            front_right < self.seek_detect_wall_dist or
            left_mid < self.seek_detect_wall_dist or
            right_mid < self.seek_detect_wall_dist
        )

        if wall_seen:
            if front <= self.seek_wall_clearance:
                self.seek_turn_until = now + self.seek_right_turn_time
                return 0.0, -self.turn_in_place_rate

            if front_left < front_right - 0.05:
                w_target = +self.seek_turn_rate
            elif front_right < front_left - 0.05:
                w_target = -self.seek_turn_rate
            else:
                w_target = 0.0

            return self.seek_forward_speed, w_target

        return min(self.v_max, 0.30), 0.0

    def control_tracking_wall(self, front, front_left, left_front, left_mid, left_rear):
        if front <= self.front_stop_clearance:
            return 0.0, -self.turn_in_place_rate

        wall_error = left_mid - self.left_target_clearance
        heading_error = left_front - left_rear
        w_target = self.k_dist * wall_error + self.k_align * heading_error

        if front < self.front_turn_clearance:
            w_target -= self.k_front * (self.front_turn_clearance - front)
        if front_left < self.front_turn_clearance:
            w_target -= 0.8 * self.k_front * (self.front_turn_clearance - front_left)

        if left_mid > self.wall_lost_clearance and front > self.front_turn_clearance:
            w_target += self.search_left_rate

        w_target = clamp(w_target, -self.w_max, self.w_max)

        stop_margin = max(0.0, front - self.front_stop_clearance)
        v_brake = math.sqrt(max(0.0, 2.0 * self.a_d * stop_margin))
        v_target = min(self.v_max, v_brake)

        if front < self.front_slow_clearance:
            v_target *= clamp(
                (front - self.front_stop_clearance) /
                max(1e-6, self.front_slow_clearance - self.front_stop_clearance),
                0.0, 1.0
            )

        turn_scale = clamp(1.0 - 0.55 * abs(w_target) / max(1e-6, self.w_max), 0.25, 1.0)
        v_target *= turn_scale

        if left_mid > self.wall_lost_clearance:
            v_target = min(v_target, 0.22)

        if front > 0.8 and abs(w_target) < 0.25:
            v_target = max(v_target, 0.18)

        return v_target, w_target

    def control_loop(self):
        now = self.now_sec()
        dt = max(1e-3, now - self.last_control_time)
        self.last_control_time = now

        self.publish_mode()

        if self.mapping_done or self.mode == self.DONE:
            self.cmd_v = 0.0
            self.cmd_w = 0.0
            self.publish_cmd(0.0, 0.0)
            return

        if self.mode == self.EXPLORE:
            self.cmd_v, self.cmd_w = self.apply_velocity_limits(0.0, 0.0, dt)
            self.publish_cmd(self.cmd_v, self.cmd_w)
            if self.explore_done:
                self.finish_mapping('explore done latched')
            return

        if self.scan_msg is None:
            self.cmd_v, self.cmd_w = self.apply_velocity_limits(0.0, 0.0, dt)
            self.publish_cmd(self.cmd_v, self.cmd_w)
            return

        front = self.sector_percentile(0.0, 24.0, q=0.25, default=5.0)
        front_left = self.sector_percentile(35.0, 22.0, q=0.25, default=5.0)
        front_right = self.sector_percentile(-35.0, 22.0, q=0.25, default=5.0)

        left_front = self.sector_percentile(65.0, 18.0, q=0.30, default=5.0)
        left_mid = self.sector_percentile(90.0, 18.0, q=0.30, default=5.0)
        left_rear = self.sector_percentile(115.0, 18.0, q=0.30, default=5.0)

        right_mid = self.sector_percentile(-90.0, 18.0, q=0.30, default=5.0)

        if now < self.recovery_until:
            if now < self.recovery_reverse_until:
                v_target = -0.08
                w_target = 0.0
            else:
                v_target = 0.0
                w_target = self.recovery_dir * self.turn_in_place_rate

            self.maybe_debug(self.mode, front, front_left, front_right, left_mid, right_mid, v_target, w_target)
            self.cmd_v, self.cmd_w = self.apply_velocity_limits(v_target, w_target, dt)
            self.publish_cmd(self.cmd_v, self.cmd_w)
            return

        if self.is_stuck(front):
            self.start_recovery(front)
            self.cmd_v, self.cmd_w = self.apply_velocity_limits(0.0, self.recovery_dir * self.turn_in_place_rate, dt)
            self.publish_cmd(self.cmd_v, self.cmd_w)
            return

        if self.mode == self.SEEK_WALL:
            v_target, w_target = self.control_seek_wall(front, front_left, front_right, left_mid, right_mid)

            if self.seek_turn_until > 0.0 and now >= self.seek_turn_until:
                self.seek_turn_until = 0.0
                self.set_mode(self.TRACKING_WALL)
                self.get_logger().info('SEEK_WALL finished -> switch to TRACKING_WALL')

        elif self.mode == self.TRACKING_WALL:
            if self.should_handoff_to_explore():
                self.handoff_to_explore('bootstrap coverage reached')
                return
            v_target, w_target = self.control_tracking_wall(front, front_left, left_front, left_mid, left_rear)

        else:
            v_target, w_target = 0.0, 0.0

        self.maybe_debug(self.mode, front, front_left, front_right, left_mid, right_mid, v_target, w_target)
        self.cmd_v, self.cmd_w = self.apply_velocity_limits(v_target, w_target, dt)
        self.publish_cmd(self.cmd_v, self.cmd_w)


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        try:
            if rclpy.ok():
                node.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()