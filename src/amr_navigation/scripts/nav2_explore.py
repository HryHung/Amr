#!/usr/bin/env python3

import math
import random
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import TransformException


class FrontierNav2Explorer(Node):
    IDLE = 'IDLE'
    EXPLORING = 'EXPLORING'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('nav2_frontier_explorer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('mode_topic', '/auto_mapper/mode')
        self.declare_parameter('explore_done_topic', '/auto_mapper/explore_done')
        self.declare_parameter('marker_topic', '/frontier_target_marker')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('frontier_occ_threshold', 65)
        self.declare_parameter('frontier_safe_inflation_cells', 10)
        self.declare_parameter('frontier_edge_margin_cells', 12)
        self.declare_parameter('frontier_min_cluster_size', 10)
        self.declare_parameter('frontier_cluster_conn', 8)
        self.declare_parameter('unknown_neighbor_radius', 1)
        self.declare_parameter('unknown_neighbor_min_count', 1)
        self.declare_parameter('goal_free_radius_cells', 5)
        self.declare_parameter('goal_obstacle_check_radius_cells', 1)
        self.declare_parameter('goal_max_allowed_obstacle_neighbors', 0)
        self.declare_parameter('frontier_approach_search_radius_cells', 8)
        self.declare_parameter('frontier_min_goal_dist', 1.5)
        self.declare_parameter('frontier_max_goal_dist', 15.0)
        self.declare_parameter('frontier_heading_weight', 0.20)
        self.declare_parameter('frontier_dist_weight', 1.00)
        self.declare_parameter('frontier_size_weight', 0.25)
        self.declare_parameter('frontier_unknown_weight', 0.12)
        self.declare_parameter('plan_validation_candidates', 25)
        self.declare_parameter('fallback_enable', True)
        self.declare_parameter('fallback_unknown_search_radius', 6)
        self.declare_parameter('fallback_min_goal_dist', 2.0)
        self.declare_parameter('fallback_max_goal_dist', 10.0)
        self.declare_parameter('fallback_unknown_weight', 2.0)
        self.declare_parameter('fallback_dist_weight', 0.20)
        self.declare_parameter('fallback_random_sample_count', 200)
        self.declare_parameter('goal_blacklist_radius', 2.5)
        self.declare_parameter('goal_timeout_sec', 150.0)
        self.declare_parameter('reached_blacklist_hold_sec', 120.0)
        self.declare_parameter('failed_blacklist_hold_sec', 180.0)
        self.declare_parameter('startup_grace_sec', 12.0)
        self.declare_parameter('progress_window_sec', 15.0)
        self.declare_parameter('progress_min_dist', 0.20)
        self.declare_parameter('feedback_progress_epsilon', 0.10)
        self.declare_parameter('finish_unknown_ratio', 0.08)
        self.declare_parameter('finish_min_known_cells', 3000)
        self.declare_parameter('control_period', 0.25)
        self.declare_parameter('replan_period_sec', 1.0)
        self.declare_parameter('debug_print_sec', 0.75)

        self.map_topic = self.get_parameter('map_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.explore_done_topic = self.get_parameter('explore_done_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.frontier_occ_threshold = int(self.get_parameter('frontier_occ_threshold').value)
        self.frontier_safe_inflation_cells = int(self.get_parameter('frontier_safe_inflation_cells').value)
        self.frontier_edge_margin_cells = int(self.get_parameter('frontier_edge_margin_cells').value)
        self.frontier_min_cluster_size = int(self.get_parameter('frontier_min_cluster_size').value)
        self.frontier_cluster_conn = int(self.get_parameter('frontier_cluster_conn').value)
        self.unknown_neighbor_radius = int(self.get_parameter('unknown_neighbor_radius').value)
        self.unknown_neighbor_min_count = int(self.get_parameter('unknown_neighbor_min_count').value)
        self.goal_free_radius_cells = int(self.get_parameter('goal_free_radius_cells').value)
        self.goal_obstacle_check_radius_cells = int(self.get_parameter('goal_obstacle_check_radius_cells').value)
        self.goal_max_allowed_obstacle_neighbors = int(self.get_parameter('goal_max_allowed_obstacle_neighbors').value)
        self.frontier_approach_search_radius_cells = int(self.get_parameter('frontier_approach_search_radius_cells').value)
        self.frontier_min_goal_dist = float(self.get_parameter('frontier_min_goal_dist').value)
        self.frontier_max_goal_dist = float(self.get_parameter('frontier_max_goal_dist').value)
        self.frontier_heading_weight = float(self.get_parameter('frontier_heading_weight').value)
        self.frontier_dist_weight = float(self.get_parameter('frontier_dist_weight').value)
        self.frontier_size_weight = float(self.get_parameter('frontier_size_weight').value)
        self.frontier_unknown_weight = float(self.get_parameter('frontier_unknown_weight').value)
        self.plan_validation_candidates = int(self.get_parameter('plan_validation_candidates').value)
        self.fallback_enable = bool(self.get_parameter('fallback_enable').value)
        self.fallback_unknown_search_radius = int(self.get_parameter('fallback_unknown_search_radius').value)
        self.fallback_min_goal_dist = float(self.get_parameter('fallback_min_goal_dist').value)
        self.fallback_max_goal_dist = float(self.get_parameter('fallback_max_goal_dist').value)
        self.fallback_unknown_weight = float(self.get_parameter('fallback_unknown_weight').value)
        self.fallback_dist_weight = float(self.get_parameter('fallback_dist_weight').value)
        self.fallback_random_sample_count = int(self.get_parameter('fallback_random_sample_count').value)
        self.goal_blacklist_radius = float(self.get_parameter('goal_blacklist_radius').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.reached_blacklist_hold_sec = float(self.get_parameter('reached_blacklist_hold_sec').value)
        self.failed_blacklist_hold_sec = float(self.get_parameter('failed_blacklist_hold_sec').value)
        self.startup_grace_sec = float(self.get_parameter('startup_grace_sec').value)
        self.progress_window_sec = float(self.get_parameter('progress_window_sec').value)
        self.progress_min_dist = float(self.get_parameter('progress_min_dist').value)
        self.feedback_progress_epsilon = float(self.get_parameter('feedback_progress_epsilon').value)
        self.finish_unknown_ratio = float(self.get_parameter('finish_unknown_ratio').value)
        self.finish_min_known_cells = int(self.get_parameter('finish_min_known_cells').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.replan_period_sec = float(self.get_parameter('replan_period_sec').value)
        self.debug_print_sec = float(self.get_parameter('debug_print_sec').value)

        self.map_msg = None
        self.auto_mapper_mode = 'SEEK_WALL'
        self.state = self.IDLE
        self.current_goal_xy = None
        self.current_goal_kind = 'NONE'
        self.current_goal_stamp = 0.0
        self.current_goal_sent = False
        self.current_goal_feedback_start = None
        self.current_goal_feedback_best = None
        self.last_plan_time = 0.0
        self.last_debug_time = 0.0
        self.done_sent = False
        self.blacklist = []
        self.goal_progress_history = deque()
        self.nav2_ready = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 10)
        self.create_subscription(String, self.mode_topic, self.mode_cb, 10)

        self.done_pub = self.create_publisher(Bool, self.explore_done_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info('Fixed Nav2 frontier explorer started')

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def map_cb(self, msg):
        self.map_msg = msg

    def mode_cb(self, msg):
        prev = self.auto_mapper_mode
        self.auto_mapper_mode = msg.data
        if self.auto_mapper_mode != 'EXPLORE':
            if prev == 'EXPLORE' and self.current_goal_sent and self.current_goal_xy is not None:
                self.cancel_active_goal()
            self.clear_current_goal()
            self.publish_goal_marker()
            self.state = self.IDLE
            self.done_sent = False
            self.goal_progress_history.clear()

    def maybe_debug(self, text, force=False):
        now = self.now_sec()
        if force or (now - self.last_debug_time) >= self.debug_print_sec:
            self.last_debug_time = now
            self.get_logger().info(text)

    def ensure_nav2_ready(self):
        if self.nav2_ready:
            return True
        try:
            self.get_logger().info('Waiting for Nav2 to become active...')
            self.navigator.waitUntilNav2Active(navigator='bt_navigator')
            self.nav2_ready = True
            self.get_logger().info('Nav2 is active')
            return True
        except Exception as e:
            self.get_logger().warn(f'Nav2 not ready yet: {e}')
            return False

    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)

    def cancel_active_goal(self):
        if not self.current_goal_sent:
            return
        try:
            self.navigator.cancelTask()
        except Exception:
            pass

    def clear_current_goal(self):
        self.current_goal_xy = None
        self.current_goal_kind = 'NONE'
        self.current_goal_stamp = 0.0
        self.current_goal_sent = False
        self.current_goal_feedback_start = None
        self.current_goal_feedback_best = None
        self.goal_progress_history.clear()

    def lookup_map_to_base(self):
        try:
            return self.tf_buffer.lookup_transform(self.global_frame, self.robot_base_frame, rclpy.time.Time())
        except TransformException:
            return None

    def get_robot_xy(self):
        tfm = self.lookup_map_to_base()
        if tfm is None:
            return None
        return (tfm.transform.translation.x, tfm.transform.translation.y)

    def get_yaw(self):
        tfm = self.lookup_map_to_base()
        if tfm is None:
            return 0.0
        q = tfm.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_robot_pose_stamped(self):
        tfm = self.lookup_map_to_base()
        if tfm is None:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tfm.transform.translation.x
        pose.pose.position.y = tfm.transform.translation.y
        pose.pose.position.z = tfm.transform.translation.z
        pose.pose.orientation = tfm.transform.rotation
        return pose

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def make_goal_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        yaw = 0.0
        robot_xy = self.get_robot_xy()
        if robot_xy is not None:
            yaw = math.atan2(y - robot_xy[1], x - robot_xy[0])
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def map_xy_to_ij(self, x, y):
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        i = int((x - info.origin.position.x) / info.resolution)
        j = int((y - info.origin.position.y) / info.resolution)
        if i < 0 or j < 0 or i >= info.width or j >= info.height:
            return None
        return i, j

    def map_ij_to_xy(self, i, j):
        info = self.map_msg.info
        x = info.origin.position.x + (i + 0.5) * info.resolution
        y = info.origin.position.y + (j + 0.5) * info.resolution
        return x, y

    def map_index(self, i, j):
        return j * self.map_msg.info.width + i

    def cell_value(self, i, j, default=100):
        if self.map_msg is None:
            return default
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        if i < 0 or j < 0 or i >= w or j >= h:
            return default
        return self.map_msg.data[self.map_index(i, j)]

    def is_unknown_cell(self, i, j):
        return self.cell_value(i, j, default=-1) == -1

    def is_occupied_cell(self, i, j):
        v = self.cell_value(i, j, default=100)
        return v >= self.frontier_occ_threshold

    def is_near_map_edge(self, i, j, margin):
        if self.map_msg is None:
            return True
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        return i < margin or j < margin or i >= (w - margin) or j >= (h - margin)

    def unknown_count_around(self, i, j, radius):
        count = 0
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if di == 0 and dj == 0:
                    continue
                if self.is_unknown_cell(i + di, j + dj):
                    count += 1
        return count

    def occupied_count_around(self, i, j, radius):
        count = 0
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                if di == 0 and dj == 0:
                    continue
                if self.is_occupied_cell(i + di, j + dj):
                    count += 1
        return count

    def has_unknown_neighbor(self, i, j):
        return self.unknown_count_around(i, j, self.unknown_neighbor_radius) >= self.unknown_neighbor_min_count

    def is_goal_region_clear(self, i, j, radius):
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                ii = i + di
                jj = j + dj
                v = self.cell_value(ii, jj, default=100)
                if v == -1 or v >= self.frontier_occ_threshold:
                    return False
        return True

    def get_unknown_ratio_in_bbox(self):
        if self.map_msg is None:
            return None, 0
        w = self.map_msg.info.width
        data = self.map_msg.data
        known_idx = [k for k, v in enumerate(data) if v != -1]
        if not known_idx:
            return None, 0
        xs = [k % w for k in known_idx]
        ys = [k // w for k in known_idx]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        total = 0
        unknown = 0
        known = 0
        for j in range(min_y, max_y + 1):
            base = j * w
            for i in range(min_x, max_x + 1):
                v = data[base + i]
                total += 1
                if v == -1:
                    unknown += 1
                else:
                    known += 1
        if total == 0:
            return None, known
        return unknown / float(total), known

    def build_blocked_grid(self):
        if self.map_msg is None:
            return None
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        blocked = [[False for _ in range(w)] for _ in range(h)]
        for j in range(h):
            for i in range(w):
                v = self.cell_value(i, j, default=100)
                if v == -1 or v >= self.frontier_occ_threshold:
                    blocked[j][i] = True
        n = max(0, self.frontier_safe_inflation_cells)
        if n <= 0:
            return blocked
        inflated = [row[:] for row in blocked]
        for j in range(h):
            for i in range(w):
                if not blocked[j][i]:
                    continue
                for dj in range(-n, n + 1):
                    for di in range(-n, n + 1):
                        ii = i + di
                        jj = j + dj
                        if 0 <= ii < w and 0 <= jj < h:
                            inflated[jj][ii] = True
        return inflated

    def find_nearest_unblocked_start(self, blocked, start_i, start_j, max_r=12):
        h = len(blocked)
        w = len(blocked[0])
        if 0 <= start_i < w and 0 <= start_j < h and not blocked[start_j][start_i]:
            return (start_i, start_j)
        for r in range(1, max_r + 1):
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    ii = start_i + di
                    jj = start_j + dj
                    if ii < 0 or jj < 0 or ii >= w or jj >= h:
                        continue
                    if not blocked[jj][ii]:
                        return (ii, jj)
        return None

    def reachable_free_cells(self, blocked, start_i, start_j):
        h = len(blocked)
        w = len(blocked[0])
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        dist = {(start_i, start_j): 0}
        q = deque([(start_i, start_j)])
        while q:
            i, j = q.popleft()
            d = dist[(i, j)]
            for di, dj in nbrs:
                ii = i + di
                jj = j + dj
                if ii < 0 or jj < 0 or ii >= w or jj >= h:
                    continue
                if blocked[jj][ii]:
                    continue
                key = (ii, jj)
                if key in dist:
                    continue
                dist[key] = d + 1
                q.append((ii, jj))
        return dist

    def reachable_frontier_cells(self, reachable):
        frontier = set()
        for i, j in reachable.keys():
            if self.is_near_map_edge(i, j, self.frontier_edge_margin_cells):
                continue
            if not self.has_unknown_neighbor(i, j):
                continue
            frontier.add((i, j))
        return frontier

    def cluster_frontiers(self, frontier_cells):
        if not frontier_cells:
            return []
        frontier_cells = set(frontier_cells)
        visited = set()
        clusters = []
        if self.frontier_cluster_conn == 8:
            nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        else:
            nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for seed in frontier_cells:
            if seed in visited:
                continue
            q = deque([seed])
            visited.add(seed)
            cluster = []
            while q:
                c = q.popleft()
                cluster.append(c)
                ci, cj = c
                for di, dj in nbrs:
                    ni = ci + di
                    nj = cj + dj
                    nk = (ni, nj)
                    if nk in frontier_cells and nk not in visited:
                        visited.add(nk)
                        q.append(nk)
            if len(cluster) >= self.frontier_min_cluster_size:
                clusters.append(cluster)
        return clusters

    def cluster_centroid_cell(self, cluster):
        if not cluster:
            return None
        mi = sum(c[0] for c in cluster) / float(len(cluster))
        mj = sum(c[1] for c in cluster) / float(len(cluster))
        best = None
        best_d2 = 1e18
        for c in cluster:
            d2 = (c[0] - mi) ** 2 + (c[1] - mj) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best = c
        return best

    def find_approach_cell(self, center_i, center_j, reachable, search_r):
        best = None
        best_score = 1e18
        for dj in range(-search_r, search_r + 1):
            for di in range(-search_r, search_r + 1):
                ii = center_i + di
                jj = center_j + dj
                key = (ii, jj)
                if key not in reachable:
                    continue
                if self.is_near_map_edge(ii, jj, self.frontier_edge_margin_cells):
                    continue
                if not self.is_goal_region_clear(ii, jj, self.goal_free_radius_cells):
                    continue
                obs_n = self.occupied_count_around(ii, jj, self.goal_obstacle_check_radius_cells)
                if obs_n > self.goal_max_allowed_obstacle_neighbors:
                    continue
                d2 = di * di + dj * dj
                unk = self.unknown_count_around(ii, jj, self.unknown_neighbor_radius + 1)
                score = d2 - 0.2 * float(unk)
                if score < best_score:
                    best_score = score
                    best = (ii, jj)
        return best

    def cleanup_blacklist(self):
        now = self.now_sec()
        self.blacklist = [b for b in self.blacklist if b[2] > now]

    def is_blacklisted(self, x, y):
        self.cleanup_blacklist()
        for bx, by, _ in self.blacklist:
            if math.hypot(x - bx, y - by) <= self.goal_blacklist_radius:
                return True
        return False

    def blacklist_goal(self, x, y, hold_sec):
        self.blacklist.append((x, y, self.now_sec() + hold_sec))

    def nav2_path_exists(self, x, y):
        start = self.get_robot_pose_stamped()
        goal = self.make_goal_pose(x, y)
        if start is None:
            return False
        try:
            path = self.navigator.getPath(start, goal)
            return path is not None and len(path.poses) > 1
        except Exception:
            return False

    def frontier_candidates(self, reachable):
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return [], 0, 0
        rx, ry = robot_xy
        yaw = self.get_yaw()
        frontier_cells = self.reachable_frontier_cells(reachable)
        clusters = self.cluster_frontiers(frontier_cells)
        candidates = []
        for cluster in clusters:
            rep = self.cluster_centroid_cell(cluster)
            if rep is None:
                continue
            approach = self.find_approach_cell(rep[0], rep[1], reachable, self.frontier_approach_search_radius_cells)
            if approach is None:
                continue
            ci, cj = approach
            tx, ty = self.map_ij_to_xy(ci, cj)
            if self.is_blacklisted(tx, ty):
                continue
            metric_dist = math.hypot(tx - rx, ty - ry)
            if metric_dist < self.frontier_min_goal_dist or metric_dist > self.frontier_max_goal_dist:
                continue
            heading_err = abs(self.wrap_angle(math.atan2(ty - ry, tx - rx) - yaw))
            cluster_size = len(cluster)
            unknown_score = self.unknown_count_around(ci, cj, self.unknown_neighbor_radius + 1)
            score = (
                self.frontier_dist_weight * metric_dist +
                self.frontier_heading_weight * heading_err -
                self.frontier_size_weight * float(cluster_size) -
                self.frontier_unknown_weight * float(unknown_score)
            )
            candidates.append({'score': score, 'x': tx, 'y': ty, 'cell': (ci, cj), 'cluster_size': cluster_size, 'unknown_score': unknown_score})
        candidates.sort(key=lambda c: c['score'])
        return candidates, len(frontier_cells), len(clusters)

    def fallback_candidates(self, reachable):
        if not self.fallback_enable:
            return []
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return []
        rx, ry = robot_xy
        candidates = []
        for i, j in reachable.keys():
            if self.is_near_map_edge(i, j, self.frontier_edge_margin_cells):
                continue
            if not self.is_goal_region_clear(i, j, self.goal_free_radius_cells):
                continue
            x, y = self.map_ij_to_xy(i, j)
            if self.is_blacklisted(x, y):
                continue
            dist_m = math.hypot(x - rx, y - ry)
            if dist_m < self.fallback_min_goal_dist or dist_m > self.fallback_max_goal_dist:
                continue
            unk = self.unknown_count_around(i, j, self.fallback_unknown_search_radius)
            if unk <= 0:
                continue
            score = self.fallback_unknown_weight * float(unk) - self.fallback_dist_weight * dist_m
            candidates.append({'score': score, 'x': x, 'y': y, 'cell': (i, j), 'unknown_score': unk})
        candidates.sort(key=lambda c: c['score'], reverse=True)
        if not candidates:
            keys = list(reachable.keys())
            random.shuffle(keys)
            sample_count = min(len(keys), self.fallback_random_sample_count)
            for idx in range(sample_count):
                i, j = keys[idx]
                if self.is_near_map_edge(i, j, self.frontier_edge_margin_cells):
                    continue
                if not self.is_goal_region_clear(i, j, self.goal_free_radius_cells):
                    continue
                x, y = self.map_ij_to_xy(i, j)
                if self.is_blacklisted(x, y):
                    continue
                dist_m = math.hypot(x - rx, y - ry)
                if dist_m < self.fallback_min_goal_dist or dist_m > self.fallback_max_goal_dist:
                    continue
                candidates.append({'score': 0.0, 'x': x, 'y': y, 'cell': (i, j), 'unknown_score': 0})
                break
        return candidates

    def find_best_target(self):
        robot_xy = self.get_robot_xy()
        if self.map_msg is None or robot_xy is None:
            return None, None, {}
        rcell = self.map_xy_to_ij(robot_xy[0], robot_xy[1])
        if rcell is None:
            return None, None, {}
        blocked = self.build_blocked_grid()
        if blocked is None:
            return None, None, {}
        start = self.find_nearest_unblocked_start(blocked, rcell[0], rcell[1])
        if start is None:
            return None, None, {}
        reachable = self.reachable_free_cells(blocked, start[0], start[1])
        frontier_cands, frontier_cell_count, cluster_count = self.frontier_candidates(reachable)
        debug = {'reachable': len(reachable), 'frontier_cells': frontier_cell_count, 'clusters': cluster_count, 'frontier_candidates': len(frontier_cands), 'fallback_candidates': 0}
        for idx, c in enumerate(frontier_cands[:self.plan_validation_candidates]):
            ok = self.nav2_path_exists(c['x'], c['y'])
            self.maybe_debug(f'state=CHECK_FRONTIER idx={idx} target=({c["x"]:.2f},{c["y"]:.2f}) cluster={c["cluster_size"]} unk={c["unknown_score"]} nav2_path={ok}')
            if ok:
                return (c['x'], c['y']), 'FRONTIER', debug
        fallback_cands = self.fallback_candidates(reachable)
        debug['fallback_candidates'] = len(fallback_cands)
        for idx, c in enumerate(fallback_cands[:self.plan_validation_candidates]):
            ok = self.nav2_path_exists(c['x'], c['y'])
            self.maybe_debug(f'state=CHECK_FALLBACK idx={idx} target=({c["x"]:.2f},{c["y"]:.2f}) unk={c["unknown_score"]} nav2_path={ok}')
            if ok:
                return (c['x'], c['y']), 'FALLBACK', debug
        return None, None, debug

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'nav2_frontier_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        if self.current_goal_xy is None:
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
            return
        marker.pose.position.x = float(self.current_goal_xy[0])
        marker.pose.position.y = float(self.current_goal_xy[1])
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.24
        marker.scale.y = 0.24
        marker.scale.z = 0.24
        marker.color.a = 0.95
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        self.marker_pub.publish(marker)

    def update_progress_history(self):
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return
        now = self.now_sec()
        self.goal_progress_history.append((now, robot_xy[0], robot_xy[1]))
        while self.goal_progress_history and (now - self.goal_progress_history[0][0]) > self.progress_window_sec:
            self.goal_progress_history.popleft()

    def feedback_distance_remaining(self):
        try:
            feedback = self.navigator.getFeedback()
        except Exception:
            return None, None
        if feedback is None:
            return None, None
        dist_rem = getattr(feedback, 'distance_remaining', None)
        nav_time = getattr(feedback, 'navigation_time', None)
        nav_sec = None
        if nav_time is not None:
            try:
                nav_sec = nav_time.sec + nav_time.nanosec * 1e-9
            except Exception:
                nav_sec = None
        return dist_rem, nav_sec

    def is_progress_stalled(self):
        if len(self.goal_progress_history) < 2:
            return False
        elapsed = self.now_sec() - self.current_goal_stamp
        if elapsed < self.startup_grace_sec:
            return False
        x0 = self.goal_progress_history[0][1]
        y0 = self.goal_progress_history[0][2]
        x1 = self.goal_progress_history[-1][1]
        y1 = self.goal_progress_history[-1][2]
        moved = math.hypot(x1 - x0, y1 - y0)
        feedback = None
        try:
            feedback = self.navigator.getFeedback()
        except Exception:
            feedback = None
        if feedback is not None:
            dist_rem = getattr(feedback, 'distance_remaining', None)
            if dist_rem is not None:
                dist_rem = float(dist_rem)
                if self.current_goal_feedback_start is None:
                    self.current_goal_feedback_start = dist_rem
                    self.current_goal_feedback_best = dist_rem
                else:
                    self.current_goal_feedback_best = min(self.current_goal_feedback_best, dist_rem)
                improvement = self.current_goal_feedback_start - self.current_goal_feedback_best
                if improvement > self.feedback_progress_epsilon:
                    return False
        return moved < self.progress_min_dist

    def control_loop(self):
        if self.auto_mapper_mode != 'EXPLORE':
            self.state = self.IDLE
            return
        if self.done_sent:
            self.publish_done()
            return
        if self.map_msg is None:
            return
        if self.lookup_map_to_base() is None:
            return
        if not self.ensure_nav2_ready():
            return
        unknown_ratio, known_cells = self.get_unknown_ratio_in_bbox()

        if self.current_goal_sent and self.current_goal_xy is not None:
            self.publish_goal_marker()
            self.update_progress_history()
            robot_xy = self.get_robot_xy()
            if robot_xy is None:
                return
            d_goal = math.hypot(self.current_goal_xy[0] - robot_xy[0], self.current_goal_xy[1] - robot_xy[1])
            d_fb, t_nav = self.feedback_distance_remaining()
            self.maybe_debug(f'state=RUN kind={self.current_goal_kind} robot=({robot_xy[0]:.2f},{robot_xy[1]:.2f}) target=({self.current_goal_xy[0]:.2f},{self.current_goal_xy[1]:.2f}) d_goal={d_goal:.2f} d_fb={(d_fb if d_fb is not None else -1):.2f} t_nav={(t_nav if t_nav is not None else -1):.1f}s')
            if (self.now_sec() - self.current_goal_stamp) > self.goal_timeout_sec:
                self.maybe_debug('state=TIMEOUT -> cancel and blacklist goal', force=True)
                self.cancel_active_goal()
                self.blacklist_goal(self.current_goal_xy[0], self.current_goal_xy[1], self.failed_blacklist_hold_sec)
                self.clear_current_goal()
                self.publish_goal_marker()
                self.last_plan_time = 0.0
                return
            if self.is_progress_stalled():
                self.maybe_debug('state=STALLED -> cancel and blacklist goal', force=True)
                self.cancel_active_goal()
                self.blacklist_goal(self.current_goal_xy[0], self.current_goal_xy[1], self.failed_blacklist_hold_sec)
                self.clear_current_goal()
                self.publish_goal_marker()
                self.last_plan_time = 0.0
                return
            if self.navigator.isTaskComplete():
                try:
                    result = self.navigator.getResult()
                except Exception:
                    result = None
                if result == TaskResult.SUCCEEDED:
                    self.blacklist_goal(self.current_goal_xy[0], self.current_goal_xy[1], self.reached_blacklist_hold_sec)
                elif result == TaskResult.FAILED:
                    self.blacklist_goal(self.current_goal_xy[0], self.current_goal_xy[1], self.failed_blacklist_hold_sec)
                elif result == TaskResult.CANCELED:
                    self.blacklist_goal(self.current_goal_xy[0], self.current_goal_xy[1], 30.0)
                else:
                    self.blacklist_goal(self.current_goal_xy[0], self.current_goal_xy[1], 30.0)
                self.clear_current_goal()
                self.publish_goal_marker()
                self.last_plan_time = 0.0
            return

        if unknown_ratio is not None and known_cells >= self.finish_min_known_cells and unknown_ratio <= self.finish_unknown_ratio:
            self.maybe_debug(f'state=DONE unknown_ratio={unknown_ratio:.3f} known={known_cells}', force=True)
            self.done_sent = True
            self.publish_done()
            self.state = self.DONE
            self.publish_goal_marker()
            return

        if (self.now_sec() - self.last_plan_time) < self.replan_period_sec:
            return
        self.last_plan_time = self.now_sec()
        target, target_kind, dbg = self.find_best_target()
        self.maybe_debug(f'state=SEARCH reachable={dbg.get("reachable", 0)} frontier_cells={dbg.get("frontier_cells", 0)} clusters={dbg.get("clusters", 0)} frontier_candidates={dbg.get("frontier_candidates", 0)} fallback_candidates={dbg.get("fallback_candidates", 0)} unknown_ratio={(unknown_ratio if unknown_ratio is not None else -1):.3f} known={known_cells}')
        if target is None:
            self.maybe_debug('state=NO_VALID_TARGET_FROM_NAV2', force=True)
            return
        self.current_goal_xy = target
        self.current_goal_kind = target_kind if target_kind is not None else 'UNKNOWN'
        self.current_goal_stamp = self.now_sec()
        self.current_goal_sent = False
        self.current_goal_feedback_start = None
        self.current_goal_feedback_best = None
        self.goal_progress_history.clear()
        self.update_progress_history()
        pose = self.make_goal_pose(target[0], target[1])
        try:
            accepted = self.navigator.goToPose(pose)
        except Exception:
            accepted = False
        if not accepted:
            self.maybe_debug(f'state=REJECTED kind={self.current_goal_kind} target=({target[0]:.2f},{target[1]:.2f})', force=True)
            self.blacklist_goal(target[0], target[1], 30.0)
            self.clear_current_goal()
            self.publish_goal_marker()
            self.last_plan_time = 0.0
            return
        self.current_goal_sent = True
        self.state = self.EXPLORING
        self.publish_goal_marker()
        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return
        d_goal = math.hypot(target[0] - robot_xy[0], target[1] - robot_xy[1])
        self.maybe_debug(f'state=NEW_GOAL kind={self.current_goal_kind} robot=({robot_xy[0]:.2f},{robot_xy[1]:.2f}) target=({target[0]:.2f},{target[1]:.2f}) d_goal={d_goal:.2f}', force=True)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNav2Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cancel_active_goal()
        except Exception:
            pass
        try:
            node.publish_goal_marker()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            node.navigator.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()