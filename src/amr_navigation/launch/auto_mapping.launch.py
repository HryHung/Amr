from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav2_params = '/home/hungubuntu/Documents/amr_ws/src/amr_navigation/config/nav2_explore_params.yaml'

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }],
        ),

        Node(
            package='amr_navigation',
            executable='auto_mapper.py',
            name='auto_mapper',
            output='screen',
            parameters=[
                {'scan_topic': '/scan'},
                {'odom_topic': '/odom'},
                {'cmd_vel_topic': '/cmd_vel'},
                {'mode_topic': '/auto_mapper/mode'},
                {'explore_done_topic': '/auto_mapper/explore_done'},
                {'base_length': 0.500},
                {'base_width': 0.400},
                {'lidar_x': 0.200},
                {'lidar_y': 0.000},
                {'v_max': 0.35},
                {'a_max': 0.4},
                {'a_d': 1.421762821},
                {'w_max': 0.9},
                {'w_accel': 2.4},
                {'left_target_clearance': 0.40},
                {'seek_wall_clearance': 0.50},
                {'front_stop_clearance': 0.22},
                {'front_turn_clearance': 0.50},
                {'front_slow_clearance': 0.95},
                {'wall_lost_clearance': 0.78},
                {'min_valid_scan': 0.05},
                {'seek_detect_wall_dist': 1.40},
                {'seek_forward_speed': 0.24},
                {'seek_turn_rate': 0.28},
                {'seek_right_turn_time': 1.45},
                {'k_dist': 2.0},
                {'k_align': 1.3},
                {'k_front': 3.2},
                {'search_left_rate': 0.38},
                {'turn_in_place_rate': 0.75},
                {'stuck_window_sec': 4.0},
                {'stuck_distance_thresh': 0.05},
                {'recovery_reverse_sec': 0.30},
                {'recovery_rotate_sec': 1.10},
                {'recovery_cooldown_sec': 3.0},
                {'stuck_min_cmd_v': 0.12},
                {'stuck_max_cmd_w': 0.35},
                {'bootstrap_min_time_sec': 20.0},
                {'bootstrap_min_travel_m': 4.0},
                {'bootstrap_frontier_handoff': True},
                {'debug_print_sec': 1.0},
                {'control_period': 0.05},
                {'enable_keyboard_debug': False},
            ],
        ),

        Node(
            package='amr_navigation',
            executable='nav2_explore.py',
            name='nav2_frontier_explorer',
            output='screen',
            parameters=[
                {'map_topic': '/map'},
                {'mode_topic': '/auto_mapper/mode'},
                {'explore_done_topic': '/auto_mapper/explore_done'},
                {'marker_topic': '/frontier_target_marker'},
                {'global_frame': 'map'},
                {'robot_base_frame': 'base_footprint'},
                {'control_period': 0.25},
                {'replan_period_sec': 1.0},
                {'debug_print_sec': 0.75},
            ],
        ),
    ])