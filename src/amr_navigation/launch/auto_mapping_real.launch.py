from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": slam_params_file
        }.items()
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": nav2_params_file,
            "use_composition": "False",
            "use_respawn": "False"
        }.items()
    )

    nav2_explore_node = Node(
        package="amr_navigation",
        executable="nav2_explore.py",
        name="nav2_explore",
        output="screen",
        parameters=[{
            "map_topic": "/map",
            "odom_topic": "/odometry/filtered",
            "navigate_action": "navigate_to_pose",
            "global_frame": "map",
            "robot_frame": "base_link",
            "planner_period": 3.0,
            "min_frontier_size": 12,
            "frontier_search_radius_cells": 120,
            "goal_blacklist_radius": 0.35,
            "goal_tolerance_for_repeat": 0.25,
            "max_goal_failures": 3,
            "prefer_far_frontiers": True
        }]
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("amr_description"),
            "rviz",
            "gazebo.rviz"
        ])]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true"
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_description"),
                "config",
                "slam_real.yaml"
            ])
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_navigation"),
                "config",
                "nav2_explore_params.yaml"
            ])
        ),

        slam_toolbox_launch,
        nav2_navigation_launch,
        nav2_explore_node,
        rviz_node,
    ])