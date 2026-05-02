from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = "/home/hungubuntu/Documents/amr_ws/src/amr_navigation/config/nav2_explore_params.yaml"

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower"
                ]
            }]
        )
    ])