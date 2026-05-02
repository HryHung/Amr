from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument('host', default_value='127.0.0.1')
    port_arg = DeclareLaunchArgument('port', default_value='8080')
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value='/home/hungubuntu/Documents/amr_ws/waypoints.json'
    )

    web_node = Node(
        package='amr_mission_manager',
        executable='web_mission_server',
        name='web_mission_server',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'waypoint_file': LaunchConfiguration('waypoint_file'),
        }]
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        waypoint_file_arg,
        web_node,
    ])
