from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_sim = get_package_share_directory('amr_simulation')
    pkg_desc = get_package_share_directory('amr_description')

    world_file = os.path.join(pkg_sim, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'amr.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'amr_robot',
            '-x', '-20',
            '-y', '-10',
            '-z', '0.05'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    return LaunchDescription([
        gazebo,
        rsp,
        delayed_spawn
    ])