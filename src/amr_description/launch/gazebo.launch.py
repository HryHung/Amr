import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_amr = get_package_share_directory('amr_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_amr, 'urdf', 'amr.urdf.xacro')
    world_file = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_amr_robot',
        output='screen',
        arguments=[
            '-entity', 'amr_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot
    ])