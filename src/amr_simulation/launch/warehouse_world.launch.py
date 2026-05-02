from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_sim = get_package_share_directory('amr_simulation')
    world_file = os.path.join(pkg_sim, 'worlds', 'warehouse.world')

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])