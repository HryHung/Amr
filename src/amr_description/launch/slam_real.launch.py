from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_params_file = LaunchConfiguration("slam_params_file")
    lidar_port = LaunchConfiguration("lidar_port")
    agent_port = LaunchConfiguration("agent_port")
    agent_baudrate = LaunchConfiguration("agent_baudrate")
    use_rviz = LaunchConfiguration("use_rviz")

    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("amr_description"),
                "launch",
                "real_bringup.launch.py"
            ])
        ]),
        launch_arguments={
            "lidar_port": lidar_port,
            "agent_port": agent_port,
            "agent_baudrate": agent_baudrate,
            "use_rviz": use_rviz
        }.items()
    )

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

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0"
        ),
        DeclareLaunchArgument(
            "agent_port",
            default_value="/dev/ttyUSB1"
        ),
        DeclareLaunchArgument(
            "agent_baudrate",
            default_value="115200"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_description"),
                "config",
                "slam_real.yaml"
            ])
        ),

        real_bringup,
        slam_toolbox_launch,
    ])