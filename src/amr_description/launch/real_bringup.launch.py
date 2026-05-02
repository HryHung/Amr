from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    robot_description_file = LaunchConfiguration("robot_description_file")

    lidar_port = LaunchConfiguration("lidar_port")
    lidar_frame = LaunchConfiguration("lidar_frame")

    agent_port = LaunchConfiguration("agent_port")
    agent_baudrate = LaunchConfiguration("agent_baudrate")

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_base = LaunchConfiguration("wheel_base")

    imu_frame = LaunchConfiguration("imu_frame")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_rate = LaunchConfiguration("imu_rate")

    ekf_params_file = LaunchConfiguration("ekf_params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            robot_description_file
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "robot_description": robot_description_content
            }
        ]
    )

    micro_ros_agent = ExecuteProcess(
        cmd=[
            "ros2", "run", "micro_ros_agent", "micro_ros_agent",
            "serial",
            "--dev", agent_port,
            "-b", agent_baudrate
        ],
        output="screen"
    )

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": lidar_port,
            "serial_baudrate": 115200,
            "frame_id": lidar_frame,
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Sensitivity"
        }]
    )

    wheel_rpm_odometry_node = Node(
        package="amr_description",
        executable="wheel_rpm_odometry.py",
        name="wheel_rpm_odometry",
        output="screen",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_base": wheel_base,
            "rpm_topic": "/wheel_rpm_feedback",
            "odom_topic": "/odom/raw",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "publish_tf": False
        }]
    )

    bno055_node = Node(
        package="amr_description",
        executable="bno055_node.py",
        name="bno055_node",
        output="screen",
        parameters=[{
            "i2c_address": 0x28,
            "frame_id": imu_frame,
            "imu_topic": imu_topic,
            "mag_topic": "/imu/mag",
            "publish_rate": imu_rate
        }]
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_description_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_description"),
                "urdf",
                "amr_real.urdf.xacro"
            ])
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB0"
        ),
        DeclareLaunchArgument(
            "lidar_frame",
            default_value="lidar_link"
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
            "wheel_radius",
            default_value="0.06"
        ),
        DeclareLaunchArgument(
            "wheel_base",
            default_value="0.465"
        ),
        DeclareLaunchArgument(
            "imu_frame",
            default_value="imu_link"
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu/data"
        ),
        DeclareLaunchArgument(
            "imu_rate",
            default_value="30.0"
        ),
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_navigation"),
                "config",
                "ekf_real.yaml"
            ])
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("amr_description"),
                "rviz",
                "gazebo.rviz"
            ])
        ),

        robot_state_publisher_node,
        micro_ros_agent,
        rplidar_node,
        wheel_rpm_odometry_node,
        bno055_node,
        ekf_node,
        rviz_node,  
    ])