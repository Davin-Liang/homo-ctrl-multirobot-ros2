import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    robot_frame_id = LaunchConfiguration("robot_frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    gyro_frame_id = LaunchConfiguration("gyro_frame_id")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="",
        description="机器人命名空间（如 robot1），留空为根命名空间"
    )
    declare_prefix = DeclareLaunchArgument(
        "prefix", default_value="",
        description="TF 帧前缀（如 robot1_），留空无前缀"
    )
    declare_robot_frame = DeclareLaunchArgument(
        "robot_frame_id",
        default_value=[prefix, "base_footprint"],
        description="机器人 base frame_id"
    )
    declare_odom_frame = DeclareLaunchArgument(
        "odom_frame_id",
        default_value=[prefix, "odom_combined"],
        description="轮式里程计 odom frame_id"
    )
    declare_gyro_frame = DeclareLaunchArgument(
        "gyro_frame_id",
        default_value=[prefix, "gyro_link"],
        description="IMU gyro frame_id"
    )

    wheeltec_node = Node(
        package="turn_on_wheeltec_robot",
        executable="wheeltec_robot_node",
        name="wheeltec_robot",
        namespace=namespace,
        output="screen",
        remappings=[("/cmd_vel", "cmd_vel")],
        parameters=[{
            "usart_port_name": "/dev/wheeltec_controller",
            "serial_baud_rate": 115200,
            "robot_frame_id": robot_frame_id,
            "odom_frame_id": odom_frame_id,
            "gyro_frame_id": gyro_frame_id,
            "car_mode": "mini_omni",
            "ranger_avoid_flag": False,
            "odom_x_scale": 1.0,
            "odom_y_scale": 1.0,
            "odom_z_scale_positive": 1.0,
            "odom_z_scale_negative": 1.0,
        }],
    )

    return LaunchDescription([
        declare_namespace,
        declare_prefix,
        declare_robot_frame,
        declare_odom_frame,
        declare_gyro_frame,
        wheeltec_node,
    ])
