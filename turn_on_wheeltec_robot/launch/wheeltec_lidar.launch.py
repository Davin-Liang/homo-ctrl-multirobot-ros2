import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    driver_config = os.path.join(
        get_package_share_directory("lslidar_driver"),
        "config", "lslidar_x10.yaml"
    )

    lidar_node = Node(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        name="lslidar_driver_node",
        namespace=namespace,
        output="screen",
        parameters=[driver_config, {
            "lidar_model": "N10Plus",
            "serial_port": "/dev/wheeltec_lidar",
            "frame_id": [prefix, "laser"],
            "laserscan_topic": "scan",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="机器人命名空间"
        ),
        DeclareLaunchArgument(
            "prefix", default_value="",
            description="TF 帧前缀，如 robot1_"
        ),
        lidar_node,
    ])
