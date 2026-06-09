from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    lidar_node = Node(
        package="lslidar_driver",
        executable="lslidar_driver_node",
        name="lslidar_driver_node",
        namespace=namespace,
        output="screen",
        parameters=[{
            "lidar_type": "X10",
            "lidar_model": "N10Plus",
            "serial_port": "/dev/wheeltec_lidar",
            "frame_id": [prefix, "laser"],
            "pointcloud_topic": "lslidar_point_cloud",
            "laserscan_topic": "scan",
            "publish_scan": True,
            "use_high_precision": True,
            "N10Plus_hz": 6,
            "min_range": 0.15,
            "max_range": 50.0,
            "angle_disable_min": [0],
            "angle_disable_max": [0],
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
