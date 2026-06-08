from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    imu_processor = Node(
        package="turn_on_wheeltec_robot",
        executable="ImuProcessor",
        name="imu_processor",
        namespace=namespace,
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="机器人命名空间"
        ),
        imu_processor,
    ])
