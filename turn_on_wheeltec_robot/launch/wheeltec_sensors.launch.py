import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    launch_dir = os.path.join(
        get_package_share_directory("turn_on_wheeltec_robot"), "launch"
    )

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "turn_on_wheeltec_robot.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
        }.items(),
    )
    lidar_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "wheeltec_lidar.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
        }.items(),
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
        wheeltec_robot,
        lidar_ros,
    ])
