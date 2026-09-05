"""Minimal real-robot bringup for pure motion-capture localization.

Starts only the wheel serial driver and robot description; lidar is optional.
It deliberately excludes ImuProcessor, rf2o, and EKF because mocap_state_adapter
owns the localization state and odom-to-base TF in this mode.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_dir = os.path.join(
        get_package_share_directory("turn_on_wheeltec_robot"), "launch")

    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "base_serial.launch.py")),
        launch_arguments={"namespace": namespace, "prefix": prefix}.items(),
    )
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_mode_description.launch.py")),
        launch_arguments={"namespace": namespace, "prefix": prefix}.items(),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "wheeltec_lidar.launch.py")),
        condition=IfCondition(launch_lidar),
        launch_arguments={"namespace": namespace, "prefix": prefix}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot1"),
        DeclareLaunchArgument("prefix", default_value="robot1_"),
        DeclareLaunchArgument(
            "launch_lidar", default_value="false",
            description="Start lslidar_driver_node for scan visualization or obstacle sensing"),
        base_serial,
        robot_description,
        lidar,
    ])
