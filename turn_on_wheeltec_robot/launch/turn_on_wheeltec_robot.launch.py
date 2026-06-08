import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    launch_dir = os.path.join(
        get_package_share_directory("turn_on_wheeltec_robot"), "launch"
    )

    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "base_serial.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
        }.items(),
    )

    robot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_mode_description.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
        }.items(),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=namespace,
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
        base_serial,
        robot_desc,
        joint_state_publisher,
    ])
