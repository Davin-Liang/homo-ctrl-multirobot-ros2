from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    bringup = LaunchConfiguration("bringup")

    declared_arguments = [
        DeclareLaunchArgument(
            "bringup",
            default_value=TextSubstitution(text="false"),
            description="Whether to include wheeltec bringup (turn_on_wheeltec_robot + lidar).",
        ),
    ]

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.join(get_package_share_directory("turn_on_wheeltec_robot"), "launch"),
                "turn_on_wheeltec_robot.launch.py",
            )
        ),
        condition=IfCondition(bringup),
    )
    wheeltec_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.join(get_package_share_directory("turn_on_wheeltec_robot"), "launch"),
                "wheeltec_lidar.launch.py",
            )
        ),
        condition=IfCondition(bringup),
    )

    return LaunchDescription(
        declared_arguments
        + [
            wheeltec_robot,
            wheeltec_lidar,
            launch_ros.actions.Node(
                parameters=[
                    get_package_share_directory("homo_multirobot_slam_toolbox")
                    + "/config/mapper_params_online_async.yaml"
                ],
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                remappings=[("odom", "odom_combined")],
            ),
        ]
    )
