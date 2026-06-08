import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    # xacro 参数
    xacro_path = PathJoinSubstitution([
        FindPackageShare("homo_multirobot_urdf"),
        "urdf", "mini_omni_robot.xacro",
    ])

    robot_desc = Command([
        "xacro ", xacro_path,
        " prefix:=", prefix,
        " use_gazebo:=false",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"robot_description": robot_desc}],
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
        robot_state_publisher,
    ])
