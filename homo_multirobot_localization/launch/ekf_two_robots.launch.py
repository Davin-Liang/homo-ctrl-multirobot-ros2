from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot1_namespace = LaunchConfiguration("robot1_namespace")
    robot2_namespace = LaunchConfiguration("robot2_namespace")

    pkg_share = FindPackageShare("homo_multirobot_localization")
    ekf_robot1_yaml = PathJoinSubstitution([pkg_share, "config", "ekf_robot1.yaml"])
    ekf_robot2_yaml = PathJoinSubstitution([pkg_share, "config", "ekf_robot2.yaml"])

    ekf_robot1 = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=robot1_namespace,
        output="screen",
        parameters=[ekf_robot1_yaml, {"use_sim_time": use_sim_time}],
    )

    ekf_robot2 = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=robot2_namespace,
        output="screen",
        parameters=[ekf_robot2_yaml, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot1_namespace", default_value="/robot1"),
            DeclareLaunchArgument("robot2_namespace", default_value="/robot2"),
            ekf_robot1,
            ekf_robot2,
        ]
    )

