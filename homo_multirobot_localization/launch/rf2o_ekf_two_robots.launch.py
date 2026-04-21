from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot1_namespace = LaunchConfiguration("robot1_namespace")
    robot2_namespace = LaunchConfiguration("robot2_namespace")
    rf2o_publish_tf = LaunchConfiguration("rf2o_publish_tf")

    pkg_share = FindPackageShare("homo_multirobot_localization")

    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "rf2o_two_robots.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot1_namespace": robot1_namespace,
            "robot2_namespace": robot2_namespace,
            "rf2o_publish_tf": rf2o_publish_tf,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "ekf_two_robots.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot1_namespace": robot1_namespace,
            "robot2_namespace": robot2_namespace,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot1_namespace", default_value="/robot1"),
            DeclareLaunchArgument("robot2_namespace", default_value="/robot2"),
            DeclareLaunchArgument(
                "rf2o_publish_tf",
                default_value="false",
                description="是否让 rf2o 发布 TF（建议 false，由 EKF 统一发布 TF）。",
            ),
            rf2o_launch,
            ekf_launch,
        ]
    )

