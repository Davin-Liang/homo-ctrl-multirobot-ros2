from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    source = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("homo_multirobot_formation_control"), "launch",
        "formation_single_follower_6d_map_hpc_artstein.launch.py"]))
    return LaunchDescription([
        DeclareLaunchArgument("forward_prediction_horizon", default_value="0.0"),
        IncludeLaunchDescription(source, launch_arguments={
            "state_source": "mocap", "use_sim_time": "false", "use_motor_delay": "false",
            "forward_prediction_horizon": LaunchConfiguration(
                "forward_prediction_horizon"),
        }.items()),
    ])
