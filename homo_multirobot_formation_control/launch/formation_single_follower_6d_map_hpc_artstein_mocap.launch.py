import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


config_file = os.path.join(
    get_package_share_directory("homo_multirobot_formation_control"),
    "config", "formation_single_follower_6d_map_hpc_artstein.yaml")
with open(config_file, encoding="utf-8") as stream:
    defaults = yaml.safe_load(stream)["/**"]["ros__parameters"]


def _launch_default(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def generate_launch_description():
    source = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("homo_multirobot_formation_control"), "launch",
        "formation_single_follower_6d_map_hpc_artstein.launch.py"]))
    return LaunchDescription([
        DeclareLaunchArgument(
            "forward_prediction_horizon",
            default_value=_launch_default(defaults["forward_prediction_horizon"])),
        IncludeLaunchDescription(source, launch_arguments={
            "state_source": "mocap", "use_sim_time": "false", "use_motor_delay": "false",
            "forward_prediction_horizon": LaunchConfiguration(
                "forward_prediction_horizon"),
        }.items()),
    ])
