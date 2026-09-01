"""6D Artstein Disc formation with scan-only static-cylinder HOCBF safety.

This launch only starts the follower controller.  It subscribes to the leader
odometry and therefore intentionally has no ``leader_speed`` argument; set the
leader speed on the separately launched leader trajectory node.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as _DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

config_file = os.path.join(
    get_package_share_directory("homo_multirobot_formation_control"),
    "config", "formation_single_follower_6d_artstein_disc_hocbf.yaml")
with open(config_file, encoding="utf-8") as stream:
    defaults = yaml.safe_load(stream)["/**"]["ros__parameters"]


def _launch_default(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def DeclareLaunchArgument(name, default_value, description=None):
    return _DeclareLaunchArgument(
        name, default_value=_launch_default(defaults[name]), description=description)


def generate_launch_description():
    parameters = {name: LaunchConfiguration(name) for name in defaults}
    node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_6d_artstein_disc_hocbf",
        name="formation_control_node_6d_artstein_disc_hocbf",
        namespace=PythonExpression(["'", LaunchConfiguration("follower_ns"), "'"]),
        output="screen", parameters=[config_file, parameters],
    )
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [node])
