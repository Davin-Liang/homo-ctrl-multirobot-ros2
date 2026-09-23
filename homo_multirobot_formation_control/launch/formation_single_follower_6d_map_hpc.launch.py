"""6D Map HPC 编队控制：不使用 Artstein 延迟补偿或前向预测。"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


config_file = os.path.join(
    get_package_share_directory("homo_multirobot_formation_control"),
    "config", "formation_single_follower_6d_map_hpc.yaml")
with open(config_file, encoding="utf-8") as stream:
    defaults = yaml.safe_load(stream)["/**"]["ros__parameters"]


def _launch_default(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=_launch_default(defaults[name]))
        for name, value in defaults.items()
    ]
    parameters = {name: LaunchConfiguration(name) for name in defaults}
    follower_ns = LaunchConfiguration("follower_ns")
    use_motor_delay = LaunchConfiguration("use_motor_delay")
    cmd_topic = PythonExpression([
        "'cmd_vel_raw' if '", use_motor_delay, "' == 'true' else 'cmd_vel'"
    ])

    controller = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_6d_map_hpc",
        name="formation_control_node_6d_map_hpc",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        remappings=[("cmd_vel", cmd_topic)],
        parameters=[config_file, parameters],
    )
    delay_node = Node(
        package="homo_multirobot_formation_control",
        executable="sim_motor_delay.py",
        name="sim_motor_delay",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        condition=IfCondition(use_motor_delay),
        output="screen",
        parameters=[{
            "input_topic": "cmd_vel_raw",
            "output_topic": "cmd_vel",
            "motor_tau": LaunchConfiguration("motor_tau"),
            "transport_delay": LaunchConfiguration("transport_delay"),
            "max_accel": LaunchConfiguration("delay_max_accel"),
            "rate": 100.0,
        }],
    )
    return LaunchDescription(arguments + [controller, delay_node])
