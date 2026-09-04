from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _vrpn_topic(rigid_name, suffix):
    return PythonExpression(["'/vrpn/' + '", rigid_name, "' + '/", suffix, "'"])


def generate_launch_description():
    server = LaunchConfiguration("server")
    port = LaunchConfiguration("port")
    robot1_rigid_name = LaunchConfiguration("robot1_rigid_name")
    robot2_rigid_name = LaunchConfiguration("robot2_rigid_name")
    timeout = LaunchConfiguration("state_timeout")
    share = FindPackageShare("homo_multirobot_localization")

    bridge = Node(
        package="vrpn_listener", executable="vrpn_listener", name="vrpn_listener", output="screen",
        parameters=[{"server": server, "port": port, "frame_id": "map"}],
    )
    robot1 = Node(
        package="homo_multirobot_localization", executable="mocap_state_adapter",
        namespace="robot1", name="mocap_state_adapter", output="screen",
        parameters=[PathJoinSubstitution([share, "config", "mocap_robot1.yaml"]), {
            "input_pose_topic": _vrpn_topic(robot1_rigid_name, "pose"),
            "input_twist_topic": _vrpn_topic(robot1_rigid_name, "twist"),
            "state_timeout": timeout,
        }],
    )
    robot2 = Node(
        package="homo_multirobot_localization", executable="mocap_state_adapter",
        namespace="robot2", name="mocap_state_adapter", output="screen",
        parameters=[PathJoinSubstitution([share, "config", "mocap_robot2.yaml"]), {
            "input_pose_topic": _vrpn_topic(robot2_rigid_name, "pose"),
            "input_twist_topic": _vrpn_topic(robot2_rigid_name, "twist"),
            "state_timeout": timeout,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("server", description="VRPN server IP or hostname"),
        DeclareLaunchArgument("port", default_value="3883"),
        DeclareLaunchArgument("robot1_rigid_name", default_value="robot1"),
        DeclareLaunchArgument("robot2_rigid_name", default_value="robot2"),
        DeclareLaunchArgument("state_timeout", default_value="0.10"),
        bridge, robot1, robot2,
    ])
