from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_share = get_package_share_directory("homo_multirobot_slam_toolbox")

    mapper_robot = LaunchConfiguration("mapper_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    declared_arguments = [
        DeclareLaunchArgument(
            "mapper_robot",
            default_value=TextSubstitution(text="robot1"),
            description="Which robot namespace runs slam_toolbox (e.g. robot1 or robot2).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=TextSubstitution(text="true"),
            description="Use /clock if available (simulation).",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [TextSubstitution(text=pkg_share), "config", "mapper_params_online_sync.yaml"]
            ),
            description="slam_toolbox parameters YAML.",
        ),
    ]

    # Frame naming convention: <prefix>odom / <prefix>base_footprint
    # where prefix == f'{mapper_robot}_'
    slam_toolbox = GroupAction(
        actions=[
            PushRosNamespace(mapper_robot),
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                # slam_toolbox may publish map topics as absolute names (/map, /map_metadata, /map_updates).
                # Remap them into the robot namespace so save_map (which runs under the same namespace)
                # can subscribe successfully.
                remappings=[
                    ("/map", "map"),
                    ("/map_metadata", "map_metadata"),
                    ("/map_updates", "map_updates"),
                ],
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "map",
                        "odom_frame": [mapper_robot, TextSubstitution(text="_odom")],
                        "base_frame": [mapper_robot, TextSubstitution(text="_base_footprint")],
                        "scan_topic": "scan",
                    },
                ],
            ),
        ]
    )

    return LaunchDescription(declared_arguments + [slam_toolbox])

