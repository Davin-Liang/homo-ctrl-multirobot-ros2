from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace


def _make_slam_toolbox_group(context, **_kwargs):
    mapper_robot = LaunchConfiguration("mapper_robot").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_in_namespace = (
        LaunchConfiguration("map_in_namespace").perform(context).strip().lower() == "true"
    )

    # Default behavior: always publish/subscribe global /map (easier for RViz).
    # If you need /<ns>/map for tooling (e.g. save_map expectations), enable map_in_namespace.
    if map_in_namespace:
        map_remaps = [
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ]
        relay_node = None
    else:
        map_remaps = [
            ("map", "/map"),
            ("map_metadata", "/map_metadata"),
            ("map_updates", "/map_updates"),
        ]
        # slam_toolbox 的 save_map 内部会在同命名空间下启动 map_saver，并订阅相对话题 "map"（即 /<ns>/map）。
        # 当我们只发布全局 /map 时，为了保持 save_map 可用，需要把 /map 转发到 /<ns>/map。
        relay_node = Node(
            package="homo_multirobot_slam_toolbox",
            executable="occupancy_grid_relay",
            name="map_relay",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time, "input_topic": "/map", "output_topic": "map"}
            ],
        )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(mapper_robot),
                Node(
                    package="slam_toolbox",
                    executable="sync_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    remappings=map_remaps,
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
                *([relay_node] if relay_node is not None else []),
            ]
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("homo_multirobot_slam_toolbox")

    mapper_robot = LaunchConfiguration("mapper_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_in_namespace = LaunchConfiguration("map_in_namespace")

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
        DeclareLaunchArgument(
            "map_in_namespace",
            default_value=TextSubstitution(text="false"),
            description="true: use /<ns>/map (remap /map -> map); false: keep global /map (remap map -> /map).",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_make_slam_toolbox_group)])

