import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _maybe_make_rviz(context, **_kwargs):
    use_rviz = LaunchConfiguration("use_rviz").perform(context).strip().lower() == "true"
    if not use_rviz:
        return []

    namespace = LaunchConfiguration("namespace").perform(context).rstrip("/")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    initialpose_topic = f"{namespace}/initialpose" if namespace else "/initialpose"

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                rviz_config,
                "--ros-args",
                "-r",
                f"/initialpose:={initialpose_topic}",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    map_name = LaunchConfiguration("map_name")
    map_start_x = LaunchConfiguration("map_start_x")
    map_start_y = LaunchConfiguration("map_start_y")
    map_start_yaw = LaunchConfiguration("map_start_yaw")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    global_frame_id = LaunchConfiguration("global_frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    scan_topic = LaunchConfiguration("scan_topic")

    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_nav"), "rviz", "slam_toolbox_loc_single_robot.rviz"]
    )
    default_odom_frame_id = PythonExpression(["'", prefix, "odom'"])
    default_base_frame_id = PythonExpression(["'", prefix, "base_footprint'"])

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("homo_multirobot_nav"), "config", "slam_toolbox_localization.yaml"]
            ),
            {
                "use_sim_time": use_sim_time,
                "map_file_name": PathJoinSubstitution(
                    [FindPackageShare("homo_multirobot_nav"), "maps", map_name]
                ),
                "map_start_pose": PythonExpression([
                    "[", map_start_x, ", ", map_start_y, ", ", map_start_yaw, "]"
                ]),
                "map_frame": global_frame_id,
                "odom_frame": odom_frame_id,
                "base_frame": base_frame_id,
                "scan_topic": scan_topic,
            },
        ],
        remappings=[
            ("map", "/map"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "namespace",
                default_value="/robot1",
                description="robot namespace (e.g. /robot1)",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="robot1_",
                description="TF frame prefix (e.g. robot1_)",
            ),
            DeclareLaunchArgument(
                "map_name",
                default_value="sim_room1",
                description="serialized map name without extension (e.g. sim_room1)",
            ),
            DeclareLaunchArgument(
                "map_start_x",
                default_value="0.0",
                description="initial pose x in map frame",
            ),
            DeclareLaunchArgument(
                "map_start_y",
                default_value="0.0",
                description="initial pose y in map frame",
            ),
            DeclareLaunchArgument(
                "map_start_yaw",
                default_value="0.0",
                description="initial pose yaw in map frame",
            ),
            DeclareLaunchArgument(
                "global_frame_id",
                default_value="map",
                description="global frame for localization",
            ),
            DeclareLaunchArgument(
                "odom_frame_id",
                default_value=default_odom_frame_id,
                description="odom frame (e.g. robot1_odom)",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value=default_base_frame_id,
                description="base frame (e.g. robot1_base_footprint)",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan topic (resolved under namespace)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="launch RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 config file path",
            ),
            slam_toolbox,
            OpaqueFunction(function=_maybe_make_rviz),
        ]
    )
