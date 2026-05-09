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

    initialpose_to = LaunchConfiguration("initialpose_to").perform(context).strip().lower()
    if initialpose_to not in ("robot1", "robot2"):
        initialpose_to = "robot1"

    ns_key = "robot1_namespace" if initialpose_to == "robot1" else "robot2_namespace"
    namespace = LaunchConfiguration(ns_key).perform(context).rstrip("/")
    initialpose_topic = f"{namespace}/initialpose" if namespace else "/initialpose"

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
    map_name = LaunchConfiguration("map_name")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    initialpose_to = LaunchConfiguration("initialpose_to")

    robot1_namespace = LaunchConfiguration("robot1_namespace")
    robot2_namespace = LaunchConfiguration("robot2_namespace")
    robot1_prefix = LaunchConfiguration("robot1_prefix")
    robot2_prefix = LaunchConfiguration("robot2_prefix")

    global_frame_id = LaunchConfiguration("global_frame_id")
    robot1_odom_frame_id = LaunchConfiguration("robot1_odom_frame_id")
    robot2_odom_frame_id = LaunchConfiguration("robot2_odom_frame_id")
    robot1_base_frame_id = LaunchConfiguration("robot1_base_frame_id")
    robot2_base_frame_id = LaunchConfiguration("robot2_base_frame_id")
    scan_topic = LaunchConfiguration("scan_topic")

    robot1_map_start_x = LaunchConfiguration("robot1_map_start_x")
    robot1_map_start_y = LaunchConfiguration("robot1_map_start_y")
    robot1_map_start_yaw = LaunchConfiguration("robot1_map_start_yaw")
    robot2_map_start_x = LaunchConfiguration("robot2_map_start_x")
    robot2_map_start_y = LaunchConfiguration("robot2_map_start_y")
    robot2_map_start_yaw = LaunchConfiguration("robot2_map_start_yaw")

    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_nav"), "rviz", "slam_toolbox_loc_two_robots.rviz"]
    )
    default_robot1_odom_frame_id = PythonExpression(["'", robot1_prefix, "odom'"])
    default_robot2_odom_frame_id = PythonExpression(["'", robot2_prefix, "odom'"])
    default_robot1_base_frame_id = PythonExpression(["'", robot1_prefix, "base_footprint'"])
    default_robot2_base_frame_id = PythonExpression(["'", robot2_prefix, "base_footprint'"])

    slam_params_yaml = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_nav"), "config", "slam_toolbox_localization.yaml"]
    )
    map_file_name = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_nav"), "maps", map_name]
    )

    slam_toolbox_robot1 = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        namespace=robot1_namespace,
        output="screen",
        parameters=[
            slam_params_yaml,
            {
                "use_sim_time": use_sim_time,
                "map_file_name": map_file_name,
                "map_start_pose": PythonExpression([
                    "[", robot1_map_start_x, ", ", robot1_map_start_y, ", ",
                    robot1_map_start_yaw, "]"
                ]),
                "map_frame": global_frame_id,
                "odom_frame": robot1_odom_frame_id,
                "base_frame": robot1_base_frame_id,
                "scan_topic": scan_topic,
            },
        ],
        remappings=[
            ("map", "/map"),
        ],
    )

    slam_toolbox_robot2 = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        namespace=robot2_namespace,
        output="screen",
        parameters=[
            slam_params_yaml,
            {
                "use_sim_time": use_sim_time,
                "map_file_name": map_file_name,
                "map_start_pose": PythonExpression([
                    "[", robot2_map_start_x, ", ", robot2_map_start_y, ", ",
                    robot2_map_start_yaw, "]"
                ]),
                "map_frame": global_frame_id,
                "odom_frame": robot2_odom_frame_id,
                "base_frame": robot2_base_frame_id,
                "scan_topic": scan_topic,
            },
        ],
        remappings=[
            ("map", "/robot2/dummy_map"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map_name",
                default_value="sim_room1",
                description="serialized map name without extension (e.g. sim_room1)",
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
            DeclareLaunchArgument(
                "initialpose_to",
                default_value="robot1",
                description="which robot receives /initialpose via RViz: robot1 or robot2",
            ),
            DeclareLaunchArgument(
                "global_frame_id",
                default_value="map",
                description="global frame for localization",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan topic (resolved under each namespace)",
            ),
            DeclareLaunchArgument(
                "robot1_namespace",
                default_value="/robot1",
                description="robot1 namespace",
            ),
            DeclareLaunchArgument(
                "robot2_namespace",
                default_value="/robot2",
                description="robot2 namespace",
            ),
            DeclareLaunchArgument(
                "robot1_prefix",
                default_value="robot1_",
                description="robot1 TF frame prefix",
            ),
            DeclareLaunchArgument(
                "robot2_prefix",
                default_value="robot2_",
                description="robot2 TF frame prefix",
            ),
            DeclareLaunchArgument(
                "robot1_odom_frame_id",
                default_value=default_robot1_odom_frame_id,
                description="robot1 odom frame",
            ),
            DeclareLaunchArgument(
                "robot2_odom_frame_id",
                default_value=default_robot2_odom_frame_id,
                description="robot2 odom frame",
            ),
            DeclareLaunchArgument(
                "robot1_base_frame_id",
                default_value=default_robot1_base_frame_id,
                description="robot1 base frame",
            ),
            DeclareLaunchArgument(
                "robot2_base_frame_id",
                default_value=default_robot2_base_frame_id,
                description="robot2 base frame",
            ),
            DeclareLaunchArgument(
                "robot1_map_start_x",
                default_value="0.0",
                description="robot1 initial pose x in map frame",
            ),
            DeclareLaunchArgument(
                "robot1_map_start_y",
                default_value="0.0",
                description="robot1 initial pose y in map frame",
            ),
            DeclareLaunchArgument(
                "robot1_map_start_yaw",
                default_value="0.0",
                description="robot1 initial pose yaw in map frame",
            ),
            DeclareLaunchArgument(
                "robot2_map_start_x",
                default_value="0.0",
                description="robot2 initial pose x in map frame",
            ),
            DeclareLaunchArgument(
                "robot2_map_start_y",
                default_value="0.0",
                description="robot2 initial pose y in map frame",
            ),
            DeclareLaunchArgument(
                "robot2_map_start_yaw",
                default_value="0.0",
                description="robot2 initial pose yaw in map frame",
            ),
            slam_toolbox_robot1,
            slam_toolbox_robot2,
            OpaqueFunction(function=_maybe_make_rviz),
        ]
    )
