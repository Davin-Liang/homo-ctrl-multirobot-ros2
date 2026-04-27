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

    # RViz 的 2D Pose Estimate 工具默认发 /initialpose；这里改到 /<ns>/initialpose
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
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    map_yaml = LaunchConfiguration("map")

    global_frame_id = LaunchConfiguration("global_frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")
    scan_topic = LaunchConfiguration("scan_topic")

    autostart = LaunchConfiguration("autostart")

    default_map_yaml = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_slam_toolbox"), "maps", "my_map1.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_nav"), "rviz", "amcl_single_robot.rviz"]
    )
    default_odom_frame_id = PythonExpression(["'", prefix, "odom'"])
    default_base_frame_id = PythonExpression(["'", prefix, "base_footprint'"])

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "yaml_filename": map_yaml},
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "global_frame_id": global_frame_id,
                "odom_frame_id": odom_frame_id,
                "base_frame_id": base_frame_id,
                "scan_topic": scan_topic,
                "tf_broadcast": True,
            }
        ],
        remappings=[
            # map_server is global; make sure AMCL always consumes /map
            ("map", "/map"),
        ],
    )

    # Run lifecycle managers in the same namespace as the nodes they manage.
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
            }
        ],
    )

    lifecycle_manager_amcl = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_amcl",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["amcl"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "namespace",
                default_value="/robot1",
                description="机器人命名空间（例如 /robot1）。",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="robot1_",
                description="TF frame 前缀（例如 robot1_）。用于生成默认 frame。",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_yaml,
                description="地图 YAML 路径（nav2_map_server yaml_filename）。建议使用绝对路径。",
            ),
            DeclareLaunchArgument(
                "global_frame_id",
                default_value="map",
                description="AMCL 全局坐标系（共图固定为 map）。",
            ),
            DeclareLaunchArgument(
                "odom_frame_id",
                default_value=default_odom_frame_id,
                description="AMCL 里程计坐标系（例如 robot1_odom）。",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value=default_base_frame_id,
                description="AMCL 机器人底盘坐标系（例如 robot1_base_footprint）。",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan 话题名（相对名会拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="生命周期节点是否自动激活。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz2（已内置 /initialpose -> <namespace>/initialpose 重映射）。",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 配置文件路径（默认 homo_multirobot_nav/rviz/amcl_single_robot.rviz）。",
            ),
            map_server,
            amcl,
            lifecycle_manager_map,
            lifecycle_manager_amcl,
            OpaqueFunction(function=_maybe_make_rviz),
        ]
    )

