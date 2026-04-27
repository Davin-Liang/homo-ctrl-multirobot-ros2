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
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
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

    default_map_yaml = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_slam_toolbox"), "maps", "my_map1.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_nav"), "rviz", "amcl_two_robots.rviz"]
    )

    default_robot1_odom_frame_id = PythonExpression(["'", robot1_prefix, "odom'"])
    default_robot2_odom_frame_id = PythonExpression(["'", robot2_prefix, "odom'"])
    default_robot1_base_frame_id = PythonExpression(["'", robot1_prefix, "base_footprint'"])
    default_robot2_base_frame_id = PythonExpression(["'", robot2_prefix, "base_footprint'"])

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "yaml_filename": map_yaml},
        ],
    )

    amcl_robot1 = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=robot1_namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "global_frame_id": global_frame_id,
                "odom_frame_id": robot1_odom_frame_id,
                "base_frame_id": robot1_base_frame_id,
                "scan_topic": scan_topic,
                "tf_broadcast": True,
            }
        ],
        remappings=[
            ("map", "/map"),
        ],
    )

    amcl_robot2 = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=robot2_namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "global_frame_id": global_frame_id,
                "odom_frame_id": robot2_odom_frame_id,
                "base_frame_id": robot2_base_frame_id,
                "scan_topic": scan_topic,
                "tf_broadcast": True,
            }
        ],
        remappings=[
            ("map", "/map"),
        ],
    )

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

    lifecycle_manager_robot1 = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_amcl",
        namespace=robot1_namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["amcl"],
            }
        ],
    )

    lifecycle_manager_robot2 = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_amcl",
        namespace=robot2_namespace,
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
                "map",
                default_value=default_map_yaml,
                description="地图 YAML 路径（nav2_map_server yaml_filename）。建议使用绝对路径。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz2（内置 /initialpose -> 指定机器人命名空间 initialpose 的重映射）。",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 配置文件路径（默认 two_robots_sim.rviz，已包含 Map QoS=Transient Local）。",
            ),
            DeclareLaunchArgument(
                "initialpose_to",
                default_value="robot1",
                description="RViz 里 2D Pose Estimate 的目标机器人：robot1 或 robot2。",
            ),
            DeclareLaunchArgument(
                "global_frame_id",
                default_value="map",
                description="AMCL 全局坐标系（共图固定为 map）。",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan 话题名（相对名会拼到各自 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "robot1_namespace",
                default_value="/robot1",
                description="机器人1命名空间（例如 /robot1）。",
            ),
            DeclareLaunchArgument(
                "robot2_namespace",
                default_value="/robot2",
                description="机器人2命名空间（例如 /robot2）。",
            ),
            DeclareLaunchArgument(
                "robot1_prefix",
                default_value="robot1_",
                description="机器人1 TF frame 前缀（例如 robot1_）。用于生成默认 frame。",
            ),
            DeclareLaunchArgument(
                "robot2_prefix",
                default_value="robot2_",
                description="机器人2 TF frame 前缀（例如 robot2_）。用于生成默认 frame。",
            ),
            DeclareLaunchArgument(
                "robot1_odom_frame_id",
                default_value=default_robot1_odom_frame_id,
                description="AMCL robot1 里程计坐标系（例如 robot1_odom）。",
            ),
            DeclareLaunchArgument(
                "robot2_odom_frame_id",
                default_value=default_robot2_odom_frame_id,
                description="AMCL robot2 里程计坐标系（例如 robot2_odom）。",
            ),
            DeclareLaunchArgument(
                "robot1_base_frame_id",
                default_value=default_robot1_base_frame_id,
                description="AMCL robot1 底盘坐标系（例如 robot1_base_footprint）。",
            ),
            DeclareLaunchArgument(
                "robot2_base_frame_id",
                default_value=default_robot2_base_frame_id,
                description="AMCL robot2 底盘坐标系（例如 robot2_base_footprint）。",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="生命周期节点是否自动激活。",
            ),
            map_server,
            amcl_robot1,
            amcl_robot2,
            lifecycle_manager_map,
            lifecycle_manager_robot1,
            lifecycle_manager_robot2,
            OpaqueFunction(function=_maybe_make_rviz),
        ]
    )

