from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_ekf_node(context, **_kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)

    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).strip().lower() == "true"
    )
    ekf_yaml_only = (
        LaunchConfiguration("ekf_yaml_only").perform(context).strip().lower() == "true"
    )

    params = [
        LaunchConfiguration("config_file").perform(context),
        {"use_sim_time": use_sim_time},
    ]

    if not ekf_yaml_only:
        params.append(
            {
                "map_frame": LaunchConfiguration("map_frame").perform(context),
                "odom_frame": LaunchConfiguration("odom_frame").perform(context),
                "base_link_frame": LaunchConfiguration("base_link_frame").perform(context),
                "world_frame": LaunchConfiguration("world_frame").perform(context),
                "odom0": LaunchConfiguration("odom_topic").perform(context),
                "imu0": LaunchConfiguration("imu_topic").perform(context),
                "frequency": float(LaunchConfiguration("ekf_frequency").perform(context)),
                "sensor_timeout": float(
                    LaunchConfiguration("ekf_sensor_timeout").perform(context)
                ),
                "publish_tf": LaunchConfiguration("ekf_publish_tf").perform(context)
                .strip()
                .lower()
                == "true",
            }
        )

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            namespace=namespace,
            output="screen",
            parameters=params,
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    # rf2o args
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    rf2o_publish_tf = LaunchConfiguration("rf2o_publish_tf")
    rf2o_freq = LaunchConfiguration("rf2o_freq")
    base_frame_id = LaunchConfiguration("base_frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")

    # ekf args
    config_file = LaunchConfiguration("config_file")
    imu_topic = LaunchConfiguration("imu_topic")
    ekf_publish_tf = LaunchConfiguration("ekf_publish_tf")
    ekf_frequency = LaunchConfiguration("ekf_frequency")
    ekf_sensor_timeout = LaunchConfiguration("ekf_sensor_timeout")
    ekf_yaml_only = LaunchConfiguration("ekf_yaml_only")
    base_link_frame = LaunchConfiguration("base_link_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    world_frame = LaunchConfiguration("world_frame")

    pkg_share = FindPackageShare("homo_multirobot_localization")

    default_base_frame_id = PythonExpression(["'", prefix, "base_footprint'"])
    default_odom_frame_id = PythonExpression(["'", prefix, "odom'"])
    default_base_link_frame = PythonExpression(["'", prefix, "base_footprint'"])
    default_odom_frame = PythonExpression(["'", prefix, "odom'"])
    default_map_frame = PythonExpression(["'", prefix, "map'"])
    default_config_file = PathJoinSubstitution([pkg_share, "config", "ekf_single_robot.yaml"])

    rf2o_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "laser_scan_topic": scan_topic,
                "odom_topic": odom_topic,
                "publish_tf": ParameterValue(rf2o_publish_tf, value_type=bool),
                "base_frame_id": base_frame_id,
                "odom_frame_id": odom_frame_id,
                "init_pose_from_topic": "",
                "freq": ParameterValue(rf2o_freq, value_type=float),
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="实机一般为 false；仿真可设为 true。",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="节点所在命名空间（例如 /robot1）。留空表示根命名空间。",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="",
                description="TF frame 前缀（例如 robot1_）。用于生成默认 frame。",
            ),
            # rf2o passthrough
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan 输入话题（相对名会拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="rf2o/odom",
                description="rf2o 输出 odom 话题（同时作为 EKF 的 odom 输入；相对名会拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "rf2o_publish_tf",
                default_value="false",
                description="是否由 rf2o 发布 TF（建议 false，由 EKF 统一发布 TF）。",
            ),
            DeclareLaunchArgument(
                "rf2o_freq",
                default_value="20.0",
                description="rf2o 处理频率（Hz）。建议不高于激光帧率太多。",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value=default_base_frame_id,
                description="rf2o base_frame_id（默认 <prefix>base_footprint）。",
            ),
            DeclareLaunchArgument(
                "odom_frame_id",
                default_value=default_odom_frame_id,
                description="rf2o odom_frame_id（默认 <prefix>odom）。",
            ),
            # ekf passthrough
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="EKF 参数文件路径（YAML）。",
            ),
            DeclareLaunchArgument(
                "ekf_yaml_only",
                default_value="true",
                description="true 时 EKF 完全以 config_file(YAML) 为准；false 时允许用命令行参数覆盖 frame/topic/frequency/publish_tf 等。",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="imu",
                description="IMU 输入话题（相对名会拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "ekf_publish_tf",
                default_value="true",
                description="是否由 EKF 发布 TF（odom->base）。推荐 true。",
            ),
            DeclareLaunchArgument(
                "ekf_frequency",
                default_value="30.0",
                description="EKF 输出频率（Hz）。",
            ),
            DeclareLaunchArgument(
                "ekf_sensor_timeout",
                default_value="0.2",
                description="EKF 输入超时时间（秒）。",
            ),
            DeclareLaunchArgument(
                "base_link_frame",
                default_value=default_base_link_frame,
                description="EKF base_link_frame（默认 <prefix>base_footprint）。",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value=default_odom_frame,
                description="EKF odom_frame（默认 <prefix>odom）。",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value=default_map_frame,
                description="EKF map_frame（默认 <prefix>map）。",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value=default_odom_frame,
                description="EKF world_frame（默认 <prefix>odom；2D 里程计常用）。",
            ),
            rf2o_node,
            OpaqueFunction(function=_make_ekf_node),
        ]
    )

