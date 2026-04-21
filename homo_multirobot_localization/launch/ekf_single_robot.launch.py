from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    config_file = LaunchConfiguration("config_file")

    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    publish_tf = LaunchConfiguration("publish_tf")
    frequency = LaunchConfiguration("frequency")
    sensor_timeout = LaunchConfiguration("sensor_timeout")

    base_link_frame = LaunchConfiguration("base_link_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    world_frame = LaunchConfiguration("world_frame")

    default_base_link_frame = PythonExpression(["'", prefix, "base_footprint'"])
    default_odom_frame = PythonExpression(["'", prefix, "odom'"])
    default_map_frame = PythonExpression(["'", prefix, "map'"])

    default_config_file = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_localization"), "config", "ekf_single_robot.yaml"]
    )

    # 2D IMU + laser-odom fusion. YAML defines fusion + topic defaults;
    # launch args can override per-robot frames/topics.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=namespace,
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_link_frame": base_link_frame,
                "world_frame": world_frame,
                "odom0": odom_topic,
                "imu0": imu_topic,
                "frequency": ParameterValue(frequency, value_type=float),
                "sensor_timeout": ParameterValue(sensor_timeout, value_type=float),
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
            }
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
                description="TF frame 前缀（例如 robot1_）。用于生成默认 base/odom/map frame。",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="EKF 参数文件路径（YAML）。默认使用本包内置 ekf_single_robot.yaml。",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="imu",
                description="IMU 输入话题名（相对名会自动拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="rf2o/odom",
                description="里程计输入话题名（通常为 rf2o 输出；相对名会自动拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="是否由 EKF 发布 TF（odom->base）。建议让 EKF 成为唯一 TF 来源。",
            ),
            DeclareLaunchArgument(
                "frequency",
                default_value="30.0",
                description="EKF 输出频率（Hz）。建议 >= 输入频率，且不要太高。",
            ),
            DeclareLaunchArgument(
                "sensor_timeout",
                default_value="0.2",
                description="输入超时时间（秒）。",
            ),
            DeclareLaunchArgument(
                "base_link_frame",
                default_value=default_base_link_frame,
                description="底盘基座 frame（默认 <prefix>base_footprint）。",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value=default_odom_frame,
                description="里程计 frame（默认 <prefix>odom）。",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value=default_map_frame,
                description="全局 frame（默认 <prefix>map）。若无全局定位，可保持默认但不使用 map->odom。",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value=default_odom_frame,
                description="robot_localization world_frame（2D 里程计场景通常设为 odom_frame）。",
            ),
            ekf_node,
        ]
    )

