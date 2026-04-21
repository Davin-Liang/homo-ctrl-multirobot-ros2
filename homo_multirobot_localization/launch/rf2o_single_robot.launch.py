from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    publish_tf = LaunchConfiguration("publish_tf")
    freq = LaunchConfiguration("freq")

    base_frame_id = LaunchConfiguration("base_frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")

    default_base_frame_id = ParameterValue(
        PythonExpression(["'", prefix, "base_footprint'"]),
        value_type=str,
    )
    default_odom_frame_id = ParameterValue(
        PythonExpression(["'", prefix, "odom'"]),
        value_type=str,
    )

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
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
                "base_frame_id": base_frame_id,
                "odom_frame_id": odom_frame_id,
                "init_pose_from_topic": "",
                "freq": ParameterValue(freq, value_type=float),
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
                description="TF frame 前缀（例如 robot1_）。用于生成默认 base/odom frame。",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan 输入话题名（相对名会自动拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="rf2o/odom",
                description="Odometry 输出话题名（相对名会自动拼到 namespace 下）。",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="是否由 rf2o 发布 TF（odom->base）。建议与 EKF 二选一，避免 TF 冲突。",
            ),
            DeclareLaunchArgument(
                "freq",
                default_value="20.0",
                description="rf2o 处理频率（Hz）。建议不高于激光帧率太多。",
            ),
            DeclareLaunchArgument(
                "base_frame_id",
                default_value=default_base_frame_id,
                description="底盘基座 frame（默认 <prefix>base_footprint）。",
            ),
            DeclareLaunchArgument(
                "odom_frame_id",
                default_value=default_odom_frame_id,
                description="里程计 frame（默认 <prefix>odom）。",
            ),
            rf2o_node,
        ]
    )

