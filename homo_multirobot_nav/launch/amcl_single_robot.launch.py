from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")

    pkg_share = FindPackageShare("homo_multirobot_nav")

    default_params_file = PathJoinSubstitution([pkg_share, "config", "amcl_robot1.yaml"])

    # Build frame names from prefix
    base_frame = PythonExpression(["'", prefix, "base_footprint'"])
    odom_frame = PythonExpression(["'", prefix, "odom'"])

    # map_server: global node (no namespace), publishes /map
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_file},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Publish static transforms on /tf with current timestamps (backup)
    tf_bridge_node = Node(
        package="homo_multirobot_nav",
        executable="tf_static_bridge",
        name="tf_static_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Pre-transform laser scans: laser_link -> base_footprint
    # This bypasses AMCL's internal TF lookup which is broken for
    # lifecycle nodes in ROS2 Humble (known issue).
    scan_xform = Node(
        package="homo_multirobot_nav",
        executable="scan_transformer",
        name="scan_transformer",
        namespace=namespace,
        output="screen",
        parameters=[
            {"base_frame": base_frame},
            {"scan_in": scan_topic},
            {"scan_out": "scan_transformed"},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Python AMCL (regular node — avoids lifecycle+TF2 bug in nav2_amcl)
    amcl_node = Node(
        package="homo_multirobot_nav",
        executable="python_amcl",
        name="amcl",
        namespace=namespace,
        output="screen",
        parameters=[
            {"base_frame_id": base_frame},
            {"odom_frame_id": odom_frame},
            {"global_frame_id": "map"},
            {"min_particles": 300},
            {"use_sim_time": use_sim_time},
            params_file,
        ],
        remappings=[
            ("scan", "scan_transformed"),
        ],
    )

    # Lifecycle manager for map_server
    lifecycle_manager_root = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {"node_names": ["map_server"]},
            {"autostart": True},
            {"use_sim_time": use_sim_time},
        ],
    )

    # No lifecycle_manager needed: python_amcl is a regular node

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="/robot1",
                description="Robot namespace (e.g. /robot1).",
            ),
            DeclareLaunchArgument(
                "prefix",
                default_value="robot1_",
                description="Robot TF frame prefix (e.g. robot1_).",
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value="",
                description="Path to map YAML file (required).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="AMCL parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan topic (relative to namespace).",
            ),
            map_server_node,
            tf_bridge_node,
            scan_xform,
            amcl_node,
            TimerAction(period=8.0, actions=[lifecycle_manager_root]),
        ]
    )
