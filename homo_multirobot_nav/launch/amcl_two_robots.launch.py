from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")

    pkg_share = FindPackageShare("homo_multirobot_nav")

    params_file_robot1 = LaunchConfiguration("params_file_robot1")
    params_file_robot2 = LaunchConfiguration("params_file_robot2")
    scan_topic = LaunchConfiguration("scan_topic")

    default_params_file_robot1 = PathJoinSubstitution(
        [pkg_share, "config", "amcl_robot1.yaml"]
    )
    default_params_file_robot2 = PathJoinSubstitution(
        [pkg_share, "config", "amcl_robot2.yaml"]
    )

    # Publish static transforms on /tf with current timestamps
    tf_bridge_node = Node(
        package="homo_multirobot_nav",
        executable="tf_static_bridge",
        name="tf_static_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

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

    # Pre-transform scans for robot1
    scan_xform1 = Node(
        package="homo_multirobot_nav",
        executable="scan_transformer",
        name="scan_transformer",
        namespace="/robot1",
        output="screen",
        parameters=[
            {"base_frame": "robot1_base_footprint"},
            {"scan_in": scan_topic},
            {"scan_out": "scan_transformed"},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Pre-transform scans for robot2
    scan_xform2 = Node(
        package="homo_multirobot_nav",
        executable="scan_transformer",
        name="scan_transformer",
        namespace="/robot2",
        output="screen",
        parameters=[
            {"base_frame": "robot2_base_footprint"},
            {"scan_in": scan_topic},
            {"scan_out": "scan_transformed"},
            {"use_sim_time": use_sim_time},
        ],
    )

    amcl_robot1 = Node(
        package="homo_multirobot_nav",
        executable="python_amcl",
        name="amcl",
        namespace="/robot1",
        output="screen",
        parameters=[
            {"base_frame_id": "robot1_base_footprint"},
            {"odom_frame_id": "robot1_odom"},
            {"global_frame_id": "map"},
            {"min_particles": 300},
            {"use_sim_time": use_sim_time},
            params_file_robot1,
        ],
        remappings=[
            ("scan", "scan_transformed"),
        ],
    )

    amcl_robot2 = Node(
        package="homo_multirobot_nav",
        executable="python_amcl",
        name="amcl",
        namespace="/robot2",
        output="screen",
        parameters=[
            {"base_frame_id": "robot2_base_footprint"},
            {"odom_frame_id": "robot2_odom"},
            {"global_frame_id": "map"},
            {"min_particles": 300},
            {"use_sim_time": use_sim_time},
            params_file_robot2,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock.",
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value="",
                description="Path to map YAML file (required).",
            ),
            DeclareLaunchArgument(
                "params_file_robot1",
                default_value=default_params_file_robot1,
                description="AMCL parameter YAML for robot1.",
            ),
            DeclareLaunchArgument(
                "params_file_robot2",
                default_value=default_params_file_robot2,
                description="AMCL parameter YAML for robot2.",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan topic (relative to namespace).",
            ),
            map_server_node,
            tf_bridge_node,
            scan_xform1,
            scan_xform2,
            amcl_robot1,
            amcl_robot2,
            TimerAction(period=8.0, actions=[lifecycle_manager_root]),
        ]
    )
