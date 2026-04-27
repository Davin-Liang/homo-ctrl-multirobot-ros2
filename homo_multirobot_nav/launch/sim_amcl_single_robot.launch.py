from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Gazebo(sim) args
    world_name = LaunchConfiguration("world_name")
    gui = LaunchConfiguration("gui")
    server = LaunchConfiguration("server")
    verbose = LaunchConfiguration("verbose")
    software_rendering = LaunchConfiguration("software_rendering")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_world_tf = LaunchConfiguration("publish_world_tf")

    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_prefix = LaunchConfiguration("robot_prefix")

    # EKF frame override
    ekf_yaml_only = LaunchConfiguration("ekf_yaml_only")
    base_link_frame = LaunchConfiguration("base_link_frame")
    odom_frame = LaunchConfiguration("odom_frame")

    # TF/odom conflict strategy
    planar_publish_odom = LaunchConfiguration("planar_publish_odom")
    planar_publish_odom_tf = LaunchConfiguration("planar_publish_odom_tf")
    rf2o_publish_tf = LaunchConfiguration("rf2o_publish_tf")

    # AMCL args
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")

    # Gazebo + localization (rf2o + EKF)
    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_localization"),
                    "launch",
                    "sim_rf2o_ekf_single_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_name": world_name,
            "gui": gui,
            "server": server,
            "verbose": verbose,
            "software_rendering": software_rendering,
            "use_rviz": use_rviz,
            "publish_world_tf": publish_world_tf,
            "robot_namespace": robot_namespace,
            "robot_prefix": robot_prefix,
            "planar_publish_odom": planar_publish_odom,
            "planar_publish_odom_tf": planar_publish_odom_tf,
            "rf2o_publish_tf": rf2o_publish_tf,
            "ekf_yaml_only": ekf_yaml_only,
            "base_link_frame": base_link_frame,
            "odom_frame": odom_frame,
        }.items(),
    )

    # AMCL + map_server
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_nav"),
                    "launch",
                    "amcl_single_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": robot_namespace,
            "prefix": robot_prefix,
            "map_yaml_file": map_yaml_file,
            "params_file": params_file,
            "scan_topic": scan_topic,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="empty.world"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("server", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="false"),
            DeclareLaunchArgument("software_rendering", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("publish_world_tf", default_value="false"),
            DeclareLaunchArgument("robot_namespace", default_value="/robot1"),
            DeclareLaunchArgument("robot_prefix", default_value="robot1_"),
            DeclareLaunchArgument(
                "ekf_yaml_only",
                default_value="false",
                description="false: override EKF frames from launch args.",
            ),
            DeclareLaunchArgument(
                "base_link_frame",
                default_value="robot1_base_footprint",
                description="EKF base_link_frame.",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="robot1_odom",
                description="EKF odom_frame.",
            ),
            DeclareLaunchArgument("planar_publish_odom", default_value="false"),
            DeclareLaunchArgument("planar_publish_odom_tf", default_value="false"),
            DeclareLaunchArgument("rf2o_publish_tf", default_value="false"),
            # AMCL args
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value="",
                description="Path to map YAML file (required).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="AMCL parameter YAML file (defaults to amcl_robot1.yaml).",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="scan",
                description="LaserScan topic (relative to namespace).",
            ),
            loc_launch,
            amcl_launch,
        ]
    )
