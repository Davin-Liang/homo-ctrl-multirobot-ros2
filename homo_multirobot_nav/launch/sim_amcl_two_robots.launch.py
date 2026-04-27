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

    robot1_namespace = LaunchConfiguration("robot1_namespace")
    robot2_namespace = LaunchConfiguration("robot2_namespace")
    robot1_prefix = LaunchConfiguration("robot1_prefix")
    robot2_prefix = LaunchConfiguration("robot2_prefix")

    # TF/odom conflict strategy
    planar_publish_odom = LaunchConfiguration("planar_publish_odom")
    planar_publish_odom_tf = LaunchConfiguration("planar_publish_odom_tf")
    rf2o_publish_tf = LaunchConfiguration("rf2o_publish_tf")

    # AMCL args
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    params_file_robot1 = LaunchConfiguration("params_file_robot1")
    params_file_robot2 = LaunchConfiguration("params_file_robot2")
    scan_topic = LaunchConfiguration("scan_topic")

    # Gazebo + localization (rf2o + EKF) for two robots
    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_localization"),
                    "launch",
                    "sim_rf2o_ekf_two_robots.launch.py",
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
            "robot1_namespace": robot1_namespace,
            "robot2_namespace": robot2_namespace,
            "robot1_prefix": robot1_prefix,
            "robot2_prefix": robot2_prefix,
            "planar_publish_odom": planar_publish_odom,
            "planar_publish_odom_tf": planar_publish_odom_tf,
            "rf2o_publish_tf": rf2o_publish_tf,
        }.items(),
    )

    # AMCL + map_server for two robots
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_nav"),
                    "launch",
                    "amcl_two_robots.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_yaml_file": map_yaml_file,
            "params_file_robot1": params_file_robot1,
            "params_file_robot2": params_file_robot2,
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
            DeclareLaunchArgument("robot1_namespace", default_value="/robot1"),
            DeclareLaunchArgument("robot2_namespace", default_value="/robot2"),
            DeclareLaunchArgument("robot1_prefix", default_value="robot1_"),
            DeclareLaunchArgument("robot2_prefix", default_value="robot2_"),
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
                "params_file_robot1",
                default_value="",
                description="AMCL parameter YAML for robot1 (defaults to amcl_robot1.yaml).",
            ),
            DeclareLaunchArgument(
                "params_file_robot2",
                default_value="",
                description="AMCL parameter YAML for robot2 (defaults to amcl_robot2.yaml).",
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
