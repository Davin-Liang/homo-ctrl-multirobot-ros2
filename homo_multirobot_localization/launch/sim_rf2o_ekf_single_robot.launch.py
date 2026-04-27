from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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

    # EKF frame override (ensure odom->base TF matches <prefix> convention)
    ekf_yaml_only = LaunchConfiguration("ekf_yaml_only")
    base_link_frame = LaunchConfiguration("base_link_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    world_frame = LaunchConfiguration("world_frame")

    # TF/odom conflict strategy
    planar_publish_odom = LaunchConfiguration("planar_publish_odom")
    planar_publish_odom_tf = LaunchConfiguration("planar_publish_odom_tf")
    rf2o_publish_tf = LaunchConfiguration("rf2o_publish_tf")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_gazebo"),
                    "launch",
                    "sim_single_robot.launch.py",
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
        }.items(),
    )

    rf2o_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_localization"),
                    "launch",
                    "rf2o_ekf_single_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": robot_namespace,
            "prefix": robot_prefix,
            "rf2o_publish_tf": rf2o_publish_tf,
            # Important: do NOT rely on ekf_single_robot.yaml for frames in sim;
            # we must align EKF TF with <prefix>odom -> <prefix>base_footprint.
            "ekf_yaml_only": ekf_yaml_only,
            "base_link_frame": base_link_frame,
            "odom_frame": odom_frame,
            "map_frame": map_frame,
            "world_frame": world_frame,
        }.items(),
    )

    default_base_link_frame = PythonExpression(["'", robot_prefix, "base_footprint'"])
    default_odom_frame = PythonExpression(["'", robot_prefix, "odom'"])
    # For pure odom fusion, keep world_frame == odom_frame (robot_localization convention).
    default_map_frame = PythonExpression(["'", robot_prefix, "map'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="test_world.world"),
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
                description="false: 用 launch 参数覆盖 EKF frame/topic（仿真推荐，避免默认 frame=odom/base_link 导致 TF 断链）。",
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
                description="EKF map_frame（默认 <prefix>map；纯里程计融合时基本不使用 map->odom）。",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value=default_odom_frame,
                description="EKF world_frame（默认 <prefix>odom）。",
            ),
            # Default: EKF is the ONLY source of odom->base TF
            DeclareLaunchArgument("planar_publish_odom", default_value="false"),
            DeclareLaunchArgument("planar_publish_odom_tf", default_value="false"),
            DeclareLaunchArgument("rf2o_publish_tf", default_value="false"),
            gazebo_launch,
            rf2o_ekf_launch,
        ]
    )

