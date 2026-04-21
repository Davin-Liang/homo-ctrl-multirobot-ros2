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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_gazebo"),
                    "launch",
                    "sim_two_robots.launch.py",
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
        }.items(),
    )

    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_localization"),
                    "launch",
                    "rf2o_two_robots.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot1_namespace": robot1_namespace,
            "robot2_namespace": robot2_namespace,
            "rf2o_publish_tf": rf2o_publish_tf,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("homo_multirobot_localization"),
                    "launch",
                    "ekf_two_robots.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot1_namespace": robot1_namespace,
            "robot2_namespace": robot2_namespace,
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
            # Default: EKF is the ONLY source of odom->base TF
            DeclareLaunchArgument("planar_publish_odom", default_value="false"),
            DeclareLaunchArgument("planar_publish_odom_tf", default_value="false"),
            DeclareLaunchArgument("rf2o_publish_tf", default_value="false"),
            gazebo_launch,
            rf2o_launch,
            ekf_launch,
        ]
    )

