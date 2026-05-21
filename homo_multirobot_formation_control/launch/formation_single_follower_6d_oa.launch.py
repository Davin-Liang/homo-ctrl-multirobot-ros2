from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower_ns = LaunchConfiguration("follower_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Formation geometry
    radius = LaunchConfiguration("radius")

    # Robot dynamics
    mass = LaunchConfiguration("mass")
    I = LaunchConfiguration("I")
    omega_d = LaunchConfiguration("omega_d")
    omega_d_theta = LaunchConfiguration("omega_d_theta")
    hpc_vel_threshold = LaunchConfiguration("hpc_vel_threshold")

    # Kinematic constraints
    wheel_radius = LaunchConfiguration("wheel_radius")
    base_radius = LaunchConfiguration("base_radius")
    wheel_max_omega = LaunchConfiguration("wheel_max_omega")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_angular_accel = LaunchConfiguration("max_angular_accel")

    # Control rate
    control_rate = LaunchConfiguration("control_rate")

    # Obstacle avoidance
    scan_topic = LaunchConfiguration("scan_topic")
    safety_distance = LaunchConfiguration("safety_distance")
    obstacle_weight = LaunchConfiguration("obstacle_weight")
    time_horizon = LaunchConfiguration("time_horizon")
    max_obstacles = LaunchConfiguration("max_obstacles")
    cluster_tolerance = LaunchConfiguration("cluster_tolerance")
    min_cluster_size = LaunchConfiguration("min_cluster_size")

    formation_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_6d_oa",
        name="formation_control_node_6d_oa",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        parameters=[{
            "leader_ns": leader_ns,
            "follower_ns": follower_ns,
            "use_sim_time": use_sim_time,
            "radius": radius,
            "mass": mass,
            "I": I,
            "omega_d": omega_d,
            "omega_d_theta": omega_d_theta,
            "hpc_vel_threshold": hpc_vel_threshold,
            "wheel_radius": wheel_radius,
            "base_radius": base_radius,
            "wheel_max_omega": wheel_max_omega,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "control_rate": control_rate,
            # obstacle avoidance
            "scan_topic": scan_topic,
            "safety_distance": safety_distance,
            "obstacle_weight": obstacle_weight,
            "time_horizon": time_horizon,
            "max_obstacles": max_obstacles,
            "cluster_tolerance": cluster_tolerance,
            "min_cluster_size": min_cluster_size,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1"),
        DeclareLaunchArgument("follower_ns", default_value="/robot2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("radius", default_value="2.0",
                              description="Formation safety circle radius (m)"),
        DeclareLaunchArgument("mass", default_value="8.0"),
        DeclareLaunchArgument("I", default_value="1.0"),
        DeclareLaunchArgument("omega_d", default_value="1.5"),
        DeclareLaunchArgument("omega_d_theta", default_value="1.5"),
        DeclareLaunchArgument("hpc_vel_threshold", default_value="0.3"),
        DeclareLaunchArgument("wheel_radius", default_value="0.03"),
        DeclareLaunchArgument("base_radius", default_value="0.11"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0"),
        DeclareLaunchArgument("max_linear_accel", default_value="2.0"),
        DeclareLaunchArgument("max_angular_accel", default_value="4.0"),
        DeclareLaunchArgument("control_rate", default_value="20.0"),
        # obstacle avoidance
        DeclareLaunchArgument("scan_topic", default_value="scan",
                              description="LaserScan topic (relative to follower ns)"),
        DeclareLaunchArgument("safety_distance", default_value="0.5",
                              description="Min safe distance to obstacle (m)"),
        DeclareLaunchArgument("obstacle_weight", default_value="1.0",
                              description="Obstacle avoidance cost weight"),
        DeclareLaunchArgument("time_horizon", default_value="0.5",
                              description="Collision prediction horizon (s)"),
        DeclareLaunchArgument("max_obstacles", default_value="10",
                              description="Max obstacle count"),
        DeclareLaunchArgument("cluster_tolerance", default_value="0.1",
                              description="Euclidean clustering distance threshold (m)"),
        DeclareLaunchArgument("min_cluster_size", default_value="5",
                              description="Min points per cluster"),
        formation_node,
    ])
