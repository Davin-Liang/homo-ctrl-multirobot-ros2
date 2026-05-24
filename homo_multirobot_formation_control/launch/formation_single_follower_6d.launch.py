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
    use_hpc = LaunchConfiguration("use_hpc")

    # Kinematic constraints
    wheel_radius = LaunchConfiguration("wheel_radius")
    base_radius = LaunchConfiguration("base_radius")
    wheel_max_omega = LaunchConfiguration("wheel_max_omega")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_angular_accel = LaunchConfiguration("max_angular_accel")

    # Control rate
    control_rate = LaunchConfiguration("control_rate")

    formation_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_6d",
        name="formation_control_node_6d",
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
            "use_hpc": use_hpc,
            "wheel_radius": wheel_radius,
            "base_radius": base_radius,
            "wheel_max_omega": wheel_max_omega,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "control_rate": control_rate,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1"),
        DeclareLaunchArgument("follower_ns", default_value="/robot2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("radius", default_value="2.0",
                              description="Formation safety circle radius (m)"),
        DeclareLaunchArgument("mass", default_value="8.0",
                              description="Mass tuning parameter"),
        DeclareLaunchArgument("I", default_value="1.0",
                              description="Inertia tuning parameter (yaw channel)"),
        DeclareLaunchArgument("omega_d", default_value="1.5",
                              description="Position channel desired bandwidth"),
        DeclareLaunchArgument("omega_d_theta", default_value="1.5",
                              description="Yaw channel desired bandwidth"),
        DeclareLaunchArgument("hpc_vel_threshold", default_value="0.3",
                              description="Leader velocity change threshold for HPC recompute"),
        DeclareLaunchArgument("use_hpc", default_value="true",
                              description="Enable homogeneous upgrade (false = pure LPC)"),
        DeclareLaunchArgument("wheel_radius", default_value="0.03"),
        DeclareLaunchArgument("base_radius", default_value="0.11"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0"),
        DeclareLaunchArgument("max_linear_accel", default_value="2.0"),
        DeclareLaunchArgument("max_angular_accel", default_value="4.0"),
        DeclareLaunchArgument("control_rate", default_value="20.0"),
        formation_node,
    ])
