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
    omega_d = LaunchConfiguration("omega_d")

    # Yaw control
    Kp_yaw = LaunchConfiguration("Kp_yaw")
    K_ff = LaunchConfiguration("K_ff")

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
        executable="formation_control_node_4d_cont",
        name="formation_control_node_4d_cont",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        parameters=[{
            "leader_ns": leader_ns,
            "follower_ns": follower_ns,
            "use_sim_time": use_sim_time,
            "radius": radius,
            "mass": mass,
            "omega_d": omega_d,
            "Kp_yaw": Kp_yaw,
            "K_ff": K_ff,
            "wheel_radius": wheel_radius,
            "base_radius": base_radius,
            "wheel_max_omega": wheel_max_omega,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "control_rate": control_rate,
            "use_hpc": LaunchConfiguration("use_hpc"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1",
                              description="Leader robot namespace"),
        DeclareLaunchArgument("follower_ns", default_value="/robot2",
                              description="Follower robot namespace"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation time"),
        DeclareLaunchArgument("radius", default_value="2.0",
                              description="Formation safety circle radius (m)"),
        DeclareLaunchArgument("mass", default_value="8.0",
                              description="Controller model mass (tuning, not physical)"),
        DeclareLaunchArgument("omega_d", default_value="1.5",
                              description="Desired damping bandwidth"),
        DeclareLaunchArgument("Kp_yaw", default_value="4.0",
                              description="Proportional yaw gain"),
        DeclareLaunchArgument("K_ff", default_value="1.0",
                              description="Feedforward yaw gain"),
        DeclareLaunchArgument("control_rate", default_value="20.0",
                              description="Control loop frequency (Hz)"),
        DeclareLaunchArgument("use_hpc", default_value="true",
                              description="Enable homogeneous upgrade (false = pure LPC)"),
        DeclareLaunchArgument("wheel_radius", default_value="0.03",
                              description="Wheel rolling radius (m)"),
        DeclareLaunchArgument("base_radius", default_value="0.11",
                              description="Distance from robot center to wheel (m)"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0",
                              description="Max wheel angular velocity (rad/s)"),
        DeclareLaunchArgument("max_linear_accel", default_value="2.0",
                              description="Max body linear acceleration (m/s^2)"),
        DeclareLaunchArgument("max_angular_accel", default_value="4.0",
                              description="Max body angular acceleration (rad/s^2)"),
        formation_node,
    ])
