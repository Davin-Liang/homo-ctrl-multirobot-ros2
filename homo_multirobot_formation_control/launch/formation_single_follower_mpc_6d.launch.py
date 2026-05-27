from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower_ns = LaunchConfiguration("follower_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Formation
    formation_radius = LaunchConfiguration("formation_radius")
    formation_offset_x = LaunchConfiguration("formation_offset_x")
    formation_offset_y = LaunchConfiguration("formation_offset_y")

    # MPC tuning
    mpc_horizon = LaunchConfiguration("mpc_horizon")
    mpc_q_px = LaunchConfiguration("mpc_q_px")
    mpc_q_py = LaunchConfiguration("mpc_q_py")
    mpc_q_theta = LaunchConfiguration("mpc_q_theta")
    mpc_q_vx = LaunchConfiguration("mpc_q_vx")
    mpc_q_vy = LaunchConfiguration("mpc_q_vy")
    mpc_q_omega = LaunchConfiguration("mpc_q_omega")
    mpc_r_ax = LaunchConfiguration("mpc_r_ax")
    mpc_r_ay = LaunchConfiguration("mpc_r_ay")
    mpc_r_alpha = LaunchConfiguration("mpc_r_alpha")
    mpc_terminal_factor = LaunchConfiguration("mpc_terminal_factor")

    # Limits
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_angular_accel = LaunchConfiguration("max_angular_accel")
    max_linear_vel = LaunchConfiguration("max_linear_vel")
    max_angular_vel = LaunchConfiguration("max_angular_vel")

    # Kinematics
    wheel_radius = LaunchConfiguration("wheel_radius")
    base_radius = LaunchConfiguration("base_radius")
    wheel_max_omega = LaunchConfiguration("wheel_max_omega")
    control_rate = LaunchConfiguration("control_rate")

    formation_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_mpc_6d",
        name="formation_control_node_mpc_6d",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        parameters=[{
            "leader_ns": leader_ns,
            "follower_ns": follower_ns,
            "use_sim_time": use_sim_time,
            "formation_radius": formation_radius,
            "formation_offset_x": formation_offset_x,
            "formation_offset_y": formation_offset_y,
            "mpc_horizon": mpc_horizon,
            "mpc_q_px": mpc_q_px,
            "mpc_q_py": mpc_q_py,
            "mpc_q_theta": mpc_q_theta,
            "mpc_q_vx": mpc_q_vx,
            "mpc_q_vy": mpc_q_vy,
            "mpc_q_omega": mpc_q_omega,
            "mpc_r_ax": mpc_r_ax,
            "mpc_r_ay": mpc_r_ay,
            "mpc_r_alpha": mpc_r_alpha,
            "mpc_terminal_factor": mpc_terminal_factor,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "max_linear_vel": max_linear_vel,
            "max_angular_vel": max_angular_vel,
            "wheel_radius": wheel_radius,
            "base_radius": base_radius,
            "wheel_max_omega": wheel_max_omega,
            "control_rate": control_rate,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1"),
        DeclareLaunchArgument("follower_ns", default_value="/robot2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        DeclareLaunchArgument("formation_radius", default_value="2.0"),
        DeclareLaunchArgument("formation_offset_x", default_value="-2.0"),
        DeclareLaunchArgument("formation_offset_y", default_value="0.0"),

        DeclareLaunchArgument("mpc_horizon", default_value="40"),
        DeclareLaunchArgument("mpc_q_px", default_value="5.0"),
        DeclareLaunchArgument("mpc_q_py", default_value="5.0"),
        DeclareLaunchArgument("mpc_q_theta", default_value="20.0"),
        DeclareLaunchArgument("mpc_q_vx", default_value="0.5"),
        DeclareLaunchArgument("mpc_q_vy", default_value="0.5"),
        DeclareLaunchArgument("mpc_q_omega", default_value="2.0"),
        DeclareLaunchArgument("mpc_r_ax", default_value="0.01"),
        DeclareLaunchArgument("mpc_r_ay", default_value="0.01"),
        DeclareLaunchArgument("mpc_r_alpha", default_value="0.01"),
        DeclareLaunchArgument("mpc_terminal_factor", default_value="10.0"),

        DeclareLaunchArgument("max_linear_accel", default_value="2.0"),
        DeclareLaunchArgument("max_angular_accel", default_value="6.0"),
        DeclareLaunchArgument("max_linear_vel", default_value="1.0"),
        DeclareLaunchArgument("max_angular_vel", default_value="2.0"),

        DeclareLaunchArgument("wheel_radius", default_value="0.03"),
        DeclareLaunchArgument("base_radius", default_value="0.11"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0"),
        DeclareLaunchArgument("control_rate", default_value="20.0"),

        formation_node,
    ])
