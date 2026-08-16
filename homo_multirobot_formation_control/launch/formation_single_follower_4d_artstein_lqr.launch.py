from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower_ns = LaunchConfiguration("follower_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_motor_delay = LaunchConfiguration("use_motor_delay")
    cmd_output_topic = PythonExpression([
        "'cmd_vel_raw' if '", use_motor_delay, "' == 'true' else 'cmd_vel'"])

    formation_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_4d_artstein_lqr",
        name="formation_control_node_4d_artstein_lqr",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        remappings=[("cmd_vel", cmd_output_topic)],
        parameters=[{
            "leader_ns": leader_ns,
            "follower_ns": follower_ns,
            "use_sim_time": use_sim_time,
            "m_p": LaunchConfiguration("m_p"),
            "radius": LaunchConfiguration("radius"),
            "tol": LaunchConfiguration("tol"),
            "mass": LaunchConfiguration("mass"),
            "tau": LaunchConfiguration("tau"),
            "tau_min": LaunchConfiguration("tau_min"),
            "tau_max": LaunchConfiguration("tau_max"),
            "v_tau_trans": LaunchConfiguration("v_tau_trans"),
            "Td": LaunchConfiguration("Td"),
            "q_px": LaunchConfiguration("q_px"),
            "q_py": LaunchConfiguration("q_py"),
            "q_vx": LaunchConfiguration("q_vx"),
            "q_vy": LaunchConfiguration("q_vy"),
            "r_ux": LaunchConfiguration("r_ux"),
            "r_uy": LaunchConfiguration("r_uy"),
            "dare_max_iter": LaunchConfiguration("dare_max_iter"),
            "dare_tol": LaunchConfiguration("dare_tol"),
            "Kp_yaw": LaunchConfiguration("Kp_yaw"),
            "K_ff": LaunchConfiguration("K_ff"),
            "wheel_radius": LaunchConfiguration("wheel_radius"),
            "base_radius": LaunchConfiguration("base_radius"),
            "max_linear_vel": LaunchConfiguration("max_linear_vel"),
            "max_angular_vel": LaunchConfiguration("max_angular_vel"),
            "enable_radial_safety": LaunchConfiguration("enable_radial_safety"),
            "wheel_max_omega": LaunchConfiguration("wheel_max_omega"),
            "max_linear_accel": LaunchConfiguration("max_linear_accel"),
            "max_angular_accel": LaunchConfiguration("max_angular_accel"),
            "control_rate": LaunchConfiguration("control_rate"),
            "leader_vel_lpf_tau": LaunchConfiguration("leader_vel_lpf_tau"),
            "min_cmd_vel": LaunchConfiguration("min_cmd_vel"),
        }],
    )

    delay_node = Node(
        package="homo_multirobot_formation_control",
        executable="sim_motor_delay.py",
        name="sim_motor_delay",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        condition=IfCondition(use_motor_delay),
        parameters=[{
            "input_topic": "cmd_vel_raw",
            "output_topic": "cmd_vel",
            "motor_tau": LaunchConfiguration("motor_tau"),
            "transport_delay": LaunchConfiguration("transport_delay"),
            "max_accel": LaunchConfiguration("delay_max_accel"),
            "rate": 100.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1"),
        DeclareLaunchArgument("follower_ns", default_value="/robot2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("m_p", default_value="4"),
        DeclareLaunchArgument("radius", default_value="2.0"),
        DeclareLaunchArgument("tol", default_value="0.1"),
        DeclareLaunchArgument("mass", default_value="2.0"),
        DeclareLaunchArgument("tau", default_value="0.43"),
        DeclareLaunchArgument("tau_min", default_value="0.25"),
        DeclareLaunchArgument("tau_max", default_value="0.55"),
        DeclareLaunchArgument("v_tau_trans", default_value="0.10"),
        DeclareLaunchArgument("Td", default_value="0.22"),
        DeclareLaunchArgument("q_px", default_value="40.0"),
        DeclareLaunchArgument("q_py", default_value="40.0"),
        DeclareLaunchArgument("q_vx", default_value="1.0"),
        DeclareLaunchArgument("q_vy", default_value="1.0"),
        DeclareLaunchArgument("r_ux", default_value="0.02"),
        DeclareLaunchArgument("r_uy", default_value="0.02"),
        DeclareLaunchArgument("dare_max_iter", default_value="10000"),
        DeclareLaunchArgument("dare_tol", default_value="1e-12"),
        DeclareLaunchArgument("Kp_yaw", default_value="4.0"),
        DeclareLaunchArgument("K_ff", default_value="1.0"),
        DeclareLaunchArgument("control_rate", default_value="20.0"),
        DeclareLaunchArgument("leader_vel_lpf_tau", default_value="0.0"),
        DeclareLaunchArgument("min_cmd_vel", default_value="0.0"),
        DeclareLaunchArgument("wheel_radius", default_value="0.03"),
        DeclareLaunchArgument("base_radius", default_value="0.11"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0"),
        DeclareLaunchArgument("max_linear_accel", default_value="0.4"),
        DeclareLaunchArgument("max_angular_accel", default_value="4.0"),
        DeclareLaunchArgument("max_linear_vel", default_value="0.5"),
        DeclareLaunchArgument("max_angular_vel", default_value="0.5"),
        DeclareLaunchArgument("enable_radial_safety", default_value="true"),
        DeclareLaunchArgument("use_motor_delay", default_value="false"),
        DeclareLaunchArgument("motor_tau", default_value="0.43"),
        DeclareLaunchArgument("transport_delay", default_value="0.0"),
        DeclareLaunchArgument("delay_max_accel", default_value="2.0"),
        formation_node,
        delay_node,
    ])
