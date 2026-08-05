"""
6D Artstein Disc 编队控制。

方向 A 架构：
  map-frame 平移 4D Artstein + yaw 2D Artstein
  -> 预测后的 6D Disc 状态
  -> leader-frame 6D Disc HPC 核心
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower_ns = LaunchConfiguration("follower_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    radius = LaunchConfiguration("radius")
    mass = LaunchConfiguration("mass")
    I = LaunchConfiguration("I")
    m_p = LaunchConfiguration("m_p")
    tol = LaunchConfiguration("tol")
    use_hpc = LaunchConfiguration("use_hpc")
    control_rate = LaunchConfiguration("control_rate")
    hpc_c_min = LaunchConfiguration("hpc_c_min")
    initial_min_lambda = LaunchConfiguration("initial_min_lambda")
    switch_min_lambda = LaunchConfiguration("switch_min_lambda")
    hpc_vel_threshold = LaunchConfiguration("hpc_vel_threshold")
    hpc_yaw_threshold = LaunchConfiguration("hpc_yaw_threshold")
    stability_margin = LaunchConfiguration("stability_margin")

    tau = LaunchConfiguration("tau")
    tau_yaw = LaunchConfiguration("tau_yaw")
    Td = LaunchConfiguration("Td")

    wheel_radius = LaunchConfiguration("wheel_radius")
    base_radius = LaunchConfiguration("base_radius")
    wheel_max_omega = LaunchConfiguration("wheel_max_omega")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_angular_accel = LaunchConfiguration("max_angular_accel")
    max_linear_vel = LaunchConfiguration("max_linear_vel")
    max_angular_vel = LaunchConfiguration("max_angular_vel")
    min_cmd_vel = LaunchConfiguration("min_cmd_vel")

    use_motor_delay = LaunchConfiguration("use_motor_delay")
    motor_tau = LaunchConfiguration("motor_tau")
    transport_delay = LaunchConfiguration("transport_delay")
    delay_max_accel = LaunchConfiguration("delay_max_accel")

    cmd_topic = PythonExpression([
        "'cmd_vel_raw' if '", use_motor_delay, "' == 'true' else 'cmd_vel'"
    ])

    controller = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_6d_artstein_disc",
        name="formation_control_node_6d_artstein_disc",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        remappings=[("cmd_vel", cmd_topic)],
        parameters=[
            {
                "leader_ns": leader_ns,
                "follower_ns": follower_ns,
                "use_sim_time": use_sim_time,
                "radius": radius,
                "mass": mass,
                "I": I,
                "m_p": m_p,
                "tol": tol,
                "use_hpc": use_hpc,
                "control_rate": control_rate,
                "hpc_c_min": hpc_c_min,
                "initial_min_lambda": initial_min_lambda,
                "switch_min_lambda": switch_min_lambda,
                "hpc_vel_threshold": hpc_vel_threshold,
                "hpc_yaw_threshold": hpc_yaw_threshold,
                "stability_margin": stability_margin,
                "tau": tau,
                "tau_yaw": tau_yaw,
                "Td": Td,
                "wheel_radius": wheel_radius,
                "base_radius": base_radius,
                "wheel_max_omega": wheel_max_omega,
                "max_linear_accel": max_linear_accel,
                "max_angular_accel": max_angular_accel,
                "max_linear_vel": max_linear_vel,
                "max_angular_vel": max_angular_vel,
                "min_cmd_vel": min_cmd_vel,
            },
        ],
    )

    delay_node = Node(
        package="homo_multirobot_formation_control",
        executable="sim_motor_delay.py",
        name="sim_motor_delay",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        condition=IfCondition(use_motor_delay),
        output="screen",
        parameters=[
            {
                "input_topic": "cmd_vel_raw",
                "output_topic": "cmd_vel",
                "motor_tau": motor_tau,
                "transport_delay": transport_delay,
                "max_accel": delay_max_accel,
                "rate": 100.0,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("leader_ns", default_value="/robot1"),
            DeclareLaunchArgument("follower_ns", default_value="/robot2"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("radius", default_value="2.0"),
            DeclareLaunchArgument("mass", default_value="2.0"),
            DeclareLaunchArgument("I", default_value="1.0"),
            DeclareLaunchArgument("m_p", default_value="4"),
            DeclareLaunchArgument("tol", default_value="0.1"),
            DeclareLaunchArgument("use_hpc", default_value="true"),
            DeclareLaunchArgument("control_rate", default_value="20.0"),
            DeclareLaunchArgument("hpc_c_min", default_value="0.5"),
            DeclareLaunchArgument("initial_min_lambda", default_value="1.0"),
            DeclareLaunchArgument("switch_min_lambda", default_value="4.0"),
            DeclareLaunchArgument("hpc_vel_threshold", default_value="0.3"),
            DeclareLaunchArgument("hpc_yaw_threshold", default_value="0.3"),
            DeclareLaunchArgument("stability_margin", default_value="0.01"),
            DeclareLaunchArgument("tau", default_value="0.43"),
            DeclareLaunchArgument("tau_yaw", default_value="0.43"),
            DeclareLaunchArgument("Td", default_value="0.22"),
            DeclareLaunchArgument("wheel_radius", default_value="0.03"),
            DeclareLaunchArgument("base_radius", default_value="0.11"),
            DeclareLaunchArgument("wheel_max_omega", default_value="20.0"),
            DeclareLaunchArgument("max_linear_accel", default_value="2.0"),
            DeclareLaunchArgument("max_angular_accel", default_value="4.0"),
            DeclareLaunchArgument("max_linear_vel", default_value="1.0"),
            DeclareLaunchArgument("max_angular_vel", default_value="0.5"),
            DeclareLaunchArgument("min_cmd_vel", default_value="0.0"),
            DeclareLaunchArgument("use_motor_delay", default_value="false"),
            DeclareLaunchArgument("motor_tau", default_value="0.43"),
            DeclareLaunchArgument("transport_delay", default_value="0.22"),
            DeclareLaunchArgument("delay_max_accel", default_value="2.0"),
            controller,
            delay_node,
        ]
    )
