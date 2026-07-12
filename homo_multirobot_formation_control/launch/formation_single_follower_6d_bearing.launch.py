"""
6D 运动学模型编队控制 — 方位角约束编队策略。

将目标编队点固定于 Leader 车体系下安全圆上指定方位角 phi_d 处。
Cartesian 位置误差同时编码径向距离误差和切向方位角误差：
  - 径向分量将 Follower 推向/拉离安全圆
  - 切向分量驱动 Follower 沿圆弧滑向目标方位
无需离散编队点的切换逻辑，编队偏移恒定，轨迹为平滑弧线。

与 formation_single_follower_6d_disc（离散编队点）的区别：
  无 m_p / tol 参数，改用 phi_d 指定唯一编队方位。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower_ns = LaunchConfiguration("follower_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 控制器参数
    radius = LaunchConfiguration("radius")
    phi_d = LaunchConfiguration("phi_d")
    mass = LaunchConfiguration("mass")
    I = LaunchConfiguration("I")
    omega_d = LaunchConfiguration("omega_d")
    omega_d_theta = LaunchConfiguration("omega_d_theta")
    hpc_vel_threshold = LaunchConfiguration("hpc_vel_threshold")
    use_hpc = LaunchConfiguration("use_hpc")
    control_rate = LaunchConfiguration("control_rate")

    # 运动学约束
    wheel_radius = LaunchConfiguration("wheel_radius")
    base_radius = LaunchConfiguration("base_radius")
    wheel_max_omega = LaunchConfiguration("wheel_max_omega")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_angular_accel = LaunchConfiguration("max_angular_accel")
    max_linear_vel = LaunchConfiguration("max_linear_vel")
    max_angular_vel = LaunchConfiguration("max_angular_vel")

    node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_6d_bearing",
        name="formation_control_node_6d_bearing",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        parameters=[
            {
                "leader_ns": leader_ns,
                "follower_ns": follower_ns,
                "use_sim_time": use_sim_time,
                "radius": radius,
                "phi_d": phi_d,
                "mass": mass,
                "I": I,
                "omega_d": omega_d,
                "omega_d_theta": omega_d_theta,
                "hpc_vel_threshold": hpc_vel_threshold,
                "use_hpc": use_hpc,
                "control_rate": control_rate,
                "wheel_radius": wheel_radius,
                "base_radius": base_radius,
                "wheel_max_omega": wheel_max_omega,
                "max_linear_accel": max_linear_accel,
                "max_angular_accel": max_angular_accel,
                "max_linear_vel": max_linear_vel,
                "max_angular_vel": max_angular_vel,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("leader_ns", default_value="/robot1"),
            DeclareLaunchArgument("follower_ns", default_value="/robot2"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("radius", default_value="2.0"),
            DeclareLaunchArgument("phi_d", default_value="3.1416"),
            DeclareLaunchArgument("mass", default_value="1.5"),
            DeclareLaunchArgument("I", default_value="0.3"),
            DeclareLaunchArgument("omega_d", default_value="0.8"),
            DeclareLaunchArgument("omega_d_theta", default_value="0.8"),
            DeclareLaunchArgument("hpc_vel_threshold", default_value="0.3"),
            DeclareLaunchArgument("use_hpc", default_value="true"),
            DeclareLaunchArgument("control_rate", default_value="20.0"),
            DeclareLaunchArgument("wheel_radius", default_value="0.03"),
            DeclareLaunchArgument("base_radius", default_value="0.11"),
            DeclareLaunchArgument("wheel_max_omega", default_value="20.0"),
            DeclareLaunchArgument("max_linear_accel", default_value="2.0"),
            DeclareLaunchArgument("max_angular_accel", default_value="4.0"),
            DeclareLaunchArgument("max_linear_vel", default_value="1.0"),
            DeclareLaunchArgument("max_angular_vel", default_value="0.5"),
            node,
        ]
    )
