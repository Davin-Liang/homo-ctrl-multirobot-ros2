"""6D Artstein Disc formation with scan-only static-cylinder HOCBF safety.

This launch only starts the follower controller.  It subscribes to the leader
odometry and therefore intentionally has no ``leader_speed`` argument; set the
leader speed on the separately launched leader trajectory node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    names = [
        ("leader_ns", "/robot1"), ("follower_ns", "/robot2"),
        ("use_sim_time", "true"), ("radius", "2.0"), ("mass", "2.0"),
        ("I", "1.0"), ("m_p", "4"), ("tol", "0.1"),
        ("use_hpc", "true"), ("control_rate", "20.0"),
        ("tau", "0.43"), ("tau_yaw", "0.43"), ("Td", "0.22"),
        ("max_linear_vel", "1.0"), ("max_angular_vel", "0.5"),
        ("max_linear_accel", "2.0"), ("max_angular_accel", "4.0"),
        ("wheel_radius", "0.03"), ("base_radius", "0.11"),
        ("wheel_max_omega", "20.0"), ("hpc_c_min", "0.5"),
        ("initial_min_lambda", "1.0"), ("switch_min_lambda", "4.0"),
        ("hpc_vel_threshold", "0.3"), ("hpc_yaw_threshold", "0.3"),
        ("stability_margin", "0.01"), ("scan_topic", "scan"),
        ("follower_radius", "0.15"), ("clearance", "0.10"),
        ("perception_margin", "0.15"), ("scan_timeout", "0.30"),
        ("use_latest_tf_fallback", "true"),
        ("cluster_tolerance", "0.10"), ("min_cluster_points", "5"),
        ("max_obstacles", "10"), ("min_cylinder_radius", "0.03"),
        ("max_cylinder_radius", "0.60"), ("max_fit_residual", "0.03"),
    ]
    parameters = {name: LaunchConfiguration(name) for name, _ in names}
    node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_6d_artstein_disc_hocbf",
        name="formation_control_node_6d_artstein_disc_hocbf",
        namespace=PythonExpression(["'", LaunchConfiguration("follower_ns"), "'"]),
        output="screen", parameters=[parameters],
    )
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in names] + [node])
