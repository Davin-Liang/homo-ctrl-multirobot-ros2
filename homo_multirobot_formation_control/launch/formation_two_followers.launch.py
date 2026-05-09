from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower1_ns = LaunchConfiguration("follower1_ns")
    follower2_ns = LaunchConfiguration("follower2_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Formation geometry
    m_p = LaunchConfiguration("m_p")
    radius = LaunchConfiguration("radius")
    tol = LaunchConfiguration("tol")

    # Robot dynamics
    mass = LaunchConfiguration("mass")

    # Yaw control
    Kp_yaw = LaunchConfiguration("Kp_yaw")
    K_ff = LaunchConfiguration("K_ff")

    # Control rate
    control_rate = LaunchConfiguration("control_rate")

    common_params = {
        "leader_ns": leader_ns,
        "use_sim_time": use_sim_time,
        "m_p": m_p,
        "radius": radius,
        "tol": tol,
        "mass": mass,
        "Kp_yaw": Kp_yaw,
        "K_ff": K_ff,
        "control_rate": control_rate,
    }

    follower1_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node",
        name="formation_control_node",
        namespace=PythonExpression(["'", follower1_ns, "'"]),
        output="screen",
        parameters=[{
            **common_params,
            "follower_ns": follower1_ns,
        }],
    )

    follower2_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node",
        name="formation_control_node",
        namespace=PythonExpression(["'", follower2_ns, "'"]),
        output="screen",
        parameters=[{
            **common_params,
            "follower_ns": follower2_ns,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1",
                              description="Leader robot namespace"),
        DeclareLaunchArgument("follower1_ns", default_value="/robot2",
                              description="Follower 1 robot namespace"),
        DeclareLaunchArgument("follower2_ns", default_value="/robot3",
                              description="Follower 2 robot namespace"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation time"),
        DeclareLaunchArgument("m_p", default_value="4",
                              description="Number of safe formation points"),
        DeclareLaunchArgument("radius", default_value="2.0",
                              description="Formation circle radius (m)"),
        DeclareLaunchArgument("tol", default_value="0.1",
                              description="Switching tolerance between formation points"),
        DeclareLaunchArgument("mass", default_value="8.0",
                              description="Controller model mass (tuning, not physical)"),
        DeclareLaunchArgument("Kp_yaw", default_value="4.0",
                              description="Proportional yaw gain"),
        DeclareLaunchArgument("K_ff", default_value="1.0",
                              description="Feedforward yaw gain"),
        DeclareLaunchArgument("control_rate", default_value="20.0",
                              description="Control loop frequency (Hz)"),
        follower1_node,
        follower2_node,
    ])
