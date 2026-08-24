from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    leader_ns = LaunchConfiguration("leader_ns")
    follower_ns = LaunchConfiguration("follower_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Formation geometry
    m_p = LaunchConfiguration("m_p")
    radius = LaunchConfiguration("radius")
    tol = LaunchConfiguration("tol")

    # Robot dynamics
    mass = LaunchConfiguration("mass")
    tau = LaunchConfiguration("tau")
    Td = LaunchConfiguration("Td")
    omega_d = LaunchConfiguration("omega_d")

    # Yaw control
    Kp_yaw = LaunchConfiguration("Kp_yaw")
    K_ff = LaunchConfiguration("K_ff")

    # Velocity limits
    max_linear_vel = LaunchConfiguration("max_linear_vel")
    max_angular_vel = LaunchConfiguration("max_angular_vel")
    enable_radial_safety = LaunchConfiguration("enable_radial_safety")

    # Kinematic constraints
    wheel_radius = LaunchConfiguration("wheel_radius")
    base_radius = LaunchConfiguration("base_radius")
    wheel_max_omega = LaunchConfiguration("wheel_max_omega")
    max_linear_accel = LaunchConfiguration("max_linear_accel")
    max_angular_accel = LaunchConfiguration("max_angular_accel")

    # Control rate
    control_rate = LaunchConfiguration("control_rate")

    # Motor delay simulation
    use_motor_delay = LaunchConfiguration("use_motor_delay")
    motor_tau = LaunchConfiguration("motor_tau")
    transport_delay = LaunchConfiguration("transport_delay")
    delay_max_accel = LaunchConfiguration("delay_max_accel")
    radial_safety_max_decel = PythonExpression([
        "float('", delay_max_accel, "') if '", use_motor_delay,
        "' == 'true' else 0.0"])
    radial_safety_effective_delay = PythonExpression([
        "(float('", transport_delay, "') + float('", motor_tau,
        "')) if '", use_motor_delay, "' == 'true' else -1.0"])

    # Delay on → controller outputs to cmd_vel_raw, delay node relays to cmd_vel
    cmd_output_topic = PythonExpression([
        "'cmd_vel_raw' if '", use_motor_delay, "' == 'true' else 'cmd_vel'"])

    formation_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_4d_artstein",
        name="formation_control_node_4d_artstein",
        namespace=PythonExpression(["'", follower_ns, "'"]),
        output="screen",
        remappings=[("cmd_vel", cmd_output_topic)],
        parameters=[{
            "leader_ns": leader_ns,
            "follower_ns": follower_ns,
            "use_sim_time": use_sim_time,
            "m_p": m_p,
            "radius": radius,
            "tol": tol,
            "mass": mass,
            "tau": tau,
            "Td": Td,
            "omega_d": omega_d,
            "Kp_yaw": Kp_yaw,
            "K_ff": K_ff,
            "wheel_radius": wheel_radius,
            "base_radius": base_radius,
            "max_linear_vel": max_linear_vel,
            "max_angular_vel": max_angular_vel,
            "enable_radial_safety": enable_radial_safety,
            "radial_safety_max_decel": radial_safety_max_decel,
            "radial_safety_effective_delay": radial_safety_effective_delay,
            "wheel_max_omega": wheel_max_omega,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "control_rate": control_rate,
            "use_hpc": LaunchConfiguration("use_hpc"),
            "hpc_c_min": LaunchConfiguration("hpc_c_min"),
            "initial_min_lambda": LaunchConfiguration("initial_min_lambda"),
            "switch_min_lambda": LaunchConfiguration("switch_min_lambda"),
            "leader_vel_lpf_tau": LaunchConfiguration("leader_vel_lpf_tau"),
            "min_cmd_vel": LaunchConfiguration("min_cmd_vel"),
            "cmd_integrator_base": LaunchConfiguration("cmd_integrator_base"),
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
            "motor_tau": motor_tau,
            "transport_delay": transport_delay,
            "max_accel": delay_max_accel,
            "rate": 100.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("leader_ns", default_value="/robot1",
                              description="Leader robot namespace"),
        DeclareLaunchArgument("follower_ns", default_value="/robot2",
                              description="Follower robot namespace"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation time"),
        DeclareLaunchArgument("m_p", default_value="4",
                              description="Number of safe formation points"),
        DeclareLaunchArgument("radius", default_value="2.0",
                              description="Formation circle radius (m)"),
        DeclareLaunchArgument("tol", default_value="0.1",
                              description="Switching tolerance between formation points"),
        DeclareLaunchArgument("mass", default_value="2.0",
                              description="Double-integrator HPC mass parameter. Default 2.0 matches the MATLAB/Python numerical tests."),
        DeclareLaunchArgument("tau", default_value="0.43",
                              description="Fixed equivalent motor time constant (s)."),
        DeclareLaunchArgument("Td", default_value="0.22",
                              description="Actuator dead time (s). Real robot ~220ms. "
                                          "Artstein reduction transforms the input-delay "
                                          "system into an equivalent delay-free system. "
                                          "Set 0.0 to disable Artstein compensation."),
        DeclareLaunchArgument("omega_d", default_value="0.7",
                              description="Desired closed-loop bandwidth (rad/s)"),
        DeclareLaunchArgument("Kp_yaw", default_value="4.0",
                              description="Proportional yaw gain"),
        DeclareLaunchArgument("K_ff", default_value="1.0",
                              description="Feedforward yaw gain"),
        DeclareLaunchArgument("control_rate", default_value="20.0",
                              description="Control loop frequency (Hz)"),
        DeclareLaunchArgument("use_hpc", default_value="true",
                              description="Enable homogeneous upgrade (false = pure LPC)"),
        DeclareLaunchArgument("hpc_c_min", default_value="0.1",
                              description="HPC warp clamp lower bound. Default 0.1 matches MATLAB lpc_hpc_distance_square.m."),
        DeclareLaunchArgument("initial_min_lambda", default_value="1.0",
                              description="Initial LPC pole lower bound, matching Python initial_min_lambda."),
        DeclareLaunchArgument("switch_min_lambda", default_value="4.0",
                              description="LPC pole lower bound after formation-point switching, matching Python switch_min_lambda."),
        DeclareLaunchArgument("leader_vel_lpf_tau", default_value="0.0",
                              description="Leader velocity LPF time constant (s)."),
        DeclareLaunchArgument("min_cmd_vel", default_value="0.03",
                              description="Minimum cmd_vel magnitude (m/s). "
                                          "Set 0.0 to disable (simulation)."),
        DeclareLaunchArgument("wheel_radius", default_value="0.03",
                              description="Wheel rolling radius (m)"),
        DeclareLaunchArgument("base_radius", default_value="0.11",
                              description="Distance from robot center to wheel (m)"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0",
                              description="Max wheel angular velocity (rad/s)"),
        DeclareLaunchArgument("max_linear_accel", default_value="2.0",
                              description="Max body linear acceleration (m/s^2). "
                                          "Acts as actuator rate constraint in 4D model; "
                                          "prevents v_cmd from jumping between steps."),
        DeclareLaunchArgument("max_angular_accel", default_value="4.0",
                              description="Max body angular acceleration (rad/s^2)"),
        DeclareLaunchArgument("max_linear_vel", default_value="1.0",
                              description="Max body linear velocity (m/s)"),
        DeclareLaunchArgument("max_angular_vel", default_value="0.5",
                              description="Max body angular velocity (rad/s)"),
        DeclareLaunchArgument("enable_radial_safety", default_value="true",
                              description="Limit inward radial velocity near the formation radius "
                                          "to account for actuator delay and braking distance."),
        DeclareLaunchArgument("use_motor_delay", default_value="false",
                              description="Simulate real motor response delay"),
        DeclareLaunchArgument("motor_tau", default_value="0.43",
                              description="Simulated motor LP time constant (s)."),
        DeclareLaunchArgument("transport_delay", default_value="0.0",
                              description="Transport delay (s, e.g. serial)."),
        DeclareLaunchArgument("delay_max_accel", default_value="0.25",
                              description="Delay node linear accel limit (m/s^2)."),
        DeclareLaunchArgument("cmd_integrator_base", default_value="pred",
                              description="Velocity command integration base: pred keeps current behavior; cmd integrates from previous published cmd_vel."),
        formation_node,
        delay_node,
    ])
