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

    # MPC weights
    mpc_horizon = LaunchConfiguration("mpc_horizon")
    q_px = LaunchConfiguration("q_px")
    q_py = LaunchConfiguration("q_py")
    q_vx = LaunchConfiguration("q_vx")
    q_vy = LaunchConfiguration("q_vy")
    r_ux = LaunchConfiguration("r_ux")
    r_uy = LaunchConfiguration("r_uy")
    terminal_factor = LaunchConfiguration("terminal_factor")
    osqp_max_iter = LaunchConfiguration("osqp_max_iter")
    osqp_eps_abs = LaunchConfiguration("osqp_eps_abs")
    osqp_eps_rel = LaunchConfiguration("osqp_eps_rel")
    osqp_polish = LaunchConfiguration("osqp_polish")

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

    # Delay on → controller outputs to cmd_vel_raw, delay node relays to cmd_vel
    cmd_output_topic = PythonExpression([
        "'cmd_vel_raw' if '", use_motor_delay, "' == 'true' else 'cmd_vel'"])

    formation_node = Node(
        package="homo_multirobot_formation_control",
        executable="formation_control_node_4d_artstein_mpc",
        name="formation_control_node_4d_artstein_mpc",
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
            "tau_min": LaunchConfiguration("tau_min"),
            "tau_max": LaunchConfiguration("tau_max"),
            "v_tau_trans": LaunchConfiguration("v_tau_trans"),
            "Td": Td,
            "mpc_horizon": mpc_horizon,
            "q_px": q_px,
            "q_py": q_py,
            "q_vx": q_vx,
            "q_vy": q_vy,
            "r_ux": r_ux,
            "r_uy": r_uy,
            "terminal_factor": terminal_factor,
            "osqp_max_iter": osqp_max_iter,
            "osqp_eps_abs": osqp_eps_abs,
            "osqp_eps_rel": osqp_eps_rel,
            "osqp_polish": osqp_polish,
            "Kp_yaw": Kp_yaw,
            "K_ff": K_ff,
            "wheel_radius": wheel_radius,
            "base_radius": base_radius,
            "max_linear_vel": max_linear_vel,
            "max_angular_vel": max_angular_vel,
            "enable_radial_safety": enable_radial_safety,
            "wheel_max_omega": wheel_max_omega,
            "max_linear_accel": max_linear_accel,
            "max_angular_accel": max_angular_accel,
            "control_rate": control_rate,
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
                              description="Motor time constant (s). With adaptive tau, "
                                          "this is the nominal value; actual tau varies "
                                          "between tau_min and tau_max based on |v_cmd|."),
        DeclareLaunchArgument("tau_min", default_value="0.25",
                              description="Adaptive tau lower bound (s)."),
        DeclareLaunchArgument("tau_max", default_value="0.55",
                              description="Adaptive tau upper bound (s)."),
        DeclareLaunchArgument("v_tau_trans", default_value="0.10",
                              description="Transition velocity (m/s) for adaptive tau."),
        DeclareLaunchArgument("Td", default_value="0.22",
                              description="Actuator dead time (s). Real robot ~220ms. "
                                          "Artstein reduction transforms the input-delay "
                                          "system into an equivalent delay-free system. "
                                          "Set 0.0 to disable Artstein compensation."),
        DeclareLaunchArgument("mpc_horizon", default_value="30",
                              description="MPC horizon steps"),
        DeclareLaunchArgument("q_px", default_value="40.0",
                              description="MPC px tracking weight"),
        DeclareLaunchArgument("q_py", default_value="40.0",
                              description="MPC py tracking weight"),
        DeclareLaunchArgument("q_vx", default_value="1.0",
                              description="MPC vx tracking weight"),
        DeclareLaunchArgument("q_vy", default_value="1.0",
                              description="MPC vy tracking weight"),
        DeclareLaunchArgument("r_ux", default_value="0.02",
                              description="MPC ux force-like input weight"),
        DeclareLaunchArgument("r_uy", default_value="0.02",
                              description="MPC uy force-like input weight"),
        DeclareLaunchArgument("terminal_factor", default_value="10.0",
                              description="MPC terminal Q multiplier"),
        DeclareLaunchArgument("osqp_max_iter", default_value="4000",
                              description="OSQP max iteration count"),
        DeclareLaunchArgument("osqp_eps_abs", default_value="1e-3",
                              description="OSQP absolute tolerance"),
        DeclareLaunchArgument("osqp_eps_rel", default_value="1e-3",
                              description="OSQP relative tolerance"),
        DeclareLaunchArgument("osqp_polish", default_value="true",
                              description="Enable OSQP polish step"),
        DeclareLaunchArgument("Kp_yaw", default_value="4.0",
                              description="Proportional yaw gain"),
        DeclareLaunchArgument("K_ff", default_value="1.0",
                              description="Feedforward yaw gain"),
        DeclareLaunchArgument("control_rate", default_value="20.0",
                              description="Control loop frequency (Hz)"),
        DeclareLaunchArgument("leader_vel_lpf_tau", default_value="0.0",
                              description="Leader velocity LPF time constant (s)."),
        DeclareLaunchArgument("min_cmd_vel", default_value="0.0",
                              description="Minimum cmd_vel magnitude (m/s). "
                                          "Set 0.0 to disable (simulation)."),
        DeclareLaunchArgument("wheel_radius", default_value="0.03",
                              description="Wheel rolling radius (m)"),
        DeclareLaunchArgument("base_radius", default_value="0.11",
                              description="Distance from robot center to wheel (m)"),
        DeclareLaunchArgument("wheel_max_omega", default_value="20.0",
                              description="Max wheel angular velocity (rad/s)"),
        DeclareLaunchArgument("max_linear_accel", default_value="0.4",
                              description="Max body linear acceleration (m/s^2). "
                                          "Acts as actuator rate constraint in 4D model; "
                                          "prevents v_cmd from jumping between steps."),
        DeclareLaunchArgument("max_angular_accel", default_value="4.0",
                              description="Max body angular acceleration (rad/s^2)"),
        DeclareLaunchArgument("max_linear_vel", default_value="0.5",
                              description="Max body linear velocity (m/s)"),
        DeclareLaunchArgument("max_angular_vel", default_value="0.5",
                              description="Max body angular velocity (rad/s)"),
        DeclareLaunchArgument("enable_radial_safety", default_value="true",
                              description="Limit inward radial velocity near the formation radius. "
                                          "Useful with actuator delay to avoid crossing into the leader."),
        DeclareLaunchArgument("use_motor_delay", default_value="false",
                              description="Simulate real motor response delay"),
        DeclareLaunchArgument("motor_tau", default_value="0.43",
                              description="Simulated motor LP time constant (s)."),
        DeclareLaunchArgument("transport_delay", default_value="0.0",
                              description="Transport delay (s, e.g. serial)."),
        DeclareLaunchArgument("delay_max_accel", default_value="2.0",
                              description="Delay node linear accel limit (m/s^2). "
                                          "Keep this above max_linear_vel / motor_tau when validating the "
                                          "Artstein first-order actuator model; lower values add an "
                                          "extra slew-rate saturation not represented in the predictor."),
        formation_node,
        delay_node,
    ])
