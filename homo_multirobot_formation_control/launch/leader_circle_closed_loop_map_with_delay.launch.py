"""Launch a delay-aware closed-loop Leader circle and motor-delay simulator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_motor_delay = LaunchConfiguration('use_motor_delay')
    cmd_output_topic = PythonExpression([
        "'cmd_vel_raw' if '", use_motor_delay,
        "' == 'true' else 'cmd_vel'",
    ])

    leader_node = Node(
        package='homo_multirobot_formation_control',
        executable='leader_circle_closed_loop_map.py',
        name='leader_circle_closed_loop_map',
        namespace=namespace,
        output='screen',
        remappings=[('cmd_vel', cmd_output_topic)],
        parameters=[{
            'use_sim_time': use_sim_time,
            'radius': LaunchConfiguration('radius'),
            'speed': LaunchConfiguration('speed'),
            'heading': LaunchConfiguration('heading'),
            'direction': LaunchConfiguration('direction'),
            'start_side': LaunchConfiguration('start_side'),
            'rate': LaunchConfiguration('rate'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
            'Td': LaunchConfiguration('Td'),
            'tau_v': LaunchConfiguration('tau_v'),
            'kp': LaunchConfiguration('kp'),
            'kv': LaunchConfiguration('kv'),
            'k_yaw': LaunchConfiguration('k_yaw'),
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_linear_accel': LaunchConfiguration('max_linear_accel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'max_angular_accel': LaunchConfiguration('max_angular_accel'),
        }],
    )

    delay_node = Node(
        package='homo_multirobot_formation_control',
        executable='sim_motor_delay.py',
        name='sim_motor_delay',
        namespace=namespace,
        output='screen',
        condition=IfCondition(use_motor_delay),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': 'cmd_vel_raw',
            'output_topic': 'cmd_vel',
            'motor_tau': LaunchConfiguration('motor_tau'),
            'transport_delay': LaunchConfiguration('transport_delay'),
            'max_accel': LaunchConfiguration('delay_max_accel'),
            'rate': LaunchConfiguration('delay_rate'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='robot1',
            description='Leader robot namespace without a leading slash'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'use_motor_delay', default_value='true',
            description='Inject sim_motor_delay between cmd_vel_raw and cmd_vel'),
        DeclareLaunchArgument('radius', default_value='2.0'),
        DeclareLaunchArgument('speed', default_value='0.2'),
        DeclareLaunchArgument('heading', default_value='0.0'),
        DeclareLaunchArgument('direction', default_value='ccw'),
        DeclareLaunchArgument(
            'start_side', default_value='top',
            description='Circle start endpoint: top or bottom'),
        DeclareLaunchArgument('rate', default_value='20.0'),
        DeclareLaunchArgument('odom_topic', default_value='odometry/filtered'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Global frame used for the closed-loop circle reference'),
        DeclareLaunchArgument(
            'Td', default_value='0.22',
            description='Leader predictor pure input dead time (s)'),
        DeclareLaunchArgument(
            'tau_v', default_value='0.43',
            description='Leader predictor velocity-response time constant (s)'),
        DeclareLaunchArgument('kp', default_value='0.8'),
        DeclareLaunchArgument('kv', default_value='0.2'),
        DeclareLaunchArgument('k_yaw', default_value='1.5'),
        DeclareLaunchArgument('max_linear_vel', default_value='0.4'),
        DeclareLaunchArgument('max_linear_accel', default_value='0.25'),
        DeclareLaunchArgument('max_angular_vel', default_value='0.8'),
        DeclareLaunchArgument('max_angular_accel', default_value='1.0'),
        DeclareLaunchArgument(
            'motor_tau', default_value='0.43',
            description='sim_motor_delay first-order response time constant (s)'),
        DeclareLaunchArgument(
            'transport_delay', default_value='0.22',
            description='sim_motor_delay pure transport delay (s)'),
        DeclareLaunchArgument(
            'delay_max_accel', default_value='0.25',
            description='sim_motor_delay per-component acceleration limit (m/s^2)'),
        DeclareLaunchArgument(
            'delay_rate', default_value='100.0',
            description='sim_motor_delay update frequency (Hz)'),
        leader_node,
        delay_node,
    ])

