"""Launch the odom-frame closed-loop Leader circle without delay injection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='robot1',
            description='Leader robot namespace without a leading slash'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
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
        Node(
            package='homo_multirobot_formation_control',
            executable='leader_circle_closed_loop.py',
            name='leader_circle_closed_loop',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'radius': LaunchConfiguration('radius'),
                'speed': LaunchConfiguration('speed'),
                'heading': LaunchConfiguration('heading'),
                'direction': LaunchConfiguration('direction'),
                'start_side': LaunchConfiguration('start_side'),
                'rate': LaunchConfiguration('rate'),
                'odom_topic': LaunchConfiguration('odom_topic'),
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
        ),
    ])
