"""Launch the map-frame closed-loop Leader figure-eight trajectory."""

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
        DeclareLaunchArgument('amplitude_x', default_value='2.0'),
        DeclareLaunchArgument('amplitude_y', default_value='1.0'),
        DeclareLaunchArgument(
            'speed', default_value='0.2',
            description='Figure-eight peak reference speed in m/s'),
        DeclareLaunchArgument('heading', default_value='0.0'),
        DeclareLaunchArgument('rate', default_value='20.0'),
        DeclareLaunchArgument('odom_topic', default_value='odometry/filtered'),
        DeclareLaunchArgument(
            'state_source', default_value='odom_tf',
            description='odom_tf or mocap'),
        DeclareLaunchArgument('mocap_pose_topic', default_value='mocap/pose'),
        DeclareLaunchArgument('mocap_twist_topic', default_value='mocap/twist'),
        DeclareLaunchArgument('mocap_state_timeout', default_value='0.10'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Global frame used for the closed-loop reference'),
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
            executable='leader_eight_closed_loop_map.py',
            name='leader_eight_closed_loop_map',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'amplitude_x': LaunchConfiguration('amplitude_x'),
                'amplitude_y': LaunchConfiguration('amplitude_y'),
                'speed': LaunchConfiguration('speed'),
                'heading': LaunchConfiguration('heading'),
                'rate': LaunchConfiguration('rate'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'state_source': LaunchConfiguration('state_source'),
                'mocap_pose_topic': LaunchConfiguration('mocap_pose_topic'),
                'mocap_twist_topic': LaunchConfiguration('mocap_twist_topic'),
                'mocap_state_timeout': LaunchConfiguration('mocap_state_timeout'),
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
        ),
    ])
