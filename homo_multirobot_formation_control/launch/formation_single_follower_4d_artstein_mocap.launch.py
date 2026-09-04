from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    source = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("homo_multirobot_formation_control"),
        "launch", "formation_single_follower_4d_artstein.launch.py"]))
    return LaunchDescription([IncludeLaunchDescription(source, launch_arguments={
        "state_source": "mocap", "use_sim_time": "false", "use_motor_delay": "false",
    }.items())])
