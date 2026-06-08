import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")

    turn_on_dir = get_package_share_directory("turn_on_wheeltec_robot")
    localization_dir = get_package_share_directory("homo_multirobot_localization")
    launch_dir = os.path.join(turn_on_dir, "launch")

    # 1. 串口驱动 (STM32 → /<ns>/imu/data_raw, /<ns>/odom)
    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "base_serial.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
        }.items(),
    )

    # 2. N10 雷达 (→ /<ns>/scan)
    wheeltec_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "wheeltec_lidar.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
        }.items(),
    )

    # 3. URDF + robot_state_publisher (TF 由 URDF fixed joints 提供)
    robot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_mode_description.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
        }.items(),
    )

    # 4. IMU 防漂移 (→ /<ns>/imu/data_filtered)
    imu_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "imu_processor.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    # 5. rf2o + EKF (→ /<ns>/rf2o/odom + odometry/filtered + TF)
    rf2o_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                localization_dir, "launch", "rf2o_ekf_single_robot.launch.py"
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "prefix": prefix,
            "use_sim_time": "false",
            "imu_topic": "imu/data_filtered",
            "ekf_yaml_only": "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="机器人命名空间（如 robot1），多机时每台车使用不同 namespace"
        ),
        DeclareLaunchArgument(
            "prefix", default_value="",
            description="TF 帧前缀（如 robot1_），与 namespace 对应"
        ),
        base_serial,
        wheeltec_lidar,
        robot_desc,
        imu_processor,
        rf2o_ekf,
    ])
