import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    odom_source = LaunchConfiguration("odom_source")

    # rf2o 模式用低金字塔层级；wheel 模式 EKF 读取轮式里程计
    ctf_levels_val = PythonExpression(["'3' if '", odom_source, "' == 'rf2o' else '5'"])
    iter_irls_val = PythonExpression(["'3' if '", odom_source, "' == 'rf2o' else '5'"])
    ekf_odom_topic_val = PythonExpression(["'odom' if '", odom_source, "' == 'wheel' else ''"])

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

    # 5. rf2o + EKF (odom_source=wheel 时 EKF 用轮式里程计，rf2o 仍运行但不参与融合)
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
            "rf2o_freq": "6.0",
            "scan_topic": "scan",
            "ctf_levels": ctf_levels_val,
            "iter_irls": iter_irls_val,
            # 显式传 odom_frame_id，避免被 base_serial 的 odom_combined 覆盖
            "odom_frame_id": [prefix, "odom"],
            # wheel 模式: EKF 读取轮式里程计 /odom 而非 rf2o/odom
            "ekf_odom_topic": ekf_odom_topic_val,
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
        DeclareLaunchArgument(
            "odom_source", default_value="wheel",
            description="里程计来源: wheel(轮式里程计，默认) / rf2o(激光里程计)"
        ),
        base_serial,
        wheeltec_lidar,
        robot_desc,
        imu_processor,
        rf2o_ekf,
    ])
