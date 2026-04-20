from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    rf2o_publish_tf = LaunchConfiguration("rf2o_publish_tf")

    robot1_namespace = LaunchConfiguration("robot1_namespace")
    robot2_namespace = LaunchConfiguration("robot2_namespace")

    rf2o_robot1 = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry_robot1",
        namespace=robot1_namespace,
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/robot1/scan",
                "odom_topic": "/robot1/rf2o/odom",
                "publish_tf": ParameterValue(rf2o_publish_tf, value_type=bool),
                "base_frame_id": "robot1_base_footprint",
                "odom_frame_id": "robot1_odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    rf2o_robot2 = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry_robot2",
        namespace=robot2_namespace,
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/robot2/scan",
                "odom_topic": "/robot2/rf2o/odom",
                "publish_tf": ParameterValue(rf2o_publish_tf, value_type=bool),
                "base_frame_id": "robot2_base_footprint",
                "odom_frame_id": "robot2_odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("robot1_namespace", default_value="/robot1"),
            DeclareLaunchArgument("robot2_namespace", default_value="/robot2"),
            DeclareLaunchArgument(
                "rf2o_publish_tf",
                default_value="false",
                description="是否让 rf2o 发布 TF（odom->base_footprint）。建议与 EKF 二选一，避免 TF 冲突。",
            ),
            rf2o_robot1,
            rf2o_robot2,
        ]
    )

