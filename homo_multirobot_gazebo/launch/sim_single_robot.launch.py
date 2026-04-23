import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _opaque_setup(context, *args, **kwargs):
    """单机器人 Gazebo 仿真入口（从 sim_two_robots 精简而来）。"""
    software = context.perform_substitution(LaunchConfiguration("software_rendering"))
    actions = []

    gazebo_pkg_share = get_package_share_directory("homo_multirobot_gazebo")
    model_root = os.path.join(gazebo_pkg_share, "gazebo_model_root")
    os.makedirs(model_root, exist_ok=True)
    link_path = os.path.join(model_root, "homo_multirobot_urdf")
    urdf_share = get_package_share_directory("homo_multirobot_urdf")
    if os.path.lexists(link_path):
        if os.path.isdir(link_path) and not os.path.islink(link_path):
            shutil.rmtree(link_path)
        else:
            os.unlink(link_path)
    os.symlink(urdf_share, link_path)

    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    merged_model_path = (
        os.pathsep.join([model_root, existing_model_path]) if existing_model_path else model_root
    )
    actions.append(SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=merged_model_path))

    # WSL 无 ALSA 设备时减少 OpenAL 刷屏（不影响仿真）
    actions.append(SetEnvironmentVariable(name="ALSOFT_DRIVERS", value="null"))

    if software.lower() == "true":
        actions.append(SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"))
        actions.append(SetEnvironmentVariable(name="__GL_SYNC_TO_VBLANK", value="0"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    planar_publish_odom = LaunchConfiguration("planar_publish_odom")
    planar_publish_odom_tf = LaunchConfiguration("planar_publish_odom_tf")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    server = LaunchConfiguration("server")
    verbose = LaunchConfiguration("verbose")

    robot_name = LaunchConfiguration("robot_name")
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_prefix = LaunchConfiguration("robot_prefix")

    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")
    robot_yaw = LaunchConfiguration("robot_yaw")

    xacro_path = PathJoinSubstitution(
        [get_package_share_directory("homo_multirobot_urdf"), "urdf", "mini_omni_robot.xacro"]
    )

    robot_urdf = Command(
        [
            "xacro",
            " ",
            xacro_path,
            " ",
            "prefix:=",
            robot_prefix,
            " ",
            "ros_namespace:=",
            robot_namespace,
            " ",
            "use_ros2_control:=",
            use_ros2_control,
            " ",
            "planar_publish_odom:=",
            planar_publish_odom,
            " ",
            "planar_publish_odom_tf:=",
            planar_publish_odom_tf,
        ]
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=robot_namespace,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(robot_urdf, value_type=str),
            },
        ],
    )

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        namespace=robot_namespace,
        parameters=[{"use_sim_time": use_sim_time}],
    )

    prefix_value = context.perform_substitution(LaunchConfiguration("robot_prefix"))
    child_base_footprint = f"{prefix_value}base_footprint"

    publish_world_tf = LaunchConfiguration("publish_world_tf")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    static_tf_world_robot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "--x",
            robot_x,
            "--y",
            robot_y,
            "--z",
            robot_z,
            "--yaw",
            robot_yaw,
            "--frame-id",
            "world",
            "--child-frame-id",
            child_base_footprint,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(publish_world_tf),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    gzserver_launch = os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
    gzclient_launch = os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch),
        launch_arguments={
            "world": world,
            "verbose": verbose,
            "init": "true",
            "factory": "true",
        }.items(),
        condition=IfCondition(server),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch),
        condition=IfCondition(gui),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        namespace=robot_namespace,
        arguments=[
            "-entity",
            robot_name,
            "-robot_namespace",
            robot_namespace,
            "-topic",
            "robot_description",
            "-x",
            robot_x,
            "-y",
            robot_y,
            "-z",
            robot_z,
            "-Y",
            robot_yaw,
        ],
    )

    actions.extend(
        [
            gzserver,
            gzclient,
            robot_state_pub,
            joint_state_pub,
            static_tf_world_robot,
            spawn_robot,
            rviz,
        ]
    )
    return actions


def generate_launch_description():
    default_world_name = "empty.world"
    world_name = LaunchConfiguration("world_name")
    default_world = PathJoinSubstitution(
        [get_package_share_directory("homo_multirobot_gazebo"), "worlds", world_name]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_gazebo"), "rviz", "two_robots_sim.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="false",
                description="true 时在 URDF 中启用 ros2_control + gazebo_ros2_control；false 时回退 gazebo_ros_planar_move。",
            ),
            DeclareLaunchArgument(
                "planar_publish_odom",
                default_value="true",
                description="仅 gazebo_ros_planar_move 模式生效：是否发布 <ns>/odom 话题。",
            ),
            DeclareLaunchArgument(
                "planar_publish_odom_tf",
                default_value="true",
                description="仅 gazebo_ros_planar_move 模式生效：是否发布 TF <prefix>odom -> <prefix>base_footprint。",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value=default_world_name,
                description="从 homo_multirobot_gazebo/worlds/ 下加载的世界文件名（例如 empty.world）。如需绝对路径请直接用 world:=/abs/path/to.world。",
            ),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("server", default_value="true"),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="gzserver 详细日志；一般 false 可略减轻输出与开销。",
            ),
            DeclareLaunchArgument(
                "software_rendering",
                default_value="false",
                description="true 时使用 LIBGL_ALWAYS_SOFTWARE=1（WSL 黑屏可试）；false 优先 GPU，界面更流畅。",
            ),
            DeclareLaunchArgument("robot_name", default_value="robot1"),
            DeclareLaunchArgument("robot_namespace", default_value="/robot1"),
            DeclareLaunchArgument("robot_prefix", default_value="robot1_"),
            DeclareLaunchArgument("robot_x", default_value="0.0"),
            DeclareLaunchArgument("robot_y", default_value="0.0"),
            DeclareLaunchArgument("robot_z", default_value="0.0"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "publish_world_tf",
                default_value="false",
                description="发布 world -> <prefix>base_footprint 静态 TF，便于 RViz 使用 Fixed Frame=world。若已有 Gazebo/里程计发布同名 TF，请设为 false。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否同时启动 RViz2。",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 配置文件路径。",
            ),
            OpaqueFunction(function=_opaque_setup),
        ]
    )

