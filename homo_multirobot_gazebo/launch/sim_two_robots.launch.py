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


def _opaque_setup(context, *args, **kwargs):
    """WSL/WSLg 下 gzclient 常黑屏：可选强制软件渲染。

    Gazebo 会把 URDF 里的 package://homo_multirobot_urdf/... 转成 model://homo_multirobot_urdf/...
    若把 .../share 整段加入 GAZEBO_MODEL_PATH，InsertModelWidget 会把 ament_index、colcon-core
    等目录也当成模型并报 Missing model.config。这里仅使用「只含 homo_multirobot_urdf 子目录」的路径。
    """
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
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    server = LaunchConfiguration("server")
    verbose = LaunchConfiguration("verbose")

    robot1_name = LaunchConfiguration("robot1_name")
    robot2_name = LaunchConfiguration("robot2_name")
    robot1_namespace = LaunchConfiguration("robot1_namespace")
    robot2_namespace = LaunchConfiguration("robot2_namespace")

    robot1_prefix = LaunchConfiguration("robot1_prefix")
    robot2_prefix = LaunchConfiguration("robot2_prefix")

    robot1_x = LaunchConfiguration("robot1_x")
    robot1_y = LaunchConfiguration("robot1_y")
    robot1_z = LaunchConfiguration("robot1_z")
    robot1_yaw = LaunchConfiguration("robot1_yaw")

    robot2_x = LaunchConfiguration("robot2_x")
    robot2_y = LaunchConfiguration("robot2_y")
    robot2_z = LaunchConfiguration("robot2_z")
    robot2_yaw = LaunchConfiguration("robot2_yaw")

    default_world = PathJoinSubstitution(
        [get_package_share_directory("homo_multirobot_gazebo"), "worlds", "empty.world"]
    )

    xacro_path = PathJoinSubstitution(
        [get_package_share_directory("homo_multirobot_urdf"), "urdf", "mini_omni_robot.xacro"]
    )

    robot1_urdf = Command(["xacro", " ", xacro_path, " ", "prefix:=", robot1_prefix])
    robot2_urdf = Command(["xacro", " ", xacro_path, " ", "prefix:=", robot2_prefix])

    robot_state_pub_robot1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=robot1_namespace,
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot1_urdf},
        ],
    )

    robot_state_pub_robot2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=robot2_namespace,
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot2_urdf},
        ],
    )

    # continuous 关节（轮子）的 TF 依赖 /joint_states；从 robot_description 话题取模型（由 robot_state_publisher 发布）
    joint_state_pub_robot1 = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        namespace=robot1_namespace,
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_pub_robot2 = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        namespace=robot2_namespace,
        parameters=[{"use_sim_time": use_sim_time}],
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

    spawn_robot1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        namespace=robot1_namespace,
        arguments=[
            "-entity",
            robot1_name,
            "-robot_namespace",
            robot1_namespace,
            "-topic",
            "robot_description",
            "-x",
            robot1_x,
            "-y",
            robot1_y,
            "-z",
            robot1_z,
            "-Y",
            robot1_yaw,
        ],
    )

    spawn_robot2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        namespace=robot2_namespace,
        arguments=[
            "-entity",
            robot2_name,
            "-robot_namespace",
            robot2_namespace,
            "-topic",
            "robot_description",
            "-x",
            robot2_x,
            "-y",
            robot2_y,
            "-z",
            robot2_z,
            "-Y",
            robot2_yaw,
        ],
    )

    actions.extend(
        [
            gzserver,
            gzclient,
            robot_state_pub_robot1,
            robot_state_pub_robot2,
            joint_state_pub_robot1,
            joint_state_pub_robot2,
            spawn_robot1,
            spawn_robot2,
        ]
    )
    return actions


def generate_launch_description():
    default_world = PathJoinSubstitution(
        [get_package_share_directory("homo_multirobot_gazebo"), "worlds", "empty.world"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
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
            DeclareLaunchArgument("robot1_name", default_value="robot1"),
            DeclareLaunchArgument("robot2_name", default_value="robot2"),
            DeclareLaunchArgument("robot1_namespace", default_value="/robot1"),
            DeclareLaunchArgument("robot2_namespace", default_value="/robot2"),
            DeclareLaunchArgument("robot1_prefix", default_value="robot1_"),
            DeclareLaunchArgument("robot2_prefix", default_value="robot2_"),
            DeclareLaunchArgument("robot1_x", default_value="0.0"),
            DeclareLaunchArgument("robot1_y", default_value="0.0"),
            DeclareLaunchArgument("robot1_z", default_value="0.0"),
            DeclareLaunchArgument("robot1_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot2_x", default_value="1.0"),
            DeclareLaunchArgument("robot2_y", default_value="0.0"),
            DeclareLaunchArgument("robot2_z", default_value="0.0"),
            DeclareLaunchArgument("robot2_yaw", default_value="0.0"),
            OpaqueFunction(function=_opaque_setup),
        ]
    )
