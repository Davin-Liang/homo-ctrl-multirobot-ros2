import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def generate_launch_description():
    prefix = LaunchConfiguration("prefix")
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    jsp_available = True
    try:
        get_package_share_directory("joint_state_publisher")
    except PackageNotFoundError:
        jsp_available = False

    jsp_gui_available = True
    try:
        get_package_share_directory("joint_state_publisher_gui")
    except PackageNotFoundError:
        jsp_gui_available = False

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_urdf"), "urdf", "mini_omni_robot.xacro"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("homo_multirobot_urdf"), "rviz", "mini_omni_robot.rviz"]
    )

    robot_description = {
        "robot_description": Command(
            ["xacro", " ", xacro_file, " ", "prefix:=", prefix]
        )
    }

    declared = [
        DeclareLaunchArgument("prefix", default_value="", description="Link/joint name prefix."),
        DeclareLaunchArgument("use_gui", default_value="false", description="Use joint_state_publisher_gui."),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz2."),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config file path.",
        ),
    ]

    def _launch_setup(context, *args, **kwargs):
        prefix_value = prefix.perform(context)

        # Make RViz fixed frame match the URDF prefix mode by patching config.
        # Note: RViz can't accept fixed-frame as a CLI arg, so we rewrite config.
        fixed_frame = f"{prefix_value}base_link" if prefix_value else "base_link"
        rviz_config_in = rviz_config.perform(context)
        rviz_config_to_use = rviz_config_in
        try:
            with open(rviz_config_in, "r", encoding="utf-8") as f:
                content = f.read()
            if "Fixed Frame:" in content:
                patched = []
                for line in content.splitlines(True):
                    if line.lstrip().startswith("Fixed Frame:"):
                        indent = line[: len(line) - len(line.lstrip())]
                        patched.append(f"{indent}Fixed Frame: {fixed_frame}\n")
                    else:
                        patched.append(line)
                content = "".join(patched)
            tmp_path = os.path.join(
                tempfile.gettempdir(),
                f"homo_multirobot_urdf_default_{fixed_frame}.rviz",
            )
            with open(tmp_path, "w", encoding="utf-8") as f:
                f.write(content)
            rviz_config_to_use = tmp_path
        except Exception:
            # Fall back to provided config path if anything goes wrong.
            rviz_config_to_use = rviz_config_in

        actions = [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            )
        ]

        if jsp_gui_available and jsp_available:
            actions.append(
                Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    output="screen",
                    condition=IfCondition(use_gui),
                )
            )
            actions.append(
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    output="screen",
                    condition=UnlessCondition(use_gui),
                )
            )
        elif jsp_available:
            actions.append(
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    output="screen",
                )
            )

        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_config_to_use],
            )
        )

        return actions

    return LaunchDescription(declared + [OpaqueFunction(function=_launch_setup)])

