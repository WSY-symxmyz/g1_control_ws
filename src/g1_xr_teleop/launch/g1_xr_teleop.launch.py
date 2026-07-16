from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("g1_xr_teleop"), "config", "xr_teleop.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="YAML parameter file for the XR teleop bridge.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="When true, compute commands but do not publish motion commands.",
            ),
            DeclareLaunchArgument(
                "enable_xr",
                default_value="false",
                description="Enable the vendored Unitree TeleVuer input adapter.",
            ),
            DeclareLaunchArgument(
                "enable_ik",
                default_value="false",
                description="Enable the vendored Unitree G1 arm IK adapter.",
            ),
            DeclareLaunchArgument(
                "arm_model",
                default_value="G1ARM5",
                description="Arm model for command mapping and IK: G1ARM5 or G1ARM7.",
            ),
            Node(
                package="g1_xr_teleop",
                executable="xr_teleop_node",
                name="g1_xr_teleop_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "enable_xr": LaunchConfiguration("enable_xr"),
                        "enable_ik": LaunchConfiguration("enable_ik"),
                        "arm_model": LaunchConfiguration("arm_model"),
                    },
                ],
            ),
        ]
    )
