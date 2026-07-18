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
                description="Enable G1 dual-arm inverse kinematics.",
            ),
            DeclareLaunchArgument(
                "ik_backend",
                default_value="native_optimization",
                description="IK backend: native_optimization or unitree_casadi.",
            ),
            DeclareLaunchArgument(
                "enable_arm",
                default_value="true",
                description="Enable arm target generation and publication.",
            ),
            DeclareLaunchArgument(
                "enable_loco",
                default_value="true",
                description="Enable locomotion command generation and publication.",
            ),
            DeclareLaunchArgument(
                "arm_model",
                default_value="G1ARM5",
                description="Arm model for command mapping and IK: G1ARM5 or G1ARM7.",
            ),
            DeclareLaunchArgument(
                "arm_tracking_mode",
                default_value="relative_clutch",
                description="Arm mapping: relative_clutch or unitree_absolute.",
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
                        "ik_backend": LaunchConfiguration("ik_backend"),
                        "enable_arm": LaunchConfiguration("enable_arm"),
                        "enable_loco": LaunchConfiguration("enable_loco"),
                        "arm_model": LaunchConfiguration("arm_model"),
                        "arm_tracking_mode": LaunchConfiguration(
                            "arm_tracking_mode"
                        ),
                    },
                ],
            ),
        ]
    )
