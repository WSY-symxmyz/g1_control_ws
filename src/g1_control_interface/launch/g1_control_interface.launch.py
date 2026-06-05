from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    share_dir = get_package_share_directory("g1_control_interface")
    config_path = os.path.join(share_dir, "config", "controller.yaml")
    limits_path = os.path.join(share_dir, "config", "joint_limits.yaml")
    arm_model = LaunchConfiguration("arm_model")
    arm_model_config = PythonExpression(
        [
            '"',
            share_dir,
            '/config/" + ("g1_arm7.yaml" if "',
            arm_model,
            '" == "G1ARM7" else "g1_arm5.yaml")',
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_model",
                default_value="G1ARM5",
                description="Upper-body joint layout: G1ARM5 or G1ARM7",
            ),
            Node(
                package="g1_control_interface",
                executable="g1_controller_main",
                name="g1_controller_node",
                output="screen",
                parameters=[
                    config_path,
                    limits_path,
                    arm_model_config,
                    {"arm_model": arm_model},
                ],
            )
        ]
    )
