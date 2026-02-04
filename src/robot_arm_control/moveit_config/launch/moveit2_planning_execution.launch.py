"""
MoveIt2 데모 런칭 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("so_arm101", package_name="robot_arm_control").to_moveit_configs()

    # MoveIt2 Launch
    move_group_launch = moveit_config.launch_file(
        config_suffix="_planning",
        launch_configs={
            "rviz_config": PathJoinSubstitution(
                [FindPackageShare("robot_arm_control"), "moveit_config", "config", "moveit.rviz"]
            ),
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
        ]
        + move_group_launch
    )
