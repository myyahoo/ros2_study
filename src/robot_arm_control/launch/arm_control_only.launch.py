"""
팔 제어만 시작하는 Launch 파일
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_control',
            executable='arm_control_node',
            name='arm_control_node',
            parameters=[
                {'num_joints': 6},
                {'joint_max_velocity': 1.0},
            ],
            output='screen'
        ),
    ])
