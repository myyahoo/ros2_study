"""
Vision만 시작하는 Launch 파일
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_control',
            executable='vision_node',
            name='vision_node',
            parameters=[
                {'camera_topic': '/camera/image_raw'},
                {'detection_topic': '/robot_arm/detection'},
                {'confidence_threshold': 0.5},
            ],
            output='screen'
        ),
    ])
