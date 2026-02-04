"""
태스크 실행 노드만 시작하는 Launch 파일
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_control',
            executable='task_executor_node',
            name='task_executor_node',
            parameters=[
                {'task_update_rate': 10},
                {'pen_detection_timeout': 5.0},
            ],
            output='screen'
        ),
    ])
