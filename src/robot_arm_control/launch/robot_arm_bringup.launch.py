"""
메인 Launch 파일: 전체 로봇 암 제어 시스템 시작
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Vision 노드
        Node(
            package='robot_arm_control',
            executable='vision_node',
            name='vision_node',
            parameters=[
                {'camera_topic': '/camera/image_raw'},
                {'detection_topic': '/robot_arm/detection'},
                {'confidence_threshold': 0.5},
                {'target_class': 'pen'},
            ],
            output='screen'
        ),
        
        # 로봇 암 제어 노드
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
        
        # 메인 태스크 실행 노드
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
