from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    로봇 팔 제어 시스템 실행
    
    실행: ros2 launch so_arm101_pkg arm_control.launch.py
    """
    
    return LaunchDescription([
        # 메인 컨트롤러 노드
        Node(
            package='so_arm101_pkg',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM1'},
                {'baudrate': 115200},
                {'update_rate': 50},
            ]
        ),
    ])
