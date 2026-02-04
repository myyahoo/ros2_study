"""
Robot Arm Demo Node
데모: 여러 위치로 순차 이동
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import time


class ArmDemoNode(Node):
    """로봇 팔 데모 노드"""

    def __init__(self):
        super().__init__('arm_demo')
        
        # 발행자
        self.joint_cmd_pub = self.create_publisher(
            Float32MultiArray,
            'arm_controller/joint_command',
            10
        )
        
        self.command_pub = self.create_publisher(
            String,
            'arm_controller/command',
            10
        )
        
        # 데모 위치들 (각도: J1, J2, J3, J4, J5, J6)
        self.demo_positions = [
            [0, 0, 0, 0, 0, 0],           # Home
            [45, 30, 45, 0, 0, 0],        # 위치 1
            [-45, 45, -30, 0, 0, 0],      # 위치 2
            [0, -30, 60, 45, 0, 0],       # 위치 3
            [90, 20, 30, -30, 0, 0],      # 위치 4
            [0, 0, 0, 0, 0, 0],           # 다시 Home
        ]
        
        # 데모 실행
        self.get_logger().info("로봇 팔 데모 시작!")
        self._run_demo()

    def _run_demo(self) -> None:
        """데모 루틴 실행"""
        
        for i, position in enumerate(self.demo_positions):
            self.get_logger().info(
                f"데모 위치 {i}: {[f'{a:.0f}°' for a in position]}"
            )
            
            # 명령 발행
            cmd_msg = Float32MultiArray()
            cmd_msg.data = position
            self.joint_cmd_pub.publish(cmd_msg)
            
            # 이동 대기 (3초)
            time.sleep(3)
        
        self.get_logger().info("데모 완료!")
        
        # 데모 종료
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArmDemoNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
