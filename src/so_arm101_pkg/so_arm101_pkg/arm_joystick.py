"""
Robot Arm Joystick Control Node
조이스틱을 통한 팔 제어
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, String
import math


class ArmJoystickNode(Node):
    """조이스틱 컨트롤러 노드"""

    def __init__(self):
        super().__init__('arm_joystick')
        
        # 현재 관절 각도
        self.joint_angles = [0.0] * 6
        self.joint_speed = 2.0  # 도/초
        
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
        
        # 구독자
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self._joy_callback,
            10
        )
        
        self.get_logger().info("조이스틱 컨트롤러 초기화 완료")

    def _joy_callback(self, msg: Joy) -> None:
        """조이스틱 입력 처리"""
        
        # 축 설정 (일반적인 조이스틱)
        # axes[0, 1]: 좌측 스틱 (X, Y)
        # axes[2, 5]: 좌측/우측 트리거
        # axes[3, 4]: 우측 스틱 (X, Y)
        
        # buttons[0-3]: A, B, X, Y
        # buttons[4, 5]: LB, RB
        # buttons[6, 7]: Back, Start
        
        if len(msg.axes) < 6 or len(msg.buttons) < 8:
            return
        
        # 좌측 스틱: J1 (회전) + J2 (수평)
        j1_delta = msg.axes[0] * self.joint_speed
        j2_delta = msg.axes[1] * self.joint_speed
        
        # 우측 스틱: J3 (높이) + J4 (pitch)
        j3_delta = msg.axes[4] * self.joint_speed
        j4_delta = msg.axes[3] * self.joint_speed
        
        # 트리거: J5 (roll), J6 (손가락)
        j5_delta = (msg.axes[2] - msg.axes[5]) * self.joint_speed  # 좌/우 트리거
        j6_delta = (msg.buttons[4] - msg.buttons[5]) * self.joint_speed  # LB/RB
        
        # 각도 업데이트
        self.joint_angles[0] += j1_delta
        self.joint_angles[1] += j2_delta
        self.joint_angles[2] += j3_delta
        self.joint_angles[3] += j4_delta
        self.joint_angles[4] += j5_delta
        self.joint_angles[5] += j6_delta
        
        # 각도 범위 제한 (-180 ~ 180)
        for i in range(6):
            while self.joint_angles[i] > 180:
                self.joint_angles[i] -= 360
            while self.joint_angles[i] < -180:
                self.joint_angles[i] += 360
        
        # 명령 발행
        cmd_msg = Float32MultiArray()
        cmd_msg.data = self.joint_angles
        self.joint_cmd_pub.publish(cmd_msg)
        
        # 특수 버튼
        if msg.buttons[0]:  # A 버튼: Home
            self._send_command("home")
        if msg.buttons[1]:  # B 버튼: Stop
            self._send_command("stop")

    def _send_command(self, command: str) -> None:
        """명령 전송"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJoystickNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
