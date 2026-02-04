"""
Robot Arm Keyboard Control Node
키보드를 통한 팔 제어
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import sys
import tty
import termios
import threading
import math


class ArmKeyboardNode(Node):
    """키보드 컨트롤러 노드"""

    def __init__(self):
        super().__init__('arm_keyboard')
        
        # 현재 관절 각도
        self.joint_angles = [0.0] * 6
        self.joint_speed = 5.0  # 도/초
        
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
        
        # 키보드 입력 스레드
        self.kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.kb_thread.start()
        
        self.get_logger().info("키보드 컨트롤러 초기화 완료")
        self._print_help()

    def _print_help(self) -> None:
        """도움말 출력"""
        help_text = """
╔════════════════════════════════════════════════════════════════╗
║            🤖 로봇 팔 키보드 제어 (Keyboard Control)           ║
╚════════════════════════════════════════════════════════════════╝

📌 관절 제어 (Joint Control):
  ┌─────────────────────────┐
  │  Q/A  : J1 (회전)       │  W/S  : J2 (수평)
  │  E/D  : J3 (높이)       │  R/F  : J4 (Pitch)
  │  T/G  : J5 (Roll)       │  Y/H  : J6 (그리퍼)
  └─────────────────────────┘

🎮 특수 명령 (Special Commands):
  H  - Home 위치로 이동     L  - 현재 각도 표시
  P  - 이전 자세 복원       C  - 각도 초기화
  Z  - 속도 감소            X  - 속도 증가
  Ctrl+C - 종료

⚡ 현재 상태:
  속도: {:.1f}°/sec
  J1={:6.1f}° | J2={:6.1f}° | J3={:6.1f}°
  J4={:6.1f}° | J5={:6.1f}° | J6={:6.1f}°

""".format(
            self.joint_speed,
            self.joint_angles[0],
            self.joint_angles[1],
            self.joint_angles[2],
            self.joint_angles[3],
            self.joint_angles[4],
            self.joint_angles[5],
        )
        print(help_text)

    def _keyboard_loop(self) -> None:
        """키보드 입력 루프"""
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                try:
                    key = sys.stdin.read(1)
                    if not key:
                        continue
                    
                    self._handle_key(key.lower())
                    
                except KeyboardInterrupt:
                    self.get_logger().info("종료...")
                    break
                    
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            rclpy.shutdown()

    def _handle_key(self, key: str) -> None:
        """키 입력 처리"""
        
        # 관절 제어
        key_map = {
            'q': (0, 1),   # J1 증가
            'a': (0, -1),  # J1 감소
            'w': (1, 1),   # J2 증가
            's': (1, -1),  # J2 감소
            'e': (2, 1),   # J3 증가
            'd': (2, -1),  # J3 감소
            'r': (3, 1),   # J4 증가
            'f': (3, -1),  # J4 감소
            't': (4, 1),   # J5 증가
            'g': (4, -1),  # J5 감소
            'y': (5, 1),   # J6 증가
            'h': (5, -1),  # J6 감소
        }
        
        if key in key_map:
            joint_idx, direction = key_map[key]
            self.joint_angles[joint_idx] += direction * self.joint_speed
            
            # 각도 범위 제한
            self.joint_angles[joint_idx] = self._clamp_angle(self.joint_angles[joint_idx])
            
            self._publish_angles()
            self._print_status(joint_idx)
        
        # 특수 명령
        elif key == 'h':
            self._send_command("home")
            self.joint_angles = [0.0] * 6
            print("\n✓ Home 위치로 이동합니다")
        
        elif key == 'l':
            print(f"\n📍 현재 관절 각도:")
            for i, angle in enumerate(self.joint_angles):
                print(f"  J{i+1}: {angle:7.1f}°")
        
        elif key == 'c':
            self.joint_angles = [0.0] * 6
            self._publish_angles()
            print("\n✓ 관절 각도 초기화")
        
        elif key == 'z':
            self.joint_speed = max(0.5, self.joint_speed - 1.0)
            print(f"\n⬇️  속도: {self.joint_speed:.1f}°/sec")
        
        elif key == 'x':
            self.joint_speed = min(30.0, self.joint_speed + 1.0)
            print(f"\n⬆️  속도: {self.joint_speed:.1f}°/sec")
        
        elif key == 'p':
            # 이전 자세 저장 기능 (나중에 구현 가능)
            print("\n⏮️  이전 자세 복원 (미구현)")
    
    def _clamp_angle(self, angle: float) -> float:
        """각도를 -180 ~ 180도 범위로 제한"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def _publish_angles(self) -> None:
        """현재 관절 각도 발행"""
        msg = Float32MultiArray()
        msg.data = self.joint_angles
        self.joint_cmd_pub.publish(msg)
    
    def _send_command(self, command: str) -> None:
        """명령 전송"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
    
    def _print_status(self, changed_joint: int) -> None:
        """관절 상태 출력"""
        joint_names = ['J1(회전)', 'J2(수평)', 'J3(높이)', 'J4(Pitch)', 'J5(Roll)', 'J6(그리퍼)']
        print(f"\r{joint_names[changed_joint]:10} = {self.joint_angles[changed_joint]:7.1f}°  |  "
              f"속도: {self.joint_speed:.1f}°/sec", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
