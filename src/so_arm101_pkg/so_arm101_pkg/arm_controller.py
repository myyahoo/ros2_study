"""
6DOF Robot Arm Main Controller Node
메인 컨트롤러 - ROS2 노드로 팔 제어
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
import logging

from .arm_serial_comm import ArmSerialComm
from .arm_kinematics import ArmKinematics

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ArmControllerNode(Node):
    """ROS2 로봇 팔 제어 노드"""

    def __init__(self):
        super().__init__('arm_controller')
        
        # 파라미터 선언
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('update_rate', 50)  # Hz
        
        # 파라미터 읽기
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # 시리얼 통신 초기화
        self.arm_comm = ArmSerialComm(port=self.serial_port, baudrate=self.baudrate)
        self.arm_kinematics = ArmKinematics()
        
        # 현재 관절 상태
        self.current_joint_angles = [0.0] * 6
        self.target_joint_angles = [0.0] * 6
        
        # 구독자 (Subscriber)
        self.joint_cmd_sub = self.create_subscription(
            Float32MultiArray,
            'arm_controller/joint_command',
            self._joint_command_callback,
            10
        )
        
        self.pose_cmd_sub = self.create_subscription(
            Float32MultiArray,
            'arm_controller/pose_command',
            self._pose_command_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            'arm_controller/command',
            self._command_callback,
            10
        )
        
        # 발행자 (Publisher)
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.feedback_pub = self.create_publisher(
            String,
            'arm_controller/feedback',
            10
        )
        
        # 타이머 (주기적 업데이트)
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self._control_loop
        )
        
        # 시리얼 연결
        if not self.arm_comm.connect():
            self.get_logger().error("시리얼 포트 연결 실패!")
            raise RuntimeError("Cannot connect to serial port")
        
        self.get_logger().info("✓ 로봇 팔 컨트롤러 초기화 완료")
        
        # 초기 자세로 이동
        self._publish_feedback("Robot arm controller ready. Moving to home position.")
        self.arm_comm.home_position()

    def _joint_command_callback(self, msg: Float32MultiArray) -> None:
        """관절 각도 명령 수신"""
        if len(msg.data) == 6:
            self.target_joint_angles = list(msg.data)
            self.get_logger().info(
                f"관절 명령 수신: J1={msg.data[0]:.1f}°, "
                f"J2={msg.data[1]:.1f}°, J3={msg.data[2]:.1f}°, "
                f"J4={msg.data[3]:.1f}°, J5={msg.data[4]:.1f}°, "
                f"J6={msg.data[5]:.1f}°"
            )
        else:
            self.get_logger().error(f"잘못된 관절 명령: {len(msg.data)}개 값")

    def _pose_command_callback(self, msg: Float32MultiArray) -> None:
        """직교좌표 자세 명령 수신 (x, y, z, rx, ry, rz)"""
        if len(msg.data) == 6:
            result = self.arm_kinematics.inverse_kinematics(
                target_pos=(msg.data[0], msg.data[1], msg.data[2]),
                target_rot=(msg.data[3], msg.data[4], msg.data[5]),
                initial_guess=self.current_joint_angles
            )
            
            if result:
                self.target_joint_angles = result
                self.get_logger().info(
                    f"자세 명령 수신 (역기구학): "
                    f"[{', '.join(f'{a:.1f}°' for a in result)}]"
                )
            else:
                self.get_logger().error("역기구학 해를 찾을 수 없습니다")
        else:
            self.get_logger().error(f"잘못된 자세 명령: {len(msg.data)}개 값")

    def _command_callback(self, msg: String) -> None:
        """문자 명령 수신"""
        command = msg.data.lower().strip()
        
        if command == "home":
            self.arm_comm.home_position()
            self._publish_feedback("Moving to home position")
        elif command == "stop":
            self.arm_comm.emergency_stop()
            self._publish_feedback("Emergency stop activated!")
        elif command.startswith("speed:"):
            try:
                speed = int(command.split(":")[1])
                self.arm_comm.set_speed(speed)
                self._publish_feedback(f"Speed set to {speed}%")
            except:
                self.get_logger().error("Invalid speed command")
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def _control_loop(self) -> None:
        """메인 제어 루프"""
        try:
            # 목표 관절 각도 전송
            if self.target_joint_angles != self.current_joint_angles:
                self.arm_comm.send_joint_angles(self.target_joint_angles)
                self.current_joint_angles = list(self.target_joint_angles)
            
            # 관절 상태 발행
            self._publish_joint_state()
            
        except Exception as e:
            self.get_logger().error(f"제어 루프 오류: {e}")

    def _publish_joint_state(self) -> None:
        """현재 관절 상태 발행"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        msg.position = [float(angle) for angle in self.current_joint_angles]
        msg.velocity = [0.0] * 6  # 속도는 나중에 센서에서 읽을 수 있음
        msg.effort = [0.0] * 6    # 토크도 나중에 추가 가능
        
        self.joint_state_pub.publish(msg)

    def _publish_feedback(self, message: str) -> None:
        """피드백 메시지 발행"""
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)

    def destroy_node(self) -> None:
        """노드 종료"""
        self.get_logger().info("로봇 팔 컨트롤러 종료...")
        self.arm_comm.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
