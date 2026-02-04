"""
6DOF Robot Arm Serial Communication Module
로봇 팔과의 시리얼 통신을 담당하는 모듈
"""

import serial
import time
import threading
from typing import Callable, Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ArmSerialComm:
    """
    시리얼 포트를 통해 6DOF 로봇 팔과 통신하는 클래스
    """

    def __init__(self, port: str = '/dev/ttyACM1', baudrate: int = 115200, timeout: float = 1.0):
        """
        Args:
            port: 시리얼 포트 (기본값: /dev/ttyACM1)
            baudrate: 통신 속도 (기본값: 115200)
            timeout: 읽기 타임아웃 (기본값: 1.0초)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_connected = False
        self.read_thread = None
        self.stop_reading = False
        self.data_callback: Optional[Callable] = None

    def connect(self) -> bool:
        """
        시리얼 포트에 연결
        Returns:
            연결 성공 여부
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            time.sleep(2)  # Arduino 재부팅 대기
            self.is_connected = True
            logger.info(f"✓ 시리얼 포트 연결 성공: {self.port} ({self.baudrate} baud)")
            
            # 읽기 스레드 시작
            self.stop_reading = False
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            return True
        except Exception as e:
            logger.error(f"✗ 시리얼 포트 연결 실패: {e}")
            return False

    def disconnect(self) -> None:
        """시리얼 포트 연결 해제"""
        if self.is_connected:
            self.stop_reading = True
            if self.read_thread:
                self.read_thread.join(timeout=2)
            if self.serial_conn:
                self.serial_conn.close()
            self.is_connected = False
            logger.info("✓ 시리얼 포트 연결 해제")

    def send_command(self, command: str) -> bool:
        """
        로봇 팔에 명령 전송
        Args:
            command: 전송할 명령 문자열
        Returns:
            전송 성공 여부
        """
        if not self.is_connected:
            logger.error("시리얼 포트가 연결되지 않았습니다")
            return False

        try:
            # 명령 끝에 개행 문자 추가
            cmd = command if command.endswith('\n') else command + '\n'
            self.serial_conn.write(cmd.encode('utf-8'))
            logger.debug(f"명령 전송: {command}")
            return True
        except Exception as e:
            logger.error(f"명령 전송 실패: {e}")
            return False

    def send_joint_angles(self, angles: list) -> bool:
        """
        관절 각도로 로봇 팔 제어
        Args:
            angles: [J1, J2, J3, J4, J5, J6] 각도 리스트 (단위: 도)
        Returns:
            전송 성공 여부
        """
        if len(angles) != 6:
            logger.error("각도는 6개여야 합니다")
            return False

        # 각도 범위 검증 (-180 ~ 180도)
        for i, angle in enumerate(angles):
            if not (-180 <= angle <= 180):
                logger.warning(f"J{i+1} 각도가 범위를 벗어났습니다: {angle}")
                angles[i] = max(-180, min(180, angle))

        # 명령 형식: "J1,J2,J3,J4,J5,J6"
        command = f"MOVE:{','.join(f'{a:.1f}' for a in angles)}"
        return self.send_command(command)

    def send_cartesian_pose(self, x: float, y: float, z: float, 
                           rx: float, ry: float, rz: float) -> bool:
        """
        직교좌표계 위치와 자세로 로봇 팔 제어
        Args:
            x, y, z: 위치 (단위: mm)
            rx, ry, rz: 회전 각도 (단위: 도)
        Returns:
            전송 성공 여부
        """
        command = f"POSE:{x},{y},{z},{rx},{ry},{rz}"
        return self.send_command(command)

    def get_joint_angles(self) -> Optional[list]:
        """
        현재 관절 각도 조회
        Returns:
            [J1, J2, J3, J4, J5, J6] 또는 None
        """
        if not self.send_command("GET_ANGLES"):
            return None
        time.sleep(0.1)
        # 응답은 _read_loop에서 콜백으로 처리됨
        return None

    def set_speed(self, speed_percent: int) -> bool:
        """
        로봇 팔의 속도 설정
        Args:
            speed_percent: 속도 (1-100%)
        Returns:
            전송 성공 여부
        """
        speed = max(1, min(100, speed_percent))
        return self.send_command(f"SPEED:{speed}")

    def set_data_callback(self, callback: Callable) -> None:
        """
        수신 데이터 콜백 함수 설정
        Args:
            callback: def callback(data: str) -> None
        """
        self.data_callback = callback

    def _read_loop(self) -> None:
        """백그라운드 스레드에서 시리얼 데이터 읽기"""
        while not self.stop_reading and self.is_connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.readline().decode('utf-8').strip()
                    if data and self.data_callback:
                        self.data_callback(data)
            except Exception as e:
                logger.error(f"데이터 읽기 오류: {e}")
                time.sleep(0.1)

    def emergency_stop(self) -> bool:
        """긴급 정지"""
        return self.send_command("STOP")

    def home_position(self) -> bool:
        """초기 자세로 이동"""
        return self.send_command("HOME")
