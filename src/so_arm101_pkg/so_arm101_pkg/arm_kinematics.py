"""
6DOF Robot Arm Kinematics Module
역기구학 계산 및 기본 기하학 연산
"""

import math
import numpy as np
from typing import Tuple, List, Optional


class ArmKinematics:
    """
    6DOF 로봇 팔의 기구학 계산
    
    DH (Denavit-Hartenberg) 파라미터 기반 정방향/역기구학
    로봇 구성: 
    - J1: 베이스 회전 (±180도)
    - J2-J6: 팔 관절 (각 ±90도)
    """

    def __init__(self, link_lengths: Optional[List[float]] = None):
        """
        Args:
            link_lengths: [L1, L2, L3, L4, L5, L6] 링크 길이 (단위: mm)
                         기본값: [50, 150, 150, 100, 100, 100]
        """
        # 기본 링크 길이 (mm)
        self.link_lengths = link_lengths or [50, 150, 150, 100, 100, 100]
        
        # DH 파라미터 (a, d, alpha)
        self.DH_PARAMS = [
            {'a': 0, 'd': self.link_lengths[0], 'alpha': math.pi/2},      # J1
            {'a': self.link_lengths[1], 'd': 0, 'alpha': 0},               # J2
            {'a': self.link_lengths[2], 'd': 0, 'alpha': 0},               # J3
            {'a': self.link_lengths[3], 'd': 0, 'alpha': math.pi/2},       # J4
            {'a': 0, 'd': self.link_lengths[4], 'alpha': -math.pi/2},      # J5
            {'a': 0, 'd': self.link_lengths[5], 'alpha': 0},               # J6
        ]

    def forward_kinematics(self, joint_angles: List[float]) -> dict:
        """
        정방향 기구학: 관절 각도 -> 끝단 위치/자세
        
        Args:
            joint_angles: [J1, J2, J3, J4, J5, J6] (단위: 도)
        
        Returns:
            {
                'position': [x, y, z],
                'rotation_matrix': 3x3 회전 행렬,
                'euler_angles': [roll, pitch, yaw]
            }
        """
        # 도를 라디안으로 변환
        theta = [math.radians(angle) for angle in joint_angles]
        
        # 변환 행렬 누적
        T = np.eye(4)
        
        for i in range(6):
            # DH 파라미터로부터 변환 행렬 계산
            T_i = self._dh_matrix(
                theta[i],
                self.DH_PARAMS[i]['d'],
                self.DH_PARAMS[i]['a'],
                self.DH_PARAMS[i]['alpha']
            )
            T = T @ T_i
        
        # 위치 추출
        position = T[0:3, 3].tolist()
        
        # 회전 행렬 추출
        rotation_matrix = T[0:3, 0:3]
        
        # 오일러 각도로 변환
        euler_angles = self._rotation_matrix_to_euler(rotation_matrix)
        
        return {
            'position': position,
            'rotation_matrix': rotation_matrix,
            'euler_angles': euler_angles
        }

    def inverse_kinematics(self, target_pos: Tuple[float, float, float],
                          target_rot: Optional[Tuple[float, float, float]] = None,
                          initial_guess: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        역기구학: 끝단 위치 -> 관절 각도
        수치적 방법 (Newton-Raphson) 사용
        
        Args:
            target_pos: [x, y, z] 목표 위치
            target_rot: [roll, pitch, yaw] 목표 자세 (선택)
            initial_guess: 초기 추정 각도
        
        Returns:
            [J1, J2, J3, J4, J5, J6] (도) 또는 None (해 없음)
        """
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0, 0, 0]
        
        # 수치적 역기구학 (간단한 구현)
        # 실제로는 보다 정교한 알고리즘 (CCD, FABRIK 등) 필요
        
        current_angles = list(initial_guess)
        max_iterations = 100
        tolerance = 1.0  # mm
        learning_rate = 0.01
        
        for iteration in range(max_iterations):
            # 현재 끝단 위치 계산
            fk_result = self.forward_kinematics(current_angles)
            current_pos = np.array(fk_result['position'])
            target_array = np.array(target_pos)
            
            # 오차 계산
            error = target_array - current_pos
            error_magnitude = np.linalg.norm(error)
            
            if error_magnitude < tolerance:
                return [round(angle, 2) for angle in current_angles]
            
            # 자코비안 수치 계산
            jacobian = self._numerical_jacobian(current_angles)
            
            # 의사 역행렬로 각속도 계산
            try:
                j_pinv = np.linalg.pinv(jacobian)
                dtheta = learning_rate * (j_pinv @ error)
                
                # 각도 업데이트 (라디안 -> 도)
                for i in range(6):
                    current_angles[i] += math.degrees(dtheta[i])
                    # 각도 범위 제한
                    current_angles[i] = self._clamp_angle(current_angles[i])
            except:
                break
        
        # 해를 찾지 못한 경우
        return None

    def _dh_matrix(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """DH 파라미터로부터 4x4 변환 행렬 생성"""
        T = np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        return T

    def _rotation_matrix_to_euler(self, R: np.ndarray) -> List[float]:
        """회전 행렬을 오일러 각도로 변환 (XYZ 순서)"""
        sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
        
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        
        return [math.degrees(x), math.degrees(y), math.degrees(z)]

    def _numerical_jacobian(self, angles: List[float], delta: float = 0.01) -> np.ndarray:
        """수치적 자코비안 계산"""
        jacobian = np.zeros((3, 6))
        
        base_fk = self.forward_kinematics(angles)
        base_pos = np.array(base_fk['position'])
        
        for i in range(6):
            perturbed_angles = list(angles)
            perturbed_angles[i] += delta
            
            perturbed_fk = self.forward_kinematics(perturbed_angles)
            perturbed_pos = np.array(perturbed_fk['position'])
            
            jacobian[:, i] = (perturbed_pos - base_pos) / delta
        
        return jacobian

    def _clamp_angle(self, angle: float) -> float:
        """각도를 -180 ~ 180도 범위로 제한"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def calculate_distance(self, pos1: Tuple[float, float, float],
                          pos2: Tuple[float, float, float]) -> float:
        """두 점 사이의 거리 계산"""
        return math.sqrt(
            (pos2[0] - pos1[0])**2 +
            (pos2[1] - pos1[1])**2 +
            (pos2[2] - pos1[2])**2
        )

    def interpolate_angles(self, start_angles: List[float],
                          end_angles: List[float],
                          steps: int) -> List[List[float]]:
        """
        두 관절 각도 사이의 선형 보간
        
        Args:
            start_angles: 시작 각도
            end_angles: 종료 각도
            steps: 보간 스텝 수
        
        Returns:
            보간된 각도 리스트
        """
        result = []
        for t in np.linspace(0, 1, steps):
            interpolated = [
                start_angles[i] * (1 - t) + end_angles[i] * t
                for i in range(6)
            ]
            result.append(interpolated)
        return result
