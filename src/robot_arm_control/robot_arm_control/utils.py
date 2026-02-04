#!/usr/bin/env python3
"""
로봇 암 유틸리티: 공통 함수 및 헬퍼
"""

import numpy as np
from typing import List, Tuple


class RobotArmUtils:
    """로봇 암 관련 유틸리티 함수"""
    
    @staticmethod
    def rad_to_deg(radians: float) -> float:
        """라디안을 도로 변환"""
        return np.degrees(radians)
    
    @staticmethod
    def deg_to_rad(degrees: float) -> float:
        """도를 라디안으로 변환"""
        return np.radians(degrees)
    
    @staticmethod
    def clamp(value: float, min_val: float, max_val: float) -> float:
        """값을 범위 내로 제한"""
        return max(min_val, min(value, max_val))
    
    @staticmethod
    def distance(point1: np.ndarray, point2: np.ndarray) -> float:
        """두 점 사이의 거리 계산"""
        return np.linalg.norm(point1 - point2)
    
    @staticmethod
    def linear_interpolation(start: np.ndarray, end: np.ndarray, 
                             t: float) -> np.ndarray:
        """선형 보간 (0 <= t <= 1)"""
        return start + t * (end - start)
    
    @staticmethod
    def smooth_step(t: float) -> float:
        """스무즈 스텝 함수 (0 <= t <= 1)"""
        return t * t * (3 - 2 * t)
    
    @staticmethod
    def velocity_profile(current: float, target: float, max_velocity: float, 
                        dt: float) -> Tuple[float, float]:
        """속도 프로필 계산"""
        error = target - current
        velocity = np.clip(error, -max_velocity, max_velocity)
        next_value = current + velocity * dt
        return next_value, velocity


class TrapezoidalVelocityProfile:
    """사다리꼴 속도 프로필 생성"""
    
    def __init__(self, max_velocity: float, max_acceleration: float):
        self.max_v = max_velocity
        self.max_a = max_acceleration
    
    def generate(self, distance: float, dt: float) -> List[float]:
        """사다리꼴 속도 프로필 생성"""
        # 가속 시간
        t_accel = self.max_v / self.max_a
        
        # 가속 거리
        s_accel = 0.5 * self.max_a * t_accel ** 2
        
        # 등속 거리
        s_constant = distance - 2 * s_accel
        
        if s_constant < 0:
            # 삼각형 프로필
            t_accel = np.sqrt(distance / self.max_a)
            t_total = 2 * t_accel
        else:
            # 사다리꼴 프로필
            t_constant = s_constant / self.max_v
            t_total = 2 * t_accel + t_constant
        
        # 속도 프로필
        velocities = []
        t = 0
        while t <= t_total:
            if t <= t_accel:
                v = self.max_a * t
            elif t <= t_total - t_accel:
                v = self.max_v
            else:
                v = self.max_v - self.max_a * (t - (t_total - t_accel))
            velocities.append(max(0, v))
            t += dt
        
        return velocities


class PIDController:
    """PID 제어기"""
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0
        self.prev_error = 0
    
    def update(self, error: float, dt: float) -> float:
        """PID 제어 업데이트"""
        # 비례 항
        p = self.kp * error
        
        # 적분 항
        self.integral += error * dt
        i = self.ki * self.integral
        
        # 미분 항
        d = self.kd * (error - self.prev_error) / dt if dt > 0 else 0
        
        self.prev_error = error
        
        return p + i + d
    
    def reset(self):
        """제어기 리셋"""
        self.integral = 0
        self.prev_error = 0


class Quaternion:
    """사원수 연산"""
    
    def __init__(self, x: float, y: float, z: float, w: float):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def normalize(self) -> 'Quaternion':
        """정규화"""
        norm = np.sqrt(self.x**2 + self.y**2 + self.z**2 + self.w**2)
        if norm == 0:
            return Quaternion(0, 0, 0, 1)
        return Quaternion(self.x/norm, self.y/norm, self.z/norm, self.w/norm)
    
    def to_euler(self) -> Tuple[float, float, float]:
        """사원수를 오일러 각으로 변환"""
        # 롤 (x축 회전)
        sinr_cosp = 2 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1 - 2 * (self.x**2 + self.y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # 피치 (y축 회전)
        sinp = 2 * (self.w * self.y - self.z * self.x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        
        # 요 (z축 회전)
        siny_cosp = 2 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1 - 2 * (self.y**2 + self.z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
