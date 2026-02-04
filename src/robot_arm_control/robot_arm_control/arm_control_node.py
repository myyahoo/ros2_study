#!/usr/bin/env python3
"""
로봇 암 제어 노드: So-arm101 (6-DOF) 제어 및 MoveIt2 통합
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from typing import List, Tuple

# MoveIt2 관련 imports
try:
    from moveit_msgs.srv import GetPositionIK, GetPositionFK
    from moveit_msgs.msg import MoveItErrorCodes
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.action import FollowJointTrajectory
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class SimpleIK:
    """So-arm101용 간단한 역기구학 솔버"""
    
    def __init__(self):
        # So-arm101의 DH 파라미터 (근사값)
        self.arm_length = [0.04, 0.15, 0.15, 0.04, 0.08, 0.05]  # 각 링크 길이 (m)
        self.num_joints = 6
    
    def forward_kinematics(self, joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """정기구학: 관절 각도 → 엔드 이펙터 위치"""
        x, y, z = 0, 0, 0
        roll, pitch, yaw = 0, 0, 0
        
        for i in range(min(3, len(joint_angles))):  # 처음 3개 관절로 XYZ 계산
            angle = joint_angles[i]
            if i == 0:  # 베이스 회전
                x += self.arm_length[i] * np.cos(angle)
                y += self.arm_length[i] * np.sin(angle)
            elif i == 1:  # 첫 번째 팔
                x += self.arm_length[i] * np.cos(joint_angles[0]) * np.cos(angle)
                y += self.arm_length[i] * np.sin(joint_angles[0]) * np.cos(angle)
                z += self.arm_length[i] * np.sin(angle)
            elif i == 2:  # 두 번째 팔
                combined_angle = joint_angles[1] + angle
                x += self.arm_length[i] * np.cos(joint_angles[0]) * np.cos(combined_angle)
                y += self.arm_length[i] * np.sin(joint_angles[0]) * np.cos(combined_angle)
                z += self.arm_length[i] * np.sin(combined_angle)
        
        position = np.array([x, y, z])
        orientation = np.array([roll, pitch, yaw])
        return position, orientation
    
    def inverse_kinematics(self, target_position: np.ndarray, 
                          target_orientation: np.ndarray = None) -> List[float]:
        """역기구학: 목표 위치 → 관절 각도"""
        x, y, z = target_position
        
        # 베이스 회전 계산
        theta1 = np.arctan2(y, x)
        
        # XY 평면 거리
        r = np.sqrt(x**2 + y**2)
        
        # 팔 길이들
        l1 = self.arm_length[1]
        l2 = self.arm_length[2]
        total_length = l1 + l2
        
        # 거리 확인 (도달 가능성)
        distance = np.sqrt(r**2 + z**2)
        if distance > total_length or distance < abs(l1 - l2):
            self.get_logger().warn(f'Target position {target_position} is not reachable')
            # 근사 솔루션 반환
            return [theta1, 0.5, 0.5, 0, 0, 0]
        
        # 엘보우 각도 계산 (코사인 법칙)
        cos_theta3 = (distance**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_theta3 = np.clip(cos_theta3, -1, 1)
        theta3 = np.arccos(cos_theta3)
        
        # 숄더 각도 계산
        alpha = np.arctan2(z, r)
        beta = np.arctan2(l2 * np.sin(theta3), l1 + l2 * np.cos(theta3))
        theta2 = alpha - beta
        
        # 손목 관절들 (간단한 설정)
        theta4 = 0  # 손목 회전
        theta5 = 0  # 손목 피치
        theta6 = 0  # 그리퍼 회전
        
        return [theta1, theta2, theta3, theta4, theta5, theta6]


class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        
        self.get_logger().info('Arm Control Node initialized')
        
        # 파라미터 선언
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('joint_max_velocity', 1.0)  # rad/s
        self.declare_parameter('use_moveit', True)
        
        self.num_joints = self.get_parameter('num_joints').value
        self.max_velocity = self.get_parameter('joint_max_velocity').value
        self.use_moveit = self.get_parameter('use_moveit').value
        
        # 현재 관절 상태
        self.current_joint_angles = np.zeros(self.num_joints)
        self.current_joint_velocities = np.zeros(self.num_joints)
        self.gripper_open = True
        
        # 역기구학 솔버 초기화
        self.ik_solver = SimpleIK()
        
        # MoveIt2 클라이언트 초기화
        self.moveit_ik_client = None
        self.moveit_fk_client = None
        
        if self.use_moveit and MOVEIT_AVAILABLE:
            self.init_moveit_clients()
        
        # 타이머 (50Hz 제어 루프)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info(f'Arm initialized with {self.num_joints} joints')
        self.get_logger().info(f'MoveIt2 integration: {"enabled" if self.use_moveit else "disabled"}')
    
    def init_moveit_clients(self):
        """MoveIt2 클라이언트 초기화"""
        try:
            self.moveit_ik_client = self.create_client(
                GetPositionIK, 
                '/compute_ik'
            )
            self.moveit_fk_client = self.create_client(
                GetPositionFK,
                '/compute_fk'
            )
            
            # 서비스 대기
            if not self.moveit_ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('IK service not available')
                self.use_moveit = False
            else:
                self.get_logger().info('MoveIt2 clients initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt2: {e}')
            self.use_moveit = False
    
    def control_loop(self):
        """메인 제어 루프"""
        # 실제 구현에서는 모터 드라이버와 통신
        # 여기서는 시뮬레이션
        pass
    
    def calculate_ik(self, target_pose: Pose) -> List[float]:
        """역기구학 계산 (MoveIt2 또는 로컬 IK 사용)"""
        
        if self.use_moveit and self.moveit_ik_client:
            return self.calculate_ik_moveit2(target_pose)
        else:
            return self.calculate_ik_local(target_pose)
    
    def calculate_ik_local(self, target_pose: Pose) -> List[float]:
        """로컬 역기구학 계산"""
        target_pos = np.array([
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        ])
        
        target_orient = np.array([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z
        ])
        
        joint_angles = self.ik_solver.inverse_kinematics(target_pos, target_orient)
        return joint_angles
    
    def calculate_ik_moveit2(self, target_pose: Pose) -> List[float]:
        """MoveIt2를 이용한 역기구학 계산"""
        try:
            from moveit_msgs.srv import GetPositionIK
            from moveit_msgs.msg import RobotState, PositionIKRequest, KinematicLimits
            
            # IK 요청 생성
            ik_request = PositionIKRequest()
            ik_request.group_name = "arm"
            ik_request.pose_stamped.header.frame_id = "base_link"
            ik_request.pose_stamped.pose = target_pose
            ik_request.timeout.sec = 1
            
            # 현재 로봇 상태 설정
            robot_state = RobotState()
            robot_state.joint_state.name = [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
            ]
            robot_state.joint_state.position = self.current_joint_angles.tolist()
            ik_request.robot_state = robot_state
            
            # 서비스 호출
            future = self.moveit_ik_client.call_async(GetPositionIK.Request(ik_request=ik_request))
            
            # 동기 처리 (실제로는 비동기로 처리하는 것이 좋음)
            import time
            timeout = time.time() + 5
            while not future.done() and time.time() < timeout:
                time.sleep(0.01)
            
            if future.done():
                response = future.result()
                if response.error_code.val == 1:  # SUCCESS
                    return list(response.solution.joint_state.position[:6])
            
            self.get_logger().warn('MoveIt2 IK failed, using local solver')
            return self.calculate_ik_local(target_pose)
            
        except Exception as e:
            self.get_logger().error(f'MoveIt2 IK error: {e}')
            return self.calculate_ik_local(target_pose)
    
    def move_to_position(self, joint_angles: List[float], duration: float):
        """관절을 지정된 각도로 이동"""
        # 속도 프로필 생성
        steps = int(duration * 50)  # 50Hz 루프
        
        angle_diff = np.array(joint_angles) - self.current_joint_angles
        
        # 사다리꼴 속도 프로필
        velocities = angle_diff / duration
        velocities = np.clip(velocities, -self.max_velocity, self.max_velocity)
        
        self.get_logger().info(f'Moving to angles: {joint_angles}')
        self.get_logger().info(f'Joint velocities: {velocities}')
    
    def set_gripper(self, open: bool):
        """그리퍼 제어"""
        self.gripper_open = open
        state = "open" if open else "close"
        self.get_logger().info(f'Gripper: {state}')


def main(args=None):
    rclpy.init(args=args)
    arm_control_node = ArmControlNode()
    
    try:
        rclpy.spin(arm_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        arm_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
