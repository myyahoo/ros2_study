#!/usr/bin/env python3
"""
메인 태스크 실행 노드: 볼펜 감지 → 집기 → 옆으로 이동
상태 머신 기반 제어 + MoveIt2 통합
"""

import rclpy
from rclpy.node import Node
from enum import Enum
import numpy as np

# MoveIt2 관련 imports
try:
    from geometry_msgs.msg import Pose, Point, Quaternion
    from moveit_msgs.srv import GetPositionIK
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False


class TaskState(Enum):
    """작업 상태"""
    IDLE = 0
    DETECT_PEN = 1
    APPROACH_PEN = 2
    GRASP_PEN = 3
    LIFT_PEN = 4
    MOVE_SIDE = 5
    RELEASE_PEN = 6
    RETURN_HOME = 7
    ERROR = 8


class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor_node')
        
        self.get_logger().info('Task Executor Node initialized')
        
        # 파라미터 선언
        self.declare_parameter('task_update_rate', 10)  # Hz
        self.declare_parameter('pen_detection_timeout', 5.0)  # 초
        self.declare_parameter('use_moveit', True)
        
        self.current_state = TaskState.IDLE
        self.previous_state = TaskState.IDLE
        self.use_moveit = self.get_parameter('use_moveit').value
        
        # 타이머 (10Hz)
        self.update_rate = self.get_parameter('task_update_rate').value
        self.task_timer = self.create_timer(1.0 / self.update_rate, self.task_loop)
        
        # 타이밍 변수들
        self.state_entry_time = self.get_clock().now()
        self.detection_timeout = self.get_parameter('pen_detection_timeout').value
        
        # 펜 위치 (시뮬레이션)
        self.pen_position = np.array([0.2, 0.0, 0.1])  # x, y, z
        self.detection_count = 0
        
        self.get_logger().info('Task state machine ready')
    
    def task_loop(self):
        """메인 상태 머신 루프"""
        if self.current_state != self.previous_state:
            self.on_state_enter(self.current_state)
            self.previous_state = self.current_state
        
        # 상태별 처리
        if self.current_state == TaskState.IDLE:
            self.handle_idle()
        elif self.current_state == TaskState.DETECT_PEN:
            self.handle_detect_pen()
        elif self.current_state == TaskState.APPROACH_PEN:
            self.handle_approach_pen()
        elif self.current_state == TaskState.GRASP_PEN:
            self.handle_grasp_pen()
        elif self.current_state == TaskState.LIFT_PEN:
            self.handle_lift_pen()
        elif self.current_state == TaskState.MOVE_SIDE:
            self.handle_move_side()
        elif self.current_state == TaskState.RELEASE_PEN:
            self.handle_release_pen()
        elif self.current_state == TaskState.RETURN_HOME:
            self.handle_return_home()
    
    def on_state_enter(self, state: TaskState):
        """상태 진입 시 호출"""
        self.state_entry_time = self.get_clock().now()
        self.get_logger().info(f'Entering state: {state.name}')
    
    def handle_idle(self):
        """IDLE 상태: 시작 신호 대기"""
        self.get_logger().info('Waiting for task start command...')
        # 자동으로 시작 (실제로는 외부 신호 대기)
        self.current_state = TaskState.DETECT_PEN
    
    def handle_detect_pen(self):
        """DETECT_PEN 상태: 볼펜 감지"""
        self.get_logger().info('Detecting pen...')
        
        # 시뮬레이션: 일정 시간 후 감지
        elapsed_time = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        if elapsed_time > 1.0:  # 1초 후 감지
            self.get_logger().info(f'Pen detected at position: {self.pen_position}')
            self.current_state = TaskState.APPROACH_PEN
        elif elapsed_time > self.detection_timeout:
            self.get_logger().error('Pen detection timeout')
            self.current_state = TaskState.ERROR
    
    def handle_approach_pen(self):
        """APPROACH_PEN 상태: 펜에 접근"""
        self.get_logger().info('Approaching pen...')
        
        # 계산된 관절 각도로 이동
        # pen_position에서 grip offset만큼 위쪽으로
        approach_position = self.pen_position + np.array([0, 0, 0.05])
        
        self.get_logger().info(f'Moving to approach position: {approach_position}')
        
        elapsed_time = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        if elapsed_time > 2.0:  # 2초 이동
            self.current_state = TaskState.GRASP_PEN
    
    def handle_grasp_pen(self):
        """GRASP_PEN 상태: 펜 집기"""
        self.get_logger().info('Grasping pen...')
        
        # 그리퍼 명령: 닫기
        self.get_logger().info('Closing gripper')
        
        elapsed_time = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        if elapsed_time > 1.0:  # 1초 대기
            self.get_logger().info('Pen grasped successfully')
            self.current_state = TaskState.LIFT_PEN
    
    def handle_lift_pen(self):
        """LIFT_PEN 상태: 펜 들어올리기"""
        self.get_logger().info('Lifting pen...')
        
        # Z 축 상승
        lift_position = self.pen_position + np.array([0, 0, 0.15])
        
        self.get_logger().info(f'Lifting to position: {lift_position}')
        
        elapsed_time = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        if elapsed_time > 1.5:
            self.current_state = TaskState.MOVE_SIDE
    
    def handle_move_side(self):
        """MOVE_SIDE 상태: 옆으로 이동"""
        self.get_logger().info('Moving pen to the side...')
        
        # Y 축으로 0.3m 이동
        side_position = self.pen_position + np.array([0, 0.3, 0.15])
        
        self.get_logger().info(f'Moving to side position: {side_position}')
        
        elapsed_time = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        if elapsed_time > 2.0:
            self.current_state = TaskState.RELEASE_PEN
    
    def handle_release_pen(self):
        """RELEASE_PEN 상태: 펜 놓기"""
        self.get_logger().info('Releasing pen...')
        
        # 그리퍼 명령: 열기
        self.get_logger().info('Opening gripper')
        
        elapsed_time = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        if elapsed_time > 1.0:
            self.get_logger().info('Pen released successfully')
            self.current_state = TaskState.RETURN_HOME
    
    def handle_return_home(self):
        """RETURN_HOME 상태: 홈 위치로 복귀"""
        self.get_logger().info('Returning to home position...')
        
        home_position = np.array([0.15, 0, 0.2])
        
        self.get_logger().info(f'Moving to home: {home_position}')
        
        elapsed_time = (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
        
        if elapsed_time > 2.0:
            self.get_logger().info('Task completed successfully!')
            self.current_state = TaskState.IDLE
    
    def print_status(self):
        """현재 상태 출력"""
        self.get_logger().info(f'Current state: {self.current_state.name}')


def main(args=None):
    rclpy.init(args=args)
    task_executor_node = TaskExecutorNode()
    
    try:
        rclpy.spin(task_executor_node)
    except KeyboardInterrupt:
        pass
    finally:
        task_executor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
