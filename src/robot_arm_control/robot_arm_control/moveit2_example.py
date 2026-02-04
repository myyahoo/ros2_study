#!/usr/bin/env python3
"""
MoveIt2를 이용한 로봇 암 제어 예제
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import numpy as np


class MoveIt2Example(Node):
    def __init__(self):
        super().__init__('moveit2_example')
        
        self.get_logger().info('MoveIt2 Example started')
        
        # MoveIt 컴포넌트 초기화
        try:
            self.robot = RobotCommander(wait_for_servers=5.0)
            self.scene = PlanningSceneInterface()
            self.move_group = MoveGroupCommander("arm", wait_for_servers=5.0)
            
            self.get_logger().info('MoveIt2 initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt2: {e}')
            return
        
        # 로봇 정보 출력
        self.print_robot_info()
        
        # 예제 실행
        self.run_examples()
    
    def print_robot_info(self):
        """로봇 정보 출력"""
        self.get_logger().info('=== Robot Information ===')
        self.get_logger().info(f'Planning Frame: {self.move_group.get_planning_frame()}')
        self.get_logger().info(f'End Effector Link: {self.move_group.get_end_effector_link()}')
        self.get_logger().info(f'Available Planning Groups: {self.robot.get_group_names()}')
        self.get_logger().info(f'Joints: {self.move_group.get_joints()}')
    
    def example1_move_to_joint_goal(self):
        """예제 1: 관절 목표로 이동"""
        self.get_logger().info('\n=== Example 1: Move to Joint Goal ===')
        
        # 관절 각도 설정 (라디안)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0.0     # joint1
        joint_goal[1] = -0.785  # joint2 (-45도)
        joint_goal[2] = 0.785   # joint3 (45도)
        
        # 이동 계획 및 실행
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
        self.get_logger().info('Moved to joint goal')
    
    def example2_move_to_pose_goal(self):
        """예제 2: 엔드 이펙터 위치로 이동"""
        self.get_logger().info('\n=== Example 2: Move to Pose Goal ===')
        
        # 목표 위치 설정
        pose_goal = Pose()
        pose_goal.position.x = 0.2
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.2
        pose_goal.orientation.w = 1.0
        
        # 이동 계획 및 실행
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.plan()
        
        if plan:
            self.move_group.execute(plan[1], wait=True)
            self.get_logger().info('Moved to pose goal')
        else:
            self.get_logger().warn('Planning failed')
        
        self.move_group.clear_pose_targets()
    
    def example3_cartesian_path(self):
        """예제 3: 직선 경로로 이동 (Cartesian Path)"""
        self.get_logger().info('\n=== Example 3: Cartesian Path ===')
        
        # 현재 위치에서 시작
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        
        # 1단계: 위로 이동
        wpose.position.z += 0.1
        waypoints.append(wpose.deepcopy())
        
        # 2단계: 앞으로 이동
        wpose.position.x += 0.1
        waypoints.append(wpose.deepcopy())
        
        # 3단계: 옆으로 이동
        wpose.position.y += 0.1
        waypoints.append(wpose.deepcopy())
        
        # Cartesian 경로 계획
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            eef_step=0.01,  # 1cm
            jump_threshold=0.0
        )
        
        self.get_logger().info(f'Cartesian path achieved: {fraction * 100:.1f}%')
        
        if fraction > 0.5:
            self.move_group.execute(plan, wait=True)
    
    def example4_add_collision_object(self):
        """예제 4: 충돌 객체 추가"""
        self.get_logger().info('\n=== Example 4: Add Collision Object ===')
        
        # 객체 생성
        box = CollisionObject()
        box.header.frame_id = self.move_group.get_planning_frame()
        box.id = "box"
        
        # 상자 정의
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.1, 0.1, 0.1]  # 10cm x 10cm x 10cm
        
        box.primitives.append(primitive)
        box.primitive_poses.append(Pose(
            position=Point(x=0.5, y=0.0, z=0.0)
        ))
        
        # 장면에 추가
        self.scene.add_object(box)
        self.get_logger().info('Added collision object: box')
    
    def example5_gripper_control(self):
        """예제 5: 그리퍼 제어"""
        self.get_logger().info('\n=== Example 5: Gripper Control ===')
        
        try:
            gripper_group = MoveGroupCommander("gripper")
            
            # 그리퍼 열기
            gripper_joints = gripper_group.get_joints()
            gripper_goal = gripper_group.get_current_joint_values()
            gripper_goal[0] = 0.0     # 왼쪽 손가락 열기
            gripper_goal[1] = 0.0     # 오른쪽 손가락 열기
            
            gripper_group.go(gripper_goal, wait=True)
            self.get_logger().info('Gripper opened')
            
            # 그리퍼 닫기
            gripper_goal[0] = -0.03   # 왼쪽 손가락 닫기
            gripper_goal[1] = -0.03   # 오른쪽 손가락 닫기
            
            gripper_group.go(gripper_goal, wait=True)
            self.get_logger().info('Gripper closed')
            
        except Exception as e:
            self.get_logger().warn(f'Gripper control failed: {e}')
    
    def run_examples(self):
        """모든 예제 실행"""
        try:
            self.example1_move_to_joint_goal()
            rclpy.spin_once(self, timeout_sec=2.0)
            
            self.example2_move_to_pose_goal()
            rclpy.spin_once(self, timeout_sec=2.0)
            
            self.example3_cartesian_path()
            rclpy.spin_once(self, timeout_sec=2.0)
            
            self.example4_add_collision_object()
            rclpy.spin_once(self, timeout_sec=1.0)
            
            self.example5_gripper_control()
            
            self.get_logger().info('\n=== All examples completed ===')
            
        except Exception as e:
            self.get_logger().error(f'Error during examples: {e}')


def main(args=None):
    rclpy.init(args=args)
    example = MoveIt2Example()
    
    try:
        rclpy.spin(example)
    except KeyboardInterrupt:
        pass
    finally:
        example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
