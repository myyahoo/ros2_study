#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Import here to avoid issues with moveit
        from moveit_commander import MoveGroupCommander, RobotCommander
        from moveit_commander import PlanningSceneInterface
        
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")
        
        # Set parameters
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(5)
        self.group.allow_replanning(True)
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)
        
        self.get_logger().info('Arm Controller initialized')
    
    def go_to_home(self):
        """Move arm to home position"""
        self.get_logger().info('Moving to home position...')
        self.group.set_named_target("home")
        plan = self.group.plan()
        
        if plan[0]:
            self.group.execute(plan[1])
            self.get_logger().info('Home position reached')
        else:
            self.get_logger().error('Failed to plan to home position')
    
    def go_to_ready(self):
        """Move arm to ready position"""
        self.get_logger().info('Moving to ready position...')
        self.group.set_named_target("ready")
        plan = self.group.plan()
        
        if plan[0]:
            self.group.execute(plan[1])
            self.get_logger().info('Ready position reached')
        else:
            self.get_logger().error('Failed to plan to ready position')
    
    def go_to_pose(self, x, y, z, roll=0, pitch=0, yaw=0):
        """Move end effector to specified pose"""
        self.get_logger().info(f'Moving to pose: x={x}, y={y}, z={z}')
        
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # Convert roll, pitch, yaw to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        
        self.group.set_pose_target(pose_goal)
        plan = self.group.plan()
        
        if plan[0]:
            self.group.execute(plan[1])
            self.get_logger().info('Pose reached')
        else:
            self.get_logger().error('Failed to plan to pose')
    
    def print_joint_values(self):
        """Print current joint values"""
        joint_values = self.group.get_current_joint_values()
        self.get_logger().info(f'Current joint values: {joint_values}')
        return joint_values
    
    def print_end_effector_pose(self):
        """Print current end effector pose"""
        pose = self.group.get_current_pose("link_6")
        self.get_logger().info(f'End effector pose: {pose.pose}')
        return pose.pose


def main():
    rclpy.init()
    
    arm_controller = ArmController()
    
    try:
        # Example 1: Go to home position
        arm_controller.go_to_home()
        arm_controller.print_joint_values()
        
        # Wait a bit
        import time
        time.sleep(2)
        
        # Example 2: Go to ready position
        arm_controller.go_to_ready()
        arm_controller.print_joint_values()
        
        # Wait a bit
        time.sleep(2)
        
        # Example 3: Move to a specific pose
        arm_controller.go_to_pose(x=0.3, y=0.1, z=0.5)
        arm_controller.print_end_effector_pose()
        
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
