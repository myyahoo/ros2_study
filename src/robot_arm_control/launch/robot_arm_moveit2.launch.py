"""
MoveIt2 + Robot Arm Control 통합 Launch 파일
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    robot_arm_pkg = FindPackageShare("robot_arm_control")
    
    return LaunchDescription([
        # URDF 발행
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": PathJoinSubstitution([
                    robot_arm_pkg, "urdf", "so_arm101.urdf"
                ])
            }]
        ),

        # RViz 시각화
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d", PathJoinSubstitution([
                    robot_arm_pkg, "moveit_config", "config", "moveit.rviz"
                ])
            ]
        ),

        # MoveIt2 Move Group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    robot_arm_pkg, "moveit_config", "config", "kinematics.yaml"
                ]),
                PathJoinSubstitution([
                    robot_arm_pkg, "moveit_config", "config", "ompl_planning.yaml"
                ]),
            ]
        ),

        # Arm Control Node (MoveIt2 통합)
        Node(
            package="robot_arm_control",
            executable="arm_control_node",
            name="arm_control_node",
            output="screen",
            parameters=[
                {"use_moveit": True},
                {"num_joints": 6},
            ]
        ),

        # Vision Node
        Node(
            package="robot_arm_control",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[
                {"confidence_threshold": 0.5},
            ]
        ),

        # Task Executor Node
        Node(
            package="robot_arm_control",
            executable="task_executor_node",
            name="task_executor_node",
            output="screen",
            parameters=[
                {"use_moveit": True},
            ]
        ),
    ])
