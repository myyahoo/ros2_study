import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_so_arm_gazebo')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'arm.urdf.xacro')
    
    # Generate URDF from xacro
    robot_description = Command(['xacro', ' ', urdf_file])
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'visualization.rviz')]
        ),
    ])
