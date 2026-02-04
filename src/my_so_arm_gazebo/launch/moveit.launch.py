import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import yaml

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_so_arm_gazebo')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'arm.urdf.xacro')
    srdf_file = os.path.join(pkg_dir, 'config', 'arm.srdf')
    
    # Generate URDF from xacro
    robot_description = Command(['xacro', ' ', urdf_file])
    
    # Load SRDF
    robot_description_semantic = open(srdf_file).read()
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        
        # MoveIt2 Node
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': load_yaml(
                        pkg_dir, 'config/kinematics.yaml'
                    ),
                    'ompl_planning_pipeline_config': load_yaml(
                        pkg_dir, 'config/ompl_planning.yaml'
                    ),
                    'use_sim_time': True,
                },
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('~monitored_planning_scene', 'planning_scene'),
            ]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'moveit.rviz')]
        ),
    ])

def load_yaml(pkg_dir, file_name):
    import yaml
    file_path = os.path.join(pkg_dir, file_name)
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)
