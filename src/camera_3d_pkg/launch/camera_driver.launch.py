#!/usr/bin/env python3
# Copyright 2024 Camera 3D Package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Simple launch file for 3D camera driver only
    
    Usage:
        ros2 launch camera_3d_pkg camera_driver.launch.py
    """
    
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='simulated',
        description='Type of camera'
    )
    
    camera_3d_node = Node(
        package='camera_3d_pkg',
        executable='camera_3d_node',
        name='camera_3d_node',
        namespace='camera',
        parameters=[
            {
                'camera_type': LaunchConfiguration('camera_type'),
                'enable_pointcloud': True,
            }
        ],
        output='screen',
    )
    
    return LaunchDescription([
        camera_type_arg,
        camera_3d_node,
    ])
