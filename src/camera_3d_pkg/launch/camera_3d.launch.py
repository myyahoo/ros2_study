#!/usr/bin/env python3
# Copyright 2024 Camera 3D Package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Main launch file for 3D camera system
    
    Usage:
        ros2 launch camera_3d_pkg camera_3d.launch.py
        ros2 launch camera_3d_pkg camera_3d.launch.py \
            camera_type:=realsense \
            enable_pointcloud:=true \
            rgb_fps:=30 \
            depth_fps:=30
    """
    
    # Declare arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='simulated',
        choices=['simulated', 'realsense', 'kinect'],
        description='Type of camera to use'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera ID or device index'
    )
    
    rgb_fps_arg = DeclareLaunchArgument(
        'rgb_fps',
        default_value='30',
        description='RGB frame rate'
    )
    
    depth_fps_arg = DeclareLaunchArgument(
        'depth_fps',
        default_value='30',
        description='Depth frame rate'
    )
    
    depth_width_arg = DeclareLaunchArgument(
        'depth_width',
        default_value='640',
        description='Depth image width'
    )
    
    depth_height_arg = DeclareLaunchArgument(
        'depth_height',
        default_value='480',
        description='Depth image height'
    )
    
    enable_rgb_arg = DeclareLaunchArgument(
        'enable_rgb',
        default_value='true',
        choices=['true', 'false'],
        description='Enable RGB stream'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        choices=['true', 'false'],
        description='Enable depth stream'
    )
    
    enable_ir_arg = DeclareLaunchArgument(
        'enable_ir',
        default_value='false',
        choices=['true', 'false'],
        description='Enable infrared stream'
    )
    
    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        choices=['true', 'false'],
        description='Enable point cloud generation'
    )
    
    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='camera',
        description='Transform frame prefix'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/camera_captures',
        description='Output directory for captured images'
    )
    
    # Camera 3D Node
    camera_3d_node = Node(
        package='camera_3d_pkg',
        executable='camera_3d_node',
        name='camera_3d_node',
        namespace='camera',
        parameters=[
            {
                'camera_type': LaunchConfiguration('camera_type'),
                'camera_id': LaunchConfiguration('camera_id'),
                'rgb_fps': LaunchConfiguration('rgb_fps'),
                'depth_fps': LaunchConfiguration('depth_fps'),
                'depth_width': LaunchConfiguration('depth_width'),
                'depth_height': LaunchConfiguration('depth_height'),
                'enable_rgb': LaunchConfiguration('enable_rgb'),
                'enable_depth': LaunchConfiguration('enable_depth'),
                'enable_ir': LaunchConfiguration('enable_ir'),
                'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
                'tf_prefix': LaunchConfiguration('tf_prefix'),
                'output_dir': LaunchConfiguration('output_dir'),
            }
        ],
        output='screen',
    )
    
    # Point Cloud Node
    pointcloud_node = Node(
        package='camera_3d_pkg',
        executable='pointcloud_node',
        name='pointcloud_node',
        namespace='camera',
        parameters=[
            {
                'camera_frame': [LaunchConfiguration('tf_prefix'), '_depth_optical_frame'],
                'min_distance': 0.1,
                'max_distance': 5.0,
                'voxel_size': 0.01,
                'enable_filtering': True,
            }
        ],
        output='screen',
    )
    
    return LaunchDescription([
        camera_type_arg,
        camera_id_arg,
        rgb_fps_arg,
        depth_fps_arg,
        depth_width_arg,
        depth_height_arg,
        enable_rgb_arg,
        enable_depth_arg,
        enable_ir_arg,
        enable_pointcloud_arg,
        tf_prefix_arg,
        output_dir_arg,
        camera_3d_node,
        pointcloud_node,
    ])
