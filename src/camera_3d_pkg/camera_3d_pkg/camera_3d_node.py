#!/usr/bin/env python3
# Copyright 2024 Camera 3D Package
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
import time
from datetime import datetime
import os

# Custom service imports - will be auto-generated
try:
    from camera_3d_interfaces.srv import CaptureFrame, GetCameraInfo, ConfigureCamera
except ImportError:
    # Fallback for development
    pass


class Camera3DNode(Node):
    """
    3D Camera Driver Node for RGB-D cameras
    Supports: RealSense, Kinect, and simulated cameras
    
    Publishes:
    - /camera/rgb/image_raw: RGB image
    - /camera/depth/image_raw: Depth image
    - /camera/ir/image_raw: Infrared image (if available)
    - /camera/rgb/camera_info: RGB camera info
    - /camera/depth/camera_info: Depth camera info
    - /camera/points: Point cloud data
    
    Services:
    - /camera/capture_frame: Capture and save frame
    - /camera/get_camera_info: Get camera specifications
    - /camera/configure_camera: Configure camera parameters
    """
    
    def __init__(self):
        super().__init__('camera_3d_node')
        
        self.get_logger().info('Initializing Camera 3D Node')
        
        # Parameters
        self.declare_parameter('camera_type', 'simulated')  # simulated, realsense, kinect
        self.declare_parameter('camera_id', '0')
        self.declare_parameter('rgb_fps', 30)
        self.declare_parameter('depth_fps', 30)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('enable_rgb', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_ir', False)
        self.declare_parameter('enable_pointcloud', True)
        self.declare_parameter('tf_prefix', 'camera')
        self.declare_parameter('output_dir', '/tmp/camera_captures')
        
        self.camera_type = self.get_parameter('camera_type').value
        self.camera_id = self.get_parameter('camera_id').value
        self.rgb_fps = self.get_parameter('rgb_fps').value
        self.depth_fps = self.get_parameter('depth_fps').value
        self.depth_width = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.enable_rgb = self.get_parameter('enable_rgb').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.enable_ir = self.get_parameter('enable_ir').value
        self.enable_pointcloud = self.get_parameter('enable_pointcloud').value
        self.tf_prefix = self.get_parameter('tf_prefix').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # QoS Profile for images
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Publishers
        self.rgb_publisher = self.create_publisher(
            Image, f'{self.tf_prefix}/rgb/image_raw', qos_profile
        )
        self.depth_publisher = self.create_publisher(
            Image, f'{self.tf_prefix}/depth/image_raw', qos_profile
        )
        self.ir_publisher = self.create_publisher(
            Image, f'{self.tf_prefix}/ir/image_raw', qos_profile
        )
        self.rgb_info_publisher = self.create_publisher(
            CameraInfo, f'{self.tf_prefix}/rgb/camera_info', qos_profile
        )
        self.depth_info_publisher = self.create_publisher(
            CameraInfo, f'{self.tf_prefix}/depth/camera_info', qos_profile
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, f'{self.tf_prefix}/points', qos_profile
        )
        
        # Services
        self.capture_service = self.create_service(
            CaptureFrame, f'{self.tf_prefix}/capture_frame',
            self.capture_frame_callback
        )
        self.camera_info_service = self.create_service(
            GetCameraInfo, f'{self.tf_prefix}/get_camera_info',
            self.get_camera_info_callback
        )
        self.configure_service = self.create_service(
            ConfigureCamera, f'{self.tf_prefix}/configure_camera',
            self.configure_camera_callback
        )
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Camera initialization
        self.camera = None
        self.seq_counter = 0
        self.init_camera()
        
        # Timer for publishing
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
        
        self.get_logger().info(
            f'Camera 3D Node initialized successfully '
            f'(Type: {self.camera_type}, ID: {self.camera_id})'
        )
    
    def init_camera(self):
        """Initialize camera based on camera_type parameter"""
        if self.camera_type == 'simulated':
            self.camera = SimulatedCamera(
                width=self.depth_width,
                height=self.depth_height
            )
        else:
            self.get_logger().warn(
                f'Camera type "{self.camera_type}" not fully supported. '
                f'Using simulated camera.'
            )
            self.camera = SimulatedCamera(
                width=self.depth_width,
                height=self.depth_height
            )
        self.get_logger().info('Camera initialized')
    
    def timer_callback(self):
        """Publish camera frames"""
        if self.camera is None:
            return
        
        try:
            # Get frames from camera
            rgb_frame, depth_frame, ir_frame = self.camera.capture_frames()
            
            if rgb_frame is None or depth_frame is None:
                return
            
            timestamp = self.get_clock().now().to_msg()
            self.seq_counter += 1
            
            # Publish RGB image
            if self.enable_rgb and rgb_frame is not None:
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
                rgb_msg.header.stamp = timestamp
                rgb_msg.header.frame_id = f'{self.tf_prefix}_rgb_optical_frame'
                rgb_msg.header.seq = self.seq_counter
                self.rgb_publisher.publish(rgb_msg)
                
                # Publish RGB camera info
                rgb_info = self.get_rgb_camera_info(timestamp)
                self.rgb_info_publisher.publish(rgb_info)
            
            # Publish Depth image
            if self.enable_depth and depth_frame is not None:
                # Convert to uint16 for standard depth representation
                depth_uint16 = (depth_frame * 1000).astype(np.uint16)
                depth_msg = self.bridge.cv2_to_imgmsg(
                    depth_uint16, encoding='mono16'
                )
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = f'{self.tf_prefix}_depth_optical_frame'
                depth_msg.header.seq = self.seq_counter
                self.depth_publisher.publish(depth_msg)
                
                # Publish Depth camera info
                depth_info = self.get_depth_camera_info(timestamp)
                self.depth_info_publisher.publish(depth_info)
            
            # Publish IR image
            if self.enable_ir and ir_frame is not None:
                ir_msg = self.bridge.cv2_to_imgmsg(ir_frame, encoding='mono8')
                ir_msg.header.stamp = timestamp
                ir_msg.header.frame_id = f'{self.tf_prefix}_ir_optical_frame'
                ir_msg.header.seq = self.seq_counter
                self.ir_publisher.publish(ir_msg)
            
            # Publish Point Cloud
            if self.enable_pointcloud and depth_frame is not None:
                pointcloud_msg = self.create_pointcloud(
                    depth_frame, rgb_frame, timestamp
                )
                self.pointcloud_publisher.publish(pointcloud_msg)
            
            # Broadcast TF
            self.broadcast_transforms(timestamp)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')
    
    def get_rgb_camera_info(self, timestamp):
        """Create RGB camera info message"""
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = f'{self.tf_prefix}_rgb_optical_frame'
        info.width = self.depth_width
        info.height = self.depth_height
        
        # Intrinsic parameters (standard for 640x480)
        fx = 606.44
        fy = 606.44
        cx = 320.0
        cy = 240.0
        
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return info
    
    def get_depth_camera_info(self, timestamp):
        """Create Depth camera info message"""
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = f'{self.tf_prefix}_depth_optical_frame'
        info.width = self.depth_width
        info.height = self.depth_height
        
        # Intrinsic parameters (typically same as RGB for RGB-D cameras)
        fx = 606.44
        fy = 606.44
        cx = 320.0
        cy = 240.0
        
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return info
    
    def create_pointcloud(self, depth_frame, rgb_frame, timestamp):
        """Create point cloud from depth and RGB images"""
        # Camera intrinsics
        fx = 606.44
        fy = 606.44
        cx = 320.0
        cy = 240.0
        
        points = []
        colors = []
        
        h, w = depth_frame.shape
        
        for v in range(h):
            for u in range(w):
                z = depth_frame[v, u]
                
                if z > 0 and z < 10:  # Valid depth range
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    points.append([x, y, z])
                    
                    if rgb_frame is not None and len(rgb_frame.shape) == 3:
                        b, g, r = rgb_frame[v, u]
                        rgb = (int(r) << 16) + (int(g) << 8) + int(b)
                        colors.append(rgb)
                    else:
                        colors.append(0xFFFFFF)
        
        if not points:
            return PointCloud2()
        
        # Create point cloud message
        points_array = np.array(points, dtype=np.float32)
        colors_array = np.array(colors, dtype=np.uint32)
        
        points_list = []
        for i in range(len(points)):
            point = [
                points_array[i, 0],
                points_array[i, 1],
                points_array[i, 2],
                colors_array[i],
            ]
            points_list.append(point)
        
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgba', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]
        
        pointcloud_msg = pc2.create_cloud(
            header=Header(
                stamp=timestamp,
                frame_id=f'{self.tf_prefix}_depth_optical_frame'
            ),
            fields=fields,
            points=points_list
        )
        
        return pointcloud_msg
    
    def broadcast_transforms(self, timestamp):
        """Broadcast camera frame transforms"""
        transforms = [
            ('world', f'{self.tf_prefix}_link', [0.0, 0.0, 0.0]),
            (f'{self.tf_prefix}_link', f'{self.tf_prefix}_rgb_optical_frame', [0.0, 0.0, 0.0]),
            (f'{self.tf_prefix}_link', f'{self.tf_prefix}_depth_optical_frame', [0.0, 0.0, 0.0]),
        ]
        
        for parent, child, translation in transforms:
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
    
    def capture_frame_callback(self, request, response):
        """Service callback for capturing and saving frames"""
        try:
            rgb_frame, depth_frame, _ = self.camera.capture_frames()
            
            if rgb_frame is None or depth_frame is None:
                response.success = False
                response.message = 'Failed to capture frames'
                return response
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            
            if request.save_image:
                save_path = request.output_path or self.output_dir
                os.makedirs(save_path, exist_ok=True)
                
                # Save RGB image
                rgb_path = os.path.join(save_path, f'rgb_{timestamp}.png')
                cv2.imwrite(rgb_path, rgb_frame)
                
                # Save Depth image
                depth_path = os.path.join(save_path, f'depth_{timestamp}.png')
                depth_uint16 = (depth_frame * 1000).astype(np.uint16)
                cv2.imwrite(depth_path, depth_uint16)
                
                self.get_logger().info(
                    f'Frames saved: RGB={rgb_path}, Depth={depth_path}'
                )
            
            response.success = True
            response.message = f'Frame captured at {timestamp}'
            response.depth_frame_id = f'{self.tf_prefix}_depth_optical_frame'
            response.rgb_frame_id = f'{self.tf_prefix}_rgb_optical_frame'
            
        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(f'Capture frame error: {str(e)}')
        
        return response
    
    def get_camera_info_callback(self, request, response):
        """Service callback for getting camera information"""
        try:
            response.camera_model = 'Simulated RGB-D Camera'
            response.serial_number = self.camera_id
            response.fx = 606.44
            response.fy = 606.44
            response.cx = 320.0
            response.cy = 240.0
            response.baseline = 0.075  # 7.5 cm baseline
            response.width = self.depth_width
            response.height = self.depth_height
            
            self.get_logger().info(f'Camera info requested for {request.camera_id}')
            
        except Exception as e:
            self.get_logger().error(f'Get camera info error: {str(e)}')
        
        return response
    
    def configure_camera_callback(self, request, response):
        """Service callback for configuring camera parameters"""
        try:
            self.enable_rgb = request.enable_rgb
            self.enable_depth = request.enable_depth
            self.enable_ir = request.enable_ir
            self.rgb_fps = request.rgb_fps
            self.depth_fps = request.depth_fps
            
            response.success = True
            response.message = (
                f'Camera configured: RGB={self.enable_rgb}, '
                f'Depth={self.enable_depth}, IR={self.enable_ir}'
            )
            
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Configuration error: {str(e)}'
            self.get_logger().error(response.message)
        
        return response


class SimulatedCamera:
    """Simulated RGB-D camera for testing purposes"""
    
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.frame_count = 0
    
    def capture_frames(self):
        """Generate synthetic RGB and depth frames"""
        self.frame_count += 1
        
        # Create synthetic RGB image (gradients and patterns)
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Add gradient
        for i in range(self.height):
            rgb[i, :] = [
                int(255 * i / self.height),
                int(128 + 127 * np.sin(self.frame_count * 0.01 + i / 100)),
                int(255 - 255 * i / self.height)
            ]
        
        # Add some patterns
        for x in range(100, 300):
            for y in range(100, 300):
                rgb[y, x] = [200, 100, 50]
        
        # Create synthetic depth image
        depth = np.zeros((self.height, self.width), dtype=np.float32)
        
        # Center point closer, edges farther
        center_x, center_y = self.width // 2, self.height // 2
        for y in range(self.height):
            for x in range(self.width):
                dist = np.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
                depth[y, x] = 2.0 + dist / 300.0  # 2-4 meters
        
        # Create synthetic IR image
        ir = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        ir = (ir * 0.7).astype(np.uint8)
        
        return rgb, depth, ir


def main(args=None):
    rclpy.init(args=args)
    node = Camera3DNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Camera 3D Node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
