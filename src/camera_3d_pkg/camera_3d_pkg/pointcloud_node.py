#!/usr/bin/env python3
# Copyright 2024 Camera 3D Package
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import sensor_msgs.point_cloud2 as pc2


class PointCloudNode(Node):
    """
    Point Cloud Processing Node
    
    Subscribes to:
    - /camera/points: Point cloud from camera
    - /camera/depth/image_raw: Depth image
    
    Publishes:
    - /camera/points_filtered: Filtered point cloud
    - /camera/object_centroid: Object centroid position
    - /camera/plane_detection: Detected plane coefficients
    """
    
    def __init__(self):
        super().__init__('pointcloud_node')
        
        self.get_logger().info('Initializing Point Cloud Node')
        
        # Parameters
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        self.declare_parameter('min_distance', 0.1)
        self.declare_parameter('max_distance', 5.0)
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('enable_filtering', True)
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.enable_filtering = self.get_parameter('enable_filtering').value
        
        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscribers
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            'camera/points',
            self.pointcloud_callback,
            qos_profile
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.depth_callback,
            qos_profile
        )
        
        # Publishers
        self.filtered_pc_publisher = self.create_publisher(
            PointCloud2, 'camera/points_filtered', qos_profile
        )
        
        self.centroid_publisher = self.create_publisher(
            Float32MultiArray, 'camera/object_centroid', qos_profile
        )
        
        self.plane_publisher = self.create_publisher(
            Float32MultiArray, 'camera/plane_detection', qos_profile
        )
        
        self.get_logger().info('Point Cloud Node initialized successfully')
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming point cloud"""
        try:
            # Convert point cloud to numpy array
            points = np.array(list(pc2.read_points(msg)))
            
            if len(points) == 0:
                self.get_logger().warn('Empty point cloud received')
                return
            
            # Extract coordinates
            xyz = points[:, :3]
            
            # Filter by distance
            if self.enable_filtering:
                distances = np.linalg.norm(xyz, axis=1)
                mask = (distances >= self.min_distance) & (distances <= self.max_distance)
                xyz_filtered = xyz[mask]
                
                self.get_logger().debug(
                    f'Filtered points: {len(xyz_filtered)}/{len(xyz)}'
                )
            else:
                xyz_filtered = xyz
            
            if len(xyz_filtered) > 0:
                # Voxel downsampling
                xyz_downsampled = self.voxel_downsample(xyz_filtered)
                
                # Create filtered point cloud message
                fields = [
                    pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                ]
                
                points_list = [[p[0], p[1], p[2]] for p in xyz_downsampled]
                
                filtered_pc = pc2.create_cloud(
                    header=msg.header,
                    fields=fields,
                    points=points_list
                )
                
                self.filtered_pc_publisher.publish(filtered_pc)
                
                # Calculate and publish centroid
                centroid = np.mean(xyz_downsampled, axis=0)
                self.publish_centroid(centroid, msg.header.stamp)
                
                # Detect and publish plane
                plane_model = self.ransac_plane_detection(xyz_filtered)
                if plane_model is not None:
                    self.publish_plane(plane_model, msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error in point cloud callback: {str(e)}')
    
    def depth_callback(self, msg: Image):
        """Process depth image"""
        # This can be extended for depth-specific processing
        pass
    
    def voxel_downsample(self, points, voxel_size=None):
        """Downsample points using voxel grid"""
        if voxel_size is None:
            voxel_size = self.voxel_size
        
        # Create voxel grid
        voxel_coords = np.floor(points / voxel_size).astype(int)
        
        # Get unique voxel coordinates
        unique_voxels, indices = np.unique(
            voxel_coords, axis=0, return_index=True
        )
        
        # Return one point per voxel
        downsampled = points[indices]
        
        return downsampled
    
    def ransac_plane_detection(self, points, max_iterations=1000, threshold=0.01):
        """
        Detect plane using RANSAC algorithm
        Returns plane coefficients [a, b, c, d] for ax + by + cz + d = 0
        """
        if len(points) < 3:
            return None
        
        best_model = None
        best_inliers = 0
        
        for _ in range(max_iterations):
            # Randomly select 3 points
            indices = np.random.choice(len(points), 3, replace=False)
            p1, p2, p3 = points[indices]
            
            # Calculate plane normal
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            
            if np.linalg.norm(normal) < 1e-10:
                continue
            
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, p1)
            
            # Count inliers
            distances = np.abs(np.dot(points, normal) + d)
            inliers = np.sum(distances < threshold)
            
            if inliers > best_inliers:
                best_inliers = inliers
                best_model = np.append(normal, d)
        
        if best_inliers > len(points) * 0.1:  # At least 10% inliers
            return best_model
        
        return None
    
    def publish_centroid(self, centroid, stamp):
        """Publish object centroid"""
        msg = Float32MultiArray()
        msg.data = [float(x) for x in centroid]
        self.centroid_publisher.publish(msg)
    
    def publish_plane(self, plane_model, stamp):
        """Publish detected plane coefficients"""
        msg = Float32MultiArray()
        msg.data = [float(x) for x in plane_model]
        self.plane_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Point Cloud Node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
