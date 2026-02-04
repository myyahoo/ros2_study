#!/usr/bin/env python3
"""
Vision 노드: 카메라에서 입력을 받아 YOLO로 볼펜을 감지하고 OpenCV로 처리
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose
from builtin_interfaces.msg import Time as ROSTime
import cv2
from cv_bridge import CvBridge
import numpy as np
import torch
from pathlib import Path

# YOLO 모델 경로
YOLO_MODEL_PATH = "/home/tech/project/ros2/my_first_ws/src/yolov10m.pt"


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.bridge = CvBridge()
        self.get_logger().info('Vision Node initialized')
        
        # 파라미터 선언
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/robot_arm/detection')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class', 'pen')
        
        # 카메라 구독
        self.camera_topic = self.get_parameter('camera_topic').value
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10
        )
        
        # 감지 결과 발행
        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_publisher = self.create_publisher(
            Image,
            self.detection_topic,
            10
        )
        
        # YOLO 모델 로드
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', 
                                       path=YOLO_MODEL_PATH, force_reload=False)
            self.model.conf = self.get_parameter('confidence_threshold').value
            self.get_logger().info(f'YOLO model loaded from {YOLO_MODEL_PATH}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.target_class = self.get_parameter('target_class').value
    
    def camera_callback(self, msg: Image):
        """카메라 입력 콜백"""
        try:
            # ROS 이미지를 OpenCV 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.model is None:
                self.get_logger().warn('YOLO model not loaded')
                return
            
            # YOLO 추론
            results = self.model(cv_image)
            
            # 결과 처리
            detections = results.xyxy[0].cpu().numpy()
            
            # 이미지에 박스 그리기
            annotated_image = cv_image.copy()
            
            for detection in detections:
                x1, y1, x2, y2, conf, cls = detection
                if conf >= self.confidence_threshold:
                    # 바운딩 박스 그리기
                    cv2.rectangle(annotated_image, (int(x1), int(y1)), 
                                 (int(x2), int(y2)), (0, 255, 0), 2)
                    
                    # 신뢰도 표시
                    label = f'Confidence: {conf:.2f}'
                    cv2.putText(annotated_image, label, (int(x1), int(y1) - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 처리된 이미지 발행
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.detection_publisher.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
