#!/usr/bin/env python3
"""
ROS 2 노드: 비디오에서 YOLO를 사용하여 사람 감지
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from yolo_msgs.msg import Detection, DetectionArray, Point2D, BoundingBox2D


class VideoPersonDetectionNode(Node):
    """ROS 2 노드: 비디오에서 YOLO를 사용하여 사람 감지"""

    def __init__(self):
        """노드 초기화"""
        super().__init__("video_person_detection_node")

        # 파라미터 선언
        self.declare_parameter("video_source", 0)  # 0: 카메라, 또는 비디오 파일 경로
        self.declare_parameter("model_path", "yolov8m.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("skip_frames", 1)

        # 파라미터 값 읽기
        self.video_source = (
            self.get_parameter("video_source").get_parameter_value().integer_value
        )
        self.model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.confidence_threshold = (
            self.get_parameter("confidence_threshold").get_parameter_value().double_value
        )
        self.skip_frames = (
            self.get_parameter("skip_frames").get_parameter_value().integer_value
        )

        # YOLO 모델 로드
        self.get_logger().info(f"YOLO 모델 로드 중: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.model.to(self.device)
        self.person_class_id = 0  # COCO 데이터셋에서 사람의 클래스 ID

        # 비디오 캡처
        self.cap = cv2.VideoCapture(self.video_source)
        if not self.cap.isOpened():
            self.get_logger().error(f"비디오를 열 수 없습니다: {self.video_source}")
            raise RuntimeError(f"Cannot open video source: {self.video_source}")

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Publisher 생성
        self.pub_detections = self.create_publisher(
            DetectionArray, "detections", qos_profile
        )
        self.pub_image = self.create_publisher(Image, "detection_image", qos_profile)

        # CvBridge 초기화
        self.bridge = CvBridge()

        # 타이머 콜백
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
        self.frame_count = 0
        self.get_logger().info("비디오 사람 감지 노드 시작됨")

    def timer_callback(self):
        """타이머 콜백: 비디오 프레임 처리"""
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("프레임 읽기 실패, 비디오 재시작")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        self.frame_count += 1

        # 프레임 스킵
        if self.frame_count % self.skip_frames != 0:
            return

        # YOLO 추론
        try:
            results = self.model(frame, conf=self.confidence_threshold)

            # 감지 결과 메시지 생성
            detection_array = DetectionArray()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = "camera"

            annotated_frame = frame.copy()

            if results and len(results) > 0:
                result = results[0]

                if result.boxes is not None:
                    for box in result.boxes:
                        # 사람 클래스만 필터링
                        if int(box.cls[0]) == self.person_class_id:
                            x1, y1, x2, y2 = map(float, box.xyxy[0])
                            confidence = float(box.conf[0])

                            # Detection 메시지 생성
                            detection = Detection()
                            detection.class_name = "person"
                            detection.class_id = 0
                            detection.score = confidence

                            # BoundingBox2D 설정
                            # center.position
                            detection.bbox.center.position.x = (x1 + x2) / 2
                            detection.bbox.center.position.y = (y1 + y2) / 2
                            detection.bbox.center.theta = 0.0
                            # size
                            detection.bbox.size.x = x2 - x1
                            detection.bbox.size.y = y2 - y1

                            detection_array.detections.append(detection)

                            # 이미지에 바운딩 박스 그리기
                            cv2.rectangle(
                                annotated_frame,
                                (int(x1), int(y1)),
                                (int(x2), int(y2)),
                                (0, 255, 0),
                                2,
                            )

                            label = f"Person: {confidence:.2f}"
                            cv2.putText(
                                annotated_frame,
                                label,
                                (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2,
                            )

            # 감지된 사람 수 표시
            cv2.putText(
                annotated_frame,
                f"Persons: {len(detection_array.detections)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )

            cv2.putText(
                annotated_frame,
                f"Frame: {self.frame_count}",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            # 메시지 발행
            self.pub_detections.publish(detection_array)

            # 이미지 메시지 발행
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.pub_image.publish(image_msg)

            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f"프레임 {self.frame_count}: {len(detection_array.detections)}명 감지됨"
                )

        except Exception as e:
            self.get_logger().error(f"감지 오류: {str(e)}")

    def destroy_node(self):
        """노드 종료"""
        self.cap.release()
        super().destroy_node()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    node = VideoPersonDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
