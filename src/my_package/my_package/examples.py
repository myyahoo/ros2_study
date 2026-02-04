#!/usr/bin/env python3
"""
YOLO 사람 감지 사용 예제
OpenCV 동영상에서 실시간 사람 감지
"""

import cv2
from pathlib import Path
import sys

# my_node.py의 YOLOPersonDetector 사용
sys.path.insert(0, str(Path(__file__).parent))
from my_node import YOLOPersonDetector


def example_camera_detection():
    """예제 1: 카메라에서 실시간 사람 감지"""
    print("\n=== 예제 1: 카메라에서 실시간 사람 감지 ===")
    print("카메라가 필요합니다.")
    print("ESC 키를 눌러 종료하세요.\n")

    detector = YOLOPersonDetector(model_path="yolov8m.pt", device="cuda:0")
    detector.process_video(video_source=0, confidence_threshold=0.5)


def example_video_file_detection():
    """예제 2: 비디오 파일에서 사람 감지"""
    print("\n=== 예제 2: 비디오 파일에서 사람 감지 ===")

    video_file = "input_video.mp4"  # 사용자의 비디오 파일 경로로 변경
    output_file = "output_video_with_detections.mp4"

    if not Path(video_file).exists():
        print(f"오류: 비디오 파일을 찾을 수 없습니다: {video_file}")
        return

    detector = YOLOPersonDetector(model_path="yolov8m.pt", device="cuda:0")
    detector.process_video(
        video_source=video_file,
        output_path=output_file,
        confidence_threshold=0.5,
        skip_frames=2,  # 성능 향상을 위해 매 2번째 프레임만 처리
    )

    print(f"결과 비디오가 저장되었습니다: {output_file}")


def example_webcam_with_recording():
    """예제 3: 웹캠에서 감지하면서 비디오 저장"""
    print("\n=== 예제 3: 웹캠에서 감지하면서 비디오 저장 ===")
    print("ESC 키를 눌러 종료하세요.\n")

    output_file = "webcam_detection.mp4"
    detector = YOLOPersonDetector(model_path="yolov8m.pt", device="cuda:0")
    detector.process_video(
        video_source=0,
        output_path=output_file,
        confidence_threshold=0.5,
        skip_frames=1,
    )

    print(f"결과 비디오가 저장되었습니다: {output_file}")


def example_image_detection():
    """예제 4: 단일 이미지에서 사람 감지"""
    print("\n=== 예제 4: 단일 이미지에서 사람 감지 ===")

    image_path = "input_image.jpg"  # 사용자의 이미지 경로로 변경

    if not Path(image_path).exists():
        print(f"오류: 이미지 파일을 찾을 수 없습니다: {image_path}")
        return

    detector = YOLOPersonDetector(model_path="yolov8m.pt", device="cuda:0")

    # 이미지 로드
    image = cv2.imread(image_path)
    if image is None:
        print(f"오류: 이미지를 읽을 수 없습니다: {image_path}")
        return

    # 사람 감지
    persons, annotated_image = detector.detect_persons(image, confidence_threshold=0.5)

    print(f"감지된 사람 수: {len(persons)}")
    for i, person in enumerate(persons):
        print(f"  사람 {i + 1}: 신뢰도 {person['confidence']:.2f}")

    # 결과 표시
    cv2.imshow("Detected Persons", annotated_image)
    cv2.imwrite("output_image.jpg", annotated_image)
    print("결과 이미지가 저장되었습니다: output_image.jpg")

    print("아무 키나 누르면 종료됩니다...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_custom_confidence():
    """예제 5: 커스텀 신뢰도 임계값으로 카메라 감지"""
    print("\n=== 예제 5: 커스텀 신뢰도 임계값으로 카메라 감지 ===")
    print("신뢰도 임계값: 0.7 (높은 신뢰도만 표시)")
    print("ESC 키를 눌러 종료하세요.\n")

    detector = YOLOPersonDetector(model_path="yolov8m.pt", device="cuda:0")
    detector.process_video(video_source=0, confidence_threshold=0.7)


def main():
    """메인 함수: 사용자가 선택할 예제 실행"""
    print("=" * 50)
    print("YOLO 사람 감지 프로그램 - 사용 예제")
    print("=" * 50)
    print("\n다음 중 하나를 선택하세요:")
    print("1. 카메라에서 실시간 사람 감지")
    print("2. 비디오 파일에서 사람 감지")
    print("3. 웹캠에서 감지하면서 비디오 저장")
    print("4. 단일 이미지에서 사람 감지")
    print("5. 커스텀 신뢰도 임계값으로 카메라 감지")
    print("6. 종료")

    choice = input("\n선택 (1-6): ").strip()

    if choice == "1":
        example_camera_detection()
    elif choice == "2":
        example_video_file_detection()
    elif choice == "3":
        example_webcam_with_recording()
    elif choice == "4":
        example_image_detection()
    elif choice == "5":
        example_custom_confidence()
    elif choice == "6":
        print("프로그램을 종료합니다.")
    else:
        print("잘못된 선택입니다.")


if __name__ == "__main__":
    main()
