import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path


class YOLOPersonDetector:
    """OpenCV 동영상에서 YOLO를 사용하여 사람을 감지하는 클래스"""

    def __init__(self, model_path="yolov8m.pt", device="cuda:0"):
        """
        YOLOPersonDetector 초기화

        Args:
            model_path: YOLO 모델 경로 (기본값: yolov8m.pt)
            device: 사용할 디바이스 (cuda:0 또는 cpu)
        """
        self.model = YOLO(model_path)
        self.model.to(device)
        self.device = device
        self.person_class_id = 0  # COCO 데이터셋에서 사람의 클래스 ID

    def detect_persons(self, frame, confidence_threshold=0.5):
        """
        프레임에서 사람 감지

        Args:
            frame: 입력 이미지 프레임
            confidence_threshold: 신뢰도 임계값

        Returns:
            탐지된 사람의 정보 리스트 및 어노테이션된 프레임
        """
        # YOLO 추론  self.predict(frame, conf=confidence_threshold) 와 동일
        #  self.model 이 인스탄스인데  model() 과 같이 함수로 사용될수 있는건 YOLO 클래스 내부에 __call__ 메서드가 정의되어 있기 때문
        # 가  다음과 같이 구현
        # class YOLO:
        #    def __call__(self, *args, **kwargs):
        #    return self.predict(*args, **kwargs)  # predict() 호출
    

        #results = self.model(frame, conf=confidence_threshold)
        results = self.model.predict(frame, conf=confidence_threshold)

        persons = []
        annotated_frame = frame.copy()

        if results and len(results) > 0:
            result = results[0]
            # 탐지된 객체 처리
            if result.boxes is not None: 
                boxes_len = len(result.boxes)   
                print(f"result.boxes length is ========={boxes_len}")
                print(f"result.boxes is ========={result.boxes}")
                for box in result.boxes:
                    print(f"box is ========={box}")   
                    # 사람 클래스만 필터링
                    if int(box.cls[0]) == self.person_class_id:
                        # 바운딩 박스 좌표
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])

                        persons.append(
                            {
                                "bbox": (x1, y1, x2, y2),
                                "confidence": confidence,
                                "class": "person",
                            }
                        )

                        # 바운딩 박스 그리기
                        cv2.rectangle(
                            annotated_frame,
                            (x1, y1),
                            (x2, y2),
                            (0, 255, 0),
                            2,
                        )

                        # 신뢰도 텍스트
                        label = f"Person: {confidence:.2f}"
                        cv2.putText(
                            annotated_frame,
                            label,
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )
                        print("==================")
        return persons, annotated_frame

    def process_video(
        self, video_source, output_path=None, confidence_threshold=0.5, skip_frames=1
    ):
        """
        비디오 파일 또는 카메라 스트림 처리

        Args:
            video_source: 비디오 파일 경로 또는 카메라 인덱스 (0, 1, ...)
            output_path: 결과 비디오 저장 경로 (선택사항)
            confidence_threshold: 신뢰도 임계값
            skip_frames: 프레임 스킵 (성능 향상용)
        """
        # 비디오 입력 열기
        cap = cv2.VideoCapture(video_source)

        if not cap.isOpened():
            print(f"Error: 비디오를 열 수 없습니다 - {video_source}")
            return

        # 비디오 정보
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        print(f"비디오 정보: {width}x{height}, FPS: {fps}, 총 프레임: {total_frames}")

        # 비디오 쓰기 준비
        out = None
        if output_path:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

        frame_count = 0
        person_count_history = []

        try:
            while True:
                ret, frame = cap.read()

                if not ret:
                    break

                frame_count += 1

                # 프레임 스킵
                if frame_count % skip_frames != 0:
                    continue
                print("########################")
                # 사람 감지
                persons, annotated_frame = self.detect_persons(
                    frame, confidence_threshold
                )
                
                # 총 사람 수 표시
                person_count = len(persons)
                person_count_history.append(person_count)

                cv2.putText(
                    annotated_frame,
                    f"Persons detected: {person_count}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )

                cv2.putText(
                    annotated_frame,
                    f"Frame: {frame_count}/{total_frames}",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                )

                # 결과 프레임 표시
                cv2.imshow("YOLO Person Detection", annotated_frame)

                # 결과 저장
                if out:
                    out.write(annotated_frame)

                # ESC 키로 종료
                if cv2.waitKey(1) & 0xFF == 27:
                    break

                if frame_count % 30 == 0:
                    print(
                        f"처리 중: {frame_count} 프레임, 감지된 사람: {person_count}"
                    )
                    break
        finally:
            # 정리
            cap.release()
            if out:
                out.release()
            cv2.destroyAllWindows()

            print(f"\n--- 처리 완료 ---")
            print(f"총 프레임: {frame_count}")
            if person_count_history:
                print(f"평균 감지 사람 수: {np.mean(person_count_history):.2f}")
                print(f"최대 감지 사람 수: {max(person_count_history)}")


def main():
    """메인 함수"""
    print("YOLO 사람 감지 프로그램 시작")

    # 모델 경로 설정 (yolov10m.pt 또는 yolov8m.pt 사용)
    model_path = "yolov8m.pt"

    # YOLOPersonDetector 생성
    detector = YOLOPersonDetector(model_path=model_path, device="cuda:0")

    # 비디오 처리 옵션
    # 1. 카메라 사용: video_source=0
    # 2. 비디오 파일 사용: video_source="path/to/video.mp4"

    # 예시 1: 카메라에서 실시간 감지
    print("\n카메라에서 실시간 감지를 시작합니다...")
    print("ESC 키를 눌러 종료하세요.")
    detector.process_video(
        video_source=0,
        confidence_threshold=0.5,
        skip_frames=1,
    )

    # 예시 2: 비디오 파일 처리 (주석 처리됨)
    # video_file = "input_video.mp4"
    # output_file = "output_video.mp4"
    # detector.process_video(
    #     video_source=video_file,
    #     output_path=output_file,
    #     confidence_threshold=0.5,
    #     skip_frames=2,  # 성능 향상을 위해 매 2번째 프레임만 처리
    # )


if __name__ == "__main__":
    main()
