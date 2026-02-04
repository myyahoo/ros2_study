# YOLO 사람 감지 패키지

OpenCV 동영상에서 YOLO를 사용하여 실시간 사람 감지를 수행하는 Python 패키지입니다.

## 기능

- **실시간 카메라 감지**: 웹캠 또는 USB 카메라에서 실시간 사람 감지
- **비디오 파일 처리**: MP4, AVI 등 다양한 형식의 비디오 파일 처리
- **비디오 저장**: 감지 결과를 새로운 비디오 파일로 저장
- **이미지 감지**: 단일 이미지에서 사람 감지
- **신뢰도 조절**: 감지 신뢰도 임계값 커스터마이징
- **ROS 2 통합**: ROS 2 노드로 실행 가능

## 설치

### 필수 패키지

```bash
pip install opencv-python
pip install ultralytics
pip install torch torchvision
pip install numpy
```

### ROS 2 환경에서

```bash
cd ~/ros2/my_first_ws
colcon build --packages-select my_package
source install/setup.bash
```

## 사용 방법

### 1. 독립 Python 프로그램으로 실행

#### 카메라에서 실시간 감지

```python
from my_node import YOLOPersonDetector

detector = YOLOPersonDetector(model_path="yolov8m.pt", device="cuda:0")
detector.process_video(video_source=0, confidence_threshold=0.5)
```

#### 비디오 파일 처리

```python
detector.process_video(
    video_source="input_video.mp4",
    output_path="output_video.mp4",
    confidence_threshold=0.5,
    skip_frames=2
)
```

#### 단일 이미지 감지

```python
import cv2

image = cv2.imread("image.jpg")
persons, annotated_image = detector.detect_persons(image)

print(f"감지된 사람 수: {len(persons)}")
cv2.imshow("Result", annotated_image)
cv2.waitKey(0)
```

### 2. 직접 실행

```bash
python3 my_node.py
```

### 3. 예제 프로그램 실행

```bash
python3 examples.py
```

메뉴에서 선택하여 다양한 예제를 실행할 수 있습니다.

### 4. ROS 2 노드로 실행

```bash
ros2 run my_package video_person_detection
```

**파라미터 설정:**

```bash
ros2 run my_package video_person_detection \
    --ros-args \
    -p video_source:=0 \
    -p model_path:="yolov8m.pt" \
    -p device:="cuda:0" \
    -p confidence_threshold:=0.5 \
    -p skip_frames:=1
```

**ROS 2 토픽 구독:**

```bash
# 감지 결과 보기
ros2 topic echo /detections

# 감지된 이미지 보기 (rviz2 사용)
ros2 run rqt_image_view rqt_image_view /detection_image
```

## API 레퍼런스

### YOLOPersonDetector 클래스

#### `__init__(model_path="yolov8m.pt", device="cuda:0")`

YOLOPersonDetector 초기화

**매개변수:**
- `model_path` (str): YOLO 모델 경로 (기본값: "yolov8m.pt")
- `device` (str): 사용할 디바이스 (기본값: "cuda:0")

#### `detect_persons(frame, confidence_threshold=0.5)`

프레임에서 사람 감지

**매개변수:**
- `frame` (numpy.ndarray): 입력 이미지 프레임
- `confidence_threshold` (float): 신뢰도 임계값 (0-1)

**반환값:**
- `persons` (list): 감지된 사람 정보 리스트
  - `bbox`: (x1, y1, x2, y2) 바운딩 박스 좌표
  - `confidence`: 신뢰도 점수
  - `class`: "person"
- `annotated_frame` (numpy.ndarray): 바운딩 박스가 표시된 프레임

#### `process_video(video_source, output_path=None, confidence_threshold=0.5, skip_frames=1)`

비디오 파일 또는 카메라 스트림 처리

**매개변수:**
- `video_source` (int 또는 str): 카메라 인덱스 또는 비디오 파일 경로
- `output_path` (str): 결과 비디오 저장 경로 (선택사항)
- `confidence_threshold` (float): 신뢰도 임계값 (기본값: 0.5)
- `skip_frames` (int): 프레임 스킵 개수 (기본값: 1, 성능 향상용)

## 주요 특징

### 성능 최적화

- **프레임 스킵**: `skip_frames` 파라미터로 처리할 프레임 선택
- **GPU 사용**: CUDA 지원으로 빠른 처리
- **배치 처리**: 여러 프레임을 효율적으로 처리

### 설정 가능한 옵션

- **신뢰도 임계값**: 감지 정확도와 성능의 균형 조절
- **모델 선택**: yolov8m, yolov8n, yolov10m 등 다양한 모델 지원
- **디바이스 선택**: GPU(cuda:0) 또는 CPU에서 실행

### 출력 형식

감지된 사람 정보:
```python
{
    'bbox': (x1, y1, x2, y2),      # 바운딩 박스 좌표
    'confidence': 0.95,            # 신뢰도 점수
    'class': 'person'              # 클래스명
}
```

## 모델 선택

사용 가능한 YOLO 모델:
- `yolov8n.pt` - 나노 (가장 빠름, 정확도 낮음)
- `yolov8s.pt` - 스몰
- `yolov8m.pt` - 미디움 (권장)
- `yolov8l.pt` - 라지
- `yolov8x.pt` - 엑스트라라지 (가장 느림, 정확도 높음)
- `yolov10m.pt` - YOLOv10 미디움
- `yolov10l.pt` - YOLOv10 라지

## 예제

### 예제 1: 기본 카메라 감지

```bash
python3 -c "from my_node import YOLOPersonDetector; YOLOPersonDetector().process_video(0)"
```

### 예제 2: 비디오 파일 처리 및 저장

```bash
python3 -c "
from my_node import YOLOPersonDetector
detector = YOLOPersonDetector()
detector.process_video('video.mp4', output_path='output.mp4')
"
```

### 예제 3: 높은 신뢰도로 감지

```bash
python3 -c "
from my_node import YOLOPersonDetector
detector = YOLOPersonDetector()
detector.process_video(0, confidence_threshold=0.7)
"
```

## 키보드 단축키

실행 중:
- **ESC**: 프로그램 종료
- **Q**: 프로그램 종료 (카메라 모드에서)

## 시스템 요구사항

- Python 3.8 이상
- CUDA 11.8 이상 (GPU 사용 시)
- 최소 2GB RAM
- GPU 메모리 최소 2GB (권장 4GB 이상)

## 트러블슈팅

### 문제: "YOLO 모델을 찾을 수 없음"

**해결:**
```bash
# 모델 다운로드
python3 -c "from ultralytics import YOLO; YOLO('yolov8m.pt')"
```

### 문제: GPU 메모리 부족

**해결:**
- CPU 사용: `device="cpu"`
- 작은 모델 사용: `model_path="yolov8n.pt"`
- 프레임 스킵: `skip_frames=2` 이상

### 문제: 느린 처리 속도

**해결:**
- GPU 사용 확인: `device="cuda:0"`
- 프레임 스킵: `skip_frames=2`
- 더 작은 모델 사용: `yolov8s.pt` 또는 `yolov8n.pt`

## ROS 2 마이그레이션

기존 카메라 토픽을 이용하여 감지하는 경우:

```python
# 카메라 토픽 구독 및 감지
ros2 topic pub /camera/image_raw sensor_msgs/Image <message>
```

## 라이선스

이 패키지는 Ultralytics YOLO의 GNU General Public License v3.0을 따릅니다.

## 참고

- [Ultralytics YOLO 공식 문서](https://docs.ultralytics.com/)
- [OpenCV 공식 문서](https://docs.opencv.org/)
- [ROS 2 공식 문서](https://docs.ros.org/en/humble/)
