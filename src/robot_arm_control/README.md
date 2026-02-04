# Robot Arm Control Package

## 개요

**robot_arm_control**은 So-arm101 6-DOF 로봇 암을 ROS2로 제어하는 패키지입니다.  
OpenCV와 YOLO를 활용한 Vision 기반 자동화 작업을 수행합니다.

### 주요 기능

- **Vision Processing**: 카메라 입력을 YOLO로 처리하여 볼펜 감지
- **6-DOF Arm Control**: So-arm101 로봇 암의 6개 모터 제어
- **Inverse Kinematics**: 목표 위치에서 관절 각도 자동 계산
- **State Machine**: 상태 기반 자동화 작업 실행
- **작업 흐름**: 볼펜 감지 → 접근 → 집기 → 들어올리기 → 옆으로 이동 → 놓기

---

## 시스템 아키텍처

### 노드 구조

```
┌─────────────────────────────────────────────────────┐
│           Robot Arm Control System                  │
├─────────────────────────────────────────────────────┤
│                                                      │
│  ┌──────────────────┐  ┌──────────────────────────┐ │
│  │  Vision Node     │  │  Arm Control Node        │ │
│  ├──────────────────┤  ├──────────────────────────┤ │
│  │ - 카메라 입력    │  │ - 역기구학 계산         │ │
│  │ - YOLO 감지      │  │ - 모터 제어             │ │
│  │ - OpenCV 처리    │  │ - 그리퍼 제어           │ │
│  └──────────────────┘  └──────────────────────────┘ │
│           │                      △                   │
│           └──────────────────────┘                   │
│                                                      │
│  ┌──────────────────────────────────────────────────┐│
│  │  Task Executor Node (State Machine)              ││
│  ├──────────────────────────────────────────────────┤│
│  │  IDLE → DETECT → APPROACH → GRASP → LIFT        ││
│  │         → MOVE_SIDE → RELEASE → HOME            ││
│  └──────────────────────────────────────────────────┘│
│                                                      │
└─────────────────────────────────────────────────────┘
```

### 주요 상태 머신

```
┌─────────┐
│  IDLE   │ ◄─────────────────────────┐
└────┬────┘                           │
     │                                │
     ▼                                │
┌──────────────┐                      │
│ DETECT_PEN   │                      │
└────┬────────┘                       │
     │                                │
     ▼                                │
┌──────────────┐                      │
│ APPROACH_PEN │                      │
└────┬────────┘                       │
     │                                │
     ▼                                │
┌──────────────┐                      │
│ GRASP_PEN    │                      │
└────┬────────┘                       │
     │                                │
     ▼                                │
┌──────────────┐                      │
│ LIFT_PEN     │                      │
└────┬────────┘                       │
     │                                │
     ▼                                │
┌──────────────┐                      │
│ MOVE_SIDE    │                      │
└────┬────────┘                       │
     │                                │
     ▼                                │
┌──────────────┐                      │
│ RELEASE_PEN  │                      │
└────┬────────┘                       │
     │                                │
     ▼                                │
┌──────────────┐                      │
│ RETURN_HOME  │                      │
└────┬────────┘                       │
     │                                │
     └─────────────────────────────────►
```

---

## 파일 구조

```
robot_arm_control/
├── robot_arm_control/              # 파이썬 패키지
│   ├── __init__.py
│   ├── vision_node.py              # Vision 처리 노드
│   ├── arm_control_node.py         # 팔 제어 노드
│   └── task_executor_node.py       # 태스크 실행 노드
├── msg/                            # ROS 메시지 정의
│   ├── ArmState.msg                # 팔 상태
│   ├── DetectionResult.msg         # 감지 결과
│   └── JointCommand.msg            # 관절 명령
├── srv/                            # ROS 서비스 정의
│   ├── GripperCommand.srv          # 그리퍼 제어
│   └── MoveArm.srv                 # 팔 이동
├── launch/                         # Launch 파일
│   ├── robot_arm_bringup.launch.py     # 전체 시스템
│   ├── vision_only.launch.py           # Vision만
│   ├── arm_control_only.launch.py      # 팔 제어만
│   └── task_executor_only.launch.py    # 태스크만
├── package.xml                     # ROS 패키지 설정
├── CMakeLists.txt                  # CMake 설정
├── setup.py                        # Python 설치 설정
├── setup.cfg                       # Python 설치 설정
└── README.md                       # 이 파일
```

---

## 설치 및 빌드

### 필수 패키지 설치

```bash
# ROS2 기본 패키지
sudo apt-get install ros-humble-rclpy
sudo apt-get install ros-humble-geometry-msgs
sudo apt-get install ros-humble-sensor-msgs

# 비전 관련 라이브러리
pip install opencv-python opencv-contrib-python
pip install torch torchvision torchaudio
pip install ultralytics

# CvBridge
sudo apt-get install ros-humble-cv-bridge
```

### 빌드

```bash
cd /home/tech/project/ros2/my_first_ws
colcon build --packages-select robot_arm_control
source install/setup.bash
```

---

## 사용 방법

### 1. 전체 시스템 실행

```bash
ros2 launch robot_arm_control robot_arm_bringup.launch.py
```

이 명령어는 다음의 3개 노드를 실행합니다:
- Vision Node (카메라/YOLO 처리)
- Arm Control Node (팔 제어)
- Task Executor Node (자동 작업)

### 2. 개별 노드 실행

#### Vision 노드만 실행
```bash
ros2 launch robot_arm_control vision_only.launch.py
```

#### 팔 제어 노드만 실행
```bash
ros2 launch robot_arm_control arm_control_only.launch.py
```

#### 태스크 실행 노드만 실행
```bash
ros2 launch robot_arm_control task_executor_only.launch.py
```

### 3. 직접 노드 실행

```bash
# Vision 노드
ros2 run robot_arm_control vision_node

# 팔 제어 노드
ros2 run robot_arm_control arm_control_node

# 태스크 실행 노드
ros2 run robot_arm_control task_executor_node
```

---

## 노드 설명

### Vision Node

**기능:**
- USB 카메라에서 프레임 입력
- YOLO v10 모델로 객체 감지
- 감지 결과를 OpenCV로 시각화
- 박스 및 신뢰도 표시

**구독 토픽:**
- `/camera/image_raw` - 카메라 이미지

**발행 토픽:**
- `/robot_arm/detection` - 처리된 감지 이미지

**파라미터:**
- `camera_topic` (기본값: `/camera/image_raw`)
- `detection_topic` (기본값: `/robot_arm/detection`)
- `confidence_threshold` (기본값: 0.5)
- `target_class` (기본값: `pen`)

**실행 예:**
```bash
ros2 run robot_arm_control vision_node --ros-args \
  -p confidence_threshold:=0.6 \
  -p target_class:='pen'
```

---

### Arm Control Node

**기능:**
- 6개 관절 제어
- 역기구학(IK) 계산
- 모터 드라이버와 통신 (실제 하드웨어 연결 필요)
- 그리퍼 제어

**제어 주기:** 50 Hz

**파라미터:**
- `num_joints` (기본값: 6) - 관절 개수
- `joint_max_velocity` (기본값: 1.0 rad/s) - 최대 관절 속도

**역기구학 계산:**

```python
# 목표 위치로부터 관절 각도 계산
from geometry_msgs.msg import Pose
target_pose = Pose()
target_pose.position.x = 0.2
target_pose.position.y = 0.0
target_pose.position.z = 0.1

joint_angles = arm_node.calculate_ik(target_pose)
# joint_angles = [θ1, θ2, θ3, θ4, θ5, θ6]
```

**So-arm101 스펙:**
- 관절 1 (Base): ±180°
- 관절 2-3 (Shoulder/Elbow): ±90°
- 관절 4-6 (Wrist): ±180°

---

### Task Executor Node

**기능:**
- 상태 머신 기반 자동 작업 흐름 관리
- 볼펜 감지 및 추적
- 팔 움직임 조정
- 그리퍼 자동 제어

**작업 흐름:**

1. **DETECT_PEN** - 카메라에서 볼펜 감지 (1초)
2. **APPROACH_PEN** - 펜 위치로 접근 (2초)
3. **GRASP_PEN** - 그리퍼로 펜 집기 (1초)
4. **LIFT_PEN** - 펜을 들어올리기 (1.5초)
5. **MOVE_SIDE** - Y축으로 0.3m 옆으로 이동 (2초)
6. **RELEASE_PEN** - 그리퍼 열기 (1초)
7. **RETURN_HOME** - 홈 위치로 복귀 (2초)

**파라미터:**
- `task_update_rate` (기본값: 10 Hz) - 상태 업데이트 속도
- `pen_detection_timeout` (기본값: 5.0초) - 감지 타임아웃

**상태 모니터링:**
```bash
ros2 topic echo /robot_arm/task_status
```

---

## ROS 토픽 및 서비스

### 토픽

| 토픽명 | 유형 | 설명 |
|--------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 카메라 원본 이미지 |
| `/robot_arm/detection` | `sensor_msgs/Image` | 감지 결과 이미지 |
| `/robot_arm/arm_state` | `ArmState` | 팔 상태 정보 |
| `/robot_arm/detection_result` | `DetectionResult` | YOLO 감지 결과 |
| `/robot_arm/task_status` | `String` | 현재 작업 상태 |

### 서비스

| 서비스명 | 유형 | 설명 |
|---------|------|------|
| `/robot_arm/move_arm` | `MoveArm` | 목표 위치로 팔 이동 |
| `/robot_arm/gripper_command` | `GripperCommand` | 그리퍼 제어 |

**서비스 사용 예:**

```bash
# 팔을 (0.2, 0, 0.1) 위치로 이동
ros2 service call /robot_arm/move_arm robot_arm_control/srv/MoveArm \
  "{target_pose: {position: {x: 0.2, y: 0.0, z: 0.1}}}"

# 그리퍼 열기
ros2 service call /robot_arm/gripper_command robot_arm_control/srv/GripperCommand \
  "command: true"
```

---

## 카메라 연결

### USB 카메라 확인

```bash
ls /dev/video*
```

### 카메라 노드 실행

기존 `my_opencv_pkg`의 카메라 노드를 사용하거나:

```bash
ros2 launch my_opencv_pkg my_pkg.launch.py
```

또는 usb_cam 패키지 사용:

```bash
sudo apt-get install ros-humble-usb-cam
ros2 launch usb_cam camera.launch.xml camera_name:=camera device:=/dev/video0
```

---

## YOLO 모델 설정

### 지원 모델

- `yolov8m.pt` (기본)
- `yolov10m.pt`
- 커스텀 학습 모델

### 모델 경로 변경

[vision_node.py](robot_arm_control/vision_node.py) 의 `YOLO_MODEL_PATH` 수정:

```python
YOLO_MODEL_PATH = "/path/to/your/model.pt"
```

### 커스텀 모델 학습 (선택사항)

```bash
# YOLO로 볼펜 감지 모델 학습
yolo detect train data=pen_dataset.yaml model=yolov8m.pt epochs=50
```

---

## 트러블슈팅

### 1. 카메라 이미지가 안 나옴

```bash
# 카메라 노드 확인
ros2 node list
ros2 topic list | grep camera

# 카메라 디바이스 확인
v4l2-ctl --list-devices
```

### 2. YOLO 모델 로드 실패

```bash
# 모델 파일 경로 확인
ls -la /home/tech/project/ros2/my_first_ws/src/yolov10m.pt

# YOLO 다시 다운로드
python3 -c "import torch; torch.hub.load('ultralytics/yolov5', 'yolov5m')"
```

### 3. 역기구학 계산 오류

팔이 도달 불가능한 위치로 이동을 시도하면 경고가 표시됩니다.  
근사 솔루션을 자동으로 계산합니다.

### 4. 그리퍼 제어 안 됨

실제 하드웨어 연결 필요. 시뮬레이션에서는 로그만 출력됩니다.

---

## 향후 개선 사항

- [ ] 실제 So-arm101 모터 드라이버 통합
- [ ] Dynamixel SDK 연동
- [ ] 고급 IK 알고리즘 (Jacobian, 다중 솔루션)
- [ ] 장애물 회피 (Collision Detection)
- [ ] 학습 기반 행동 (Reinforcement Learning)
- [ ] 웹 대시보드 (ROS Bridge + Web UI)
- [ ] 시뮬레이션 (Gazebo)

---

## 라이선스

Apache-2.0

## 참고 자료

- [ROS 2 공식 문서](https://docs.ros.org)
- [YOLO 공식 저장소](https://github.com/ultralytics/yolov5)
- [OpenCV 공식 문서](https://docs.opencv.org)
- [So-arm101 사양서](https://example.com/so-arm101-manual)

---

**작성일:** 2026.01.21  
**버전:** 0.0.1
