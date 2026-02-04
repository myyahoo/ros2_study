# 패키지 생성 완료 요약

## 📦 생성된 패키지: robot_arm_control

### 프로젝트 목표
So-arm101 (6-DOF) 로봇 암을 ROS2로 제어하여 **앞의 볼펜을 집어서 옆으로 이동**시키는 자동화 시스템 구현

---

## 🏗️ 패키지 구조

```
robot_arm_control/
├── robot_arm_control/              # 메인 Python 패키지
│   ├── __init__.py                 # 패키지 초기화
│   ├── vision_node.py              # Vision 처리 노드 (카메라 + YOLO + OpenCV)
│   ├── arm_control_node.py         # 로봇 팔 제어 노드 (IK + 모터 제어)
│   ├── task_executor_node.py       # 메인 작업 실행 노드 (상태 머신)
│   └── utils.py                    # 공용 유틸리티 함수
├── launch/                         # ROS2 Launch 파일
│   ├── robot_arm_bringup.launch.py     # 전체 시스템 실행
│   ├── vision_only.launch.py           # Vision 노드만 실행
│   ├── arm_control_only.launch.py      # 팔 제어 노드만 실행
│   └── task_executor_only.launch.py    # 태스크 노드만 실행
├── msg/                            # 사용자 정의 메시지
│   ├── ArmState.msg                # 팔 상태 정보 (관절각, 그리퍼)
│   ├── DetectionResult.msg         # YOLO 감지 결과
│   └── JointCommand.msg            # 관절 제어 명령
├── srv/                            # 사용자 정의 서비스
│   ├── GripperCommand.srv          # 그리퍼 제어 서비스
│   └── MoveArm.srv                 # 팔 이동 서비스
├── config/                         # 설정 파일
│   └── robot_config.py             # 로봇 파라미터 설정
├── test/                           # 테스트 파일
│   └── test_robot_arm_control.py   # 단위 테스트
├── resource/                       # 리소스 파일
├── package.xml                     # ROS2 패키지 정의
├── CMakeLists.txt                  # CMake 빌드 설정
├── setup.py                        # Python 패키지 설정
├── setup.cfg                       # 추가 설정
├── requirements.txt                # Python 의존성
├── README.md                       # 상세 문서 (한국어)
└── test_demo.py                    # 데모 및 테스트 스크립트
```

---

## 🤖 노드별 기능

### 1️⃣ Vision Node (`vision_node.py`)
- **목적**: 카메라에서 이미지를 수신하고 YOLO로 볼펜 감지
- **기능**:
  - USB 카메라에서 실시간 이미지 수신
  - YOLOv10/v8 모델로 객체 감지 (신뢰도 필터링)
  - OpenCV로 감지 결과 시각화 (바운딩 박스 표시)
  - 처리된 이미지 발행
- **구독**: `/camera/image_raw` (원본 이미지)
- **발행**: `/robot_arm/detection` (처리된 이미지)
- **실행**: `ros2 run robot_arm_control vision_node`

### 2️⃣ Arm Control Node (`arm_control_node.py`)
- **목적**: 6-DOF 로봇 암 제어 및 역기구학 계산
- **기능**:
  - So-arm101 로봇 암 제어 (6개 모터)
  - 역기구학(IK) 계산: 목표 위치 → 관절 각도
  - 모터 속도 프로필 생성
  - 그리퍼 제어 (열기/닫기)
  - 50Hz 제어 루프
- **파라미터**:
  - `num_joints`: 6 (관절 수)
  - `joint_max_velocity`: 1.0 rad/s
- **실행**: `ros2 run robot_arm_control arm_control_node`

### 3️⃣ Task Executor Node (`task_executor_node.py`)
- **목적**: 상태 머신 기반 자동 작업 흐름 관리
- **작업 단계**:
  1. **IDLE**: 대기 상태
  2. **DETECT_PEN**: 볼펜 감지
  3. **APPROACH_PEN**: 펜 위치로 접근
  4. **GRASP_PEN**: 그리퍼로 집기
  5. **LIFT_PEN**: Z축 위로 들어올리기
  6. **MOVE_SIDE**: Y축으로 0.3m 옆으로 이동
  7. **RELEASE_PEN**: 그리퍼 열기
  8. **RETURN_HOME**: 홈 위치로 복귀
- **업데이트 주기**: 10Hz
- **실행**: `ros2 run robot_arm_control task_executor_node`

---

## 🚀 빠른 시작

### 1. 패키지 빌드
```bash
cd /home/tech/project/ros2/my_first_ws
colcon build --packages-select robot_arm_control
source install/setup.bash
```

### 2. 전체 시스템 실행
```bash
# 터미널 1: 카메라 노드 (별도 패키지)
ros2 launch usb_cam camera.launch.xml camera_name:=camera device:=/dev/video0

# 터미널 2: 로봇 암 제어 시스템 (전체)
ros2 launch robot_arm_control robot_arm_bringup.launch.py

# 또는 개별 실행
ros2 launch robot_arm_control vision_only.launch.py        # Vision만
ros2 launch robot_arm_control arm_control_only.launch.py   # 팔 제어만
ros2 launch robot_arm_control task_executor_only.launch.py # 태스크만
```

### 3. 시스템 모니터링
```bash
# 토픽 확인
ros2 topic list

# 이미지 확인
ros2 topic echo /robot_arm/detection

# 노드 확인
ros2 node list

# 로그 확인
ros2 launch robot_arm_control robot_arm_bringup.launch.py | grep -E "INFO|ERROR"
```

---

## 📊 역기구학 (Inverse Kinematics)

### So-arm101 스펙
- **관절 수**: 6-DOF
- **링크 길이**: [0.04, 0.15, 0.15, 0.04, 0.08, 0.05] m
- **관절 범위**:
  - θ1 (Base): ±180°
  - θ2-3 (Shoulder/Elbow): ±90°
  - θ4-6 (Wrist): ±180°

### IK 계산 알고리즘
```
목표 위치 (x, y, z) → 역기구학 계산 → 관절 각도 [θ1-θ6]

1. Base 회전: θ1 = arctan2(y, x)
2. 도달 거리 확인
3. 엘보우 각도 (코사인 법칙): θ3 = arccos((d²-l1²-l2²)/(2*l1*l2))
4. 숄더 각도: θ2 = α - β
5. 손목 관절 (간단한 설정): θ4-6 = 0
```

---

## 📡 ROS2 토픽 및 서비스

### 구독 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 카메라 원본 이미지 |

### 발행 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/robot_arm/detection` | `sensor_msgs/Image` | 감지 결과 이미지 (YOLO + OpenCV) |
| `/robot_arm/arm_state` | `robot_arm_control/ArmState` | 팔 상태 (관절각, 그리퍼) |

### 서비스
| 서비스 | 요청 | 응답 | 설명 |
|--------|------|------|------|
| `/robot_arm/move_arm` | `geometry_msgs/Pose` | `joint_angles`, `success` | 팔을 목표 위치로 이동 |
| `/robot_arm/gripper_command` | `bool` | `success`, `message` | 그리퍼 제어 (열기/닫기) |

---

## 🛠️ 유틸리티 함수 (utils.py)

- `RobotArmUtils`: 기본 수학 연산
  - `distance()`: 두 점 사이 거리
  - `rad_to_deg()`, `deg_to_rad()`: 각도 변환
  - `clamp()`: 값 범위 제한
  
- `TrapezoidalVelocityProfile`: 속도 프로필 생성
  - 가속/정속/감속 구간 자동 생성
  
- `PIDController`: PID 제어기
  - 비례/적분/미분 제어
  - 에러 추적 및 수렴
  
- `Quaternion`: 사원수 연산
  - 사원수 ↔ 오일러 각 변환

---

## 📋 설정 파일 (config/robot_config.py)

```python
# So-arm101 스펙
ARM_LINK_LENGTHS = [0.04, 0.15, 0.15, 0.04, 0.08, 0.05]
ARM_DOF = 6

# 모터 설정 (Dynamixel)
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
MOTOR_PORT = '/dev/ttyUSB0'

# 그리퍼 설정
GRIPPER_MOTOR_ID = 7
GRIPPER_CLOSE_POSITION = 2048
GRIPPER_OPEN_POSITION = 512

# 카메라 설정
CAMERA_DEVICE = '/dev/video0'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# YOLO 설정
YOLO_MODEL = 'yolov10m'
YOLO_CONFIDENCE_THRESHOLD = 0.5

# 작업 파라미터
APPROACH_HEIGHT = 0.05  # 펜 위쪽
LIFT_HEIGHT = 0.15      # 들어올리는 높이
SIDE_MOVE_DISTANCE = 0.3  # 옆으로 이동 거리
```

---

## ✅ 테스트 및 데모

### 데모 실행
```bash
python3 test_demo.py
```

**테스트 항목:**
- ✓ 역기구학 솔버 (다양한 위치 테스트)
- ✓ 유틸리티 함수 (거리, 각도, PID 제어)
- ✓ 상태 머신 (작업 단계 순서)
- ✓ 의존성 확인

---

## 📝 주요 코드 예제

### 역기구학 사용
```python
from robot_arm_control.arm_control_node import ArmControlNode
import numpy as np

node = ArmControlNode()
target_pos = np.array([0.2, 0.0, 0.1])
angles = node.ik_solver.inverse_kinematics(target_pos)
print(angles)  # [θ1, θ2, θ3, θ4, θ5, θ6]
```

### PID 제어기 사용
```python
from robot_arm_control.utils import PIDController

pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
for _ in range(100):
    error = target - current
    output = pid.update(error, dt=0.01)
    current += output * 0.01
```

### 상태 머신 (개념)
```python
state = TaskState.IDLE

while running:
    if state == TaskState.DETECT_PEN:
        # 볼펜 감지 로직
        if pen_detected:
            state = TaskState.APPROACH_PEN
    
    elif state == TaskState.APPROACH_PEN:
        # 접근 로직
        if reached_approach_position:
            state = TaskState.GRASP_PEN
    
    # ... 상태 전이 계속
```

---

## 🔧 커스터마이징 가이드

### 1. 모터 연결 변경
- [config/robot_config.py](config/robot_config.py) 수정
  ```python
  MOTOR_PORT = '/dev/ttyUSB0'  # 포트 변경
  MOTOR_IDS = [1, 2, 3, 4, 5, 6]  # ID 변경
  ```

### 2. YOLO 모델 변경
- [robot_arm_control/vision_node.py](robot_arm_control/vision_node.py) 수정
  ```python
  YOLO_MODEL_PATH = "/path/to/custom/model.pt"
  ```

### 3. 작업 파라미터 조정
- Launch 파일에서 파라미터 수정
  ```bash
  ros2 launch robot_arm_control robot_arm_bringup.launch.py \
    --ros-args -p confidence_threshold:=0.7
  ```

### 4. 링크 길이 수정
- [robot_arm_control/arm_control_node.py](robot_arm_control/arm_control_node.py)
  ```python
  self.arm_length = [0.04, 0.15, 0.15, 0.04, 0.08, 0.05]
  ```

---

## 🚨 트러블슈팅

| 문제 | 원인 | 해결책 |
|------|------|--------|
| 카메라 이미지 안 나옴 | 카메라 미연결 | `v4l2-ctl --list-devices` 확인 |
| YOLO 모델 로드 실패 | 모델 파일 경로 오류 | 파일 존재 여부 확인 |
| 역기구학 오류 | 도달 불가능한 위치 | 근사 솔루션 자동 계산 |
| 그리퍼 안 움직임 | 모터 미연결 | 실제 하드웨어 연결 필요 |

---

## 📚 참고 자료

- [ROS 2 공식 문서](https://docs.ros.org)
- [YOLO 저장소](https://github.com/ultralytics/yolov5)
- [OpenCV 문서](https://docs.opencv.org)
- [Dynamixel SDK](https://emanual.robotis.com)

---

## 📈 향후 개선 사항

- [ ] Dynamixel 모터 직접 제어 (SDK 통합)
- [ ] 고급 IK 알고리즘 (Jacobian 행렬)
- [ ] 장애물 회피 (Collision Detection)
- [ ] 데이터 기반 학습 (Reinforcement Learning)
- [ ] Web Dashboard (ROS Bridge)
- [ ] Gazebo 시뮬레이션
- [ ] 다중 객체 추적 (MOT)

---

## 📞 문의 및 지원

패키지 관련 질문은 README.md의 참고 자료를 참고하세요.

---

**생성일**: 2026-01-21  
**패키지 버전**: 0.0.1  
**라이선스**: Apache-2.0
