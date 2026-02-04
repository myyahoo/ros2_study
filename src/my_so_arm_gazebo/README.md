# 6DOF Robotic Arm with MoveIt2 and Gazebo

ROS2 Jazzy 환경에서 MoveIt2와 Gazebo를 사용한 6DOF 로봇 팔 시뮬레이션 패키지입니다. 카메라 1개가 장착되어 있습니다.

## 시스템 요구사항

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.10+

## 필수 패키지 설치

```bash
# 필수 패키지 설치
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-gazebo \
  ros-jazzy-gazebo-ros \
  ros-jazzy-moveit \
  ros-jazzy-moveit-commander \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-kdl-kinematics-plugin \
  ros-jazzy-tf-transformations
```

## 빌드 방법

### 1단계: 워크스페이스로 이동
```bash
cd ~/project/ros2/my_first_ws
```

### 2단계: 의존성 설치
```bash
rosdep install -y --from-paths src --ignore-src --rosdistro=jazzy
```

### 3단계: 빌드
```bash
colcon build --symlink-install
```

### 4단계: 환경 설정
```bash
source install/setup.bash
```

## 실행 방법

### 1. URDF 시각화 (RViz에서만 보기)
```bash
ros2 launch my_so_arm_gazebo display.launch.py
```
- `joint_state_publisher_gui` 창에서 슬라이더로 관절을 제어할 수 있습니다.

### 2. Gazebo 시뮬레이션 실행
```bash
ros2 launch my_so_arm_gazebo gazebo.launch.py
```
- Gazebo에서 로봇이 생성됩니다.
- 물리 시뮬레이션이 실행됩니다.

### 3. MoveIt2 플래닝 인터페이스 실행
```bash
ros2 launch my_so_arm_gazebo moveit.launch.py
```
- MoveIt2 플래닝이 활성화됩니다.
- RViz에서 경로 계획을 수행할 수 있습니다.

## 로봇 제어 - Python 예제

### 기본 제어 (arm_controller.py 사용)

```bash
# 터미널 1: MoveIt 실행
ros2 launch my_so_arm_gazebo moveit.launch.py

# 터미널 2: 제어 스크립트 실행
python3 -m my_so_arm_gazebo.arm_controller
```

### 직접 Python 스크립트로 제어

```python
import rclpy
from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose

# 초기화
rclpy.init()
robot = RobotCommander()
group = MoveGroupCommander("manipulator")

# 홈 포지션으로 이동
group.set_named_target("home")
group.go()

# 특정 자세로 이동 (IK 사용)
pose_goal = Pose()
pose_goal.position.x = 0.3
pose_goal.position.y = 0.1
pose_goal.position.z = 0.5
pose_goal.orientation.w = 1.0

group.set_pose_target(pose_goal)
group.go()

rclpy.shutdown()
```

## 로봇 사양

### 관절 정보
| 관절명 | 타입 | 범위 | 모터토크 | 속도 |
|------|------|------|--------|------|
| shoulder_pan_joint | Revolute | -π to π | 100 N⋅m | 1.5 rad/s |
| shoulder_lift_joint | Revolute | -π/2 to π/2 | 100 N⋅m | 1.5 rad/s |
| elbow_joint | Revolute | -π to π | 50 N⋅m | 1.5 rad/s |
| wrist_1_joint | Revolute | -π to π | 30 N⋅m | 2.0 rad/s |
| wrist_2_joint | Revolute | -π/2 to π/2 | 30 N⋅m | 2.0 rad/s |
| wrist_3_joint | Revolute | -π to π | 20 N⋅m | 2.0 rad/s |

### 카메라 정보
- 위치: End Effector (link_6) 앞쪽
- 프레임: camera_optical_frame
- 용도: 시뮬레이션에서 시각화

## 파일 구조

```
my_so_arm_gazebo/
├── urdf/
│   └── arm.urdf.xacro           # 로봇 URDF 정의
├── config/
│   ├── arm.srdf                 # MoveIt2 SRDF 설정
│   ├── kinematics.yaml          # IK 솔버 설정
│   ├── ompl_planning.yaml       # 경로 계획 설정
│   ├── controllers.yaml         # 컨트롤러 설정
│   ├── visualization.rviz       # RViz 설정 (디스플레이 모드)
│   └── moveit.rviz              # RViz 설정 (MoveIt 모드)
├── launch/
│   ├── display.launch.py        # URDF 시각화
│   ├── gazebo.launch.py         # Gazebo 시뮬레이션
│   └── moveit.launch.py         # MoveIt2 실행
├── my_so_arm_gazebo/
│   ├── __init__.py
│   └── arm_controller.py        # Python 제어 스크립트
├── package.xml                   # 패키지 메타데이터
├── CMakeLists.txt               # CMake 설정
└── setup.py                      # Python 설정

```

## 일반적인 명령어

### 로봇 상태 확인
```bash
# 현재 관절 값 확인
ros2 topic echo /joint_states

# 현재 TF 확인
ros2 run tf2_tools view_frames
```

### RViz에서 경로 계획
1. MoveIt2를 실행합니다.
2. RViz 왼쪽 패널의 "Planning" 섹션에서:
   - "Planning Group"을 "manipulator"로 선택합니다.
   - 목표 위치를 설정합니다.
   - "Plan"을 클릭하여 경로를 계획합니다.
   - "Execute"를 클릭하여 실행합니다.

## 카메라 활용

카메라는 end effector에 장착되어 있습니다. 카메라 프레임:
- `camera_link`: 카메라 물리 프레임
- `camera_optical_frame`: 카메라 광학 프레임 (시각 처리용)

### 카메라 이미지 구독 (필요시 설정)
```python
import rclpy
from sensor_msgs.msg import Image

def image_callback(msg):
    print(f"Received image: {msg.width}x{msg.height}")

node = rclpy.create_node('camera_subscriber')
subscription = node.create_subscription(
    Image,
    '/camera/image_raw',
    image_callback,
    10
)
```

## 트러블슈팅

### 1. "gazebo" 명령을 찾을 수 없음
```bash
sudo apt-get install gazebo
```

### 2. MoveIt 플래닝이 실패함
- 충돌 제약 조건 확인: RViz의 "Scene" 탭에서 환경을 확인합니다.
- IK 시간 제한 증가: `kinematics.yaml`에서 `kinematics_solver_timeout` 값을 증가시킵니다.

### 3. 관절이 움직이지 않음
- 컨트롤러 상태 확인: `ros2 node list`로 실행 중인 노드 확인
- 토픽 상태 확인: `ros2 topic list`

## 다음 단계

1. **시각 피드백 추가**: 카메라를 Gazebo에 통합하고 실제 이미지 처리
2. **End Effector 추가**: 그리퍼나 흡입기 등 추가 가능
3. **장애물 회피**: 계획 장면에 객체 추가 및 경로 계획
4. **실제 로봇 제어**: 실제 하드웨어에 연결

## 참고 자료

- [ROS 2 공식 문서](https://docs.ros.org/en/jazzy/)
- [MoveIt 2 튜토리얼](https://moveit.ros.org/documents/concepts/concepts/)
- [Gazebo 공식 문서](http://gazebosim.org/)

## 라이선스

Apache-2.0
