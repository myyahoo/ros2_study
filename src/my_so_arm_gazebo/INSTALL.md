# 6DOF 로봇 팔 설정 - 단계별 가이드

## 📋 준비사항
- Ubuntu 24.04
- ROS 2 Jazzy 설치됨
- Gazebo 설치됨

## 🚀 빌드 및 실행 방법

### 1️⃣ 필수 패키지 설치

```bash
# 터미널에서 다음 명령 실행
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

### 2️⃣ 워크스페이스 빌드

```bash
# 워크스페이스로 이동
cd ~/project/ros2/my_first_ws

# 의존성 설치 (선택사항, 이미 설치된 경우 스킵)
rosdep install -y --from-paths src --ignore-src --rosdistro=jazzy

# 빌드
colcon build --symlink-install --packages-select my_so_arm_gazebo

# 환경 설정 (매번 터미널 열 때마다 실행 또는 ~/.bashrc에 추가)
source install/setup.bash
```

### 3️⃣ 실행 방법 (3가지 모드)

#### 🎨 모드 1: RViz에서 시각화 (시뮬레이션 없음)
```bash
ros2 launch my_so_arm_gazebo display.launch.py
```
- 순수 URDF 시각화
- Joint State Publisher GUI에서 슬라이더로 관절 제어 가능
- CPU 사용량 적음

#### 🏃 모드 2: Gazebo 시뮬레이션
```bash
ros2 launch my_so_arm_gazebo gazebo.launch.py
```
- 물리 시뮬레이션 포함
- Gazebo 3D 환경에서 로봇 시뮬레이션
- 관성, 충돌, 중력 적용됨

#### 🤖 모드 3: MoveIt2 경로 계획 (추천)
```bash
ros2 launch my_so_arm_gazebo moveit.launch.py
```
- MoveIt2 동작 계획 활성화
- RViz에서 그래픽 인터페이스로 경로 계획 가능
- 역운동학(IK) 솔버 포함

## 🎮 RViz에서 로봇 제어 (MoveIt2 모드)

1. **Planning Group 선택**
   - 좌측 패널 → "Planning Request"
   - Group: "manipulator" 선택

2. **목표 포지션 설정**
   - 3D 뷰에서 마우스로 로봇을 드래그
   - 또는 "Query Goal State" → 수동으로 입력

3. **경로 계획**
   - "Plan" 버튼 클릭
   - 경로가 계획되면 주황색 선으로 표시됨

4. **실행**
   - "Execute" 버튼 클릭
   - 로봇이 계획된 경로를 따라 이동

## 🐍 Python 스크립트로 제어

MoveIt2를 실행한 상태에서:

```python
# 새 터미널에서:
python3 -m my_so_arm_gazebo.arm_controller
```

또는 직접 Python 코드:

```python
import rclpy
from moveit_commander import MoveGroupCommander, RobotCommander

# 초기화
rclpy.init()
robot = RobotCommander()
group = MoveGroupCommander("manipulator")

# 홈 포지션으로 이동
group.set_named_target("home")
plan = group.plan()
group.execute(plan[1])

# 특정 포즈로 이동 (역운동학 사용)
from geometry_msgs.msg import Pose
pose = Pose()
pose.position.x = 0.3
pose.position.y = 0.1
pose.position.z = 0.5
pose.orientation.w = 1.0

group.set_pose_target(pose)
group.go()

rclpy.shutdown()
```

## 📊 로봇 사양

**6DOF 관절:**
- `shoulder_pan_joint`: ±180° 회전
- `shoulder_lift_joint`: ±90° 들었다 내렸다
- `elbow_joint`: ±180° 팔꿈치
- `wrist_1_joint`: ±180° 손목 회전
- `wrist_2_joint`: ±90° 손목 위아래
- `wrist_3_joint`: ±180° 손목 좌우

**카메라:**
- 위치: End Effector에 장착
- 프레임: `camera_optical_frame`

## 🐛 문제 해결

### 오류: "Package gazebo_ros not found"
```bash
sudo apt-get install ros-jazzy-gazebo-ros
```

### 오류: "Module moveit not found"
```bash
sudo apt-get install ros-jazzy-moveit-commander
```

### RViz에서 로봇이 안 보임
- 좌측 패널에서 "Displays" 클릭
- "RobotModel"이 체크되었는지 확인
- "robot_description"이 선택되었는지 확인

### 경로 계획이 너무 느림
- `config/ompl_planning.yaml`에서 타임아웃 증가
- 더 간단한 계획 알고리즘 선택 (RRT → EST)

## 📁 주요 파일 설명

| 파일 | 설명 |
|------|------|
| `urdf/arm.urdf.xacro` | 로봇 모델 정의 |
| `config/arm.srdf` | MoveIt2 설정 |
| `config/kinematics.yaml` | 역운동학 솔버 설정 |
| `config/ompl_planning.yaml` | 경로 계획 알고리즘 설정 |
| `launch/*.launch.py` | 실행 스크립트 |
| `my_so_arm_gazebo/arm_controller.py` | Python 제어 라이브러리 |

## ✅ 설치 확인

모든 것이 제대로 설치되었는지 확인:

```bash
# 환경 설정
source ~/project/ros2/my_first_ws/install/setup.bash

# 패키지 확인
ros2 pkg list | grep my_so_arm_gazebo

# Launch 파일 확인
ros2 launch my_so_arm_gazebo display.launch.py --show-args
```

## 🎯 다음 단계

1. **카메라 이미지 처리**: OpenCV와 연동
2. **그리퍼 추가**: End Effector 손가락 추가
3. **장애물 회피**: 계획 장면에 객체 추가
4. **실제 로봇 제어**: 실제 하드웨어 로봇에 연동

---

**질문이 있으시면 README.md를 참조하세요!**
