# MoveIt2 통합 가이드

## 개요

**robot_arm_control**에 MoveIt2가 통합되었습니다. MoveIt2는 다음 기능을 제공합니다:

- **역기구학(IK)**: 자동 계산
- **모션 플래닝**: OMPL 알고리즘 지원 (RRT, RRT*, TRRT 등)
- **충돌 회피**: 장애물 인식 경로 계획
- **시각화**: RViz를 통한 실시간 모니터링
- **그리퍼 제어**: 손가락 제어 통합

---

## 설치

### 1. MoveIt2 패키지 설치

```bash
sudo apt-get install ros-humble-moveit
sudo apt-get install ros-humble-moveit2
sudo apt-get install ros-humble-moveit-configs-utils
sudo apt-get install ros-humble-moveit-commander
```

### 2. KDL 기하학 라이브러리 설치

```bash
sudo apt-get install ros-humble-kdl-parser
sudo apt-get install liborocos-kdl-dev
```

### 3. OMPL 플래너 설치

```bash
sudo apt-get install ros-humble-ompl
```

---

## 빌드 및 실행

### 빌드

```bash
cd /home/tech/project/ros2/my_first_ws
colcon build --packages-select robot_arm_control
source install/setup.bash
```

### MoveIt2 시스템 실행

```bash
ros2 launch robot_arm_control robot_arm_moveit2.launch.py
```

이 명령어는 다음을 실행합니다:
- RViz 시각화
- MoveIt2 Move Group
- Robot State Publisher
- Arm Control Node (MoveIt2 통합)
- Vision Node
- Task Executor Node

### 개별 컴포넌트 실행

**RViz + MoveIt2만 실행:**
```bash
ros2 launch robot_arm_control moveit2_planning_execution.launch.py
```

**팔 제어 노드 (MoveIt2 비활성화):**
```bash
ros2 run robot_arm_control arm_control_node --ros-args -p use_moveit:=false
```

---

## 사용 예제

### 예제 1: 관절 목표로 이동

```python
from geometry_msgs.msg import Pose
import numpy as np

# 관절 각도 설정
joint_goal = [0.0, -0.785, 0.785, 0.0, 0.0, 0.0]

# MoveIt 실행
move_group.go(joint_goal, wait=True)
move_group.stop()
```

### 예제 2: 엔드 이펙터 위치로 이동

```python
from geometry_msgs.msg import Pose

# 목표 위치 설정
pose_goal = Pose()
pose_goal.position.x = 0.2
pose_goal.position.y = 0.1
pose_goal.position.z = 0.2
pose_goal.orientation.w = 1.0

# 계획 및 실행
move_group.set_pose_target(pose_goal)
plan = move_group.plan()
move_group.execute(plan[1], wait=True)
```

### 예제 3: Cartesian 경로 (직선)

```python
# 웨이포인트 정의
waypoints = []
wpose = move_group.get_current_pose().pose

# 위로 이동
wpose.position.z += 0.1
waypoints.append(wpose.deepcopy())

# 앞으로 이동
wpose.position.x += 0.1
waypoints.append(wpose.deepcopy())

# Cartesian 경로 계획
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints,
    eef_step=0.01,  # 1cm
    jump_threshold=0.0
)

# 실행
move_group.execute(plan, wait=True)
```

### 전체 예제 실행

```bash
ros2 run robot_arm_control moveit2_example
```

---

## RViz 조작

### 1. Interactive Marker 사용

1. RViz에서 "MotionPlanning" 디스플레이 활성화
2. 로봇의 엔드 이펙터 근처에 목표 마커 표시
3. 마우스로 드래그하여 목표 위치 설정
4. "Plan" 버튼 클릭
5. "Execute" 버튼 클릭

### 2. 장애물 추가

```python
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point

box = CollisionObject()
box.header.frame_id = "base_link"
box.id = "obstacle"

primitive = SolidPrimitive()
primitive.type = SolidPrimitive.BOX
primitive.dimensions = [0.1, 0.1, 0.1]

box.primitives.append(primitive)
box.primitive_poses.append(Pose(position=Point(x=0.5, y=0.0, z=0.0)))

scene.add_object(box)
```

### 3. 충돌 회피 경로 계획

MoveIt2는 자동으로 장애물을 회피합니다.

---

## 설정 파일

### kinematics.yaml
- IK 솔버: KDL (Kinematics and Dynamics Library)
- 해상도: 0.005 rad
- 타임아웃: 0.05초

### ompl_planning.yaml
- 플래너: RRTConnect, RRTstar, TRRT, PRMstar
- 기본 플래너: RRTConnect

### controllers.yaml
- Arm 컨트롤러: FollowJointTrajectory
- Gripper 컨트롤러: FollowJointTrajectory

---

## 문제 해결

### 1. "compute_ik" 서비스 없음

**증상:** `IK service not available`

**해결책:**
```bash
# MoveIt2 Move Group이 실행 중인지 확인
ros2 node list | grep move_group

# 없으면 다시 시작
ros2 launch robot_arm_control robot_arm_moveit2.launch.py
```

### 2. 계획 실패

**증상:** "Planning failed"

**원인:**
- 도달 불가능한 위치
- 장애물 충돌
- IK 솔루션 없음

**해결책:**
- 목표 위치 조정
- 장애물 제거
- 시작 자세 변경

### 3. 느린 계획

**증상:** 플래닝이 오래 걸림

**해결책:**
```yaml
# ompl_planning.yaml 수정
planner_configs:
  RRTConnect:
    range: 0.5  # 증가
```

### 4. 그리퍼 제어 안 됨

**증상:** 그리퍼가 움직이지 않음

**해결책:**
```bash
# 그리퍼 컨트롤러 확인
ros2 control list_controllers

# 활성화
ros2 control load_controller gripper_controller
ros2 control switch_controllers --activate gripper_controller
```

---

## 성능 최적화

### 1. IK 성능 향상

```yaml
# kinematics.yaml
general:
  kinematics_solver_timeout: 0.1  # 타임아웃 증가
  kinematics_solver_search_resolution: 0.01  # 해상도 감소
```

### 2. 모션 플래닝 최적화

```yaml
# ompl_planning.yaml
request_adapters:
  - default_planner_request_adapters/AddTimeOptimalParameterization
  - default_planner_request_adapters/FixWorkspaceBounds
```

### 3. 속도 프로필 설정

```yaml
# joint_limits.yaml
joint_limits:
  joint1:
    max_velocity: 2.0  # rad/s (증가)
    max_acceleration: 1.0  # rad/s² (증가)
```

---

## 실제 로봇 연결

### Dynamixel 모터와 연결

```python
# arm_control_node.py
from dynamixel_sdk import *

def connect_motors(self):
    # 포트 열기
    if not portHandler.openPort():
        print("Failed to open port")
        return
    
    # 통신 속도 설정
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set baud rate")
        return
```

### Gazebo 시뮬레이션 (선택사항)

```bash
# Gazebo 시뮬레이션 실행
ros2 launch robot_arm_control sim_gazebo.launch.py
```

---

## 추가 리소스

- [MoveIt2 공식 문서](https://moveit.ros.org)
- [MoveIt2 튜토리얼](https://moveit.ros.org/movingrobot/move_group_interface/move_group_interface_tutorial/)
- [OMPL 플래너](https://ompl.kavrakilab.org)
- [RViz MoveIt Plugin](https://github.com/ros-planning/moveit_visual_tools)

---

**마지막 업데이트:** 2026-01-21  
**버전:** 0.0.2 (MoveIt2 지원)
