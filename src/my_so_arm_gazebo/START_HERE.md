# ✅ 완전 설치 완료 - 실행 방법

패키지 설치가 완료되었습니다! 이제 다음 명령으로 실행하면 됩니다.

## 🚀 실행 준비

**새 터미널에서 매번 실행해야 할 환경 설정:**

```bash
cd ~/project/ros2/my_first_ws
source install/setup.bash
```

또는 `.bashrc`에 이미 추가되어 있으므로 **새 터미널을 열면 자동으로 설정됩니다.**

## 🎯 3가지 실행 모드

### **1️⃣ URDF 시각화 (RViz만, 가벼움)**

```bash
ros2 launch my_so_arm_gazebo display.launch.py
```

✨ **특징:**
- 빠른 로드 (1-2초)
- CPU 사용량 적음
- URDF 모델 시각화
- Joint State Publisher GUI에서 슬라이더로 조종 가능

---

### **2️⃣ Gazebo 물리 시뮬레이션**

```bash
ros2 launch my_so_arm_gazebo gazebo.launch.py
```

✨ **특징:**
- 중력, 충돌, 관성 포함
- 실제 물리 시뮬레이션
- 무거운 작업 (로드 시간 길음)

---

### **3️⃣ MoveIt2 경로 계획 (🌟 추천)**

```bash
ros2 launch my_so_arm_gazebo moveit.launch.py
```

✨ **특징:**
- 가장 강력한 기능
- 그래픽 경로 계획
- 역운동학(IK) 솔버
- 충돌 회피

**사용 방법:**
1. RViz 창에서 3D 뷰를 마우스로 드래그하여 로봇의 목표 포즈 설정
2. 왼쪽 "Planning" 패널 → "Plan" 클릭
3. 주황색 선으로 경로가 표시됨
4. "Execute" 클릭 → 로봇 이동

---

## 🐍 Python 예제 코드

### **MoveIt2 모드에서 로봇 제어**

```python
import rclpy
from moveit_commander import MoveGroupCommander

# 초기화
rclpy.init()
group = MoveGroupCommander("manipulator")

# 1. 홈 포지션 이동
group.set_named_target("home")
group.go()

# 2. 준비 포지션 이동
group.set_named_target("ready")
group.go()

# 3. 특정 포즈로 이동 (역운동학)
from geometry_msgs.msg import Pose
pose = Pose()
pose.position.x = 0.3  # X 좌표
pose.position.y = 0.1  # Y 좌표
pose.position.z = 0.5  # Z 좌표 (높이)
pose.orientation.w = 1.0

group.set_pose_target(pose)
group.go()

rclpy.shutdown()
```

---

## 📊 로봇 사양 확인

### **6DOF 관절:**
- `shoulder_pan_joint`: ±180° (회전)
- `shoulder_lift_joint`: ±90° (들었다 내림)
- `elbow_joint`: ±180° (팔꿈치)
- `wrist_1_joint`: ±180° (손목 회전)
- `wrist_2_joint`: ±90° (손목 위아래)
- `wrist_3_joint`: ±180° (손목 좌우)

### **카메라:**
- 위치: End Effector에 장착
- 프레임: `camera_optical_frame`

---

## 🔧 유용한 명령어

### **패키지 정보 확인**
```bash
# 패키지 위치
ros2 pkg prefix my_so_arm_gazebo

# Launch 파일 확인
ros2 launch my_so_arm_gazebo display.launch.py --show-args

# 패키지 목록 조회
ros2 pkg list | grep my_so_arm_gazebo
```

### **ROS2 정보 확인**
```bash
# 실행 중인 노드 확인
ros2 node list

# 토픽 확인
ros2 topic list

# 서비스 확인
ros2 service list

# 현재 관절 상태
ros2 topic echo /joint_states
```

### **TF (Transform) 확인**
```bash
# TF 트리 생성
ros2 run tf2_tools view_frames

# 특정 프레임 확인
ros2 run tf2_ros tf2_echo base_link link_6
```

---

## 💾 파일 구조

```
~/project/ros2/my_first_ws/src/my_so_arm_gazebo/
├── urdf/
│   └── arm.urdf.xacro          # 로봇 모델 정의
├── config/
│   ├── arm.srdf                # MoveIt 설정
│   ├── kinematics.yaml         # IK 솔버
│   ├── ompl_planning.yaml      # 경로 계획 알고리즘
│   ├── controllers.yaml        # 컨트롤러
│   ├── visualization.rviz      # RViz 설정 (디스플레이)
│   └── moveit.rviz             # RViz 설정 (MoveIt)
├── launch/
│   ├── display.launch.py       # URDF 시각화
│   ├── gazebo.launch.py        # Gazebo 시뮬레이션
│   └── moveit.launch.py        # MoveIt2 실행
├── my_so_arm_gazebo/
│   └── arm_controller.py       # Python 제어 라이브러리
└── package.xml                 # 패키지 메타데이터
```

---

## 🐛 문제 해결

### **"Package not found" 오류**
```bash
# 새 터미널 열기 → 자동으로 설정됨
# 또는 수동으로:
cd ~/project/ros2/my_first_ws
source install/setup.bash
```

### **RViz에서 로봇이 안 보임**
1. 좌측 "Displays" 클릭
2. "RobotModel"이 체크되어 있는지 확인
3. "robot_description"이 선택되어 있는지 확인

### **경로 계획이 실패함**
- 충돌 제약 조건 확인
- 목표 포즈가 로봇 reach 범위 내인지 확인
- `config/ompl_planning.yaml`에서 계획 시간 증가

### **Gazebo가 안 열림**
```bash
sudo apt-get install gazebo
gazebo --version  # 확인
```

---

## 📚 다음 단계

1. **카메라 이미지 처리**: OpenCV와 연동하여 시각 피드백 추가
2. **End Effector 추가**: 그리퍼나 흡입기 추가
3. **장애물 회피**: 계획 장면에 객체 추가
4. **실제 로봇 제어**: 실제 하드웨어 로봇과 연동

---

## 📖 참고 문서

- [README.md](README.md) - 상세 설명
- [INSTALL.md](INSTALL.md) - 설치 가이드
- [QUICKSTART_FIX.md](QUICKSTART_FIX.md) - 오류 해결
- [ROS2 공식 문서](https://docs.ros.org/en/jazzy/)
- [MoveIt2 튜토리얼](https://moveit.ros.org/)

---

**이제 바로 실행해보세요! 🎉**

```bash
ros2 launch my_so_arm_gazebo moveit.launch.py
```
