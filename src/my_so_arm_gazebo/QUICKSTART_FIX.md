# 🚀 빠른 시작 가이드 (패키지 찾기 오류 해결)

## ✅ 오류 원인

`Package 'my_so_arm_gazebo' not found` 오류는 **ROS 환경 설정이 제대로 되지 않아서** 발생합니다.

## 🔧 해결 방법

### **방법 1: 새 터미널에서 직접 실행 (권장)**

```bash
# 1. 새 터미널 열기
# 2. 워크스페이스로 이동
cd ~/project/ros2/my_first_ws

# 3. 환경 설정
source install/setup.bash

# 4. Launch 실행
ros2 launch my_so_arm_gazebo display.launch.py
```

### **방법 2: setup_env.sh 사용 (자동 설정)**

```bash
# 1. setup_env.sh 스크립트에 권한 부여
chmod +x ~/project/ros2/my_first_ws/src/my_so_arm_gazebo/setup_env.sh

# 2. 스크립트 실행
source ~/project/ros2/my_first_ws/src/my_so_arm_gazebo/setup_env.sh

# 3. 이제 바로 launch 실행 가능
ros2 launch my_so_arm_gazebo display.launch.py
```

### **방법 3: .bashrc에 자동 추가 (매번 자동)**

이미 다음이 추가되어 있습니다:
```bash
source /opt/ros/jazzy/setup.bash
source ~/project/ros2/my_first_ws/install/setup.bash
```

**새 터미널을 열면 자동으로 설정됩니다.**

## 📋 전체 실행 단계

### **1단계: 환경 설정**
```bash
cd ~/project/ros2/my_first_ws
source install/setup.bash
```

### **2단계: 선택 - 3가지 모드**

#### **모드 1️⃣: URDF 시각화 (RViz만)**
```bash
ros2 launch my_so_arm_gazebo display.launch.py
```
- **사용 시기**: 로봇 모델만 보고 싶을 때
- **특징**: 가볍고 빠름, 물리 시뮬레이션 없음
- **제어**: Joint State Publisher GUI에서 슬라이더로 조종

#### **모드 2️⃣: Gazebo 시뮬레이션**
```bash
ros2 launch my_so_arm_gazebo gazebo.launch.py
```
- **사용 시기**: 물리 시뮬레이션이 필요할 때
- **특징**: 중력, 충돌, 관성 포함
- **제어**: 명령으로 직접 제어

#### **모드 3️⃣: MoveIt2 전체 (🌟 추천)**
```bash
ros2 launch my_so_arm_gazebo moveit.launch.py
```
- **사용 시기**: 경로 계획/로봇 제어
- **특징**: 경로 계획, 충돌 회피, GUI 제어
- **제어**: RViz에서 그래픽으로 포즈 설정 → 경로 계획 → 실행

## 🎯 MoveIt2 모드에서 로봇 제어하기

1. **새 터미널에서:**
   ```bash
   cd ~/project/ros2/my_first_ws
   source install/setup.bash
   ros2 launch my_so_arm_gazebo moveit.launch.py
   ```

2. **RViz 창이 열리면:**
   - 3D 뷰에서 마우스로 로봇 드래그 → 목표 포즈 설정
   - 왼쪽 "Plan" 버튼 → 경로 계획
   - "Execute" 버튼 → 로봇 이동

3. **또는 Python으로 제어:**
   ```bash
   # 새 터미널에서
   python3 -m my_so_arm_gazebo.arm_controller
   ```

## 🔍 패키지 확인

```bash
# 패키지가 설치되었는지 확인
source ~/project/ros2/my_first_ws/install/setup.bash
ros2 pkg list | grep my_so_arm_gazebo

# launch 파일 확인
ros2 launch my_so_arm_gazebo display.launch.py --show-args
```

## 🛠️ 재빌드 필요한 경우

```bash
cd ~/project/ros2/my_first_ws

# 깨끗하게 빌드
rm -rf build install log
colcon build --symlink-install --packages-select my_so_arm_gazebo

# 환경 다시 설정
source install/setup.bash
```

## 📚 파일 위치

| 항목 | 경로 |
|------|------|
| URDF 모델 | `urdf/arm.urdf.xacro` |
| MoveIt 설정 | `config/arm.srdf` |
| Launch 파일 | `launch/*.launch.py` |
| 제어 코드 | `my_so_arm_gazebo/arm_controller.py` |

## ✅ 검증

```bash
# 1. 환경 설정
source ~/project/ros2/my_first_ws/install/setup.bash

# 2. 패키지 확인
ros2 pkg list | grep my_so_arm_gazebo
# 출력: my_so_arm_gazebo

# 3. Launch 파일 확인
ls -la ~/project/ros2/my_first_ws/install/my_so_arm_gazebo/share/my_so_arm_gazebo/launch/
```

---

**이제 준비 완료입니다! 위의 단계를 따라 실행하세요.** 🎉
