# 6DOF Robot Arm Control Package

ROS2 기반 6DOF 로봇 팔 제어 패키지입니다. `/dev/ttyACM1` 시리얼 포트를 통해 로봇 팔을 제어합니다.

## 📋 패키지 구조

```
so_arm101_pkg/
├── src/so_arm101_pkg/
│   ├── __init__.py                 # 패키지 초기화
│   ├── arm_serial_comm.py          # 시리얼 통신 모듈
│   ├── arm_kinematics.py           # 기구학 계산 모듈
│   ├── arm_controller.py           # 메인 컨트롤러 (ROS2 노드)
│   ├── arm_joystick.py             # 조이스틱 제어
│   └── arm_demo.py                 # 데모 프로그램
├── launch/
│   └── arm_control.launch.py       # Launch 파일
├── package.xml                     # 패키지 설정
├── setup.py                        # Python 설정
└── README.md                       # 이 파일
```

## 🔧 설치 및 빌드

### 1. 필수 패키지 설치

```bash
cd ~/project/ros2/my_first_ws
sudo apt-get install python3-serial python3-numpy

# 또는 pip 사용
pip install pyserial numpy
```

### 2. 패키지 빌드

```bash
cd ~/project/ros2/my_first_ws
colcon build --packages-select so_arm101_pkg
source install/setup.bash
```

## 🚀 사용 방법

### 1. 메인 컨트롤러 실행

```bash
ros2 launch so_arm101_pkg arm_control.launch.py
```

또는 직접 실행:

```bash
ros2 run so_arm101_pkg arm_controller
```

### 2. 명령 전송

#### 관절 각도로 제어

```bash
ros2 topic pub /arm_controller/joint_command std_msgs/msg/Float32MultiArray \
  "data: [0.0, 30.0, -45.0, 0.0, 0.0, 0.0]"
```

#### 직교좌표 자세로 제어 (역기구학 자동 계산)

```bash
ros2 topic pub /arm_controller/pose_command std_msgs/msg/Float32MultiArray \
  "data: [200.0, 100.0, 300.0, 0.0, 0.0, 0.0]"
```

#### 특수 명령

```bash
# Home 위치로 이동
ros2 topic pub /arm_controller/command std_msgs/msg/String "data: 'home'"

# 긴급 정지
ros2 topic pub /arm_controller/command std_msgs/msg/String "data: 'stop'"

# 속도 설정 (1-100%)
ros2 topic pub /arm_controller/command std_msgs/msg/String "data: 'speed:50'"
```

### 3. 조이스틱 제어

```bash
# 조이스틱 드라이버 설치
sudo apt-get install ros-jazzy-joy

# 터미널 1: 조이스틱 노드 실행
ros2 run joy joy_node

# 터미널 2: 로봇 팔 컨트롤러 실행
ros2 run so_arm101_pkg arm_controller

# 터미널 3: 조이스틱 제어 노드 실행
ros2 run so_arm101_pkg arm_joy
```

### 4. 키보드 제어

```bash
# 터미널 1: 로봇 팔 컨트롤러 실행
ros2 run so_arm101_pkg arm_controller

# 터미널 2: 키보드 제어 노드 실행
ros2 run so_arm101_pkg arm_keyboard
```

### 5. 데모 실행

```bash
ros2 run so_arm101_pkg arm_demo
```

## 📡 시리얼 포트 설정

### 포트 확인

```bash
# Arduino 연결 확인
ls -la /dev/ttyACM*

# 포트 권한 설정 (udev 규칙)
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인
```

### 포트 변경

`launch/arm_control.launch.py`에서 수정:

```python
parameters=[
    {'serial_port': '/dev/ttyACM0'},  # 또는 다른 포트
    {'baudrate': 115200},
]
```

## 🎮 조이스틱 매핑

| 입력 | 기능 |
|------|------|
| 좌측 스틱 X축 | J1 (베이스 회전) |
| 좌측 스틱 Y축 | J2 (팔 이동) |
| 우측 스틱 Y축 | J3 (높이 조정) |
| 우측 스틱 X축 | J4 (Pitch) |
| L/R 트리거 | J5 (Roll) |
| L/R 버튼 | J6 (그리퍼) |
| A 버튼 | Home 위치로 이동 |
| B 버튼 | 긴급 정지 |

## ⌨️ 키보드 매핑

### 관절 제어

| 키 | 기능 | 키 | 기능 |
|------|---------|------|---------|
| **Q/A** | J1 회전 (±) | **W/S** | J2 수평 (±) |
| **E/D** | J3 높이 (±) | **R/F** | J4 Pitch (±) |
| **T/G** | J5 Roll (±) | **Y/H** | J6 그리퍼 (±) |

### 특수 명령

| 키 | 기능 |
|------|---------|
| **H** | Home 위치로 이동 |
| **L** | 현재 각도 표시 |
| **C** | 각도 초기화 (0도) |
| **Z** | 속도 감소 (-1°/sec) |
| **X** | 속도 증가 (+1°/sec) |
| **Ctrl+C** | 종료 |

**예시:**
- `Q` 누르기: J1 시계 방향 회전 (+5°)
- `A` 누르기: J1 반시계 방향 회전 (-5°)
- `X` 누르기: 속도 증가 (기본값: 5°/sec)

## 📦 ROS2 토픽

### Publish (발행)

- `/joint_states` - 현재 관절 상태
- `/arm_controller/feedback` - 피드백 메시지

### Subscribe (구독)

- `/arm_controller/joint_command` - 관절 각도 명령
- `/arm_controller/pose_command` - 직교좌표 자세 명령
- `/arm_controller/command` - 문자 기반 명령

## 💻 Arduino 코드 예시

로봇 팔을 제어하는 Arduino 코드 예시입니다:

```cpp
#include <Servo.h>

Servo joint[6];
const int SERVO_PINS[6] = {3, 5, 6, 9, 10, 11};

void setup() {
  Serial.begin(115200);
  
  // 서보모터 초기화
  for(int i = 0; i < 6; i++) {
    joint[i].attach(SERVO_PINS[i]);
    joint[i].write(90);  // 중앙 위치
  }
}

void loop() {
  if(Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if(command.startsWith("MOVE:")) {
      // MOVE:45.0,30.0,-45.0,0.0,0.0,0.0
      parseAndMove(command);
    } else if(command == "HOME") {
      for(int i = 0; i < 6; i++) {
        joint[i].write(90);
      }
    } else if(command == "STOP") {
      // 정지 처리
    } else if(command.startsWith("SPEED:")) {
      // 속도 설정
    }
  }
}

void parseAndMove(String cmd) {
  cmd = cmd.substring(5);  // "MOVE:" 제거
  
  for(int i = 0; i < 6; i++) {
    int commaPos = cmd.indexOf(',');
    float angle = cmd.substring(0, commaPos).toFloat();
    
    // 각도를 서보 펄스로 변환 (0-180도)
    int pwm = map(angle, -180, 180, 0, 180);
    joint[i].write(pwm);
    
    cmd = cmd.substring(commaPos + 1);
  }
}
```

## 🔍 디버깅

### 자동 진단 도구 (권장)

```bash
cd /home/tech/project/ros2/my_first_ws
source install/setup.bash

# 자동 진단: HOME → 45도 → 30도 등 테스트
python3 -m so_arm101_pkg.serial_diagnostic

# 수동 테스트: 직접 명령 입력
python3 -m so_arm101_pkg.serial_diagnostic --manual
```

**수동 테스트 모드 명령:**
```
HOME                           → 홈 위치로 이동
MOVE:45,0,0,0,0,0            → J1만 45도로 이동
MOVE:0,30,0,0,0,0            → J2만 30도로 이동
STOP                          → 긴급 정지
SPEED:50                      → 속도 50%로 설정
quit                          → 종료
```

### 모터가 움직이지 않을 때 확인사항

1. **시리얼 통신 확인**
   ```bash
   python3 -m so_arm101_pkg.serial_diagnostic
   ```
   - 명령 전송 성공 여부 확인
   - Arduino 응답 여부 확인

2. **Arduino 코드 검증**
   - Arduino IDE Serial Monitor에서 수동 테스트:
     ```
     HOME
     MOVE:45.0,0.0,0.0,0.0,0.0,0.0
     SPEED:75
     ```

3. **모터 연결 확인**
   - 서보모터 핀 번호 확인: `const int SERVO_PINS[6] = {3, 5, 6, 9, 10, 11};`
   - 전원 연결 확인 (5V, 최소 2A)
   - GND(접지) 연결 확인

4. **전원 공급 부족**
   - USB 포트만으로는 부족할 수 있음
   - 별도 전원 공급(배터리/AC 어댑터) 권장
   - 전원 폴드(GND)를 Arduino와 공유

5. **포트 권한 확인**
   ```bash
   sudo usermod -a -G dialout $USER
   # 로그아웃 후 다시 로그인
   ```

### ROS2 로그 확인

```bash
# 노드 상태 확인
ros2 node list

# 토픽 확인
ros2 topic list
ros2 topic echo /joint_states

# 로그 레벨 설정
export ROS_LOG_DIR=/tmp/ros_logs
```

## 🛠️ 커스터마이징

### 기구학 파라미터 수정

`arm_kinematics.py`의 `__init__` 메소드에서:

```python
self.link_lengths = [50, 150, 150, 100, 100, 100]  # 링크 길이 (mm)
```

### 속도 제한 추가

`arm_controller.py`에서 속도 프로필 구현

### 충돌 감지 추가

`arm_kinematics.py`에서 collision checking 함수 추가

## 📚 참고 자료

- ROS2 공식 문서: https://docs.ros.org/en/jazzy/
- PySerial 문서: https://pyserial.readthedocs.io/
- DH 파라미터: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters

## 📝 라이선스

Apache License 2.0

---

**개발자**: ROS Development Team  
**버전**: 0.0.1  
**마지막 업데이트**: 2026-01-24
