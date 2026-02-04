#!/usr/bin/env python3
"""
빠른 시작 가이드 및 데모 스크립트
Robot Arm Control 패키지 테스트
"""

import sys
import numpy as np


def print_header(text: str):
    """헤더 출력"""
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)


def demo_ik_solver():
    """역기구학 데모"""
    print_header("역기구학 솔버 데모")
    
    try:
        from robot_arm_control.arm_control_node import SimpleIK
        
        ik = SimpleIK()
        
        # 테스트 위치들
        test_positions = [
            np.array([0.20, 0.0, 0.10]),   # 정면
            np.array([0.15, 0.15, 0.05]),  # 왼쪽 대각
            np.array([0.15, -0.15, 0.05]), # 오른쪽 대각
        ]
        
        for i, pos in enumerate(test_positions, 1):
            print(f"\n테스트 {i}: 위치 = {pos}")
            angles = ik.inverse_kinematics(pos)
            print(f"  계산된 관절각도:")
            for j, angle in enumerate(angles):
                print(f"    θ{j+1} = {np.degrees(angle):.2f}°")
        
        print("\n✓ 역기구학 솔버 정상 작동")
        
    except Exception as e:
        print(f"✗ 에러: {e}")
        return False
    
    return True


def demo_utils():
    """유틸리티 함수 데모"""
    print_header("유틸리티 함수 데모")
    
    try:
        from robot_arm_control.utils import RobotArmUtils, PIDController
        
        # 거리 계산
        p1 = np.array([0, 0, 0])
        p2 = np.array([0.3, 0.4, 0])
        dist = RobotArmUtils.distance(p1, p2)
        print(f"두 점 사이 거리: {dist:.3f} m")
        
        # 각도 변환
        rad = np.pi / 4
        deg = RobotArmUtils.rad_to_deg(rad)
        print(f"π/4 라디안 = {deg:.1f}°")
        
        # PID 제어기
        pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        error = 0.5
        output = pid.update(error, dt=0.01)
        print(f"PID 제어 출력 (error={error}): {output:.3f}")
        
        print("\n✓ 유틸리티 함수 정상 작동")
        
    except Exception as e:
        print(f"✗ 에러: {e}")
        return False
    
    return True


def demo_state_machine():
    """상태 머신 데모 (시뮬레이션)"""
    print_header("상태 머신 시뮬레이션")
    
    try:
        from robot_arm_control.task_executor_node import TaskState
        
        # 상태 전이 시뮬레이션
        states = [
            TaskState.IDLE,
            TaskState.DETECT_PEN,
            TaskState.APPROACH_PEN,
            TaskState.GRASP_PEN,
            TaskState.LIFT_PEN,
            TaskState.MOVE_SIDE,
            TaskState.RELEASE_PEN,
            TaskState.RETURN_HOME,
        ]
        
        print("작업 상태 순서:")
        for i, state in enumerate(states, 1):
            print(f"  {i}. {state.name}")
        
        print("\n✓ 상태 머신 정상 작동")
        
    except Exception as e:
        print(f"✗ 에러: {e}")
        return False
    
    return True


def check_dependencies():
    """필수 라이브러리 확인"""
    print_header("필수 라이브러리 확인")
    
    required_packages = {
        'rclpy': 'ROS2 Python 라이브러리',
        'numpy': 'NumPy',
        'cv2': 'OpenCV',
    }
    
    optional_packages = {
        'torch': 'PyTorch',
        'ultralytics': 'Ultralytics YOLO',
    }
    
    print("\n[필수 패키지]")
    all_required_ok = True
    for pkg, name in required_packages.items():
        try:
            __import__(pkg)
            print(f"  ✓ {name}")
        except ImportError:
            print(f"  ✗ {name} (설치 필요)")
            all_required_ok = False
    
    print("\n[선택 패키지]")
    for pkg, name in optional_packages.items():
        try:
            __import__(pkg)
            print(f"  ✓ {name}")
        except ImportError:
            print(f"  - {name} (Vision 기능 사용 시 필요)")
    
    return all_required_ok


def print_quick_start():
    """빠른 시작 가이드 출력"""
    print_header("빠른 시작 가이드")
    
    print("""
1. 패키지 빌드:
   cd /home/tech/project/ros2/my_first_ws
   colcon build --packages-select robot_arm_control
   source install/setup.bash

2. 전체 시스템 실행:
   ros2 launch robot_arm_control robot_arm_bringup.launch.py

3. 개별 노드 실행:
   - Vision만:      ros2 launch robot_arm_control vision_only.launch.py
   - 팔 제어만:     ros2 launch robot_arm_control arm_control_only.launch.py
   - 태스크만:      ros2 launch robot_arm_control task_executor_only.launch.py

4. 카메라 실행 (별도 터미널):
   ros2 launch usb_cam camera.launch.xml camera_name:=camera device:=/dev/video0

5. ROS2 토픽 모니터링:
   ros2 topic list
   ros2 topic echo /robot_arm/detection

더 자세한 정보는 README.md를 참고하세요.
    """)


def main():
    """메인 함수"""
    print("\n" + "=" * 60)
    print("  Robot Arm Control - 빠른 시작 & 테스트")
    print("=" * 60)
    
    # 의존성 확인
    deps_ok = check_dependencies()
    
    if not deps_ok:
        print("\n⚠️  필수 패키지가 설치되지 않았습니다.")
        print("다음 명령어로 설치하세요:")
        print("  pip install -r requirements.txt")
        print("  또는")
        print("  sudo apt-get install ros-humble-rclpy")
    
    print()
    
    # 데모 실행
    results = {
        "역기구학 솔버": demo_ik_solver(),
        "유틸리티 함수": demo_utils(),
        "상태 머신": demo_state_machine(),
    }
    
    # 결과 요약
    print_header("테스트 결과 요약")
    
    for test_name, result in results.items():
        status = "✓ 통과" if result else "✗ 실패"
        print(f"  {test_name}: {status}")
    
    passed = sum(1 for r in results.values() if r)
    total = len(results)
    
    print(f"\n총 {passed}/{total} 테스트 통과")
    
    # 빠른 시작 가이드
    print_quick_start()
    
    print("\n" + "=" * 60)
    print("  테스트 완료!")
    print("=" * 60 + "\n")


if __name__ == '__main__':
    main()
