#!/bin/bash

# 환경 초기화 및 설정 스크립트
# Usage: source setup_env.sh

echo "===== ROS2 Jazzy 환경 설정 ====="

# 1. ROS2 Jazzy 환경 설정
source /opt/ros/jazzy/setup.bash
echo "✓ ROS2 Jazzy 설정 완료"

# 2. 워크스페이스 설정
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
source $WORKSPACE_DIR/install/setup.bash
echo "✓ 워크스페이스 설정 완료: $WORKSPACE_DIR"

# 3. 패키지 확인
if ros2 pkg list | grep -q my_so_arm_gazebo; then
    echo "✓ my_so_arm_gazebo 패키지 찾음"
else
    echo "✗ 경고: my_so_arm_gazebo 패키지를 찾을 수 없습니다"
    echo "  해결: colcon build를 다시 실행하세요"
fi

# 4. 실행 준비 완료
echo ""
echo "===== 준비 완료! ====="
echo ""
echo "실행 명령:"
echo "  1. RViz 시각화:    ros2 launch my_so_arm_gazebo display.launch.py"
echo "  2. Gazebo 시뮬레이션: ros2 launch my_so_arm_gazebo gazebo.launch.py"
echo "  3. MoveIt2 (추천):   ros2 launch my_so_arm_gazebo moveit.launch.py"
echo ""
