#!/bin/bash

# ROS2 Jazzy 6DOF 로봇 팔 - 빠른 시작 가이드
# This script helps setup and run the 6DOF robotic arm

set -e

echo "================================"
echo "6DOF Robotic Arm - Quick Start"
echo "================================"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 필수 패키지 설치 확인
echo -e "\n${YELLOW}Step 1: Checking required packages...${NC}"

REQUIRED_PACKAGES=(
    "gazebo"
    "ros-jazzy-gazebo"
    "ros-jazzy-moveit"
    "ros-jazzy-ros2-control"
)

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo -e "${GREEN}✓${NC} $pkg installed"
    else
        echo -e "${RED}✗${NC} $pkg not installed"
    fi
done

# 2. 빌드
echo -e "\n${YELLOW}Step 2: Building workspace...${NC}"
cd "$(dirname "$0")/../.."
colcon build --symlink-install --packages-select my_so_arm_gazebo
echo -e "${GREEN}✓ Build complete${NC}"

# 3. 환경 설정
echo -e "\n${YELLOW}Step 3: Setting up environment...${NC}"
source install/setup.bash
echo -e "${GREEN}✓ Environment configured${NC}"

# 4. 메뉴
echo -e "\n${YELLOW}Step 4: Choose mode:${NC}"
echo "1) Display in RViz (visualization only)"
echo "2) Gazebo Simulation"
echo "3) MoveIt2 Planning (Full setup)"
echo -e "\nEnter choice (1-3): "

read -r choice

case $choice in
    1)
        echo -e "${GREEN}Launching RViz display...${NC}"
        ros2 launch my_so_arm_gazebo display.launch.py
        ;;
    2)
        echo -e "${GREEN}Launching Gazebo simulation...${NC}"
        ros2 launch my_so_arm_gazebo gazebo.launch.py
        ;;
    3)
        echo -e "${GREEN}Launching MoveIt2...${NC}"
        ros2 launch my_so_arm_gazebo moveit.launch.py
        ;;
    *)
        echo -e "${RED}Invalid choice${NC}"
        exit 1
        ;;
esac
