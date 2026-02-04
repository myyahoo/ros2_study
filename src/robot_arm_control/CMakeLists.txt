cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROS2 메시지/서비스 파일
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmState.msg"
  "msg/DetectionResult.msg"
  "msg/JointCommand.msg"
  "srv/GripperCommand.srv"
  "srv/MoveArm.srv"
)

ament_package()
