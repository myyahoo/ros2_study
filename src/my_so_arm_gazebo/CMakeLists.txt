cmake_minimum_required(VERSION 3.8)
project(my_so_arm_gazebo)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY my_so_arm_gazebo DESTINATION lib/python3.10/site-packages)

ament_package()
