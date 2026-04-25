cmake_minimum_required(VERSION 3.8)
project(aries)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gripper_controllers REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(
  DIRECTORY ard_connect config launch meshes urdf worlds 
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ard_connect/ros2_to_arduino_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()