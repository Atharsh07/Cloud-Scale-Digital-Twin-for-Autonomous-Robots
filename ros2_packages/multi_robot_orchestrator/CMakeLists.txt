cmake_minimum_required(VERSION 3.5)
project(multi_robot_orchestrator)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(digital_twin_core REQUIRED)

install(
  DIRECTORY orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
