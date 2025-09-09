cmake_minimum_required(VERSION 3.5)
project(digital_twin_core)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  DIRECTORY digital_twin_core
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
