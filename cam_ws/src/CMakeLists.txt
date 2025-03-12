cmake_minimum_required(VERSION 3.8)
project(camera)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY camera/
    DESTINATION share/${PROJECT_NAME}/camera)

ament_package()

