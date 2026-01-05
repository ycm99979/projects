cmake_minimum_required(VERSION 3.8)
project(robot_web_interface)

find_package(ament_cmake REQUIRED)

install(DIRECTORY www launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
