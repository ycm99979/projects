cmake_minimum_required(VERSION 3.8)
project(robot_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (의존성 패키지 명시)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(slam_toolbox REQUIRED)

# build testing (의존성이 존재하는 패키지들이 실제로 있는지 확인)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# install (colcon build 시 install 폴더에 project_name 폴더 내부에 붙여넣는 부분)
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_package()