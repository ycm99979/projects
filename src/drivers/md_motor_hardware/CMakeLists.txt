cmake_minimum_required(VERSION 3.8)
project(md_motor_hardware)

# C++17 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================================================
# MD Motor 4WD Hardware Interface Library (4륜 개별 제어)
# ============================================================================
add_library(md_4wd_hardware SHARED
  src/md_4wd_hardware.cpp
)

target_include_directories(md_4wd_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${serial_INCLUDE_DIRS}
)

target_link_libraries(md_4wd_hardware
  ${serial_LIBRARIES}
)

ament_target_dependencies(md_4wd_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  serial
  geometry_msgs
  sensor_msgs
  std_msgs
)

# ============================================================================
# Pluginlib Export
# ============================================================================
pluginlib_export_plugin_description_file(hardware_interface md_hardware_plugin.xml)

# ============================================================================
# Install
# ============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS md_4wd_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Launch 파일 설치 (test로 이동됨 - 주석처리)
# install(
#   DIRECTORY launch config urdf
#   DESTINATION share/${PROJECT_NAME}
# )

# ============================================================================
# Export
# ============================================================================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  serial
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
