cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensors_pkg REQUIRED)

# === pigpio Daemon Interface ===
find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2 REQUIRED)
find_path(PIGPIOD_IF2_INCLUDE_DIR pigpiod_if2.h REQUIRED)

add_executable(motor_controller_node src/motor_controller_node.cpp)
target_include_directories(motor_controller_node PUBLIC
  ${PIGPIOD_IF2_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motor_controller_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(motor_controller_node rclcpp std_msgs sensors_pkg)
target_link_libraries(motor_controller_node ${PIGPIOD_IF2_LIBRARY})

install(TARGETS motor_controller_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
