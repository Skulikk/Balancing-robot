cmake_minimum_required(VERSION 3.8)
project(sensors_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === WiringPi ===
find_library(WIRINGPI_LIB wiringPi)
if(NOT WIRINGPI_LIB)
  message(FATAL_ERROR "Failed to find WiringPi library")
endif()

# === Custom Messages ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderData.msg"
  "msg/IMUData.msg"
  DEPENDENCIES std_msgs
)

# Get the include directories for generated messages
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# === IMU Node ===
add_executable(imu_node src/imu_node.cpp)
target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(imu_node rclcpp std_msgs)
target_link_libraries(imu_node 
  ${WIRINGPI_LIB}
  "${cpp_typesupport_target}")
install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})

# === Encoder Node ===
add_executable(encoder_node src/encoder_node.cpp)
target_include_directories(encoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(encoder_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(encoder_node rclcpp std_msgs)
target_link_libraries(encoder_node
  ${WIRINGPI_LIB}
  "${cpp_typesupport_target}")
install(TARGETS encoder_node DESTINATION lib/${PROJECT_NAME})

# === Testing ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Export dependencies ===
ament_export_dependencies(rosidl_default_runtime)
ament_package()