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
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# === Find RTIMULib ===
find_path(RTIMULIB_INCLUDE_DIR RTIMULib.h
  HINTS /usr/local/include
  PATH_SUFFIXES RTIMULib)
find_library(RTIMULIB_LIBRARY RTIMULib
  HINTS /usr/local/lib)

if(NOT RTIMULIB_INCLUDE_DIR OR NOT RTIMULIB_LIBRARY)
  message(FATAL_ERROR "Failed to find RTIMULib!")
endif()

message(STATUS "Found RTIMULib: ${RTIMULIB_LIBRARY}")
set(RTIMULIB_LIBRARIES ${RTIMULIB_LIBRARY})
set(RTIMULIB_INCLUDE_DIRS ${RTIMULIB_INCLUDE_DIR})

# === Find pigpio ===
find_library(PIGPIOD_IF2_LIBRARY pigpiod_if2 REQUIRED)
find_path(PIGPIOD_IF2_INCLUDE_DIR pigpiod_if2.h REQUIRED)

find_path(PIGPIO_INCLUDE_DIR pigpio.h HINTS /usr/local/include REQUIRED)
find_library(PIGPIO_LIBRARY pigpio HINTS /usr/local/lib REQUIRED)

if(NOT PIGPIOD_IF2_LIBRARY OR NOT PIGPIOD_IF2_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find pigpiod_if2 (daemon interface for pigpio)")
endif()

if(NOT PIGPIO_INCLUDE_DIR OR NOT PIGPIO_LIBRARY)
  message(FATAL_ERROR "Failed to find pigpio!")
endif()

message(STATUS "Found pigpio: ${PIGPIO_LIBRARY}")

# === Custom Messages ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderData.msg"
  "msg/IMUData.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# === IMU Node ===
add_executable(imu_node src/imu_node.cpp)
target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${RTIMULIB_INCLUDE_DIRS} ${PIGPIO_INCLUDE_DIR}
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(imu_node rclcpp std_msgs)
target_link_libraries(imu_node 
  ${RTIMULIB_LIBRARIES}
  ${PIGPIO_LIBRARY}
  pthread rt
  "${cpp_typesupport_target}")
install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})

# === Encoder Node (using libgpiod v1 C API) ===
add_executable(encoder_node src/encoder_node.cpp)

target_include_directories(encoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GPIOD_INCLUDE_DIRS}
)

target_compile_features(encoder_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(encoder_node
  rclcpp
  std_msgs
)

target_link_libraries(encoder_node
  ${GPIOD_LIBRARIES}
  pthread
  rt
  "${cpp_typesupport_target}"
)

install(TARGETS encoder_node DESTINATION lib/${PROJECT_NAME})

# === ULTRA_S Node ===
add_executable(ultra_s_node src/ultra_s_node.cpp)

target_include_directories(ultra_s_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${PIGPIOD_IF2_INCLUDE_DIR} ${PIGPIO_INCLUDE_DIR}
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ultra_s_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ultra_s_node rclcpp std_msgs)

target_link_libraries(ultra_s_node
  ${PIGPIOD_IF2_LIBRARY}
  ${PIGPIO_LIBRARY}
  pthread
  rt
  "${cpp_typesupport_target}"
)

install(TARGETS ultra_s_node DESTINATION lib/${PROJECT_NAME})

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
