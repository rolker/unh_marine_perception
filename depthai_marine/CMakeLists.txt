cmake_minimum_required(VERSION 3.14.4)
project(depthai_marine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(lifecycle_msgs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  list_devices
  src/list_devices.cpp
)

target_link_libraries(list_devices
  depthai::core
  depthai_bridge::depthai_bridge
  rclcpp::rclcpp
)

install(TARGETS list_devices
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  display_time
  src/display_time.cpp
)

install(TARGETS display_time
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  measure_timing
  src/measure_timing.cpp
)

target_link_libraries(measure_timing
  depthai::opencv
  depthai_bridge::depthai_bridge
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS measure_timing
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
  wide_stereo
  src/wide_stereo.cpp
)

target_link_libraries(wide_stereo
  depthai::core
  depthai::opencv
  depthai_bridge::depthai_bridge
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS wide_stereo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
