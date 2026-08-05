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
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(lifecycle_msgs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(depthai_marine_lib
  src/camera_base.cpp
  src/ffmpeg_publisher.cpp
  src/image_publisher.cpp
)

target_include_directories(depthai_marine_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(depthai_marine_lib
  depthai::core
  depthai::opencv
  depthai_bridge::depthai_bridge
  ${ffmpeg_image_transport_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

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
  depthai_marine_lib
)

install(TARGETS wide_stereo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS depthai_marine_lib
  EXPORT depthai_marine_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT depthai_marine_lib
  DESTINATION share/${PROJECT_NAME}
  FILE depthai_marine_lib.cmake
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(depthai_marine_lib)
ament_export_include_directories(include)
ament_export_dependencies(
  depthai
  depthai_bridge
  ffmpeg_image_transport_msgs
  rclcpp
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_h265_params test/test_h265_params.cpp)
  target_link_libraries(test_h265_params
    depthai_marine_lib
  )
  ament_add_gtest(test_h265_bitrate_dynamic test/test_h265_bitrate_dynamic.cpp)
  target_link_libraries(test_h265_bitrate_dynamic
    depthai_marine_lib
  )
endif()

ament_package()
