cmake_minimum_required(VERSION 3.14.4)
project(sea_surface_segmentation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(lifecycle_msgs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  sea_surface_segmentation
  src/sea_surface_segmentation.cpp
)

target_link_libraries(sea_surface_segmentation
  depthai::core
  depthai_bridge::depthai_bridge
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}

)

install(TARGETS sea_surface_segmentation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
