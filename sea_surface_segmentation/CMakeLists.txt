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
find_package(cv_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(lifecycle_msgs)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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

add_executable(
  segments_to_pointcloud
  src/segments_to_pointcloud.cpp
)

target_link_libraries(segments_to_pointcloud
  image_geometry::image_geometry
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}

)

ament_target_dependencies(segments_to_pointcloud
  cv_bridge
  pcl_ros
)

install(TARGETS segments_to_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
