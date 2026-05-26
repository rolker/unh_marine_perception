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
find_package(depthai_marine REQUIRED)
find_package(image_geometry REQUIRED)
find_package(lifecycle_msgs)
find_package(nav2_costmap_2d REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pluginlib REQUIRED)
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
  depthai_marine::depthai_marine_lib
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

add_library("sea_surface_layer" SHARED
  src/sea_surface_layer.cpp
)
add_library("sea_surface_layer::sea_surface_layer" ALIAS "sea_surface_layer")

target_link_libraries(sea_surface_layer 
  image_geometry::image_geometry
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)

ament_target_dependencies(sea_surface_layer
  cv_bridge
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(
  TARGETS sea_surface_layer
  EXPORT export_sea_surface_layer
  DESTINATION lib
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_id_resolver test/test_frame_id_resolver.cpp)
  # Resolver header is private to the executable (lives next to its
  # .cpp in src/), so the test target needs its own path into src/.
  target_include_directories(test_frame_id_resolver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # segments_apply.hpp is private to the layer (lives next to
  # sea_surface_layer.cpp in src/); test target reaches into src/
  # the same way test_frame_id_resolver does. Links against
  # nav2_costmap_2d_core for Costmap2D.
  ament_add_gtest(test_segments_apply test/test_segments_apply.cpp)
  target_include_directories(test_segments_apply PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_segments_apply
    nav2_costmap_2d::nav2_costmap_2d_core
  )

  # segments_projection.hpp lives next to segments_to_pointcloud.cpp
  # in src/ for the same reason; pure-logic helper exercised via this
  # GTest. Links against image_geometry for PinholeCameraModel and
  # uses sensor_msgs CameraInfo for the synthetic fixture.
  ament_add_gtest(test_segments_projection test/test_segments_projection.cpp)
  target_include_directories(test_segments_projection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_segments_projection
    image_geometry::image_geometry
    ${sensor_msgs_TARGETS}
  )
endif()

ament_package()
