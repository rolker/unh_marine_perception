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
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(depthai_marine REQUIRED)
find_package(image_geometry REQUIRED)
find_package(grid_map_core REQUIRED)
# grid_map_core's extras inject -DEIGEN_*_PLUGIN globally via add_definitions, so
# every TU compiled past this point needs grid_map_core's include dir to find the
# plugin headers — not just the targets that link grid_map_core::grid_map_core.
# The legacy *_INCLUDE_DIRS var is empty under the modern target export, so pull
# the path from the imported target and add it globally.
get_target_property(grid_map_core_INC grid_map_core::grid_map_core INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${grid_map_core_INC})
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
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
  ${diagnostic_msgs_TARGETS}
  ${diagnostic_updater_TARGETS}
  ${lifecycle_msgs_TARGETS}
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
  find_package(launch_testing_ament_cmake REQUIRED)

  # Bag fixture for the launch_testing integration. Installed under
  # share/ so the test (which uses get_package_share_directory) can
  # find it both in-source and after install.
  install(
    DIRECTORY test/fixtures
    DESTINATION share/${PROJECT_NAME}/test
  )

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
  # uses sensor_msgs CameraInfo for the synthetic fixture. tf2 is linked
  # only to cross-check rotation_matrix_from_quaternion against
  # tf2::Matrix3x3 (the production conversion this header replaced).
  find_package(tf2 REQUIRED)
  ament_add_gtest(test_segments_projection test/test_segments_projection.cpp)
  target_include_directories(test_segments_projection PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_segments_projection
    image_geometry::image_geometry
    tf2::tf2
    ${sensor_msgs_TARGETS}
  )

  # occupancy_buffer.hpp is the pure log-odds buffer (grid_map storage +
  # rolling); private to the layer in src/, exercised via this GTest. Links
  # grid_map_core for GridMap.
  ament_add_gtest(test_occupancy_buffer test/test_occupancy_buffer.cpp)
  target_include_directories(test_occupancy_buffer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_occupancy_buffer
    grid_map_core::grid_map_core
  )

  # launch_testing integration: replay a 20s slice of the 2026-05-22
  # deployment bag through segments_to_pointcloud in reflex-mode and
  # assert obstacle points land in the forward danger sector. This
  # is the deployment-evidence regression for #17.
  add_launch_test(test/test_segments_to_pointcloud_bag.py
    TIMEOUT 60
  )
endif()

ament_package()
