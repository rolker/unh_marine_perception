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
# Exported public headers live in include/sea_surface_segmentation/
# (occupancy_buffer, segments_projection, segments_apply, occupancy_accumulator).
# Global build-time include so every target below resolves
# "sea_surface_segmentation/<hdr>.hpp"; the install + ament_export_* near the end
# expose the same dir to downstream packages (unh_marine_perception#23).
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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
  src/sea_surface_relay_layer.cpp
)
add_library("sea_surface_layer::sea_surface_layer" ALIAS "sea_surface_layer")

target_link_libraries(sea_surface_layer
  grid_map_core::grid_map_core
  image_geometry::image_geometry
  nav2_costmap_2d::nav2_costmap_2d_core
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${nav_msgs_TARGETS}
)

ament_target_dependencies(sea_surface_layer
  cv_bridge
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(
  TARGETS sea_surface_layer
  DESTINATION lib
)

# Offline bag → costmap video tool (#19 verification): replays recorded
# segmentation through the OccupancyBuffer with the corrected INVERSE (cell→pixel)
# projection, rendering a mosaic (4-cam tiles | live boat costmap | OURS-inverse)
# for review before deployment. The forward `project_observations` lives in
# segments_projection.hpp for the reflex pointcloud feed but is not used here.
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
add_executable(bag_to_costmap_video tools/bag_to_costmap_video.cpp)
# Core headers come from include/ (global include above); no src/ reach-in needed.
target_link_libraries(bag_to_costmap_video
  grid_map_core::grid_map_core
  image_geometry::image_geometry
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  tf2::tf2
  ${OpenCV_LIBS}
  ${sensor_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)
ament_target_dependencies(bag_to_costmap_video cv_bridge)
install(TARGETS bag_to_costmap_video DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Export the header-only algorithm core for downstream reuse (unh_marine_perception#23).
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
# Public deps reachable through the exported HEADER-ONLY cores: grid_map_core
# (occupancy_buffer), image_geometry + OpenCV (segments_projection /
# occupancy_accumulator cv::Mat API), nav2_costmap_2d (segments_apply Costmap2D
# bridge). NB: we deliberately do NOT ament_export_targets the sea_surface_layer
# library — it's a pluginlib plugin loaded at runtime, nothing links it, and
# exporting it would force every header-only consumer to resolve the layer's full
# link closure (cv_bridge/rclcpp/tf2_ros/…). Header consumers only need these deps.
ament_export_dependencies(grid_map_core image_geometry nav2_costmap_2d OpenCV)

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

  # segments_apply.hpp is an exported core header (include/, on the path via the
  # global include_directories above). Links nav2_costmap_2d_core for Costmap2D.
  ament_add_gtest(test_segments_apply test/test_segments_apply.cpp)
  target_link_libraries(test_segments_apply
    nav2_costmap_2d::nav2_costmap_2d_core
  )

  # segments_projection.hpp is an exported core header (include/). Pure-logic
  # helper exercised via this GTest. Links image_geometry for PinholeCameraModel,
  # uses sensor_msgs CameraInfo for the synthetic fixture. tf2 is linked only to
  # cross-check rotation_matrix_from_quaternion against tf2::Matrix3x3 (the
  # production conversion this header replaced).
  find_package(tf2 REQUIRED)
  ament_add_gtest(test_segments_projection test/test_segments_projection.cpp)
  target_link_libraries(test_segments_projection
    image_geometry::image_geometry
    tf2::tf2
    ${sensor_msgs_TARGETS}
  )

  # occupancy_buffer.hpp is the pure log-odds buffer (grid_map storage + rolling),
  # an exported core header (include/), exercised via this GTest. Links
  # grid_map_core for GridMap.
  # cost_mapping.hpp (also exercised here) pulls nav2_costmap_2d cost constants.
  ament_add_gtest(test_occupancy_buffer test/test_occupancy_buffer.cpp)
  target_link_libraries(test_occupancy_buffer
    grid_map_core::grid_map_core
    nav2_costmap_2d::nav2_costmap_2d_core
  )

  # occupancy_accumulator.hpp drives the per-frame move→decay→project→hit/miss
  # path shared by the bag→costmap tool and the offline tuner. Links grid_map_core
  # (buffer) + image_geometry (PinholeCameraModel); uses sensor_msgs CameraInfo for
  # the synthetic fixture.
  ament_add_gtest(test_occupancy_accumulator test/test_occupancy_accumulator.cpp)
  target_link_libraries(test_occupancy_accumulator
    grid_map_core::grid_map_core
    image_geometry::image_geometry
    ${sensor_msgs_TARGETS}
  )

  # Plugin smoke test: loads each costmap_2d plugin via pluginlib AND calls
  # `Layer::initialize` against a real LayeredCostmap + LifecycleNode. The
  # initialize path is what surfaces declaration-order bugs in `onInitialize`
  # (e.g. registering an on-set-parameters callback before declaring a param
  # the callback would reject — throws InvalidParameterValueException during
  # `declare_parameter`'s initial-value validation, abort onInitialize, layer
  # fails to load at deployment). Unit tests that exercise the pure helpers
  # don't reach this path; this test does. Links nav2_costmap_2d_core +
  # nav2_util for LayeredCostmap + LifecycleNode and pluginlib for ClassLoader.
  find_package(nav2_util REQUIRED)
  ament_add_gtest(test_layer_plugin_loading test/test_layer_plugin_loading.cpp)
  target_link_libraries(test_layer_plugin_loading
    nav2_costmap_2d::nav2_costmap_2d_core
    tf2_ros::tf2_ros
  )
  ament_target_dependencies(test_layer_plugin_loading
    nav2_util
    pluginlib
    rclcpp
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
