#pragma once

// Private implementation header for the sea_surface_segmentation node.
// Lives in src/ rather than include/ because the helper has no downstream
// consumers — it's exercised by `src/sea_surface_segmentation.cpp` and by
// `test/test_segmentation_stamp.cpp`, nothing else.

#include <chrono>
#include <cstdint>

#include "rclcpp/time.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace sea_surface_segmentation
{

using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock>;

// Convert a DepthAI device capture timestamp (NNData::getTimestamp(), in the
// steady-clock domain) to a ROS time, mirroring how the depthai_marine camera
// publishers stamp their frames. This is what replaced the old, wrong
// `header.stamp = now()` (issue #28): now() discarded the ~123.5 ms of fixed
// pipeline latency, so every downstream TF lookup resolved a stale pose.
//
// The base offset is re-anchored to the live ROS clock on *every* call
// (`updateBaseTime`), so the result tracks the same clock `/tf` is stamped on
// and cannot accumulate the frozen-anchor drift that a once-at-construction
// anchor would (the latent issue the camera publishers also carried). After the
// re-anchor, `getFrameTime` returns `now_ros + (device_timestamp - now_steady)`,
// i.e. the true capture time expressed against the current ROS clock.
//
// `ros_base_time` / `total_ns_change` are mutated by the re-anchor and must be
// the caller's persistent members (one per stamping path). `steady_base_time` is
// copied by `updateBaseTime`, so its stored value is immaterial after the first
// call — kept as a parameter to match the depthai_bridge convention.
//
// Takes the already-extracted `time_point` rather than the `dai::NNData` so the
// helper has no device dependency and unit-tests without OAK hardware.
inline rclcpp::Time deviceFrameStamp(
  rclcpp::Time & ros_base_time,
  const SteadyTimePoint & steady_base_time,
  int64_t & total_ns_change,
  const SteadyTimePoint & device_timestamp)
{
  dai::ros::updateBaseTime(steady_base_time, ros_base_time, total_ns_change);
  return dai::ros::getFrameTime(ros_base_time, steady_base_time, device_timestamp);
}

}  // namespace sea_surface_segmentation
