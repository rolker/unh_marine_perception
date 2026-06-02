#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "segmentation_stamp.hpp"

using sea_surface_segmentation::deviceFrameStamp;
using sea_surface_segmentation::SteadyTimePoint;

// Regression guard for issue #28: the segmentation stamp must derive from the
// device capture timestamp, NOT wall-clock now(). A frame captured ~200 ms ago
// must yield a stamp ~200 ms in the past relative to now(); a regression to
// now() would collapse that lag to ~0.
TEST(SegmentationStamp, DerivesFromDeviceTimestampNotNow)
{
  rclcpp::Clock clock;  // RCL_SYSTEM_TIME — the same clock updateBaseTime reads
  rclcpp::Time ros_base = clock.now();
  SteadyTimePoint steady_base = std::chrono::steady_clock::now();
  int64_t total_ns_change = 0;

  constexpr auto kCaptureLag = std::chrono::milliseconds(200);
  const SteadyTimePoint device_ts = std::chrono::steady_clock::now() - kCaptureLag;

  const rclcpp::Time stamp =
    deviceFrameStamp(ros_base, steady_base, total_ns_change, device_ts);

  const double lag_s = (clock.now() - stamp).seconds();
  // Bounds are deliberately wide: the regression this guards (a revert to now())
  // collapses the lag to ~0, which the lower bound catches with huge margin. The
  // upper bound is only a gross sign/scale sanity ceiling — scheduler stalls and
  // forward clock steps push the lag UP, so a tight ceiling would flake on CI
  // without adding regression-catching power.
  EXPECT_GT(lag_s, 0.1) << "stamp ~= now() — device timestamp was ignored (the #28 bug)";
  EXPECT_LT(lag_s, 1.0) << "stamp implausibly far in the past — conversion sign/scale error";
}

// The device timestamp must actually drive the result: a later capture (smaller
// lag) yields a later stamp. Guards against the conversion degenerating to a
// constant.
TEST(SegmentationStamp, LaterCaptureYieldsLaterStamp)
{
  rclcpp::Clock clock;
  SteadyTimePoint steady_base = std::chrono::steady_clock::now();
  int64_t change_older = 0;
  int64_t change_newer = 0;

  const auto t = std::chrono::steady_clock::now();
  const SteadyTimePoint older = t - std::chrono::milliseconds(300);
  const SteadyTimePoint newer = t - std::chrono::milliseconds(100);

  rclcpp::Time ros_base_older = clock.now();
  rclcpp::Time ros_base_newer = clock.now();
  const rclcpp::Time stamp_older =
    deviceFrameStamp(ros_base_older, steady_base, change_older, older);
  const rclcpp::Time stamp_newer =
    deviceFrameStamp(ros_base_newer, steady_base, change_newer, newer);

  EXPECT_GT(stamp_newer.nanoseconds(), stamp_older.nanoseconds());
}
