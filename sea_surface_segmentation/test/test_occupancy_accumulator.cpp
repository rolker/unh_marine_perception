#include <gtest/gtest.h>

#include <cmath>

#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sea_surface_segmentation/occupancy_accumulator.hpp"
#include "sea_surface_segmentation/occupancy_buffer.hpp"
#include "sea_surface_segmentation/segments_projection.hpp"

using sea_surface_segmentation::accumulate_frame;
using sea_surface_segmentation::AccumulateParams;
using sea_surface_segmentation::OccupancyBuffer;
using sea_surface_segmentation::OccupancyParams;
using sea_surface_segmentation::project_observations_inverse;

namespace
{

// Distortion-free pinhole CameraInfo (mirrors test_segments_projection.cpp).
sensor_msgs::msg::CameraInfo make_pinhole_info(
  unsigned int width, unsigned int height, double fx, double fy)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = {
    fx, 0.0, static_cast<double>(width) / 2.0,
    0.0, fy, static_cast<double>(height) / 2.0,
    0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {
    fx, 0.0, static_cast<double>(width) / 2.0, 0.0,
    0.0, fy, static_cast<double>(height) / 2.0, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return info;
}

// 16×12 @ fx=fy=10, looking straight down from 2 m. The centre pixel (8, 6)
// back-projects to world (0, 0); neighbouring cells project to neighbouring
// pixels, so a single red pixel at (row 6, col 8) marks exactly the (0, 0) cell.
image_geometry::PinholeCameraModel make_small_nadir_camera()
{
  image_geometry::PinholeCameraModel m;
  m.fromCameraInfo(make_pinhole_info(16, 12, 10.0, 10.0));
  return m;
}

// Optical (z fwd, x right, y down) → nadir target (looking straight down).
cv::Matx33d nadir_rotation()
{
  return cv::Matx33d(
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0);
}

const cv::Vec3d kNadirOrigin(0.0, 0.0, 2.0);
// res 0.25 is FP-exact (binary) and the 4.25 m buffer gives an ODD cell count,
// so world (0,0) is a true cell *centre* — not a boundary. That keeps
// logOdds(0,0) unambiguous across a move() (a boundary query can resolve to the
// neighbour cell after the origin shifts).
constexpr double kRes = 0.25;
constexpr double kBufferSize = 4.25;  // 4.25 / 0.25 = 17 cells (odd → centre cell at 0,0)
constexpr double kHalfExtent = 1.5;
constexpr double kLogOddsTol = 1e-5;  // float storage in OccupancyBuffer

cv::Mat all_water(int w = 16, int h = 12)
{
  return cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 200, 0));  // green-dominant = water
}

cv::Mat all_sky(int w = 16, int h = 12)
{
  return cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 200));  // blue-dominant = sky (skipped)
}

OccupancyBuffer make_buffer(const OccupancyParams & params)
{
  return OccupancyBuffer(kBufferSize, kBufferSize, kRes, grid_map::Position(0.0, 0.0), params);
}

AccumulateParams acc_params()
{
  return AccumulateParams{/*max_range=*/100.0, kRes, kHalfExtent};
}

}  // namespace

// A single all-water frame applies a miss to every observed water cell and
// returns the number of observations applied — and that count matches a direct
// project_observations_inverse() call (locks the driver's forwarding).
TEST(AccumulateFrame, AllWaterAppliesMissesAndReturnsObservationCount)
{
  OccupancyParams params;
  auto buffer = make_buffer(params);
  const cv::Mat mask = all_water();
  const auto model = make_small_nadir_camera();
  const auto p = acc_params();

  const std::size_t applied = accumulate_frame(
    buffer, mask, model, kNadirOrigin, nadir_rotation(),
    /*boat_x=*/0.0, /*boat_y=*/0.0, /*stamp_s=*/100.0, p);

  const auto direct = project_observations_inverse(
    mask, model, kNadirOrigin, nadir_rotation(),
    p.max_range, 0.0, 0.0, p.res, p.half_extent, p.plane_z, p.min_grazing_angle_deg);

  EXPECT_GT(applied, 0u) << "in-footprint water cells should be observed";
  EXPECT_EQ(applied, direct.size()) << "driver must apply exactly the projected observations";
  // The centre pixel (8,6) covers world (0,0); under all-water it is a miss.
  EXPECT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)), params.miss_log_odds, kLogOddsTol);
}

// A single obstacle contact pixel at image centre marks the world-(0,0) cell as
// a hit (the contact), while neighbouring water cells are missed.
TEST(AccumulateFrame, ContactPixelMarksHitCell)
{
  OccupancyParams params;
  auto buffer = make_buffer(params);
  cv::Mat mask = all_water();
  mask.at<cv::Vec3b>(6, 8) = cv::Vec3b(200, 0, 0);  // R-dominant contact (row7=water below)
  const auto model = make_small_nadir_camera();

  accumulate_frame(
    buffer, mask, model, kNadirOrigin, nadir_rotation(),
    0.0, 0.0, 100.0, acc_params());

  // (0,0) ← pixel (8,6) = the contact → one hit.
  EXPECT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)), params.hit_log_odds, kLogOddsTol);
  // A neighbour cell (0.25,0) ← a water pixel → miss (negative).
  EXPECT_LT(buffer.logOdds(grid_map::Position(0.25, 0.0)), 0.0);
}

// move() preserves accumulated evidence at its WORLD position across a window
// shift. Frame 1 hits world (0,0); frame 2 re-centres the window on a shifted
// boat with an all-sky mask (no new observations) and the SAME stamp (dt=0, so
// decay is a no-op) — the (0,0) cell must still read its frame-1 hit value.
TEST(AccumulateFrame, MovePreservesOverlappingEvidence)
{
  OccupancyParams params;
  auto buffer = make_buffer(params);
  const auto model = make_small_nadir_camera();

  cv::Mat contact = all_water();
  contact.at<cv::Vec3b>(6, 8) = cv::Vec3b(200, 0, 0);
  accumulate_frame(buffer, contact, model, kNadirOrigin, nadir_rotation(),
    /*boat_x=*/0.0, /*boat_y=*/0.0, /*stamp_s=*/100.0, acc_params());
  ASSERT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)), params.hit_log_odds, kLogOddsTol);

  // Shift the window by 0.5 m (2 cells, parity preserved so (0,0) stays a cell
  // centre); (0,0) stays inside the 4.25 m window. Sky mask → zero observations,
  // same stamp → no decay.
  const std::size_t applied = accumulate_frame(
    buffer, all_sky(), model, kNadirOrigin, nadir_rotation(),
    /*boat_x=*/0.5, /*boat_y=*/0.0, /*stamp_s=*/100.0, acc_params());

  EXPECT_EQ(applied, 0u) << "sky frame contributes no observations";
  EXPECT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)), params.hit_log_odds, kLogOddsTol)
    << "move() must preserve the prior hit at its world position";
}

// decay() between frames attenuates prior evidence by the elapsed-time factor.
// Frame 1 hits world (0,0) at t=100; frame 2 (all-sky, no new obs) at t = 100 +
// one half-life must halve the stored log-odds. This locks the move→decay→ingest
// ordering inside accumulate_frame (decay is applied before ingest, to existing
// evidence, not to this frame's fresh observations).
TEST(AccumulateFrame, DecayAttenuatesPriorHitBetweenFrames)
{
  OccupancyParams params;
  params.decay_half_life_s = 10.0;
  auto buffer = make_buffer(params);
  const auto model = make_small_nadir_camera();

  cv::Mat contact = all_water();
  contact.at<cv::Vec3b>(6, 8) = cv::Vec3b(200, 0, 0);
  accumulate_frame(buffer, contact, model, kNadirOrigin, nadir_rotation(),
    0.0, 0.0, /*stamp_s=*/100.0, acc_params());
  ASSERT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)), params.hit_log_odds, kLogOddsTol);

  // One half-life later, with no new observation at (0,0): value halves.
  accumulate_frame(buffer, all_sky(), model, kNadirOrigin, nadir_rotation(),
    0.0, 0.0, /*stamp_s=*/110.0, acc_params());

  EXPECT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)),
    params.hit_log_odds * 0.5, 1e-3)
    << "decay() must attenuate prior evidence by the half-life factor before ingest";
}

// Locks the decay-BEFORE-ingest ordering, which the test above does not (its
// frame 2 ingests nothing). move() and decay() commute on the overlap (decay is
// a uniform scalar multiply, move only relabels world positions), so the
// behaviourally-significant order is that this frame's fresh hit is applied
// AFTER decay, not decayed itself. Frame 1 hits (0,0)=0.85 at t=100; frame 2
// re-hits (0,0) one half-life later. Correct order: decay 0.85→0.425, then add a
// fresh 0.85 → 1.275. If decay ran after ingest it would be (0.85+0.85)*0.5 =
// 0.85 — so this value distinguishes the two orderings.
TEST(AccumulateFrame, DecayPrecedesFreshIngest)
{
  OccupancyParams params;
  params.decay_half_life_s = 10.0;
  auto buffer = make_buffer(params);
  const auto model = make_small_nadir_camera();

  cv::Mat contact = all_water();
  contact.at<cv::Vec3b>(6, 8) = cv::Vec3b(200, 0, 0);
  accumulate_frame(buffer, contact, model, kNadirOrigin, nadir_rotation(),
    0.0, 0.0, /*stamp_s=*/100.0, acc_params());
  ASSERT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)), params.hit_log_odds, kLogOddsTol);

  // Same contact, one half-life later: decay the prior hit, THEN add the fresh one.
  accumulate_frame(buffer, contact, model, kNadirOrigin, nadir_rotation(),
    0.0, 0.0, /*stamp_s=*/110.0, acc_params());

  EXPECT_NEAR(buffer.logOdds(grid_map::Position(0.0, 0.0)),
    params.hit_log_odds * 0.5 + params.hit_log_odds, 1e-3)
    << "fresh hit must be applied after decay, not decayed with the prior evidence";
}
