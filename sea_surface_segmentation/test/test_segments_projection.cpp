#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "segments_projection.hpp"

using sea_surface_segmentation::is_obstacle_pixel;
using sea_surface_segmentation::is_waterline_contact_pixel;
using sea_surface_segmentation::project_observations;
using sea_surface_segmentation::project_observations_inverse;
using sea_surface_segmentation::OccupancyObservation;
using sea_surface_segmentation::project_obstacle_pixels;
using sea_surface_segmentation::ProjectedPoint;
using sea_surface_segmentation::ProjectionStats;
using sea_surface_segmentation::rotation_matrix_from_quaternion;

namespace
{

// Build a simple distortion-free pinhole CameraInfo: 640x480, fx=fy=320
// (≈ 90° horizontal FOV), principal point at the image center.
sensor_msgs::msg::CameraInfo make_pinhole_info(
  unsigned int width = 640, unsigned int height = 480,
  double fx = 320.0, double fy = 320.0)
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

image_geometry::PinholeCameraModel make_camera_model()
{
  image_geometry::PinholeCameraModel m;
  m.fromCameraInfo(make_pinhole_info());
  return m;
}

// Rotation that maps the camera optical frame (z forward, x right,
// y down) into a "nadir" target frame: target_z = -optical_z (down),
// target_x = optical_x (right), target_y = -optical_y (i.e., world
// +y is camera -y / up). Right-handed, det = +1.
cv::Matx33d nadir_rotation()
{
  return cv::Matx33d(
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0);
}

// Rotation pitching the optical frame down by `pitch_deg`, with no
// roll or yaw. Maps optical_z (forward) to (cos θ, 0, -sin θ) in the
// target frame (x forward, z up). Optical_x stays target_-y (camera
// right = world right). Optical_y maps as the right-handed completion.
cv::Matx33d pitched_rotation(double pitch_deg)
{
  const double c = std::cos(pitch_deg * M_PI / 180.0);
  const double s = std::sin(pitch_deg * M_PI / 180.0);
  // Columns are (optical_x_in_target, optical_y_in_target, optical_z_in_target).
  // Optical_x = right → target_-y → (0, -1, 0).
  // Optical_z = forward → pitched down → (c, 0, -s).
  // Optical_y = down → cross(optical_z_in_t, optical_x_in_t) per right-hand =
  //                  cross((c,0,-s),(0,-1,0)) = (0*0-(-s)*(-1), (-s)*0-c*0, c*(-1)-0*0) = (-s,0,-c).
  return cv::Matx33d(
    0.0, -s, c,
    -1.0, 0.0, 0.0,
    0.0, -c, -s);
}

// Build a single-pixel red dot at (col, row) on a black mask of the
// given size. Black is non-obstacle under the R-dominant heuristic
// (0 > 0 is false).
cv::Mat make_red_dot_mask(int width, int height, int col, int row)
{
  cv::Mat mask(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  mask.at<cv::Vec3b>(row, col) = cv::Vec3b(200, 50, 50);  // R-dominant
  return mask;
}

constexpr double kRangeTolerance = 0.05;  // 5 cm — projection should be tight.
constexpr double kLateralTolerance = 0.05;

}  // namespace

// is_obstacle_pixel: red-dominant in, neutral/green/blue out.
TEST(IsObstaclePixel, RedDominantTrue)
{
  EXPECT_TRUE(is_obstacle_pixel(cv::Vec3b(200, 0, 0)));
  EXPECT_TRUE(is_obstacle_pixel(cv::Vec3b(200, 50, 50)));
}

TEST(IsObstaclePixel, NonRedDominantFalse)
{
  EXPECT_FALSE(is_obstacle_pixel(cv::Vec3b(0, 0, 0)));         // black
  EXPECT_FALSE(is_obstacle_pixel(cv::Vec3b(0, 200, 0)));       // pure green
  EXPECT_FALSE(is_obstacle_pixel(cv::Vec3b(0, 0, 200)));       // pure blue
  EXPECT_FALSE(is_obstacle_pixel(cv::Vec3b(200, 200, 200)));   // gray (ties don't dominate)
  EXPECT_FALSE(is_obstacle_pixel(cv::Vec3b(200, 200, 0)));     // red-tied-with-green
}

// is_waterline_contact_pixel: only the lowest obstacle pixel in a column
// (the one with water directly below) is the waterline contact; obstacle
// pixels stacked above it are the object's body and must NOT count as
// contacts (their cells become the false "shadow").
TEST(WaterlineContactPixel, OnlyLowestObstaclePixelInColumnIsContact)
{
  // 8x8 mask, all water (green). Column 4 holds a 3-px-tall obstacle in
  // rows 2,3,4 with water below it (rows 5..7).
  cv::Mat mask(8, 8, CV_8UC3, cv::Scalar(0, 200, 0));  // BGR-agnostic: G-dominant = water
  for (int row = 2; row <= 4; ++row) {
    mask.at<cv::Vec3b>(row, 4) = cv::Vec3b(200, 0, 0);  // R-dominant = obstacle
  }

  EXPECT_TRUE(is_waterline_contact_pixel(mask, 4, 4))   // base: water (row 5) below
    << "lowest obstacle pixel in the column is the waterline contact";
  EXPECT_FALSE(is_waterline_contact_pixel(mask, 3, 4))  // obstacle below (row 4)
    << "body pixel with obstacle below is not a contact";
  EXPECT_FALSE(is_waterline_contact_pixel(mask, 2, 4))  // obstacle below (row 3)
    << "top body pixel is not a contact";
}

TEST(WaterlineContactPixel, WaterPixelIsNeverContact)
{
  cv::Mat mask(8, 8, CV_8UC3, cv::Scalar(0, 200, 0));
  EXPECT_FALSE(is_waterline_contact_pixel(mask, 3, 3));
}

TEST(WaterlineContactPixel, ObstacleOnBottomRowIsContact)
{
  // Nothing below the bottom row to disqualify it → nearest possible return.
  cv::Mat mask(8, 8, CV_8UC3, cv::Scalar(0, 200, 0));
  mask.at<cv::Vec3b>(7, 4) = cv::Vec3b(200, 0, 0);
  EXPECT_TRUE(is_waterline_contact_pixel(mask, 7, 4));
}

TEST(WaterlineContactPixel, IsolatedObstaclePixelWithWaterBelowIsContact)
{
  cv::Mat mask(8, 8, CV_8UC3, cv::Scalar(0, 200, 0));
  mask.at<cv::Vec3b>(3, 4) = cv::Vec3b(200, 0, 0);  // single px, water below
  EXPECT_TRUE(is_waterline_contact_pixel(mask, 3, 4));
}

// Center pixel under a nadir camera 1 m above the z=0 plane projects
// to the target-frame origin.
TEST(ProjectObstaclePixels, NadirCenterPixelHitsOrigin)
{
  const auto model = make_camera_model();
  const cv::Mat mask = make_red_dot_mask(640, 480, 320, 240);

  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);
  const auto rotation = nadir_rotation();

  const auto points = project_obstacle_pixels(mask, model, camera_origin, rotation);

  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].x, 0.0, kLateralTolerance);
  EXPECT_NEAR(points[0].y, 0.0, kLateralTolerance);
  EXPECT_NEAR(points[0].z, 0.0, 1e-9);
  EXPECT_EQ(points[0].intensity, 200);
}

// A pixel to the right of the principal point under a nadir camera
// projects to a point in the camera's right direction. With our
// nadir_rotation, optical_+x maps to target_+x, so the projected
// point should be at +x in the target frame.
TEST(ProjectObstaclePixels, NadirOffsetPixelOffsetsLaterally)
{
  const auto model = make_camera_model();
  // 320 pixels right of center at fx=320 → 45° → x = h*tan(45°) = h.
  const cv::Mat mask = make_red_dot_mask(640, 480, 640 - 1, 240);

  const cv::Vec3d camera_origin(0.0, 0.0, 2.0);
  const auto rotation = nadir_rotation();

  const auto points = project_obstacle_pixels(mask, model, camera_origin, rotation);

  ASSERT_EQ(points.size(), 1u);
  // 319.5 / 320 ≈ 0.998 — multiplied by height 2 m → ≈ 1.997 m.
  EXPECT_NEAR(points[0].x, 2.0 * (319.5 / 320.0), kRangeTolerance);
  EXPECT_NEAR(points[0].y, 0.0, kLateralTolerance);
  EXPECT_NEAR(points[0].z, 0.0, 1e-9);
}

// A camera mounted at 1 m above the plane and pitched 30° down. The
// principal-point pixel (image center) corresponds to optical +z,
// which after the pitched rotation points (cos 30°, 0, -sin 30°) in
// target. Camera origin at (0,0,1). Plane intersection at u =
// (0 - 1) / -sin 30° = 2. Point = (cos 30° * 2, 0, 0) ≈ (1.732, 0, 0).
TEST(ProjectObstaclePixels, PitchedCameraCenterPixelHitsExpectedRange)
{
  const auto model = make_camera_model();
  const cv::Mat mask = make_red_dot_mask(640, 480, 320, 240);

  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);
  const auto rotation = pitched_rotation(30.0);

  const auto points = project_obstacle_pixels(mask, model, camera_origin, rotation);

  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].x, std::cos(M_PI / 6.0) / std::sin(M_PI / 6.0), kRangeTolerance);
  EXPECT_NEAR(points[0].y, 0.0, kLateralTolerance);
  EXPECT_NEAR(points[0].z, 0.0, 1e-9);
}

// With a pitched camera, a pixel BELOW the principal point (which is
// in optical +y, i.e. "more down" in the image) should project to a
// nearer point than the principal-point pixel does.
TEST(ProjectObstaclePixels, PitchedCameraImageBottomIsCloser)
{
  const auto model = make_camera_model();
  const cv::Mat mask_center = make_red_dot_mask(640, 480, 320, 240);
  const cv::Mat mask_bottom = make_red_dot_mask(640, 480, 320, 380);  // 140 px below center

  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);
  const auto rotation = pitched_rotation(30.0);

  const auto center_points = project_obstacle_pixels(mask_center, model, camera_origin, rotation);
  const auto bottom_points = project_obstacle_pixels(mask_bottom, model, camera_origin, rotation);

  ASSERT_EQ(center_points.size(), 1u);
  ASSERT_EQ(bottom_points.size(), 1u);

  EXPECT_LT(bottom_points[0].x, center_points[0].x)
    << "image-bottom pixel should project to a closer range than image-center for a "
       "forward-pitched camera";
  EXPECT_GT(bottom_points[0].x, 0.0)
    << "image-bottom pixel should still project in front of the camera";
}

// Pixels whose back-projected rays go away from the plane (e.g., a
// pixel ABOVE the principal point on a forward-pitched camera, when
// the pitch is shallow enough that the upward ray never reaches z=0)
// must be silently dropped — not produce spurious behind-camera
// points.
TEST(ProjectObstaclePixels, RaysAwayFromPlaneAreDropped)
{
  const auto model = make_camera_model();
  // Pixel well ABOVE the principal point. At 5° pitch the principal-
  // point ray is barely below horizontal; a pixel 100 px above center
  // is well above horizontal in the target frame → ray goes up.
  const cv::Mat mask = make_red_dot_mask(640, 480, 320, 140);

  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);
  const auto rotation = pitched_rotation(5.0);

  const auto points = project_obstacle_pixels(mask, model, camera_origin, rotation);

  EXPECT_TRUE(points.empty())
    << "ray pointing upward (away from plane below) must not produce a point";
}

// All non-obstacle pixels are silently dropped — no points produced.
TEST(ProjectObstaclePixels, NonObstaclePixelsProduceNoOutput)
{
  const auto model = make_camera_model();

  cv::Mat mask(480, 640, CV_8UC3, cv::Scalar(0, 200, 0));  // green everywhere

  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);
  const auto rotation = nadir_rotation();

  const auto points = project_obstacle_pixels(mask, model, camera_origin, rotation);
  EXPECT_TRUE(points.empty());
}

// projection_plane_z relocates the intersection plane in the target
// frame. With a nadir camera at z=1.5 and plane_z=0.5, the center
// pixel still projects to the camera's projected ground-track origin
// — but at z=0.5, not 0.0.
TEST(ProjectObstaclePixels, PlaneZParameterRelocatesIntersection)
{
  const auto model = make_camera_model();
  const cv::Mat mask = make_red_dot_mask(640, 480, 320, 240);

  const cv::Vec3d camera_origin(0.0, 0.0, 1.5);
  const auto rotation = nadir_rotation();

  const auto points = project_obstacle_pixels(
    mask, model, camera_origin, rotation, /*plane_z=*/0.5);

  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].x, 0.0, kLateralTolerance);
  EXPECT_NEAR(points[0].y, 0.0, kLateralTolerance);
  EXPECT_NEAR(points[0].z, 0.5, 1e-9);
}

// Honest rotation-application test. The helper just applies whatever
// `rotation_cam_to_target` the caller hands it — the "roll being
// rotated out" property the reflex-safety pipeline relies on is owned
// by `mru_transform`'s definition of `base_link_level` (heading-only,
// roll+pitch zeroed), not by this header.
//
// What we *can* unit-test here: when a roll appears inside the
// supplied rotation matrix, the projected point shifts laterally by
// the predicted amount — i.e. the helper honors the rotation rather
// than ignoring it.
TEST(ProjectObstaclePixels, RotationHonoredEvenWhenItContainsRoll)
{
  const auto model = make_camera_model();

  // Pixel 100 px below the principal point.
  const cv::Mat mask = make_red_dot_mask(640, 480, 320, 340);
  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);

  // Reference: pure 30° pitch, no roll → pixel-below-center
  // projects forward with zero lateral offset.
  const auto unrolled = project_obstacle_pixels(
    mask, model, camera_origin, pitched_rotation(30.0));
  ASSERT_EQ(unrolled.size(), 1u);
  EXPECT_NEAR(unrolled[0].y, 0.0, kLateralTolerance);
  EXPECT_GT(unrolled[0].x, 0.0);

  // Now compose a roll into the rotation by 15° about optical z
  // BEFORE the pitch. This represents a target frame that does NOT
  // factor roll out (i.e. would-be-`base_link` not
  // `base_link_level`). The lateral offset should appear,
  // demonstrating the helper applied the rotation.
  const double cr = std::cos(15.0 * M_PI / 180.0);
  const double sr = std::sin(15.0 * M_PI / 180.0);
  const cv::Matx33d add_roll(
    cr, -sr, 0.0,
    sr, cr, 0.0,
    0.0, 0.0, 1.0);
  const cv::Matx33d rolled_rotation = pitched_rotation(30.0) * add_roll;

  const auto rolled = project_obstacle_pixels(
    mask, model, camera_origin, rolled_rotation);

  ASSERT_EQ(rolled.size(), 1u);
  EXPECT_NE(rolled[0].y, unrolled[0].y)
    << "roll inside the rotation matrix must produce a different lateral offset";
  // The reference case is the world we want at runtime: a
  // heading-only target gives zero lateral offset for an
  // image-centerline pixel. mru_transform's job is to make sure the
  // rotation we feed this helper is the unrolled one.
}

// The node feeds the camera-to-target rotation through
// rotation_matrix_from_quaternion (it replaced the in-node
// tf2::Quaternion → tf2::Matrix3x3 → cv::Matx33d copy). This pins the
// helper to tf2's matrix convention element-wise, so the production
// conversion path can't drift (sign / transpose / element-order) without
// this test failing — the high-risk math the unit tests above can't cover
// because they hand-build their rotations.
TEST(RotationFromQuaternion, MatchesTf2Matrix3x3)
{
  const std::vector<std::array<double, 4>> quats = {
    {0.0, 0.0, 0.0, 1.0},                          // identity
    {0.5, 0.5, 0.5, 0.5},                          // 120° about (1,1,1)
    {0.0, 0.7071067811865476, 0.0, 0.7071067811865476},   // 90° about y
    {0.18257, 0.36515, 0.54772, 0.73030},          // arbitrary, already ~unit
    {-0.2, 0.4, -0.1, 0.88},                        // arbitrary, non-axis
  };

  for (const auto & q : quats) {
    tf2::Quaternion tq(q[0], q[1], q[2], q[3]);
    tq.normalize();
    const tf2::Matrix3x3 expected(tq);
    const cv::Matx33d actual = rotation_matrix_from_quaternion(q[0], q[1], q[2], q[3]);

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        EXPECT_NEAR(actual(i, j), expected[i][j], 1e-12)
          << "mismatch at (" << i << ", " << j << ") for quaternion "
          << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3];
      }
    }
  }
}

// A non-unit quaternion (numeric drift from a TF producer) must yield the
// same rotation as its normalized form, not a scaled/garbage matrix.
TEST(RotationFromQuaternion, NormalizesNonUnitInput)
{
  const cv::Matx33d unit = rotation_matrix_from_quaternion(0.5, 0.5, 0.5, 0.5);
  const cv::Matx33d scaled = rotation_matrix_from_quaternion(1.5, 1.5, 1.5, 1.5);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(unit(i, j), scaled(i, j), 1e-12);
    }
  }
}

// A zero-norm (or non-finite) quaternion can't define a rotation; the
// helper returns identity rather than propagating NaN into the cloud.
TEST(RotationFromQuaternion, DegenerateQuaternionReturnsIdentity)
{
  const cv::Matx33d r = rotation_matrix_from_quaternion(0.0, 0.0, 0.0, 0.0);
  const cv::Matx33d eye = cv::Matx33d::eye();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(r(i, j), eye(i, j), 1e-12);
    }
  }
}

// A degenerate CameraInfo (zero focal length — an uncalibrated-camera
// placeholder) makes projectPixelTo3dRay produce non-finite rays. The
// helper must drop them (counted in stats), not push NaN points into the
// cloud — the silent-corruption mode that matters for a safety feed.
TEST(ProjectObstaclePixels, DegenerateCameraInfoDropsNonFinite)
{
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(make_pinhole_info(640, 480, /*fx=*/0.0, /*fy=*/0.0));
  const cv::Mat mask = make_red_dot_mask(640, 480, 320, 240);

  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);
  const auto rotation = nadir_rotation();

  ProjectionStats stats;
  const auto points = project_obstacle_pixels(
    mask, model, camera_origin, rotation, /*plane_z=*/0.0, &stats);

  EXPECT_TRUE(points.empty()) << "non-finite projections must not be emitted";
  EXPECT_EQ(stats.projected, 0u);
  EXPECT_EQ(stats.obstacle_pixels, 1u);
  EXPECT_GE(stats.dropped_nonfinite, 1u);
}

// The optional stats out-param accounts for obstacle pixels seen and
// points produced — the counters the node forwards to /diagnostics.
TEST(ProjectObstaclePixels, StatsAccountForPixelsAndProjections)
{
  const auto model = make_camera_model();
  const cv::Mat mask = make_red_dot_mask(640, 480, 320, 240);

  const cv::Vec3d camera_origin(0.0, 0.0, 1.0);
  ProjectionStats stats;
  const auto points = project_obstacle_pixels(
    mask, model, camera_origin, nadir_rotation(), /*plane_z=*/0.0, &stats);

  ASSERT_EQ(points.size(), 1u);
  EXPECT_EQ(stats.obstacle_pixels, 1u);
  EXPECT_EQ(stats.projected, 1u);
  EXPECT_EQ(stats.dropped_nonfinite, 0u);
}

// ---- project_observations: the shared per-frame logic (waterline-contact +
//      occlusion + water-as-free) used by both the layer and the bag utility. ----

namespace
{
// Small nadir camera (16×12 @ fx=fy=10, 2 m up looking straight down) so every
// pixel projects to a valid ground point — lets the classification counts be
// asserted exactly without geometry dropping pixels.
image_geometry::PinholeCameraModel make_small_nadir_camera()
{
  image_geometry::PinholeCameraModel m;
  m.fromCameraInfo(make_pinhole_info(16, 12, 10.0, 10.0));
  return m;
}
const cv::Vec3d kNadirOrigin(0.0, 0.0, 2.0);

std::pair<int, int> count_obs(const std::vector<OccupancyObservation> & obs)
{
  int obstacle = 0, free = 0;
  for (const auto & o : obs) { (o.obstacle ? obstacle : free)++; }
  return {obstacle, free};
}
}  // namespace

// All-water mask → every pixel is a free observation, no obstacles.
TEST(ProjectObservations, AllWaterIsAllFree)
{
  cv::Mat mask(12, 16, CV_8UC3, cv::Scalar(0, 200, 0));  // green = water
  const auto obs = project_observations(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(), /*max_range=*/100.0);
  auto [obstacle, free] = count_obs(obs);
  EXPECT_EQ(obstacle, 0);
  EXPECT_EQ(free, 16 * 12);
}

// A 3-pixel-tall obstacle in one column (rows 2,3,4, water below) yields exactly
// ONE obstacle observation (the waterline contact, row 4); the two body pixels
// are occluded and skipped (not free, not obstacle); all water → free. This is
// the shadow/occlusion behavior.
TEST(ProjectObservations, TallObstacleYieldsOneContactNotBody)
{
  cv::Mat mask(12, 16, CV_8UC3, cv::Scalar(0, 200, 0));
  for (int row = 2; row <= 4; ++row) {
    mask.at<cv::Vec3b>(row, 8) = cv::Vec3b(200, 0, 0);  // red = obstacle
  }
  const auto obs = project_observations(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(), /*max_range=*/100.0);
  auto [obstacle, free] = count_obs(obs);
  EXPECT_EQ(obstacle, 1) << "only the waterline contact, not each body pixel";
  EXPECT_EQ(free, 16 * 12 - 3) << "all water pixels free; 3 obstacle pixels are not free";
  EXPECT_EQ(static_cast<int>(obs.size()), 16 * 12 - 2) << "2 body pixels skipped entirely";
}

// An isolated obstacle pixel with water directly below is a contact → 1 obstacle.
TEST(ProjectObservations, IsolatedContactIsObstacle)
{
  cv::Mat mask(12, 16, CV_8UC3, cv::Scalar(0, 200, 0));
  mask.at<cv::Vec3b>(6, 8) = cv::Vec3b(200, 0, 0);  // row 7 below is water
  const auto obs = project_observations(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(), /*max_range=*/100.0);
  auto [obstacle, free] = count_obs(obs);
  EXPECT_EQ(obstacle, 1);
  EXPECT_EQ(free, 16 * 12 - 1);
}

// ---- project_observations_inverse: the per-cell→pixel classifier the layer uses
//      (and the offline utility shares). Cell-iteration centred on (cx,cy);
//      same contact-only marking + sky-skip semantics. ----

// All-water mask → every in-footprint cell is missed (free); no obstacles.
TEST(ProjectObservationsInverse, AllWaterAllMisses)
{
  cv::Mat mask(12, 16, CV_8UC3, cv::Scalar(0, 200, 0));  // green-dominant = water
  const auto obs = project_observations_inverse(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(),
    /*max_range=*/100.0, /*cx=*/0.0, /*cy=*/0.0, /*res=*/0.2, /*half_extent=*/1.5);
  auto [obstacle, free] = count_obs(obs);
  EXPECT_EQ(obstacle, 0);
  EXPECT_GT(free, 0) << "every in-footprint cell should be classified as water-miss";
}

// All-sky mask → every cell projects to a sky pixel → SKIP (no obs), not miss.
// The sky-as-miss bug would over-clear cells; sky-skip preserves them.
TEST(ProjectObservationsInverse, AllSkyAllSkippedNotCleared)
{
  cv::Mat mask(12, 16, CV_8UC3, cv::Scalar(0, 0, 200));  // blue-dominant = sky
  const auto obs = project_observations_inverse(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(),
    /*max_range=*/100.0, /*cx=*/0.0, /*cy=*/0.0, /*res=*/0.2, /*half_extent=*/1.5);
  EXPECT_TRUE(obs.empty()) << "sky-projecting cells must be skipped, not marked free";
}

// A column with a 2-px-tall obstacle above water: only cells projecting to the
// contact row (lowest obstacle pixel) produce a hit; cells projecting to the
// body pixel above are skipped (occluded); water cells around → miss.
TEST(ProjectObservationsInverse, ContactCellHitsBodyCellsSkipped)
{
  cv::Mat mask(12, 16, CV_8UC3, cv::Scalar(0, 200, 0));  // water everywhere
  // 2-tall obstacle in column 8: rows 5 (body) and 6 (contact), water row 7+.
  mask.at<cv::Vec3b>(5, 8) = cv::Vec3b(200, 0, 0);
  mask.at<cv::Vec3b>(6, 8) = cv::Vec3b(200, 0, 0);
  const auto obs = project_observations_inverse(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(),
    /*max_range=*/100.0, /*cx=*/0.0, /*cy=*/0.0, /*res=*/0.2, /*half_extent=*/1.5);
  auto [obstacle, free] = count_obs(obs);
  EXPECT_GT(obstacle, 0) << "the contact cell must produce at least one hit";
  EXPECT_GT(free, 0) << "water cells must still be missed";
  // The body pixel above the contact is occluded; no cell should be flagged as
  // obstacle there. Verify by ensuring obstacle count is bounded — the contact
  // is one pixel, so only cells projecting to that one pixel become hits.
  EXPECT_LE(obstacle, 4) << "only the contact-pixel's cell footprint is hit, not the body's";
}

// Cells beyond max_range (even when in-image) are dropped by the Euclidean
// range gate. With camera at z=2 and max_range=2.05, only the cells directly
// under (slant distance ≈ 2.0–2.05 m) survive; an iteration covering several
// metres horizontally should produce far fewer observations than the same
// iteration with a generous range cap.
TEST(ProjectObservationsInverse, MaxRangeDropsFarCells)
{
  cv::Mat mask(12, 16, CV_8UC3, cv::Scalar(0, 200, 0));
  const auto loose = project_observations_inverse(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(),
    /*max_range=*/100.0, 0.0, 0.0, 0.2, 1.5);
  const auto tight = project_observations_inverse(
    mask, make_small_nadir_camera(), kNadirOrigin, nadir_rotation(),
    /*max_range=*/2.05, 0.0, 0.0, 0.2, 1.5);
  EXPECT_LT(tight.size(), loose.size())
    << "tight max_range must drop far cells the loose range admits";
  EXPECT_GT(tight.size(), 0u) << "near-overhead cells should still pass the tight gate";
}
