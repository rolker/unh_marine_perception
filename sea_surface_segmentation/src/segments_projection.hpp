#pragma once

// Private implementation header for the segments_to_pointcloud node.
// Lives in src/ rather than include/ because the projection helper has
// no downstream consumers — it's exercised by `src/segments_to_pointcloud.cpp`
// and by `test/test_segments_projection.cpp`, nothing else. Mirrors the
// pattern of `frame_id_resolver.hpp` and `segments_apply.hpp`.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"

namespace sea_surface_segmentation {

// One projected obstacle point in the target frame (3D) with the
// red-channel intensity copied through from the mask pixel.
struct ProjectedPoint
{
  double x;
  double y;
  double z;
  std::uint8_t intensity;
};

// Per-call projection accounting. Optional output of
// `project_obstacle_pixels`; the node forwards these counters to its
// `/diagnostics` health task so a silently-degraded feed (e.g. every ray
// dropped as non-finite) is observable rather than just an empty cloud.
struct ProjectionStats
{
  std::size_t obstacle_pixels = 0;             // pixels passing is_obstacle_pixel
  std::size_t projected = 0;                   // points produced
  std::size_t dropped_nonfinite = 0;           // NaN/Inf ray, u, or point
  std::size_t dropped_behind_or_parallel = 0;  // ray parallel to / behind plane
};

// True if the rgb8 pixel passes the "obstacle" classification used by
// the forward-OAK segmentation output — red-dominant.
inline bool is_obstacle_pixel(const cv::Vec3b & pixel)
{
  return pixel[0] > pixel[1] && pixel[0] > pixel[2];
}

// Convert a quaternion (x, y, z, w) into the 3×3 rotation matrix that maps
// a direction in the camera optical frame to the target frame — i.e. the
// `rotation_cam_to_target` argument of `project_obstacle_pixels`. The input
// is normalized first so a slightly non-unit quaternion (numeric drift from
// a TF producer) still yields a proper rotation. A zero-norm or non-finite
// quaternion cannot define a rotation; we return identity so the caller
// degrades to "no rotation" instead of propagating NaN into the cloud — the
// node's TF-failure diagnostics surface the bad transform separately.
//
// For a finite, non-zero quaternion the matrix matches
// `tf2::Matrix3x3(tf2::Quaternion(x,y,z,w))` element-wise (cross-checked in
// test_segments_projection.cpp); degenerate inputs return identity as noted
// above. So swapping the node's tf2 conversion for this keeps the geometry
// identical for valid transforms while keeping this header free of a tf2
// dependency.
inline cv::Matx33d rotation_matrix_from_quaternion(
  double x, double y, double z, double w)
{
  const double norm_sq = x * x + y * y + z * z + w * w;
  if (!std::isfinite(norm_sq) || norm_sq <= 0.0) {
    return cv::Matx33d::eye();
  }
  const double s = 1.0 / std::sqrt(norm_sq);
  x *= s;
  y *= s;
  z *= s;
  w *= s;

  const double xx = x * x, yy = y * y, zz = z * z;
  const double xy = x * y, xz = x * z, yz = y * z;
  const double wx = w * x, wy = w * y, wz = w * z;

  return cv::Matx33d(
    1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
    2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
    2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy));
}

// Project obstacle pixels from a segmentation mask onto a horizontal
// plane (`z = plane_z`) expressed in some target frame, returning the
// 3D intersection points in that target frame.
//
// Inputs:
//   - `mask_rgb8`            — segmentation mask in `rgb8` encoding.
//   - `camera_model`         — must already be populated from a
//                              CameraInfo message.
//   - `camera_origin`        — optical-center position expressed in the
//                              target frame.
//   - `rotation_cam_to_target` — 3×3 rotation that maps a direction
//                              vector expressed in the camera optical
//                              frame to the same direction in the
//                              target frame. Translation is supplied
//                              separately via `camera_origin`.
//   - `plane_z`              — z-coordinate of the projection plane in
//                              the target frame. Defaults to 0.0
//                              (i.e. the target frame's xy-plane).
//
//   - `stats`                — optional out-param; when non-null, receives
//                              per-call counts (obstacle pixels seen, points
//                              produced, rays dropped). The node forwards
//                              these to `/diagnostics`. Pass nullptr to skip.
//
// Behavior:
//   - Iterates every pixel of `mask_rgb8`. Obstacle pixels (per
//     `is_obstacle_pixel`) are back-projected through the pinhole
//     camera model into the target frame and intersected with the
//     plane `z = plane_z`.
//   - Pixels whose ray points away from the plane (or is parallel to
//     it) are silently dropped.
//   - Non-finite results are dropped, not emitted. A degenerate
//     CameraInfo (e.g. zero focal length from an uncalibrated camera)
//     makes `projectPixelTo3dRay` return NaN/Inf; without this guard the
//     `== 0.0` parallel test would pass them through (NaN compares false)
//     and push NaN points into the cloud, silently corrupting a safety
//     feed. Dropped non-finite rays are counted in `stats` so the
//     condition is observable instead of invisible.
//   - The returned vector preserves the per-pixel intensity (red
//     channel value) for downstream consumers.
//
// Pure logic: no ROS, no TF, no PCL. The caller (the node) is
// responsible for the TF lookup that produces `camera_origin` and
// `rotation_cam_to_target`, and for assembling the resulting points
// into whichever ROS message type the consumer wants.
inline std::vector<ProjectedPoint> project_obstacle_pixels(
  const cv::Mat & mask_rgb8,
  const image_geometry::PinholeCameraModel & camera_model,
  const cv::Vec3d & camera_origin,
  const cv::Matx33d & rotation_cam_to_target,
  double plane_z = 0.0,
  ProjectionStats * stats = nullptr)
{
  std::vector<ProjectedPoint> points;

  for (int row = 0; row < mask_rgb8.rows; ++row) {
    for (int col = 0; col < mask_rgb8.cols; ++col) {
      const cv::Vec3b pixel_value = mask_rgb8.at<cv::Vec3b>(row, col);
      if (!is_obstacle_pixel(pixel_value)) {
        continue;
      }
      if (stats) {
        ++stats->obstacle_pixels;
      }

      // Ray direction in the camera optical frame.
      const cv::Point2d pixel(col, row);
      const cv::Point3d ray_cam = camera_model.projectPixelTo3dRay(pixel);
      const cv::Vec3d ray_cam_vec(ray_cam.x, ray_cam.y, ray_cam.z);

      // Same direction in the target frame.
      const cv::Vec3d ray_target = rotation_cam_to_target * ray_cam_vec;

      // Reject non-finite rays (e.g. from a zero-focal-length CameraInfo)
      // before any arithmetic — a NaN/Inf ray would otherwise slip past the
      // `== 0.0` parallel test and produce a NaN point.
      if (!std::isfinite(ray_target[0]) || !std::isfinite(ray_target[1]) ||
          !std::isfinite(ray_target[2]))
      {
        if (stats) {
          ++stats->dropped_nonfinite;
        }
        continue;
      }

      // Parametric ray-plane intersection in target frame:
      //   P = camera_origin + u * ray_target
      //   P.z = plane_z   →   u = (plane_z - camera_origin[2]) / ray_target[2]
      // Skip rays parallel to the plane (denominator == 0) and rays
      // that would intersect behind the camera (u <= 0).
      if (ray_target[2] == 0.0) {
        if (stats) {
          ++stats->dropped_behind_or_parallel;
        }
        continue;
      }
      const double u = (plane_z - camera_origin[2]) / ray_target[2];
      if (!std::isfinite(u) || u <= 0.0) {
        if (stats) {
          ++stats->dropped_behind_or_parallel;
        }
        continue;
      }

      ProjectedPoint p;
      p.x = camera_origin[0] + u * ray_target[0];
      p.y = camera_origin[1] + u * ray_target[1];
      p.z = plane_z;
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        if (stats) {
          ++stats->dropped_nonfinite;
        }
        continue;
      }
      p.intensity = pixel_value[0];
      points.push_back(p);
      if (stats) {
        ++stats->projected;
      }
    }
  }

  return points;
}

}  // namespace sea_surface_segmentation
