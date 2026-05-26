#pragma once

// Private implementation header for the segments_to_pointcloud node.
// Lives in src/ rather than include/ because the projection helper has
// no downstream consumers — it's exercised by `src/segments_to_pointcloud.cpp`
// and by `test/test_segments_projection.cpp`, nothing else. Mirrors the
// pattern of `frame_id_resolver.hpp` and `segments_apply.hpp`.

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

// True if the rgb8 pixel passes the "obstacle" classification used by
// the forward-OAK segmentation output — red-dominant.
inline bool is_obstacle_pixel(const cv::Vec3b & pixel)
{
  return pixel[0] > pixel[1] && pixel[0] > pixel[2];
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
// Behavior:
//   - Iterates every pixel of `mask_rgb8`. Obstacle pixels (per
//     `is_obstacle_pixel`) are back-projected through the pinhole
//     camera model into the target frame and intersected with the
//     plane `z = plane_z`.
//   - Pixels whose ray points away from the plane (or is parallel to
//     it) are silently dropped.
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
  double plane_z = 0.0)
{
  std::vector<ProjectedPoint> points;

  for (int row = 0; row < mask_rgb8.rows; ++row) {
    for (int col = 0; col < mask_rgb8.cols; ++col) {
      const cv::Vec3b pixel_value = mask_rgb8.at<cv::Vec3b>(row, col);
      if (!is_obstacle_pixel(pixel_value)) {
        continue;
      }

      // Ray direction in the camera optical frame.
      const cv::Point2d pixel(col, row);
      const cv::Point3d ray_cam = camera_model.projectPixelTo3dRay(pixel);
      const cv::Vec3d ray_cam_vec(ray_cam.x, ray_cam.y, ray_cam.z);

      // Same direction in the target frame.
      const cv::Vec3d ray_target = rotation_cam_to_target * ray_cam_vec;

      // Parametric ray-plane intersection in target frame:
      //   P = camera_origin + u * ray_target
      //   P.z = plane_z   →   u = (plane_z - camera_origin[2]) / ray_target[2]
      // Skip rays parallel to the plane (denominator ≈ 0) and rays
      // that would intersect behind the camera (u <= 0).
      if (ray_target[2] == 0.0) {
        continue;
      }
      const double u = (plane_z - camera_origin[2]) / ray_target[2];
      if (u <= 0.0) {
        continue;
      }

      ProjectedPoint p;
      p.x = camera_origin[0] + u * ray_target[0];
      p.y = camera_origin[1] + u * ray_target[1];
      p.z = plane_z;
      p.intensity = pixel_value[0];
      points.push_back(p);
    }
  }

  return points;
}

}  // namespace sea_surface_segmentation
