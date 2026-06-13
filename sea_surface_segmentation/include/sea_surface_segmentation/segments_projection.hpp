#pragma once

// Exported core header (part of the package's public include/ API). The
// header-only projection helpers are reused in-package by
// `src/segments_to_pointcloud.cpp` and `src/sea_surface_layer.cpp`, by the
// `bag_to_costmap_video` tool via `occupancy_accumulator.hpp`, and out-of-package
// by offline tools (e.g. the `sea_surface_tuner` in `marine_perception_tools`),
// so they all compute the costmap with the real code. See
// rolker/unh_marine_perception#23.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"

namespace sea_surface_segmentation {

// Per-observation log-odds increment for a pixel with red value `R` (0..255 =
// 255·P(obstacle) from the per-pixel softmax). A prior-shifted, capped logit:
//
//   logit_R     = ln((R + eps) / (255 - R + eps))   — observed evidence in log-odds
//   logit_prior = ln(prob_min / (1 - prob_min))     — the decision prior
//   d           = logit_R - logit_prior             — zero-crossing at P(obs)==prob_min
//
// Shifting by the prior puts the zero-crossing at `obstacle_prob_min` rather than
// 0.5, so a buoy where water is marginally more likely (e.g. P_obs=0.47 with
// prob_min=0.35) still contributes POSITIVE evidence. `max_step` caps any single
// frame for flicker rejection (and preserves ~2-frame-to-lethal at defaults).
//
// Defensive: this is an exported utility also called by offline tools/tuners
// (rolker/unh_marine_perception#23) that don't run the layer's parameter
// validation. Out-of-range inputs return 0.0 (contribute no evidence) rather
// than feeding NaN/Inf — or tripping std::clamp's lo<=hi precondition — into the
// occupancy buffer. The live layer still validates these at init + on set.
inline double pixel_log_odds(int R, double obstacle_prob_min, double max_step)
{
  if (R < 0) { R = 0; } else if (R > 255) { R = 255; }
  if (!(obstacle_prob_min > 0.0 && obstacle_prob_min < 1.0) ||
    !std::isfinite(max_step) || max_step <= 0.0)
  {
    return 0.0;  // invalid params — no opinion rather than NaN/Inf/UB
  }
  const double eps = 0.5;
  const double logit_R     = std::log((R + eps) / (255.0 - R + eps));
  const double logit_prior = std::log(obstacle_prob_min / (1.0 - obstacle_prob_min));
  double d = logit_R - logit_prior;          // zero-crossing at P(obs)==obstacle_prob_min
  return std::clamp(d, -max_step, max_step);
}

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
  std::size_t dropped_low_confidence = 0;      // P(obstacle) below obstacle_prob_min
};

// True if the rgb8 pixel passes the "obstacle" classification used by
// the forward-OAK segmentation output — red-dominant.
inline bool is_obstacle_pixel(const cv::Vec3b & pixel)
{
  return pixel[0] > pixel[1] && pixel[0] > pixel[2];
}

// True if an obstacle pixel is a "waterline contact" — the point where an
// obstacle meets the water surface — rather than part of the obstacle's body
// sticking up out of the water.
//
// An obstacle pixel is a contact iff the pixel directly below it is water
// (non-obstacle), or it sits on the bottom image row (nothing below to
// disqualify it — the nearest possible return for that bearing). Pixels with
// more obstacle below them are the object's body.
//
// Why it matters: the costmap layer back-projects every obstacle pixel onto
// the z=0 water plane. That assumption holds only at the waterline; an
// above-water body pixel back-projects far beyond the real obstacle, smearing
// a false radial "shadow" of lethal cells out toward maximum_range. Only the
// contact is a trustworthy footprint; the region the body occludes should be
// left unknown (NO_INFORMATION), not marked lethal. Callers use this to mark
// the contact lethal and skip the rest.
inline bool is_waterline_contact_pixel(const cv::Mat & mask, int row, int col)
{
  if (row < 0 || col < 0 || row >= mask.rows || col >= mask.cols) {
    return false;  // defensive: out-of-range never a contact
  }
  if (!is_obstacle_pixel(mask.at<cv::Vec3b>(row, col))) {
    return false;
  }
  if (row + 1 >= mask.rows) {
    return true;  // bottom image row: nearest possible return for this bearing
  }
  return !is_obstacle_pixel(mask.at<cv::Vec3b>(row + 1, col));
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
//   - `obstacle_prob_min`    — confidence floor. An obstacle pixel is
//                              projected only if P(obstacle) = R/(R+G+B) >=
//                              this (0.0 disables the gate; higher = stricter).
//                              Because the mask channels are a softmax x255
//                              (they sum to 255), R/(R+G+B) == R/255, i.e. the
//                              same P(obstacle) the costmap layer thresholds.
//                              Default 0.0 keeps the historical
//                              "any argmax-obstacle pixel" behavior; raising it
//                              (e.g. 0.60) rejects low-confidence returns such
//                              as calm-water reflections, bringing the reflex
//                              feed to parity with the costmap layer's own
//                              obstacle_prob_min. Placed last with a default so
//                              existing (incl. out-of-package) callers compile
//                              unchanged.
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
  ProjectionStats * stats = nullptr,
  double obstacle_prob_min = 0.0)
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

      // Confidence floor. The mask channels carry the per-class softmax
      // (R = obstacle, G = water, B = sky), so P(obstacle) = R / (R+G+B).
      // obstacle_prob_min == 0.0 disables the gate (back-compat); a positive
      // value drops low-confidence obstacle pixels (e.g. calm-water
      // reflections measured at ~0.55) before they ever enter the cloud.
      if (obstacle_prob_min > 0.0) {
        const double denom = static_cast<double>(pixel_value[0]) +
          static_cast<double>(pixel_value[1]) + static_cast<double>(pixel_value[2]);
        const double p_obstacle =
          denom > 0.0 ? static_cast<double>(pixel_value[0]) / denom : 0.0;
        if (p_obstacle < obstacle_prob_min) {
          if (stats) {
            ++stats->dropped_low_confidence;
          }
          continue;
        }
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

// A ground-plane occupancy observation produced from one segmentation pixel:
// a world (x, y) point on the plane `z = plane_z`, and whether it is an
// obstacle (a waterline contact) or free water.
struct OccupancyObservation
{
  double x;
  double y;
  bool obstacle;
  double log_odds = 0.0;  // graded per-observation evidence increment (signed)
};

// Back-project a segmentation mask into ground-plane occupancy observations for
// the rolling log-odds buffer. This is the shared per-frame logic used by BOTH
// the costmap layer and the offline bag→costmap utility, so they produce
// identical results.
//
// Per image COLUMN, the lowest obstacle pixel with water directly below is the
// waterline contact (`is_waterline_contact_pixel`) → one **obstacle**
// observation at its ground intersection. Obstacle pixels above the contact are
// the object's body: occluded, so they are skipped (not projected — this is the
// shadow fix). Water pixels → **free** observations (the camera positively sees
// water there). Rays that miss the plane (parallel / point away) or land beyond
// `max_range` are dropped (same geometry as `project_obstacle_pixels`).
//
// Pure logic: no ROS/TF. The caller supplies the camera pose in the target
// (world) frame via `camera_origin` + `rotation_cam_to_target` and applies each
// observation to the buffer (obstacle → hit, free → miss).
inline std::vector<OccupancyObservation> project_observations(
  const cv::Mat & mask_rgb8,
  const image_geometry::PinholeCameraModel & camera_model,
  const cv::Vec3d & camera_origin,
  const cv::Matx33d & rotation_cam_to_target,
  double max_range,
  double plane_z = 0.0)
{
  std::vector<OccupancyObservation> observations;

  for (int col = 0; col < mask_rgb8.cols; ++col) {
    // Lowest (nearest) waterline contact in this column, scanning up from the
    // bottom. -1 if the column has no obstacle-over-water transition.
    int contact_row = -1;
    for (int row = mask_rgb8.rows - 1; row >= 0; --row) {
      if (is_waterline_contact_pixel(mask_rgb8, row, col)) {
        contact_row = row;
        break;
      }
    }

    for (int row = 0; row < mask_rgb8.rows; ++row) {
      const cv::Vec3b pixel_value = mask_rgb8.at<cv::Vec3b>(row, col);
      bool obstacle_obs;
      if (is_obstacle_pixel(pixel_value)) {
        if (row != contact_row) {
          continue;  // occluded body above the waterline contact — skip
        }
        obstacle_obs = true;  // the waterline contact itself
      } else {
        obstacle_obs = false;  // water → free observation
      }

      // Ray → plane intersection in the target frame (cf. project_obstacle_pixels).
      const cv::Point3d ray_cam = camera_model.projectPixelTo3dRay(cv::Point2d(col, row));
      const cv::Vec3d ray_target = rotation_cam_to_target *
        cv::Vec3d(ray_cam.x, ray_cam.y, ray_cam.z);
      if (!std::isfinite(ray_target[0]) || !std::isfinite(ray_target[1]) ||
        !std::isfinite(ray_target[2]) || ray_target[2] == 0.0)
      {
        continue;
      }
      const double u = (plane_z - camera_origin[2]) / ray_target[2];
      if (!std::isfinite(u) || u <= 0.0) {
        continue;  // ray parallel to / behind the plane
      }
      const double wx = camera_origin[0] + u * ray_target[0];
      const double wy = camera_origin[1] + u * ray_target[1];
      if (!std::isfinite(wx) || !std::isfinite(wy)) {
        continue;
      }
      const double dx = wx - camera_origin[0];
      const double dy = wy - camera_origin[1];
      const double dz = plane_z - camera_origin[2];
      if (std::sqrt(dx * dx + dy * dy + dz * dz) > max_range) {
        continue;  // beyond sensor range
      }
      // Forward path is not the live costmap path; populate log_odds for
      // self-consistency only, using a neutral 0.5 prior so a red-dominant
      // pixel yields positive and water yields negative evidence.
      observations.push_back(
        {wx, wy, obstacle_obs, pixel_log_odds(pixel_value[0], 0.5, 0.85)});
    }
  }

  return observations;
}

// INVERSE (cell→pixel) ground-plane projection — sibling to `project_observations`.
// Instead of iterating image pixels and back-projecting each to one world cell, this
// iterates world cells in an axis-aligned window centred on (cx, cy) and asks
// "which pixel covers this cell?". One large/distant pixel fills EVERY cell its
// ray covers (no gaps), and iteration is naturally range-bounded to the window
// (a near-horizon pixel can't smear past the window edge).
//
// Pixel classification (rgb8 channels: R=obstacle, G=water, B=sky):
//   - obstacle pixel that is the column's waterline contact → hit
//   - obstacle pixel above the contact (occluded body) → skip (unobserved)
//   - water pixel (green-dominant) → miss (positively observed free)
//   - sky pixel (blue-dominant) or ambiguous → skip (a ground cell on z=0 sampling
//     a sky pixel is a geometric inconsistency; clearing on it would erase
//     legitimate hits from other frames / cameras)
//
// Pure logic: no ROS/TF. Caller supplies the camera pose in the target (world)
// frame via `camera_origin` + `rotation_cam_to_target`. `(cx, cy)` is the iteration
// centre in the target frame (the layer passes the camera origin's XY; the offline
// utility passes the boat-XY for boat-centred visualisation). `half_extent` is the
// iteration half-width of the square AABB; `res` is the cell pitch.
inline std::vector<OccupancyObservation> project_observations_inverse(
  const cv::Mat & mask_rgb8,
  const image_geometry::PinholeCameraModel & camera_model,
  const cv::Vec3d & camera_origin,
  const cv::Matx33d & rotation_cam_to_target,
  double max_range,
  double cx, double cy, double res, double half_extent,
  double plane_z = 0.0,
  double min_grazing_angle_deg = 0.0,
  double obstacle_prob_min = 0.5,
  double max_evidence_step = 0.85,
  bool max_pool_bins = true)
{
  // Graded obstacle gate: a pixel counts as obstacle-ish when its red channel
  // (255·P(obstacle)) implies P(obstacle) >= obstacle_prob_min. Replaces the
  // argmax `is_obstacle_pixel`/`is_waterline_contact_pixel` gate on the costmap
  // path so marginal-but-positive evidence (e.g. a buoy where water is barely
  // more likely) is not silently dropped before the graded log-odds step.
  auto is_obstacle_ish = [&](const cv::Vec3b & px) {
    return px[0] >= obstacle_prob_min * 255.0;
  };

  // Lowest (largest-row) obstacle-ish pixel per column whose pixel directly
  // below is NOT obstacle-ish (or which sits on the bottom row). Same waterline
  // contact geometry as forward, but using the graded gate.
  std::vector<int> contact_row(mask_rgb8.cols, -1);
  for (int col = 0; col < mask_rgb8.cols; ++col) {
    for (int row = mask_rgb8.rows - 1; row >= 0; --row) {
      if (!is_obstacle_ish(mask_rgb8.at<cv::Vec3b>(row, col))) {
        continue;
      }
      if (row + 1 >= mask_rgb8.rows ||
        !is_obstacle_ish(mask_rgb8.at<cv::Vec3b>(row + 1, col)))
      {
        contact_row[col] = row;
        break;
      }
    }
  }

  // Grazing-angle cutoff: reject rays where |dz| / |d| < sin(min_grazing) —
  // i.e. rays that hit the water plane at less than the configured angle from
  // horizontal. Squared form avoids the per-cell sqrt: |dz|^2 >= s^2 * |d|^2
  // (both sides non-negative, squaring preserves the inequality). A camera at
  // height h enforces max horizontal range ≈ h / tan(min_grazing): at h=1.5 m
  // and 5°, that's ~17 m; at 2°, ~43 m. Default 0° = no filter (back-compat).
  const double min_grazing_sin = std::sin(min_grazing_angle_deg * M_PI / 180.0);
  const double min_grazing_sin_sq = min_grazing_sin * min_grazing_sin;

  const cv::Matx33d rotation_target_to_cam = rotation_cam_to_target.t();
  std::vector<OccupancyObservation> observations;
  const int n = static_cast<int>(std::lround(2.0 * half_extent / res));
  for (int iy = 0; iy < n; ++iy) {
    for (int ix = 0; ix < n; ++ix) {
      const double wx = cx + (ix - n / 2) * res;
      const double wy = cy + (iy - n / 2) * res;
      const cv::Vec3d d(wx - camera_origin[0], wy - camera_origin[1], plane_z - camera_origin[2]);
      const double range_sq = d.dot(d);
      if (range_sq > max_range * max_range) { continue; }  // beyond sensor range
      if (min_grazing_sin > 0.0 && d[2] * d[2] < min_grazing_sin_sq * range_sq) {
        continue;  // ray strikes water plane too shallowly — high ground-error per pitch arc-sec
      }
      const cv::Vec3d pc = rotation_target_to_cam * d;    // cell in camera optical frame
      if (pc[2] <= 0.0) { continue; }                     // behind the camera (+z forward)
      const cv::Point2d uv = camera_model.project3dToPixel(cv::Point3d(pc[0], pc[1], pc[2]));
      if (!std::isfinite(uv.x) || !std::isfinite(uv.y)) { continue; }
      const int u = static_cast<int>(std::lround(uv.x));
      const int v = static_cast<int>(std::lround(uv.y));
      if (u < 0 || u >= mask_rgb8.cols || v < 0 || v >= mask_rgb8.rows) { continue; }
      const cv::Vec3b px = mask_rgb8.at<cv::Vec3b>(v, u);
      if (is_obstacle_ish(px)) {
        if (v == contact_row[u]) {
          observations.push_back(
            {wx, wy, true,
             pixel_log_odds(px[0], obstacle_prob_min, max_evidence_step)});  // waterline contact → +evidence
        }
        // else: above (or below an unselected) contact = body → leave unobserved
      } else if (px[1] > px[0] && px[1] > px[2]) {        // green-dominant = water
        observations.push_back(
          {wx, wy, false,
           pixel_log_odds(px[0], obstacle_prob_min, max_evidence_step)});  // observed water → -evidence
      }
      // else: sky (blue-dominant) or ambiguous → skip (no observation)
    }
  }

  // Per-column contact backstop: the cell loop above only marks cells whose
  // (rounded) projection lands exactly on contact_row[u]. At close range the
  // angular resolution dwarfs the cell-grid step — few or zero cells round to
  // the exact contact pixel, so a clearly-detected close buoy can produce no
  // marks at all. This pass back-projects each detected contact pixel directly
  // to z = plane_z and pushes one observation at the contact's true world
  // position. The mark lands AT the obstacle's footprint (not the body
  // back-projected past it), so no radial false shadow is introduced. Same
  // range / grazing gates as the cell loop.
  const double cam_z = camera_origin[2];
  for (int col = 0; col < mask_rgb8.cols; ++col) {
    const int crow = contact_row[col];
    if (crow < 0) { continue; }

    const cv::Point2d cpixel(col, crow);
    const cv::Point3d ray_cam = camera_model.projectPixelTo3dRay(cpixel);
    const cv::Vec3d ray_target =
      rotation_cam_to_target * cv::Vec3d(ray_cam.x, ray_cam.y, ray_cam.z);
    if (!std::isfinite(ray_target[2]) || ray_target[2] == 0.0) { continue; }
    const double u_param = (plane_z - cam_z) / ray_target[2];
    if (!std::isfinite(u_param) || u_param <= 0.0) { continue; }

    const double wxc = camera_origin[0] + u_param * ray_target[0];
    const double wyc = camera_origin[1] + u_param * ray_target[1];
    if (!std::isfinite(wxc) || !std::isfinite(wyc)) { continue; }

    const double dx = wxc - camera_origin[0];
    const double dy = wyc - camera_origin[1];
    const double range_sq_xy = dx * dx + dy * dy;
    if (range_sq_xy > max_range * max_range) { continue; }
    if (min_grazing_sin > 0.0) {
      const double dz = plane_z - cam_z;
      const double range_sq_full = range_sq_xy + dz * dz;
      if (dz * dz < min_grazing_sin_sq * range_sq_full) { continue; }
    }

    const cv::Vec3b cpx = mask_rgb8.at<cv::Vec3b>(crow, col);
    observations.push_back(
      {wxc, wyc, true,
       pixel_log_odds(cpx[0], obstacle_prob_min, max_evidence_step)});
  }

  // Max-pool to one observation per accumulator cell (#26). The forward backstop
  // above emits a contact's obstacle evidence at its true footprint, but the
  // inverse cell loop also emits a WATER (free) observation for the cell whose
  // centre-ray sampled adjacent water — and the downstream buffer accumulates
  // every observation ADDITIVELY, so a small obstacle's single contact is washed
  // out by the co-located water and the cell nets free. Collapsing each cell to
  // its MAX (obstacle-preferring: obstacle log-odds are +ve, water -ve) lets the
  // contact override the co-located water instead of cancelling against it, so a
  // faint-but-real buoy marks without lowering the global obstacle threshold.
  // Cells with only water still emit free; free coverage is unchanged.
  // Gated for A/B + on-water rollback.
  if (!max_pool_bins) {
    return observations;
  }
  std::unordered_map<int64_t, double> bin_log_odds;
  bin_log_odds.reserve(observations.size());
  for (const auto & o : observations) {
    const int ix = static_cast<int>(std::lround((o.x - cx) / res)) + n / 2;
    const int iy = static_cast<int>(std::lround((o.y - cy) / res)) + n / 2;
    if (ix < 0 || ix >= n || iy < 0 || iy >= n) { continue; }
    const int64_t key = static_cast<int64_t>(iy) * n + ix;
    auto it = bin_log_odds.find(key);
    if (it == bin_log_odds.end() || o.log_odds > it->second) {
      bin_log_odds[key] = o.log_odds;
    }
  }
  std::vector<OccupancyObservation> pooled;
  pooled.reserve(bin_log_odds.size());
  for (const auto & kv : bin_log_odds) {
    const int ix = static_cast<int>(kv.first % n);
    const int iy = static_cast<int>(kv.first / n);
    pooled.push_back(
      {cx + (ix - n / 2) * res, cy + (iy - n / 2) * res, kv.second > 0.0, kv.second});
  }
  return pooled;
}

}  // namespace sea_surface_segmentation
