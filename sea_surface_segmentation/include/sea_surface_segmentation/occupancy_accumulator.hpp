#ifndef SEA_SURFACE_SEGMENTATION__OCCUPANCY_ACCUMULATOR_HPP_
#define SEA_SURFACE_SEGMENTATION__OCCUPANCY_ACCUMULATOR_HPP_

#include <cstddef>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/core.hpp>

#include "image_geometry/pinhole_camera_model.hpp"

#include "sea_surface_segmentation/occupancy_buffer.hpp"
#include "sea_surface_segmentation/segments_projection.hpp"

namespace sea_surface_segmentation
{

// Tunable knobs for one `accumulate_frame` call. Defaults mirror
// `project_observations_inverse` so this struct is the single source of those
// defaults — `accumulate_frame` forwards them and keeps none of its own.
struct AccumulateParams
{
  double max_range;                    // metres; rays landing beyond are dropped
  double res;                          // cell pitch (m) of the iteration window
  double half_extent;                  // half-width (m) of the square iteration window
  double plane_z = 0.0;                // water-plane z in the world/target frame
  double min_grazing_angle_deg = 0.0;  // reject rays striking the plane shallower; 0 = no filter
};

// Per-frame driver: re-centre the rolling buffer on the boat, decay stale
// evidence, project this frame's segmentation onto the water plane (the INVERSE
// cell→pixel projection), and accumulate hit/miss into the buffer. This is the
// single shared accumulate path used by both the offline `bag_to_costmap_video`
// tool and the `sea_surface_tuner` (marine_perception_tools), so they produce
// identical costmaps from the real algorithm (rolker/unh_marine_perception#23).
//
// Pure boundary: the caller decodes the segmentation to an `rgb8` `cv::Mat` and
// resolves the camera/boat poses from TF, then passes the geometry in. This
// keeps the header free of `cv_bridge` / `sensor_msgs` / `tf2`. (The projected
// segmentation is a raw rgb8 Image in both callers; the ffmpeg-encoded stream is
// the display camera imagery, never projected.)
//
// Iteration is centred on `(boat_x, boat_y)` — the buffer window centre —
// matching the tool's boat-centred visualisation window. The live
// `SeaSurfaceLayer` centres iteration on the camera instead; either is valid,
// the helper iterates a square axis-aligned window.
//
// Call order matters and is locked here: move() (roll the window to the boat) →
// decay() (attenuate stale evidence by elapsed time) → project + hit/miss. The
// `OccupancyBuffer` preserves overlapping evidence across a move() and seeds the
// decay clock on the first decay() call.
//
// Returns the number of observations applied to the buffer (hits + misses) —
// useful for per-camera "did this frame contribute anything" diagnostics.
inline std::size_t accumulate_frame(
  OccupancyBuffer & buffer,
  const cv::Mat & mask_rgb8,
  const image_geometry::PinholeCameraModel & camera_model,
  const cv::Vec3d & camera_origin,
  const cv::Matx33d & rotation_cam_to_target,
  double boat_x, double boat_y,
  double stamp_s,
  const AccumulateParams & params)
{
  // Moving window: re-centre the buffer on the boat, decay, then ingest.
  buffer.move(grid_map::Position(boat_x, boat_y));
  buffer.decay(stamp_s);

  const std::vector<OccupancyObservation> observations =
    project_observations_inverse(
      mask_rgb8, camera_model, camera_origin, rotation_cam_to_target,
      params.max_range, boat_x, boat_y, params.res, params.half_extent,
      params.plane_z, params.min_grazing_angle_deg);

  for (const auto & o : observations) {
    const grid_map::Position p(o.x, o.y);
    if (o.obstacle) {
      buffer.hit(p);
    } else {
      buffer.miss(p);
    }
  }
  return observations.size();
}

}  // namespace sea_surface_segmentation

#endif  // SEA_SURFACE_SEGMENTATION__OCCUPANCY_ACCUMULATOR_HPP_
