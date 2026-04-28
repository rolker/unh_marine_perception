#pragma once

// Private implementation header for the sea_surface_segmentation node.
// Lives in src/ rather than include/ because the resolver has no
// downstream consumers — it's exercised by `src/sea_surface_segmentation.cpp`
// and by `test/test_frame_id_resolver.cpp`, nothing else.

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace sea_surface_segmentation {

// Resolve the per-camera `frame_id` used to stamp segmentation outputs.
//
// Behavior:
//   - `frame_ids` empty → every camera gets the historical default
//     `<camera_name>_optical_frame`.
//   - `frame_ids.size() == camera_names.size()` → element-wise override,
//     except an empty string entry falls back to the historical default
//     for that camera.
//   - Any other size → `std::invalid_argument`.
//
// The default branch must match the literal hard-coded in
// `SegmentorCamera`'s constructor before the refactor.
inline std::vector<std::string> resolve_frame_ids(
  const std::vector<std::string> & camera_names,
  const std::vector<std::string> & frame_ids)
{
  if (!frame_ids.empty() && frame_ids.size() != camera_names.size()) {
    throw std::invalid_argument(
      "frame_ids length (" + std::to_string(frame_ids.size()) +
      ") must match camera_names length (" + std::to_string(camera_names.size()) +
      ") when set");
  }

  std::vector<std::string> resolved;
  resolved.reserve(camera_names.size());
  for (std::size_t i = 0; i < camera_names.size(); ++i) {
    const bool have_override = i < frame_ids.size() && !frame_ids[i].empty();
    resolved.push_back(have_override ? frame_ids[i] : (camera_names[i] + "_optical_frame"));
  }
  return resolved;
}

}  // namespace sea_surface_segmentation
