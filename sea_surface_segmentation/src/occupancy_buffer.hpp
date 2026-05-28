#ifndef SEA_SURFACE_SEGMENTATION__OCCUPANCY_BUFFER_HPP_
#define SEA_SURFACE_SEGMENTATION__OCCUPANCY_BUFFER_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <grid_map_core/grid_map_core.hpp>

namespace sea_surface_segmentation
{

// Tunable parameters for the log-odds occupancy buffer. All are runtime-
// reconfigurable on the live layer (validated via `validate()` before apply).
struct OccupancyParams
{
  double hit_log_odds = 0.85;       // added to a cell on an obstacle observation
  double miss_log_odds = -0.40;     // added on a free-space (water) observation; negative
  double clamp = 5.0;               // |log-odds| bound, so evidence stays revisable (no saturation)
  double lethal_threshold = 1.0;    // log-odds >= this => the cell reads as a lethal obstacle
  double decay_half_life_s = 30.0;  // unobserved evidence halves every this many seconds
};

// A rolling, world-frame occupancy grid that accumulates obstacle evidence as
// per-cell log-odds and forgets it over time (decay toward the prior).
//
// Design notes:
//   - Storage and rolling are delegated to `grid_map::GridMap`: `move()` shifts
//     the window by circular-buffer index (no per-cell copy), preserves the
//     overlapping data at its world position, and fills newly-exposed cells with
//     NaN. We treat NaN as "unobserved" (the prior) — a cell only becomes lethal
//     after enough obstacle observations accumulate, so single-frame flicker is
//     rejected, and evidence persists through gaps until it decays away.
//   - Pure logic, no ROS/nav2 types, so it is unit-testable in isolation. The
//     costmap layer owns the grid_map<->Costmap2D bridge and the camera geometry.
class OccupancyBuffer
{
public:
  OccupancyBuffer(
    double size_x_m, double size_y_m, double resolution,
    const grid_map::Position & center, const OccupancyParams & params)
  : map_(std::vector<std::string>{"log_odds"}), params_(params)
  {
    map_.setGeometry(grid_map::Length(size_x_m, size_y_m), resolution, center);
    map_["log_odds"].setConstant(NAN);  // start fully unobserved
  }

  // Roll the window so it re-centers on `position`. Overlapping cells keep their
  // accumulated evidence at the same world location; newly-exposed cells are NaN
  // (unobserved). Sub-cell motion is absorbed by grid_map's internal offset, so a
  // sequence of fractional-meter shifts does not drift evidence to a wrong cell.
  bool move(const grid_map::Position & position) { return map_.move(position); }

  // Accumulate an obstacle observation at `position` (clamped log-odds).
  bool hit(const grid_map::Position & position) { return apply(position, params_.hit_log_odds); }

  // Accumulate a free-space (water) observation at `position` (clamped log-odds).
  bool miss(const grid_map::Position & position) { return apply(position, params_.miss_log_odds); }

  // Decay all observed cells toward the prior based on wall-clock elapsed time.
  // Call once per update cycle with the current time; the first call only seeds
  // the clock. NaN (unobserved) cells are unaffected (NaN * factor == NaN).
  void decay(double now_s)
  {
    if (!seeded_) { seeded_ = true; last_decay_s_ = now_s; return; }
    const double dt = now_s - last_decay_s_;
    // On a non-positive dt (backward time jump / clock reset) do nothing AND
    // don't advance the reference, so the next forward step measures true
    // elapsed time and a time discontinuity never erases evidence (the safe
    // direction for an obstacle layer).
    if (dt <= 0.0 || params_.decay_half_life_s <= 0.0) { return; }
    last_decay_s_ = now_s;
    const float factor = static_cast<float>(std::pow(0.5, dt / params_.decay_half_life_s));
    grid_map::Matrix & data = map_["log_odds"];
    data = (data.array() * factor).matrix();
  }

  // True iff `position` is inside the window and its accumulated log-odds has
  // reached the lethal threshold. Unobserved (NaN) cells read as not-lethal.
  bool isLethal(const grid_map::Position & position) const
  {
    if (!map_.isInside(position)) { return false; }
    const float v = map_.atPosition("log_odds", position);
    return std::isfinite(v) && static_cast<double>(v) >= params_.lethal_threshold;
  }

  // Raw log-odds at `position`; NaN if unobserved or outside the window.
  double logOdds(const grid_map::Position & position) const
  {
    if (!map_.isInside(position)) { return NAN; }
    return static_cast<double>(map_.atPosition("log_odds", position));
  }

  // Forget everything — reset all cells to unobserved and re-seed the decay
  // clock on the next decay() call.
  void clear()
  {
    map_["log_odds"].setConstant(NAN);
    seeded_ = false;
  }

  const grid_map::GridMap & map() const { return map_; }
  const OccupancyParams & params() const { return params_; }

  // Replace the tunable parameters (e.g. from a runtime param callback). Existing
  // accumulated evidence is preserved; a new threshold reinterprets it on the next
  // read, new increments/decay take effect on the next update. Caller must
  // `validate()` first.
  void setParams(const OccupancyParams & params) { params_ = params; }

  // Reject values that would corrupt the buffer's interpretation (so a stray
  // runtime `ros2 param set` can't silently poison a safety layer). On failure
  // returns false and sets `why`.
  static bool validate(const OccupancyParams & p, std::string & why)
  {
    if (!std::isfinite(p.hit_log_odds) || p.hit_log_odds <= 0.0) {
      why = "hit_log_odds must be finite and > 0"; return false;
    }
    if (!std::isfinite(p.miss_log_odds) || p.miss_log_odds >= 0.0) {
      why = "miss_log_odds must be finite and < 0"; return false;
    }
    if (!std::isfinite(p.clamp) || p.clamp <= 0.0) {
      why = "clamp must be finite and > 0"; return false;
    }
    if (!std::isfinite(p.lethal_threshold) || p.lethal_threshold <= 0.0 ||
      p.lethal_threshold > p.clamp)
    {
      // Lower bound matters: a threshold of 0 makes a single hit lethal (defeats
      // flicker rejection); a negative one makes a water `miss` read lethal — a
      // safety inversion on an obstacle layer.
      why = "lethal_threshold must be finite and in (0, clamp]"; return false;
    }
    if (!std::isfinite(p.decay_half_life_s) || p.decay_half_life_s <= 0.0) {
      why = "decay_half_life_s must be finite and > 0"; return false;
    }
    return true;
  }

private:
  bool apply(const grid_map::Position & position, double increment)
  {
    if (!map_.isInside(position)) { return false; }
    float & v = map_.atPosition("log_odds", position);
    const double current = std::isfinite(v) ? static_cast<double>(v) : 0.0;  // NaN => prior
    v = static_cast<float>(std::clamp(current + increment, -params_.clamp, params_.clamp));
    return true;
  }

  grid_map::GridMap map_;
  OccupancyParams params_;
  double last_decay_s_ = 0.0;
  bool seeded_ = false;
};

}  // namespace sea_surface_segmentation

#endif  // SEA_SURFACE_SEGMENTATION__OCCUPANCY_BUFFER_HPP_
