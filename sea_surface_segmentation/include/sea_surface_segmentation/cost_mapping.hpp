#ifndef SEA_SURFACE_SEGMENTATION__COST_MAPPING_HPP_
#define SEA_SURFACE_SEGMENTATION__COST_MAPPING_HPP_

#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace sea_surface_segmentation
{

// Map a published occupancy value [0..100] (or -1 = "no opinion") to a nav2
// costmap cost, or -1 = "leave the master cell untouched". This is the only
// place the graded occupancy ramp meets nav2's cost scale, so the costmap layer
// and the relay layer share one mapping.
//
//   occ <= 0 / -1  → -1     (no opinion — never clears another layer's marks)
//   occ == 100     → LETHAL_OBSTACLE (254)
//   occ 1..99      → soft ramp 1..252 (stays strictly below INSCRIBED = 253, so
//                                       a graded sea-surface cost never reads as
//                                       an inscribed-radius collision)
//
// Pure logic — nav2_costmap_2d cost constants only (no rclcpp/grid_map).
inline int occupancy_to_cost(int occ)
{
  if (occ <= 0) { return -1; }
  if (occ >= 100) { return nav2_costmap_2d::LETHAL_OBSTACLE; }
  return 1 + static_cast<int>(std::lround((occ - 1) / 98.0 * 251.0));
}

}  // namespace sea_surface_segmentation

#endif  // SEA_SURFACE_SEGMENTATION__COST_MAPPING_HPP_
