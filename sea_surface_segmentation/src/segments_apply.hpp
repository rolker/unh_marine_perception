#pragma once

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace sea_surface_layer
{

// Copy non-NO_INFORMATION cells from `segments` into `master_grid` over the
// half-open index range [min_i, max_i) × [min_j, max_j), keeping the per-cell
// maximum cost. Bounds are nav2's convention: `max_i`, `max_j` are exclusive
// and may equal the grid size — iterating up to and including them reads past
// the buffer end. (See unh_marine_perception#14.)
inline void apply_segments_to_master(
  const nav2_costmap_2d::Costmap2D & segments,
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      const unsigned char cost = segments.getCost(i, j);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      if (cost > master_grid.getCost(i, j)) {
        master_grid.setCost(i, j, cost);
      }
    }
  }
}

}  // namespace sea_surface_layer
