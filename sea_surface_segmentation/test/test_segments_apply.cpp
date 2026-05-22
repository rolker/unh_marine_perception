#include <gtest/gtest.h>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "segments_apply.hpp"

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::NO_INFORMATION;
using sea_surface_layer::apply_segments_to_master;

namespace
{

// Helper: build an N×N Costmap2D initialised to a uniform value.
Costmap2D make_filled(unsigned int n, unsigned char value)
{
  Costmap2D m(n, n, 1.0, 0.0, 0.0, value);
  return m;
}

}  // namespace

// Calling with max_i, max_j BELOW the grid size must leave cells at index
// max_i and max_j untouched. Pre-fix `<=` would write into them; post-fix `<`
// does not.
TEST(ApplySegmentsToMaster, HalfOpenBoundsRespected)
{
  constexpr unsigned int N = 4;
  Costmap2D segments = make_filled(N, LETHAL_OBSTACLE);
  Costmap2D master = make_filled(N, FREE_SPACE);

  apply_segments_to_master(segments, master, /*min_i=*/0, /*min_j=*/0, /*max_i=*/3, /*max_j=*/3);

  // [0, 3) × [0, 3) should be LETHAL_OBSTACLE.
  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      EXPECT_EQ(master.getCost(i, j), LETHAL_OBSTACLE) << "i=" << i << " j=" << j;
    }
  }
  // Row i==3 and column j==3 must remain FREE_SPACE.
  for (unsigned int k = 0; k < N; ++k) {
    EXPECT_EQ(master.getCost(3, k), FREE_SPACE) << "row 3, col " << k;
    EXPECT_EQ(master.getCost(k, 3), FREE_SPACE) << "col 3, row " << k;
  }
}

// Regression test for unh_marine_perception#14: with max_i == size_x and
// max_j == size_y (the LayeredCostmap clamp under a rolling-window costmap
// where the union of all layers' updateBounds covers the full grid), the
// loop must stop *before* the buffer end. Pre-fix `<=` read past the end of
// segments_costmap_.costmap_ → SIGSEGV in the field.
TEST(ApplySegmentsToMaster, GridEdgeBoundsDoesNotCrash)
{
  constexpr unsigned int N = 4;
  Costmap2D segments = make_filled(N, LETHAL_OBSTACLE);
  Costmap2D master = make_filled(N, FREE_SPACE);

  ASSERT_NO_FATAL_FAILURE(
    apply_segments_to_master(segments, master, 0, 0,
                             static_cast<int>(N), static_cast<int>(N)));

  // All N×N cells of master should now be LETHAL_OBSTACLE.
  for (unsigned int i = 0; i < N; ++i) {
    for (unsigned int j = 0; j < N; ++j) {
      EXPECT_EQ(master.getCost(i, j), LETHAL_OBSTACLE) << "i=" << i << " j=" << j;
    }
  }
}

// NO_INFORMATION cells in segments must not overwrite master.
TEST(ApplySegmentsToMaster, NoInformationCellsSkipped)
{
  constexpr unsigned int N = 4;
  Costmap2D segments = make_filled(N, NO_INFORMATION);
  Costmap2D master = make_filled(N, FREE_SPACE);

  apply_segments_to_master(segments, master, 0, 0, N, N);

  for (unsigned int i = 0; i < N; ++i) {
    for (unsigned int j = 0; j < N; ++j) {
      EXPECT_EQ(master.getCost(i, j), FREE_SPACE) << "i=" << i << " j=" << j;
    }
  }
}

// Per-cell max-keep: a lower segments cost must not lower a higher master cost.
TEST(ApplySegmentsToMaster, MasterMaxKept)
{
  constexpr unsigned int N = 2;
  Costmap2D segments = make_filled(N, FREE_SPACE);
  Costmap2D master = make_filled(N, LETHAL_OBSTACLE);

  apply_segments_to_master(segments, master, 0, 0, N, N);

  for (unsigned int i = 0; i < N; ++i) {
    for (unsigned int j = 0; j < N; ++j) {
      EXPECT_EQ(master.getCost(i, j), LETHAL_OBSTACLE) << "i=" << i << " j=" << j;
    }
  }
}
