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

// Half-open bounds: calling with max_i, max_j BELOW the grid size must leave
// cells at index max_i and max_j untouched. This is the load-bearing
// regression test for unh_marine_perception#14 — under the pre-fix `<=`
// loop, the row at i==max_i and column at j==max_j would be written to,
// and the FREE_SPACE assertions below would fail deterministically.
TEST(ApplySegmentsToMaster, HalfOpenBoundsRespectedAtOrigin)
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

// Same half-open invariant with a window that doesn't touch the origin.
// Catches an indexing regression tied to a non-zero min_i / min_j that the
// origin-anchored test above would miss.
TEST(ApplySegmentsToMaster, HalfOpenBoundsRespectedOffOrigin)
{
  constexpr unsigned int N = 5;
  Costmap2D segments = make_filled(N, LETHAL_OBSTACLE);
  Costmap2D master = make_filled(N, FREE_SPACE);

  apply_segments_to_master(segments, master, /*min_i=*/1, /*min_j=*/1, /*max_i=*/3, /*max_j=*/3);

  // [1, 3) × [1, 3) should be LETHAL_OBSTACLE (4 cells).
  for (unsigned int i = 1; i < 3; ++i) {
    for (unsigned int j = 1; j < 3; ++j) {
      EXPECT_EQ(master.getCost(i, j), LETHAL_OBSTACLE) << "i=" << i << " j=" << j;
    }
  }
  // Everything outside that window must remain FREE_SPACE.
  for (unsigned int i = 0; i < N; ++i) {
    for (unsigned int j = 0; j < N; ++j) {
      if (i >= 1 && i < 3 && j >= 1 && j < 3) continue;
      EXPECT_EQ(master.getCost(i, j), FREE_SPACE) << "outside window i=" << i << " j=" << j;
    }
  }
}

// Smoke test for the field crash scenario (max_i == size_x, max_j == size_y).
// Note: on a small heap-allocated buffer this test can PASS under the pre-fix
// `<=` because the off-by-one read often lands in mapped heap memory rather
// than triggering a SIGSEGV. The deterministic crash signal lived in the
// field repro (1600×1600 grid, gabby 2026-05-22 gdb session); the
// deterministic regression catch in the unit-test layer is
// `HalfOpenBoundsRespectedAtOrigin` above. Keep this test as a documented
// smoke of the field code path.
TEST(ApplySegmentsToMaster, GridEdgeBoundsCompletes)
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
