#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <grid_map_core/grid_map_core.hpp>

#include "sea_surface_segmentation/cost_mapping.hpp"
#include "sea_surface_segmentation/occupancy_buffer.hpp"

using sea_surface_segmentation::occupancy_to_cost;
using sea_surface_segmentation::OccupancyBuffer;
using sea_surface_segmentation::OccupancyParams;

namespace
{
// 4 m square @ 0.25 m → 16×16 cells, centered at the origin. Roomy enough to
// roll the window and still have an overlap region and a newly-exposed region.
OccupancyBuffer make_buffer(const OccupancyParams & p = OccupancyParams{})
{
  return OccupancyBuffer(4.0, 4.0, 0.25, grid_map::Position(0.0, 0.0), p);
}
const grid_map::Position kP(0.3, 0.0);  // a representative observed cell
}  // namespace

// A single small positive increment must NOT mark a cell lethal — flicker
// rejection: one marginal observation shouldn't read as a confirmed obstacle.
TEST(OccupancyBuffer, SingleObservationBelowThreshold)
{
  auto buf = make_buffer();  // lethal_threshold = 1.0
  EXPECT_TRUE(buf.accumulate(kP, 0.85));
  EXPECT_NEAR(buf.logOdds(kP), 0.85, 1e-5);
  EXPECT_FALSE(buf.isLethal(kP)) << "one observation < threshold must not be lethal";
}

// Repeated positive increments accumulate past the threshold — the obstacle is
// remembered once it's confirmed.
TEST(OccupancyBuffer, RepeatedObservationsCrossThreshold)
{
  auto buf = make_buffer();
  buf.accumulate(kP, 0.85);
  buf.accumulate(kP, 0.85);
  EXPECT_NEAR(buf.logOdds(kP), 1.70, 1e-5);
  EXPECT_TRUE(buf.isLethal(kP));
}

// Negative (water) increments drive an obstacle back below the threshold — the
// layer can clear a stale mark when it sees water there.
TEST(OccupancyBuffer, FreeObservationsClearObstacle)
{
  auto buf = make_buffer();
  buf.accumulate(kP, 0.85); buf.accumulate(kP, 0.85); buf.accumulate(kP, 0.85);  // 2.55 → lethal
  ASSERT_TRUE(buf.isLethal(kP));
  buf.accumulate(kP, -0.40); buf.accumulate(kP, -0.40);
  buf.accumulate(kP, -0.40); buf.accumulate(kP, -0.40);  // -1.6 → 0.95
  EXPECT_NEAR(buf.logOdds(kP), 0.95, 1e-5);
  EXPECT_FALSE(buf.isLethal(kP));
}

// Evidence is bounded to [clear_floor, obstacle_clamp] so it stays revisable
// (no runaway saturation that would take forever to clear).
TEST(OccupancyBuffer, ClampBoundsEvidence)
{
  auto buf = make_buffer();  // obstacle_clamp = 5.0, clear_floor = -2.0
  for (int i = 0; i < 50; ++i) { buf.accumulate(kP, 100.0); }  // huge +increments
  EXPECT_NEAR(buf.logOdds(kP), 5.0, 1e-5) << "saturates at obstacle_clamp";
  for (int i = 0; i < 50; ++i) { buf.accumulate(kP, -100.0); }  // huge -increments
  EXPECT_NEAR(buf.logOdds(kP), -2.0, 1e-5) << "saturates at clear_floor";
}

// occupancyAt: unobserved is -1; small positive between free and lethal ramps
// 1..99; at/above lethal is 100; at/below free_threshold (e.g. negative) is -1.
TEST(OccupancyBuffer, OccupancyAtRampAndBounds)
{
  auto buf = make_buffer();  // free_threshold=0, lethal_threshold=1.0

  // Unobserved → -1.
  EXPECT_EQ(buf.occupancyAt(kP), -1);

  // Half-way up the ramp (0.5 of [0,1]) → ~50.
  buf.accumulate(kP, 0.5);
  const int mid = buf.occupancyAt(kP);
  EXPECT_GE(mid, 1);
  EXPECT_LE(mid, 99);
  EXPECT_NEAR(mid, 50, 2) << "linear ramp should land near the midpoint";

  // Push to/over lethal → 100.
  buf.accumulate(kP, 1.0);  // total 1.5 >= 1.0
  EXPECT_EQ(buf.occupancyAt(kP), 100);

  // A purely negative (water) cell sits at/below free_threshold → -1.
  const grid_map::Position kWater(0.3, 0.5);
  buf.accumulate(kWater, -0.4);
  EXPECT_EQ(buf.occupancyAt(kWater), -1)
    << "<= free_threshold reads as no opinion (we only ADD cost)";
}

// A never-observed cell is unknown (NaN), not free and not lethal.
TEST(OccupancyBuffer, UnobservedCellIsUnknown)
{
  auto buf = make_buffer();
  EXPECT_TRUE(std::isnan(buf.logOdds(kP)));
  EXPECT_FALSE(buf.isLethal(kP));
  EXPECT_EQ(buf.occupancyAt(kP), -1);
}

// "Remember, but not forever": a confirmed obstacle decays back below the
// threshold once it stops being observed, and toward the prior over time.
TEST(OccupancyBuffer, DecayForgetsObstacleOverTime)
{
  auto buf = make_buffer();  // half-life 30 s
  buf.accumulate(kP, 0.85); buf.accumulate(kP, 0.85);  // 1.70 → lethal
  buf.decay(0.0);            // first call only seeds the clock
  EXPECT_TRUE(buf.isLethal(kP)) << "no time elapsed yet";
  buf.decay(25.0);           // 1.70 * 0.5^(25/30) ≈ 0.954
  EXPECT_NEAR(buf.logOdds(kP), 1.70 * std::pow(0.5, 25.0 / 30.0), 1e-4);
  EXPECT_FALSE(buf.isLethal(kP)) << "decayed below threshold";
  buf.decay(1000.0);
  EXPECT_LT(std::fabs(buf.logOdds(kP)), 0.01) << "decays toward the prior (0)";
}

// A backward time jump (clock reset / sim-time discontinuity) must not decay and
// must not rewind the decay reference: evidence is held, and the next forward
// step measures true elapsed time from the original reference (not the blip).
TEST(OccupancyBuffer, BackwardTimeDoesNotDecayOrRewind)
{
  auto buf = make_buffer();  // half-life 30 s
  buf.accumulate(kP, 0.85); buf.accumulate(kP, 0.85);  // 1.70 → lethal
  buf.decay(100.0);          // seed reference at t=100
  buf.decay(50.0);           // backward: must not decay, must not rewind reference
  EXPECT_NEAR(buf.logOdds(kP), 1.70, 1e-5) << "backward time must not decay";
  EXPECT_TRUE(buf.isLethal(kP));
  buf.decay(130.0);          // dt measured from 100 (not 50) → exactly one half-life
  EXPECT_NEAR(buf.logOdds(kP), 0.85, 1e-4)
    << "elapsed measured from the held reference, not the backward blip";
}

// Rolling the window through a sequence of origin shifts must preserve
// accumulated evidence at its true world cell (not drift it to a neighbour or
// wipe it), and newly-exposed cells must read as unobserved.
TEST(OccupancyBuffer, RollingPreservesEvidenceAtWorldCell)
{
  auto buf = make_buffer();
  buf.accumulate(kP, 0.85); buf.accumulate(kP, 0.85);  // lethal at world (0.3, 0)
  ASSERT_TRUE(buf.isLethal(kP));

  buf.move(grid_map::Position(0.3, 0.0));
  buf.move(grid_map::Position(0.6, 0.0));
  buf.move(grid_map::Position(1.0, 0.0));

  EXPECT_TRUE(buf.isLethal(kP))
    << "evidence must survive fractional rolling at its world location";

  const grid_map::Position exposed(2.5, 0.0);
  EXPECT_TRUE(std::isnan(buf.logOdds(exposed)));
  EXPECT_FALSE(buf.isLethal(exposed));
}

// Out-of-window observations are no-ops (don't throw, don't mark).
TEST(OccupancyBuffer, OutOfBoundsObservationIsNoOp)
{
  auto buf = make_buffer();
  const grid_map::Position outside(100.0, 100.0);
  EXPECT_FALSE(buf.accumulate(outside, 0.85));
  EXPECT_FALSE(buf.isLethal(outside));
  EXPECT_EQ(buf.occupancyAt(outside), -1);
}

// Parameter validation rejects values that would corrupt the buffer's meaning —
// the guard for runtime `ros2 param set` on a safety layer.
TEST(OccupancyBuffer, ValidateRejectsBadParams)
{
  std::string why;
  EXPECT_TRUE(OccupancyBuffer::validate(OccupancyParams{}, why)) << why;

  OccupancyParams p;
  p.obstacle_clamp = -1.0;  // must be > 0
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.clear_floor = 0.5;  // must be < 0
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.clear_floor = 0.0;  // must be strictly < 0
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.decay_half_life_s = 0.0;  // must be > 0
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.lethal_threshold = 99.0;  // must be <= obstacle_clamp
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.free_threshold = 1.0; p.lethal_threshold = 1.0;  // free < lethal
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.free_threshold = 2.0;  // free >= lethal (1.0)
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.obstacle_clamp = std::nan("");  // finite required
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.free_threshold = std::nan("");  // finite required
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));
}

// setParams reinterprets accumulated evidence immediately — a single increment
// that sat just under the default lethal_threshold (1.0) becomes lethal once a
// lower threshold (0.3) is applied at runtime, without re-observing the cell.
TEST(OccupancyBuffer, SetParamsReinterpretsAccumulatedEvidence)
{
  auto buf = make_buffer();  // defaults: lethal_threshold = 1.0
  EXPECT_TRUE(buf.accumulate(kP, 0.85));
  EXPECT_FALSE(buf.isLethal(kP)) << "0.85 < 1.0, not yet lethal at default threshold";

  OccupancyParams looser = OccupancyParams{};
  looser.lethal_threshold = 0.3;  // < 0.85; same accumulated evidence now reads lethal
  std::string why;
  ASSERT_TRUE(OccupancyBuffer::validate(looser, why)) << why;
  buf.setParams(looser);

  EXPECT_TRUE(buf.isLethal(kP))
    << "lowered threshold must reinterpret accumulated 0.85 log-odds as lethal";
}

// occupancy_to_cost boundaries: -1/0 → -1 (untouched); 1 → 1; 99 → 252 (below
// INSCRIBED 253); 100 → LETHAL_OBSTACLE (254).
TEST(CostMapping, OccupancyToCostBoundaries)
{
  EXPECT_EQ(occupancy_to_cost(-1), -1);
  EXPECT_EQ(occupancy_to_cost(0), -1);
  EXPECT_EQ(occupancy_to_cost(1), 1);
  EXPECT_EQ(occupancy_to_cost(99), 252);
  EXPECT_EQ(occupancy_to_cost(100), 254);  // LETHAL_OBSTACLE
  // Monotonic across the soft ramp, and never reaching INSCRIBED (253).
  int prev = 0;
  for (int occ = 1; occ <= 99; ++occ) {
    const int c = occupancy_to_cost(occ);
    EXPECT_GE(c, prev);
    EXPECT_LE(c, 252) << "soft cost must stay below INSCRIBED_INFLATED_OBSTACLE (253)";
    prev = c;
  }
}
