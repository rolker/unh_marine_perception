#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <grid_map_core/grid_map_core.hpp>

#include "occupancy_buffer.hpp"

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

// A single obstacle observation must NOT mark a cell lethal — this is the
// flicker rejection that the per-frame-reset layer lacked (one stray
// segmentation pixel shouldn't become an obstacle).
TEST(OccupancyBuffer, SingleObservationBelowThreshold)
{
  auto buf = make_buffer();  // hit=0.85, threshold=1.0
  EXPECT_TRUE(buf.hit(kP));
  EXPECT_NEAR(buf.logOdds(kP), 0.85, 1e-5);
  EXPECT_FALSE(buf.isLethal(kP)) << "one observation < threshold must not be lethal";
}

// Repeated observations accumulate past the threshold — the obstacle is
// remembered once it's confirmed.
TEST(OccupancyBuffer, RepeatedObservationsCrossThreshold)
{
  auto buf = make_buffer();
  buf.hit(kP);
  buf.hit(kP);
  EXPECT_NEAR(buf.logOdds(kP), 1.70, 1e-5);
  EXPECT_TRUE(buf.isLethal(kP));
}

// Free-space (water) observations drive an obstacle back below the threshold —
// the layer can clear a stale mark when it sees water there.
TEST(OccupancyBuffer, FreeObservationsClearObstacle)
{
  auto buf = make_buffer();
  buf.hit(kP); buf.hit(kP); buf.hit(kP);  // 2.55 → lethal
  ASSERT_TRUE(buf.isLethal(kP));
  buf.miss(kP); buf.miss(kP); buf.miss(kP); buf.miss(kP);  // -1.6 → 0.95
  EXPECT_NEAR(buf.logOdds(kP), 0.95, 1e-5);
  EXPECT_FALSE(buf.isLethal(kP));
}

// Evidence is bounded so it stays revisable (no runaway saturation that would
// take forever to clear).
TEST(OccupancyBuffer, ClampBoundsEvidence)
{
  auto buf = make_buffer();  // clamp = 5.0
  for (int i = 0; i < 20; ++i) { buf.hit(kP); }
  EXPECT_NEAR(buf.logOdds(kP), 5.0, 1e-5);
  for (int i = 0; i < 40; ++i) { buf.miss(kP); }
  EXPECT_NEAR(buf.logOdds(kP), -5.0, 1e-5);
}

// A never-observed cell is unknown (NaN), not free and not lethal.
TEST(OccupancyBuffer, UnobservedCellIsUnknown)
{
  auto buf = make_buffer();
  EXPECT_TRUE(std::isnan(buf.logOdds(kP)));
  EXPECT_FALSE(buf.isLethal(kP));
}

// "Remember, but not forever": a confirmed obstacle decays back below the
// threshold once it stops being observed, and toward the prior over time.
TEST(OccupancyBuffer, DecayForgetsObstacleOverTime)
{
  auto buf = make_buffer();  // half-life 30 s
  buf.hit(kP); buf.hit(kP);  // 1.70 → lethal
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
  buf.hit(kP); buf.hit(kP);  // 1.70 → lethal
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
// wipe it), and newly-exposed cells must read as unobserved. grid_map's move()
// snaps each shift to whole cells and carries the sub-cell residual internally,
// so this exercises no-drift-across-rolling + exposed-unobserved — the core of
// review-plan must-fix #1 — rather than the residual arithmetic itself.
TEST(OccupancyBuffer, RollingPreservesEvidenceAtWorldCell)
{
  auto buf = make_buffer();
  buf.hit(kP); buf.hit(kP);  // lethal at world (0.3, 0)
  ASSERT_TRUE(buf.isLethal(kP));

  // Roll the window through successive origin steps to an end center of
  // (1.0, 0). kP stays inside the window throughout.
  buf.move(grid_map::Position(0.3, 0.0));
  buf.move(grid_map::Position(0.6, 0.0));
  buf.move(grid_map::Position(1.0, 0.0));

  EXPECT_TRUE(buf.isLethal(kP))
    << "evidence must survive fractional rolling at its world location";

  // A cell that was outside the original window (|x|<=2) but inside the rolled
  // one (x in [-1,3]) must be unobserved, not lethal.
  const grid_map::Position exposed(2.5, 0.0);
  EXPECT_TRUE(std::isnan(buf.logOdds(exposed)));
  EXPECT_FALSE(buf.isLethal(exposed));
}

// Out-of-window observations are no-ops (don't throw, don't mark).
TEST(OccupancyBuffer, OutOfBoundsObservationIsNoOp)
{
  auto buf = make_buffer();
  const grid_map::Position outside(100.0, 100.0);
  EXPECT_FALSE(buf.hit(outside));
  EXPECT_FALSE(buf.isLethal(outside));
}

// Parameter validation rejects values that would corrupt the buffer's meaning —
// the guard for runtime `ros2 param set` on a safety layer.
TEST(OccupancyBuffer, ValidateRejectsBadParams)
{
  std::string why;
  EXPECT_TRUE(OccupancyBuffer::validate(OccupancyParams{}, why)) << why;

  OccupancyParams p;
  p.hit_log_odds = -1.0;  // must be > 0
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.miss_log_odds = 0.5;  // must be < 0
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.decay_half_life_s = 0.0;  // must be > 0
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.lethal_threshold = 99.0;  // must be <= clamp
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.lethal_threshold = 0.0;   // must be > 0 (else 1 hit = lethal)
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.lethal_threshold = -1.0;  // negative inverts safety (water => lethal)
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));

  p = OccupancyParams{}; p.hit_log_odds = std::nan("");  // finite required
  EXPECT_FALSE(OccupancyBuffer::validate(p, why));
}
