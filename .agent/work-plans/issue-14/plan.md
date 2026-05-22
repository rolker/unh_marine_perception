# Plan: SeaSurfaceLayer segfault with multiple instances

## Issue

https://github.com/rolker/unh_marine_perception/issues/14

## Context

`SeaSurfaceLayer` segfaults in `controller_server` when 4 instances are
configured in the local costmap (forward/port/starboard/aft). Single-instance
has been running in the field without a crash. Issue #6's locking fix (PR #11)
and the bounds-extension fix (PR #13) are in place but did not resolve #14.

Gdb-on-gabby (2026-05-22, boat at the pier) captured a deterministic backtrace
on the second relaunch cycle:

```
#0  nav2_costmap_2d::Costmap2D::getCost(unsigned int, unsigned int) const
#1  sea_surface_layer::SeaSurfaceLayer::updateCosts(this=..., master_grid=...,
                                                    min_i=…, min_j=0,
                                                    max_i=1600, max_j=1600)
       at sea_surface_layer.cpp:107
       Locals: j=1600  i=1072  lock={...}
#2  nav2_costmap_2d::LayeredCostmap::updateMap(double, double, double)
```

The local costmap is `width: 400 m × resolution: 0.25 m` → 1600 × 1600 cells.
nav2's `LayeredCostmap::updateMap` clamps `max_i`, `max_j` to **equal** the
costmap's size in cells (half-open `[0, size)`), and `ObstacleLayer::updateCosts`
loops `i < max_i; ++i`. `SeaSurfaceLayer::updateCosts` instead loops
`i <= max_i; ++i`, so when the union of all layers' `updateBounds` covers the
full rolling-window grid, the loop calls `segments_costmap_.getCost(i, 1600)`
which reads past the buffer end → SIGSEGV.

**Why intermittent + multi-instance-specific:** PR #13 changed the
`updateBounds` clobber pattern to `std::min/std::max`. With 4 layers + chart_layer
all contributing bounds, the union reliably reaches the grid edge every cycle.
With a single SeaSurfaceLayer, max_i often stayed below size_x and the
off-by-one read landed within mapped heap memory (silently wrong, no crash).

Three sites in `sea_surface_layer.cpp` have the same inclusive-vs-exclusive
confusion:

1. `updateCosts` outer loop: `i <= max_i` (line 103) — **the crash**.
2. `updateCosts` inner loop: `j <= max_j` (line 105) — **the crash**.
3. `segmentsCallback`'s `resetMapToValue(0, 0, count_x_-1, count_y_-1, …)`
   (line 165) — silent: leaves last row/column uncleared.
4. `reset()`'s `resetMapToValue(0, 0, count_x_-1, count_y_-1, …)` (line 68) —
   silent: same.

`Costmap2D::resetMapToValue` takes `(x0, y0, xn, yn)` with `xn`/`yn`
exclusive (`for (y = y0; y < yn; …)` internally), so the right arguments are
`(0, 0, count_x_, count_y_, …)`.

## Approach

1. **Fix the two loop bounds in `updateCosts`** — change `<=` to `<`.
   This is the load-bearing fix that stops the crash.
2. **Fix the two `resetMapToValue` arg sites** — change `count_x_-1` to
   `count_x_` (and `count_y_-1` to `count_y_`). The `count_x_ == 0 ||
   count_y_ == 0` guards remain — they're still needed if `matchSize` hasn't
   run yet.
3. **Verification** — field re-cycle on gabby with the 4-layer config that
   currently crashes ~1-in-3 launches. Per issue #6's pattern (PR #11),
   field re-test is the meaningful regression check, not a synthetic unit test;
   the stack trace already names the exact code path and the fix is mechanical.
   Re-cycle the launch 5+ times on gabby and confirm no SIGSEGV.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | `<=` → `<` in `updateCosts` (2 sites); `count_x_-1` → `count_x_`, `count_y_-1` → `count_y_` in `reset()` and `segmentsCallback`'s `resetMapToValue` (2 sites) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Field verification (cycle-the-launch repro is documented in the PR body) is the regression check. Unit-test scaffolding for a costmap_2d::Layer subclass requires a `LayeredCostmap` fixture + `tf2_ros::Buffer` — adding it for a 5-line mechanical fix is over-scoped. Surfaced as a scope decision, not a silent skip. |
| Test what breaks | The failure mode (read past buffer end when `max_i == size_x`) was caught in the field with gdb, not by tests. A regression test would need to invoke `updateCosts` directly with grid-edge bounds — that's a follow-up issue if it becomes a recurring problem class. |
| Only what's needed | No threading-hygiene rework (umbrella #10 items E/F) — the gdb trace is in the `mapUpdateLoop` thread, not a subscription callback. The race hypotheses in #10 are not the cause. |
| Improve incrementally | Minimum-viable patch for the crash. Leaves the `count_x_ == 0` zero-guards in place. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 (ROS 2 conventions) | Yes | The fix aligns this layer with nav2's `[min, max)` half-open bounds convention (matches `ObstacleLayer::updateCosts` and `Costmap2D::resetMapToValue`'s internal iteration). |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `updateCosts` loop bounds | umbrella issue #10 items E/F (threading-race hypotheses) | Yes — close them as "not the cause; see PR" rather than leaving them open as speculative items. |
| The layer's behavior at grid edges | `seafloor_echoboat_project11`'s 4-layer config (PR #19 / field workaround PR #21) | Yes — once merged, revert PR #21's workaround on gitcloud's `main` so the 4-layer config is the active deployment state. Logged as a follow-up step, not part of this PR. |
| `segmentsCallback`'s `resetMapToValue` bounds | Nothing else — internal to the layer. | n/a |

## Open Questions

- None at plan stage.

## Estimated Scope

Single PR, ~5-line diff in `sea_surface_layer.cpp`. Field re-cycle (5+ launch
attempts post-merge on gabby) is the verification step.

## Followups (not in this PR)

- **Close umbrella #10 items E/F** with a note pointing at this PR — the
  threading races were the wrong hypothesis; the gdb trace shows the crash
  is in `mapUpdateLoop`, not in `segmentsCallback`. Item L (no regression
  test) stands.
- **Revert `seafloor_echoboat_project11#21`** field workaround on gitcloud
  once gabby's verified 5-cycle-clean post-fix.
- **Decide on the multi-camera redesign** — per
  `[[project_sea_surface_layer_one_camera_design]]`, the 4-instance pattern
  may eventually be replaced by a single layer that ingests N streams. If
  so, the off-by-one fix is still correct on the way to that redesign.
