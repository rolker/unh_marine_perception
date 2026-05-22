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
`updateBounds` clobber pattern to `std::min/std::max`. With that fix in place,
`chart_layer`'s wider bounds (set first in the plugin list) now **survive**
each SeaSurfaceLayer's `updateBounds` call instead of being clobbered. With 4
SeaSurfaceLayer instances + chart_layer contributing, the surviving union
reliably reaches the grid edge every cycle. Pre-#13 single-instance had max_i
often below size_x, so the off-by-one read landed within mapped heap memory
(silently wrong, no crash). The trigger is "chart_layer bounds survive" more
than "4 layers" per se — but the 4-layer config is what made it deterministic
enough to surface in the field.

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
3. **Extract `apply_segments_to_master` as a pure free function** in a new
   header `src/segments_apply.hpp` — same pattern as `frame_id_resolver.hpp`
   was extracted for testability. `updateCosts` becomes a thin wrapper that
   takes the mutex then delegates. The free function operates on two
   `Costmap2D&` references — no `Layer` base, no `LayeredCostmap`, no TF —
   so it's directly unit-testable.
4. **Add gtest cases** in `test/test_segments_apply.cpp`:
   - **`HalfOpenBoundsRespectedAtOrigin`** — call with `max_i < size_x`
     from `min_i=0` and verify the row at `max_i` is untouched. Pre-fix
     `<=` would set those cells, failing the assertion → load-bearing
     deterministic regression catch for #14.
   - **`HalfOpenBoundsRespectedOffOrigin`** — same invariant with a
     non-zero `min_i`, `min_j` (added per review-code feedback to lock
     down indexing tied to offsets that an origin-anchored test would
     miss).
   - **`GridEdgeBoundsCompletes`** — smoke test for the field-crash
     code path (`max_i == size_x, max_j == size_y`). On a small heap
     buffer this can pass under the pre-fix `<=` (OOB read often lands
     in mapped memory), so it documents the field scenario rather
     than serving as the deterministic catch — `HalfOpenBoundsRespected*`
     are the load-bearing catches.
   - **`NoInformationCellsSkipped`** and **`MasterMaxKept`** — confirm
     the `NO_INFORMATION` skip and the `cost > master.getCost(i, j)`
     max-keep semantics are preserved across the refactor.
5. **Verification** — locally: `colcon test --packages-select
   sea_surface_segmentation` passes the new tests. In the field on gabby:
   re-cycle the nav launch 5+ times with the 4-layer config that crashed
   ~1-in-2 launches in the 2026-05-22 gdb session; confirm no SIGSEGV.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/segments_apply.hpp` (new) | Header-only free function `apply_segments_to_master(segments, master, min_i, min_j, max_i, max_j)` with half-open loop bounds (`<`, not `<=`). |
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | `updateCosts` calls the new free function; `count_x_-1` → `count_x_`, `count_y_-1` → `count_y_` in `reset()` and `segmentsCallback`'s `resetMapToValue` (2 sites). |
| `sea_surface_segmentation/test/test_segments_apply.cpp` (new) | Five gtest cases: half-open bounds at origin + off-origin (the deterministic regression catches), grid-edge smoke, NO_INFORMATION skip, max-keep semantics. |
| `sea_surface_segmentation/CMakeLists.txt` | Register the new test target (mirroring the `test_frame_id_resolver` pattern). |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Pure-function extraction + unit tests land with the fix. Field re-cycle on gabby is documented in the PR body as the in-context verification. |
| Test what breaks | `HalfOpenBoundsRespectedAtOrigin` (and the off-origin variant) deterministically fail under the pre-fix `<=` loop. `GridEdgeBoundsCompletes` documents the field scenario but isn't load-bearing on small buffers. `NoInformationCellsSkipped`/`MasterMaxKept` pin the refactor's behavioral equivalence. |
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
| The layer's behavior at grid edges | `seafloor_echoboat_project11`'s 4-layer config (PR #19 / field workaround PR #21) | Yes — supersede the field workaround once gabby verifies clean: if `seafloor_echoboat_project11#21` is still open, close it without merging and the 4-layer config remains the deployed state; if it merged ahead, file a follow-up PR re-enabling the 3 commented-out layers. Logged as a follow-up step, not part of this PR. |
| `segmentsCallback`'s `resetMapToValue` bounds | Nothing else — internal to the layer. | n/a |

## Open Questions

- None at plan stage.

## Estimated Scope

Single PR. Code: ~25 lines (header + thin `updateCosts` wrapper + 3 one-character
fixes). Test: ~80 lines (4 focused gtest cases). CMakeLists: +5 lines. Total
~110-line diff. Field re-cycle on gabby is the in-context verification.

## Followups (not in this PR)

- **Close umbrella #10 items E/F** with a note pointing at this PR — the
  threading races were the wrong hypothesis; the gdb trace shows the crash
  is in `mapUpdateLoop`, not in `segmentsCallback`. Item L (no regression
  test) stands.
- **Supersede `seafloor_echoboat_project11#21`** field workaround on gitcloud
  once gabby's verified 5-cycle-clean post-fix. Exact action depends on PR
  #21's state at the time: close without merging (if still open) or open a
  re-enable PR (if it merged ahead).
- **Decide on the multi-camera redesign** — per
  `[[project_sea_surface_layer_one_camera_design]]`, the 4-instance pattern
  may eventually be replaced by a single layer that ingests N streams. If
  so, the off-by-one fix is still correct on the way to that redesign.
