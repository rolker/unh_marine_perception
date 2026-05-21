# Plan: SeaSurfaceLayer::matchSize() segfault in controller_server

## Issue

https://github.com/rolker/unh_marine_perception/issues/6

## Context

`SeaSurfaceLayer` segfaults inside `controller_server` when included in the local
costmap plugin list. Current workaround on bizzyboat: layer excluded, leaving
chart_layer + base layers only. Blocks end-to-end OAK→costmap validation (#7).

Root cause was diagnosed in umbrella issue #10:

- **A.** `matchSize()` reallocates `segments_costmap_` via `Costmap2D::resizeMap`
  (which does `delete[] costmap_; costmap_ = new …`) without holding
  `costmap_mutex_`. `updateBounds()` calls `matchSize()` whenever the parent's
  origin/size/resolution drifts — and bizzy's local costmap is `rolling_window:
  true`, so the origin shifts every cycle. Meanwhile `segmentsCallback` iterates
  `setCost` under the lock. Use-after-free → segfault. `planner_server`'s
  non-rolling global costmap calls `matchSize` once at init, which is why the
  bug is `controller_server`-specific.
- **B.** `reset()` calls `resetMapToValue(0, 0, count_x_-1, count_y_-1, …)`.
  When called before `matchSize` has run, `count_x_ == 0` underflows to
  `UINT_MAX` → OOB write. Latent under normal lifecycle (onInitialize calls
  matchSize first), but the invariant isn't enforced.

## Approach

1. **Acquire `costmap_mutex_` in `matchSize()`** — wrap the `resizeMap` and
   field updates in a `lock_guard`. Safe to acquire in `onInitialize` (no
   subscribers exist yet, no contention). No deadlock risk: no other
   `costmap_mutex_`-holding code path calls `matchSize`.
2. **Acquire `costmap_mutex_` and guard against zero-size in `reset()`** —
   skip the `resetMapToValue` call when `count_x_ == 0 || count_y_ == 0`.
3. **Add gtest `test_sea_surface_layer_lifecycle.cpp`** — construct a
   `LayeredCostmap` master, add the layer via the plugin loader (or directly,
   if simpler), and exercise: (a) `matchSize` with several distinct parent
   sizes/origins/resolutions; (b) `reset()` called before any `matchSize`
   (regression for B); (c) concurrent `matchSize` + a stand-in for
   `segmentsCallback` writes (regression for A). The goal is not a perfect
   reproducer of the race — those are flaky — but to exercise the lifecycle
   paths under load enough that TSan or a debug build catches misuse.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | Lock in `matchSize()`; lock + zero-size guard in `reset()` |
| `sea_surface_segmentation/test/test_sea_surface_layer_lifecycle.cpp` | New gtest (lifecycle + concurrent matchSize) |
| `sea_surface_segmentation/CMakeLists.txt` | Wire new test into `BUILD_TESTING` block; link `nav2_costmap_2d` to test target |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Test added in same PR (acceptance criterion). Umbrella #10 items C/G/J intentionally deferred to a separate cleanup PR — surfaced as scope decision, not silent deferral. |
| Test what breaks | Test exercises the actual failure mode (matchSize lifecycle, pre-init reset), not getter/setter glue. Race repro is best-effort; primary value is no-crash assertions across realistic call patterns. |
| Only what's needed | No restructure of `updateBounds` (items C/D from #10), no rewrite of the per-cell loop (item I). Fix is minimum-viable for the crash. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 (ROS 2 conventions) | Yes | gtest via `ament_add_gtest`, matching existing `test_frame_id_resolver` style. No new lint exceptions. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `SeaSurfaceLayer` locking | `controller_server` costmap config on bizzy (re-enable layer) | No — deliberately deferred to #7 (boat verification step) |
| Add gtest | `CMakeLists.txt`, `package.xml` test_depends if needed | Yes |
| Mutex semantics | umbrella #10 items E/F (other unsynchronized accesses) | No — separate threading-hygiene pass, tracked in #10 |

## Open Questions

- None at plan stage. Test fixture style (direct plugin construction vs.
  `pluginlib` load) decided during implementation based on what builds
  cleanly against `nav2_costmap_2d`'s headers.

## Estimated Scope

Single PR. Issue #6 stays open after merge until boat re-enable on bizzy
confirms no crash (verification step is #7).
