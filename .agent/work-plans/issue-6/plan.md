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
   subscribers exist yet, no contention). No deadlock risk in this file's own
   call paths.
2. **Acquire `costmap_mutex_` and guard against zero-size in `reset()`** —
   skip the `resetMapToValue` call when `count_x_ == 0 || count_y_ == 0`.
3. **Reorder `onInitialize` so `matchSize()` runs before `create_subscription`** —
   without this, a multi-threaded executor with pre-existing publishers can
   dispatch `cameraInfoCallback` then `segmentsCallback` in the window between
   subscriber creation and `matchSize`, hitting the same `count_x_-1`
   underflow on the `segmentsCallback` `resetMapToValue` path. Surfaced by
   Copilot review on PR #11.
4. **Add the same zero-size guard in `segmentsCallback`** before its
   `resetMapToValue` call. Defensive belt-and-suspenders so future reorderings
   can't reintroduce the underflow class.

Issue #6's "Reproduce on a recent build" and "Capture stack trace" acceptance
criteria are satisfied by umbrella #10's static diagnosis (mechanism, trigger,
and code path all named) — end-to-end runtime reproduction is the verification
step in #7, not a prerequisite for landing the patch.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | Lock in `matchSize()`; lock + zero-size guard in `reset()` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Test deferred to threading-hygiene PR with #10 items E+F+L (test design lands once against the final locking strategy, instead of being built now and rewritten when E/F refine the locks). Umbrella #10 items C/G/J also deferred — both deferrals surfaced as scope decisions, not silent. |
| Test what breaks | Boat verification (#7) is the meaningful runtime test for this crash; synthetic unit tests against the current half-locked state would be throwaway once E/F land. |
| Only what's needed | No restructure of `updateBounds` (items C/D from #10), no rewrite of the per-cell loop (item I), no test plumbing built twice. Fix is minimum-viable for the crash. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 (ROS 2 conventions) | Yes | nav2 costmap layer convention: mutex around buffer-reallocation paths. No new lint exceptions. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `SeaSurfaceLayer` locking | `controller_server` costmap config on bizzy (re-enable layer) | No — deliberately deferred to #7 (boat verification step) |
| Mutex semantics | umbrella #10 items E/F (other unsynchronized accesses) | No — separate threading-hygiene pass, tracked in #10 |
| Test coverage | umbrella #10 item L (no regression test) | No — bundled with the E+F threading-hygiene PR so tests are designed once against the final locking strategy |

## Open Questions

- None at plan stage.

## Estimated Scope

Single PR, small (~10-line diff in `sea_surface_layer.cpp`). Issue #6 stays
open after merge until boat re-enable on bizzy confirms no crash (verification
step is #7).
