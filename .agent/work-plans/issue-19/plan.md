# Plan: Re-architect SeaSurfaceLayer as a single multi-camera fused occupancy layer (temporal persistence + decay)

## Issue

https://github.com/rolker/unh_marine_perception/issues/19

## Context

Today: four independent `SeaSurfaceLayer` instances (forward/port/starboard/aft), each with a
private buffer that `segmentsCallback` wipes (`resetMapToValue` line 159) every frame and re-marks
only the current FOV. Consequences proven on the 2026-05-26 bag: no obstacle memory (forgotten the
instant it leaves the FOV — the small-buoy resume), ~50% of dynamic marks flicker even with FOV held
fixed, and tall obstacles smear false "shadow" lethal cells onto the z=0 plane. Per-camera buffers
will break the FOV handoff once persistence is added. Overlaps #10 items D, I, H, E/F, L.

A tested `is_waterline_contact_pixel` helper + the marking rule already landed (`fe337f6`) against the
old architecture; the helper/tests carry forward, the integration moves into the rewrite below.

## Approach

Phased, atomic commits on one branch / one PR. The layer becomes **multi-source-capable but
back-compatible with a single source**, so the code can land without forcing a simultaneous config
flip; the seafloor config migration (which activates cross-camera fusion) is the dependent follow-up.

1. **Decouple buffer from rolling-window resize (#10 D)** — replace the `matchSize()`-on-every-shift
   wipe with `updateOrigin()`-style cell-shifting; only `resizeMap` on true size/resolution change.
   Prerequisite for any persistence. Fixes #10 D.
2. **Pure log-odds occupancy buffer** — new header `occupancy_buffer.hpp` (mirrors the
   `segments_apply.hpp` / `segments_projection.hpp` pure-logic + unit-test pattern): per-cell
   log-odds, `hit`/`miss` updates, time-based decay toward prior, threshold→LETHAL. Fully unit-tested.
3. **Inverted projection + waterline-contact + occlusion (#10 I, H + shadow)** — per segmentation
   frame, scan image columns, take the waterline contact (`is_waterline_contact_pixel`), back-project
   it to the ground plane reusing `project_obstacle_pixels`; contact → `hit`, observed water → `miss`;
   the occluded region beyond a contact is left unobserved (decays, not marked). Folds in `fe337f6`,
   fixes #10 I and H.
4. **Multi-source subscriptions (consolidate 4→1)** — accept a list of camera sources (segmentation +
   camera_info + per-source `maximum_range`); all feed the one shared world-frame buffer (so an
   obstacle reinforces the same cell across cameras — the handoff fix). A single source remains valid
   (back-compat).
5. **Thread-safety (#10 E/F)** — guard the shared buffer and per-source camera models against the N
   concurrent camera/camera_info callbacks; fix the `camera_model_` / `count_x_` races.
6. **Live-tunable parameters via a param callback (#10 K)** — register an
   `OnSetParametersCallbackHandle` so the tuning scalars can be changed at runtime (`ros2 param set`)
   without a stack restart, for on-the-fly tuning on the water: **decay half-life, hit/miss log-odds
   increments, lethal threshold, per-source max range**. Applied under the buffer lock — a new
   threshold reinterprets the existing accumulated buffer immediately; new decay/increment values take
   effect on the next update. Source list / topics remain configure-time. A unit test asserts the
   callback actually applies the change (the current layer silently ignores runtime params — the
   #10-K failure mode).
7. **Tests (#10 L)** — occupancy buffer (hit raises / miss+decay lowers / threshold / persists across
   a rolling-origin shift / decays to prior over time), waterline-contact + occlusion, two-source
   fusion reinforcing one world cell.
8. **Docs + config follow-up** — document new params; migrate seafloor `nav2_params.yaml` (4 blocks →
   1 multi-source) in a dependent PR (different repo); note IzzyBoat parity.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | Rewrite: multi-source subs, shared persistent buffer (updateOrigin not resize), inverted projection w/ waterline-contact + occlusion, log-odds update, decay, thread-safe, param callback |
| `sea_surface_segmentation/src/occupancy_buffer.hpp` | New pure log-odds buffer (hit/miss/decay/threshold) |
| `sea_surface_segmentation/src/segments_projection.hpp` | `is_waterline_contact_pixel` (done); reuse `project_obstacle_pixels` for back-projection |
| `sea_surface_segmentation/test/test_occupancy_buffer.cpp` | New: log-odds + decay + rolling-shift persistence tests |
| `sea_surface_segmentation/test/test_segments_projection.cpp` | Waterline tests (done) + fusion/occlusion cases |
| `sea_surface_segmentation/CMakeLists.txt` | Register new test |
| `sea_surface_segmentation/README*` / params doc | Document sources, decay, thresholds |
| seafloor `echoboat_project11/config/nav2_params.yaml` *(follow-up PR, separate repo)* | 4 instances → 1 multi-source block |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | Phased atomic commits; back-compat single-source lets the code land before the config flip |
| A change includes its consequences | Tests, param docs, and config migration tracked (config is a separate-repo follow-up, owned here) |
| Test what breaks | Persistence across rolling shift, decay, fusion, flicker rejection, concurrency |
| Only what's needed | Log-odds is justified by the noise data; no marking modes beyond what the data shows |
| Human control & transparency | Tuning scalars (decay/threshold/increments/range) are live-tunable at runtime; decay model decided (log-odds) |
| Capture decisions | Plan + PR record rationale; #10 flagged this as ADR-like (no project ADR mechanism → captured here) |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | Adopts nav2 ObstacleLayer multi-source idiom + `OnSetParametersCallbackHandle`; target Rolling; no deviation needing its own ADR |
| 0001 — Adopt ADRs | Watch | Design rationale + decay-model decision captured in plan/PR (no project-level ADR mechanism in this repo) |
| 0002 — Worktree isolation | Yes | Work on `feature/issue-19` layer worktree |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `SeaSurfaceLayer` params/behavior | seafloor `nav2_params.yaml` (4→1 multi-source) | Follow-up PR (separate repo, owned here) |
| ... | IzzyBoat config parity (#120/#181) | Noted for follow-up |
| The layer | package README / param docs | Yes |
| Layer logic | regression tests (#10 L) | Yes |
| (closes #10 items) | reconcile #10: D, I, H, E/F, L addressed | Note in PR + comment on #10 |

## Open Questions

- **Decay/clearing model — DECIDED: log-odds occupancy** (confirmed 2026-05-27). Robust handling of
  conflicting evidence + graceful clearing. Tuning scalars are live-reconfigurable (phase 6).
- **Sequencing** — land the perception capability PR first (back-compat single-source), then the
  seafloor config-migration PR to activate cross-camera fusion? (recommended: yes)
- **Default tuning** — decay half-life, hit/miss increments, lethal threshold: propose conservative
  defaults, tune on water.

## Estimated Scope

One PR in `unh_marine_perception` (atomic commits per phase) + one dependent follow-up PR in
`seafloor_echoboat_project11` for the config migration (separate repo — unavoidable; handled here, no
bookkeeping for the user). Larger than a typical single PR but kept cohesive per the back-compat
sequencing; review-plan should confirm it doesn't need a hard split.
