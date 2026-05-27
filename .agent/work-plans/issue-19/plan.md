# Plan: Re-architect SeaSurfaceLayer as a single multi-camera fused occupancy layer (temporal persistence + decay)

## Issue

https://github.com/rolker/unh_marine_perception/issues/19

_Revised 2026-05-27 per review-plan (`c72005e`, changes-requested) and follow-up design calls:
log-odds buffer backed by **grid_map** (resolves the float-buffer rolling risk via `GridMap::move()`);
#10 H tide-drift confirmed a **non-issue for a floating platform** and dropped._

## Context

Today: four independent `SeaSurfaceLayer` instances (forward/port/starboard/aft), each with a
private buffer that `segmentsCallback` wipes (`resetMapToValue` line 159) every frame and re-marks
only the current FOV. Consequences proven on the 2026-05-26 bag: no obstacle memory (forgotten the
instant it leaves the FOV — the small-buoy resume), ~50% of dynamic marks flicker even with FOV held
fixed, and tall obstacles smear false "shadow" lethal cells (above-waterline pixels back-projected
onto the water plane land far beyond the real obstacle). Per-camera buffers will break the FOV handoff
once persistence is added. Overlaps #10 items D, I, E/F, L (H is resolved as a non-issue — see phase 3).

A tested `is_waterline_contact_pixel` helper + the marking rule already landed (`fe337f6`) against the
old architecture; the helper/tests carry forward, the integration moves into the rewrite below.

## Approach

Phased, atomic commits. The layer becomes **multi-source-capable but back-compatible with a single
source**, so this perception PR can land and be reviewed without the config flip. Per review-plan
(`c72005e`): **this perception PR is _Part of_ #19** (delivers the capability); the dependent seafloor
config-migration PR activates cross-camera fusion, verifies the handoff AC on bag/sim, and **finalizes
#19**. *Per-phase invariant:* every commit leaves the layer buildable AND single-source-correct.

1. **Roll the persistent buffer instead of wiping it (#10 D), via grid_map** — stop the
   `matchSize()`-on-every-shift wipe. Back the log-odds buffer with a **`grid_map::GridMap`** float
   layer and roll it with `GridMap::move()`, which shifts by circular-buffer index (no copy), clears
   only newly-exposed cells, and carries a sub-cell offset so **fractional-meter origin steps** are
   handled correctly — exactly the rolling-window case `Costmap2D::updateOrigin()` (uint8-only) can't
   serve. Sync the grid_map position to the master costmap's rolling origin each cycle. Load-bearing
   prerequisite; addresses #10 D. Test: evidence survives a *sequence* of sub-cell origin shifts and
   lands in the correct world cell. (Also revisit `isClearable()` — hard-coded `false` today — now that
   the layer clears via decay.)
2. **Log-odds occupancy logic** — the buffer is a grid_map float layer (`log_odds`), optionally with a
   `last_update` layer for time-based decay. Keep the update math (hit/miss, decay toward prior,
   threshold→LETHAL) in a **pure, unit-tested** helper (`occupancy_buffer.hpp`, mirroring the
   `segments_apply.hpp` pure-logic + test pattern) operating on the layer data — grid_map owns storage
   and rolling, the helper owns the evidence math.
3. **Inverted projection + waterline-contact + occlusion (#10 I; shadow)** — per segmentation frame,
   scan image columns for the waterline contact (`is_waterline_contact_pixel`) and back-project **only
   the contact** onto the water surface (z=0 in `map_tide`) → `hit`; raytrace the free cells between the
   camera and the contact → `miss` (clearing); leave the occluded region beyond the contact unobserved
   (decays, never marked). Needs a **new** pure helper (contact-only projection + camera→contact
   free-space raytrace) — `project_obstacle_pixels` emits points for *all* obstacle pixels and is the
   wrong primitive; it stays unchanged for its other consumer `segments_to_pointcloud.cpp` (reflex feed),
   confirmed unaffected. Folds in `fe337f6`; addresses #10 I. **z=0 is correct here**: the boat floats
   and `map_tide` z=0 tracks the water surface, so projecting onto z=0 *is* projecting onto the real
   surface at any tide — #10 H's vertical-drift worry does not apply to a floating platform (residuals:
   TF stamp consistency, already handled via `lookupTransform` at the image stamp; wave heave, a small
   transient handled by `base_link_level`). Note this on #10. *Quantization:* on the 128×96 mask,
   contact-range uncertainty is range-dependent (one pixel-row ≈ meters near max range), so
   decay/threshold defaults must tolerate a contact jittering across a few cells without re-creating
   flicker — covered by a tuning note + test.
4. **Multi-source subscriptions (consolidate 4→1)** — accept a list of camera sources (segmentation +
   camera_info + per-source `maximum_range`); all feed the one shared world-frame buffer (so an obstacle
   reinforces the same cell across cameras — the handoff fix). A single source remains valid (back-compat).
5. **Thread-safety (#10 E/F)** — guard the shared buffer and per-source camera models against the N
   concurrent camera/camera_info callbacks; fix the `camera_model_` / `count_x_` races.
6. **Live-tunable parameters via a param callback (#10 K)** — register an `OnSetParametersCallbackHandle`
   so the tuning scalars change at runtime (`ros2 param set`) without a stack restart, for on-water
   tuning: **decay half-life, hit/miss log-odds increments, lethal threshold, per-source max range**.
   The callback **validates** inputs (reject NaN/negative/out-of-range → `successful=false`) so a
   fat-fingered set can't silently poison the buffer interpretation, applies under the buffer lock
   without re-taking it (a new threshold reinterprets the accumulated buffer immediately; new
   decay/increments take effect next update). Source list / topics stay configure-time. A test asserts
   the callback applies valid changes AND rejects invalid ones (the current layer silently ignores
   runtime params — the #10-K failure mode).
7. **Tests (#10 L)** — occupancy buffer (hit raises / miss+decay lowers / threshold / decays to prior
   over time / **evidence survives a sequence of fractional-meter `grid_map::move()` shifts**),
   waterline-contact + free-space-miss + occlusion, two-source fusion reinforcing one world cell, param
   validate+apply.
8. **Docs + config follow-up (this is what finalizes #19)** — document new params; the dependent seafloor
   `nav2_params.yaml` migration (4 blocks → 1 multi-source) **is the PR that finalizes #19**, since it
   activates cross-camera fusion and is where the handoff AC is verified (bag/sim). Note IzzyBoat parity.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | Rewrite: multi-source subs, shared log-odds buffer as a `grid_map` float layer rolled via `move()` and converted to the master via `grid_map_costmap_2d`, inverted projection w/ waterline-contact + free-space-miss + occlusion, decay, thread-safe, validating param callback, `isClearable()` revisit |
| `sea_surface_segmentation/src/occupancy_buffer.hpp` | New pure log-odds update math (hit/miss/decay/threshold) over a grid_map layer; grid_map owns storage + rolling |
| `sea_surface_segmentation/src/segments_projection.hpp` | `is_waterline_contact_pixel` (done) + **new** contact-only / free-space-miss projection helper; do NOT repurpose `project_obstacle_pixels` (shared with `segments_to_pointcloud.cpp`) |
| `sea_surface_segmentation/package.xml` | Add deps: `grid_map_core`, `grid_map_ros`, `grid_map_costmap_2d` (rosdep-resolvable; already a workspace dep via camp) |
| `sea_surface_segmentation/test/test_occupancy_buffer.cpp` | New: log-odds + decay + `grid_map::move()` fractional-shift persistence tests |
| `sea_surface_segmentation/test/test_segments_projection.cpp` | Waterline tests (done) + contact/miss/occlusion + fusion cases |
| `sea_surface_segmentation/CMakeLists.txt` | `find_package(grid_map_*)` + link; register new test |
| `sea_surface_segmentation/README*` / params doc | Document sources, decay, thresholds, runtime-tunable params |
| seafloor `echoboat_project11/config/nav2_params.yaml` *(follow-up PR — finalizes #19)* | 4 instances → 1 multi-source block (local_costmap only; global_costmap doesn't use the layer) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | Phased atomic commits, each buildable + single-source-correct; phase 1 (grid_map buffer + rolling) is independently splittable |
| A change includes its consequences | Tests, param docs, shared-header consumer check, new dependency, and config migration tracked |
| Test what breaks | Fractional-shift persistence, decay, fusion, flicker rejection, concurrency, param validate/apply |
| Only what's needed | Log-odds justified by the noise data; **grid_map reused** (already a workspace dep) rather than hand-rolling buffer rolling math |
| Human control & transparency | Tuning scalars live-tunable (validated) at runtime; decay model decided (log-odds) |
| Capture decisions | Plan + PR record rationale; #10 flagged this as ADR-like (no project ADR mechanism → captured here) |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | nav2 multi-source idiom + validating `OnSetParametersCallbackHandle`; buffer rolling via `grid_map::move()` (idiomatic float-grid primitive) + `grid_map_costmap_2d` bridge; target Rolling; no deviation needing its own ADR |
| 0001 — Adopt ADRs | Watch | Design rationale + decay-model decision captured in plan/PR (no project-level ADR mechanism in this repo) |
| 0002 — Worktree isolation | Yes | Work on `feature/issue-19` layer worktree |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `SeaSurfaceLayer` params/behavior | seafloor `nav2_params.yaml` (4→1 multi-source, local_costmap only) | Follow-up PR — **finalizes #19** |
| ... | IzzyBoat config parity (#120/#181) | Noted for follow-up |
| Add `grid_map` dependency | `package.xml` + `CMakeLists.txt` (rosdep-resolvable; camp precedent) | Yes |
| `segments_projection.hpp` (shared header) | confirm `segments_to_pointcloud.cpp` (reflex feed) unaffected by the new helper | Yes |
| The layer | package README / param docs | Yes |
| Layer logic | regression tests (#10 L) | Yes |
| (reconcile #10) | D, I, E/F, L addressed; **H = non-issue for a floating platform** (note on #10, no code change) | Note in PR + comment on #10 |

## Open Questions

- **Decay model — DECIDED: log-odds** (2026-05-27). Tuning scalars live-reconfigurable (phase 6).
- **#19 closure / sequencing — DECIDED** (2026-05-27, review-plan): this perception PR is _Part of_ #19
  (capability); the seafloor config PR activates fusion, verifies the handoff, and finalizes #19.
- **#10 H tide z-drift — RESOLVED** (2026-05-27): non-issue for a floating boat (z=0 = water surface,
  tracked by `map_tide`); no `plane_z` work needed; leave a note on #10.
- **Default tuning** — decay half-life, hit/miss increments, lethal threshold: propose conservative
  defaults, tune on water (non-blocking).

## Estimated Scope

This perception PR (atomic commits per phase, each buildable + single-source-correct) + a dependent
`seafloor_echoboat_project11` config PR that **finalizes #19**. Phase 1 (grid_map-backed log-odds buffer
+ `move()`-rolling + fractional-shift test) is the core, independently reviewable piece — de-risked by
leaning on grid_map rather than hand-rolled buffer math. If the single perception PR proves unwieldy in
review, split phase 1 out first. Bookkeeping handled here; no per-ticket overhead for the user.
