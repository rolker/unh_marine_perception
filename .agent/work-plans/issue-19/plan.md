# Plan: Re-architect SeaSurfaceLayer as a single multi-camera fused occupancy layer (temporal persistence + decay)

## Issue

https://github.com/rolker/unh_marine_perception/issues/19

_Revised 2026-05-27 per review-plan (`c72005e`, verdict changes-requested) — see phases 1/3/6/8,
Consequences, and Open Questions._

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

Phased, atomic commits. The layer becomes **multi-source-capable but back-compatible with a single
source**, so this perception PR can land and be reviewed without the config flip. Per review-plan
(`c72005e`): **this perception PR is _Part of_ #19** (delivers the capability); the dependent seafloor
config-migration PR activates cross-camera fusion, verifies the handoff AC on bag/sim, and **closes
#19**. *Per-phase invariant:* every commit leaves the layer buildable AND single-source-correct.

1. **Decouple the buffer from the rolling-window resize (#10 D)** — stop wiping on every origin shift.
   The persistent buffer is the phase-2 **log-odds float array**, so nav2's `Costmap2D::updateOrigin()`
   (built for `uint8` cost cells) cannot be reused directly: implement an integer-cell partition-copy
   of the float array keyed to the origin delta, filling newly-exposed border cells with the **prior**
   (log-odds 0); `resizeMap` only on a true size/resolution change. Fractional-meter shifts quantize to
   the cell grid — test that evidence survives a *sequence* of sub-cell origin shifts and lands in the
   correct world cell, not just one clean cell-aligned shift. Load-bearing prerequisite. Fixes #10 D.
   (Also revisit `isClearable()` — hard-coded `false` today — now that the layer clears via decay.)
2. **Pure log-odds occupancy buffer** — new header `occupancy_buffer.hpp` (mirrors the
   `segments_apply.hpp` / `segments_projection.hpp` pure-logic + unit-test pattern): per-cell log-odds,
   `hit`/`miss` updates, time-based decay toward prior, threshold→LETHAL. Fully unit-tested.
3. **Inverted projection + waterline-contact + occlusion (#10 I; _partial_ H; shadow)** — per
   segmentation frame, scan image columns for the waterline contact (`is_waterline_contact_pixel`) and
   back-project **only the contact** to the ground plane → `hit`; raytrace the free cells between the
   camera and the contact → `miss` (clearing); leave the occluded region beyond the contact unobserved
   (decays, never marked). This needs a **new** pure helper (contact-only projection + camera→contact
   free-space raytrace) — `project_obstacle_pixels` emits points for *all* obstacle pixels and is the
   wrong primitive; it stays unchanged for its other consumer `segments_to_pointcloud.cpp` (the reflex
   feed), which must be confirmed unaffected. Folds in `fe337f6`; fixes #10 I. **Only _partially_
   addresses #10 H**: marking at the waterline removes the tall-obstacle smear, but the contact still
   back-projects to a fixed `plane_z=0`; the `map_tide` vertical-tide-drift component of H is orthogonal
   — parameterize `plane_z` from the tide frame, or defer it with a note on #10. *Quantization:* on the
   128×96 mask, contact-range uncertainty is range-dependent (one pixel-row ≈ meters near max range), so
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
   over time / **survives a sequence of fractional-meter origin shifts**), waterline-contact +
   free-space-miss + occlusion, two-source fusion reinforcing one world cell, param validate+apply.
8. **Docs + config follow-up (this is what closes #19)** — document new params; the dependent seafloor
   `nav2_params.yaml` migration (4 blocks → 1 multi-source) **is the PR that closes #19**, since it
   activates cross-camera fusion and is where the handoff AC is verified (bag/sim). Note IzzyBoat parity.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | Rewrite: multi-source subs, shared persistent log-odds buffer (float-array origin shift, not `updateOrigin`/resize), inverted projection w/ waterline-contact + free-space-miss + occlusion, decay, thread-safe, validating param callback, `isClearable()` revisit |
| `sea_surface_segmentation/src/occupancy_buffer.hpp` | New pure log-odds buffer (hit/miss/decay/threshold + origin-shift) |
| `sea_surface_segmentation/src/segments_projection.hpp` | `is_waterline_contact_pixel` (done) + **new** contact-only / free-space-miss projection helper; do NOT repurpose `project_obstacle_pixels` (shared with `segments_to_pointcloud.cpp`) |
| `sea_surface_segmentation/test/test_occupancy_buffer.cpp` | New: log-odds + decay + fractional-shift persistence tests |
| `sea_surface_segmentation/test/test_segments_projection.cpp` | Waterline tests (done) + contact/miss/occlusion + fusion cases |
| `sea_surface_segmentation/CMakeLists.txt` | Register new test |
| `sea_surface_segmentation/README*` / params doc | Document sources, decay, thresholds, runtime-tunable params |
| seafloor `echoboat_project11/config/nav2_params.yaml` *(follow-up PR — closes #19)* | 4 instances → 1 multi-source block (local_costmap only; global_costmap doesn't use the layer) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | Phased atomic commits, each buildable + single-source-correct; phase 1 is the riskiest, independently splittable piece |
| A change includes its consequences | Tests, param docs, shared-header consumer check, and config migration tracked |
| Test what breaks | Fractional-shift persistence, decay, fusion, flicker rejection, concurrency, param validate/apply |
| Only what's needed | Log-odds justified by the noise data; no marking modes beyond what the data shows |
| Human control & transparency | Tuning scalars live-tunable (validated) at runtime; decay model decided (log-odds) |
| Capture decisions | Plan + PR record rationale; #10 flagged this as ADR-like (no project ADR mechanism → captured here) |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes | nav2 multi-source idiom + validating `OnSetParametersCallbackHandle`; target Rolling; float-buffer origin shift is custom (uint8 `updateOrigin` doesn't fit) — documented, no deviation needing its own ADR |
| 0001 — Adopt ADRs | Watch | Design rationale + decay-model decision captured in plan/PR (no project-level ADR mechanism in this repo) |
| 0002 — Worktree isolation | Yes | Work on `feature/issue-19` layer worktree |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `SeaSurfaceLayer` params/behavior | seafloor `nav2_params.yaml` (4→1 multi-source, local_costmap only) | Follow-up PR — **closes #19** |
| ... | IzzyBoat config parity (#120/#181) | Noted for follow-up |
| `segments_projection.hpp` (shared header) | confirm `segments_to_pointcloud.cpp` (reflex feed) unaffected by the new helper | Yes |
| The layer | package README / param docs | Yes |
| Layer logic | regression tests (#10 L) | Yes |
| (reconcile #10) | D, I, E/F, L addressed; **H only partially** (shadow, not tide z-drift) | Note in PR + comment on #10 |

## Open Questions

- **Decay model — DECIDED: log-odds** (2026-05-27). Tuning scalars live-reconfigurable (phase 6).
- **#19 closure / sequencing — DECIDED** (2026-05-27, review-plan): this perception PR is _Part of_ #19
  (capability); the seafloor config PR activates fusion, verifies the handoff, and closes #19.
- **#10 H tide z-drift — OPEN (non-blocking)**: parameterize `plane_z` from the tide frame in this work,
  or defer as a separate #10 sub-item? (lean: parameterize if cheap, else defer with a #10 note.)
- **Default tuning** — decay half-life, hit/miss increments, lethal threshold: propose conservative
  defaults, tune on water (non-blocking).

## Estimated Scope

This perception PR (atomic commits per phase, each buildable + single-source-correct) + a dependent
`seafloor_echoboat_project11` config PR that **closes #19**. Phase 1 (origin-decoupled float log-odds
buffer + its fractional-shift drift test) is the riskiest, independently reviewable piece — if the
single perception PR proves unwieldy in review, split phase 1 out first. Bookkeeping handled here; no
per-ticket overhead for the user.
