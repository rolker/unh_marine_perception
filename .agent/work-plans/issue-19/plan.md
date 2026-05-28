# Plan: Re-architect SeaSurfaceLayer as a single multi-camera fused occupancy layer (temporal persistence + decay)

## Issue

https://github.com/rolker/unh_marine_perception/issues/19

_Revised 2026-05-27 per review-plan (`c72005e`, changes-requested) and follow-up design calls:
log-odds buffer backed by **grid_map** (resolves the float-buffer rolling risk via `GridMap::move()`);
#10 H tide-drift confirmed a **non-issue for a floating platform** and dropped._

_Revised again 2026-05-27 after offline review on the 2026-05-26 bag (episode #21): (1) **phase 3
projection corrected forward → inverse** (the committed `project_observations` was a regression vs the
pre-#19 inverse layer — forward leaves gaps/speckle and never reaches lethal); (2) **new global-costmap
propagation** so the planner can route around buoys — `SeaSurfaceLayer` publishes its contribution and a
thin `SeaSurfaceRelayLayer` feeds the global costmap (publish-and-relay, mirroring the S57 producer/
consumer pattern). See the new section + Open Questions._

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
3. **Projection: INVERSE (cell→pixel) + waterline-contact + occlusion (#10 I; shadow)** — for each
   world cell in the buffer window, project the cell's ground point (z=0 in `map_tide`) into the camera
   and classify it by the pixel it lands on: a cell whose pixel is its column's **waterline contact**
   (`is_waterline_contact_pixel`) → `hit`; a cell seeing **water** → `miss` (free where water is
   positively observed); a cell behind the contact (projecting to an above-contact body pixel) → left
   **unobserved** (occluded; decays, never marked). This is the **inverse** direction the pre-#19 layer
   used (iterate cells, sample the covering pixel): one large/distant pixel fills **every cell in its
   footprint** (no gaps), and the search is **range-bounded to the window** (a near-horizon pixel can't
   smear past the window edge). `project_obstacle_pixels` stays unchanged for its other consumer
   `segments_to_pointcloud.cpp` (reflex feed). **z=0 is correct here** (floating boat; `map_tide` z=0
   tracks the surface — #10 H non-issue). Add a per-camera **FOV frustum cull** so iterating window
   cells × cameras stays cheap at ~10 Hz × 4 cameras.

   > **Regression caught 2026-05-27 — this phase must change.** The first phase-3 implementation
   > (`f10666c`) drifted to **forward** per-pixel projection (`project_observations`: each pixel → one
   > world point → one cell), despite this phase's "inverted projection / #10-I loop-inversion" intent.
   > Forward leaves **gaps** at range, makes **lone jittery hits** that never reach the 2-hit lethal
   > threshold, and forward-projects near-horizon pixels to far cells (the speckle "ring"). Offline A/B
   > on the 2026-05-26 bag (episode #21, 2.0 m/s approach→STOP) measured **forward 0.1 % lethal vs
   > inverse 3.2 %** (live fused costmap 6.8 %); inverse produces a coherent obstacle footprint that
   > tracks the live costmap through the maneuver while forward stays incoherent speckle. **Fix: restore
   > inverse projection feeding the buffer**, keeping contact-only marking (the shadow fix lived in the
   > inverse loop in `fe337f6`, so both are compatible). Verified offline by the `bag_to_costmap_video`
   > forward-vs-inverse mosaic before the boat. **Note:** the layer **currently deployed on bizzy**
   > (4 instances, pre-#19, via the bizzy overlay) is **inverse**, so the live costmap in the A/B already
   > reflects inverse segmentation — the forward rewrite would regress vs the behaviour running today.
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

## Global-costmap propagation — planner buoy-avoidance (new, 2026-05-27)

**Problem surfaced during the #21 review.** The controller (`marine_nav_crabbing_path_follower::CrabbingPathFollower`)
does **not** consult the costmap, and the global planner (`SmacPlannerHybrid`) plans on the **global**
costmap — which carries charted obstacles (`s57_layer`) + inflation but **no segmentation**. So an
uncharted buoy the cameras detect never reaches the planner; today the local costmap (where the
sea-surface layer lives) is effectively **operator-display-only** for planning. For the planner to route
around buoys, the detection must reach the **global** costmap. (`local_costmap` and `global_costmap` are
separate `Costmap2DROS` instances — in `controller_server` and `planner_server` — with independent layer
plugin instances; a layer in one is not in the other.)

**Decision — publish-and-relay (mirrors the existing S57 producer/consumer pattern).** `s57_grids`
already produces chart `grid_map`s once and both costmaps' `s57_layer` instances subscribe; copy that:
- The local `SeaSurfaceLayer` **publishes its own contribution** (the grid_map log-odds buffer,
  thresholded) on a dedicated topic — cheap, since the buffer is already a `grid_map::GridMap`. It must
  publish the **layer's contribution, NOT the fused local master** (that would double-count `s57_layer`
  and import local's 150 m inflation into global).
- A new **thin `SeaSurfaceRelayLayer`** (subscribe-and-rasterize, like `s57_layer` minus dataset
  discovery) is added to the **global** costmap, ordered **before** `inflation_layer` so the planner
  inflates the buoys. (Stock `nav2_costmap_2d::StaticLayer` composes poorly with a **rolling** global
  costmap — `global_costmap` is `rolling_window: true` — so a small custom relay layer is safer.)
- Projection + persistence run **once** (in the local layer); both costmaps consume the result. No
  second projection, no second buffer.

**Tradeoff (accepted).** The producer is coupled to `controller_server`'s lifecycle (restart → global
loses buoys until republish). Promotable to a standalone producer node later (full S57-style split)
without changing the relay side.

**Scope.** This is **new work beyond the original #19 split** and is cross-repo (new relay layer here +
global-costmap config in seafloor), but **all tracked under #19** (decided 2026-05-27 — no separate issue).

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_layer.cpp` | Rewrite: multi-source subs, shared log-odds buffer as a `grid_map` float layer rolled via `move()` and converted to the master via `grid_map_costmap_2d`, **INVERSE (cell→pixel)** projection w/ waterline-contact + free-space-miss + occlusion + FOV frustum cull, decay, thread-safe, validating param callback, `isClearable()` revisit, **publish the buffer's contribution as a `grid_map`/`OccupancyGrid` topic** (for the global relay) |
| `sea_surface_segmentation/src/occupancy_buffer.hpp` | New pure log-odds update math (hit/miss/decay/threshold) over a grid_map layer; grid_map owns storage + rolling |
| `sea_surface_segmentation/src/segments_projection.hpp` | `is_waterline_contact_pixel` + per-column contact map (image-space primitives the inverse loop samples); do NOT repurpose `project_obstacle_pixels` (shared with `segments_to_pointcloud.cpp`). **Drop the forward `project_observations` from the layer's path** (keep/relocate any reuse for the offline tool) |
| `sea_surface_segmentation/` — new `SeaSurfaceRelayLayer` | New thin costmap layer: subscribe to the published contribution grid and rasterize into the costmap. Added to the **global** costmap before inflation. Mirrors `s57_layer`'s subscribe-and-rasterize. |
| `sea_surface_segmentation/package.xml` | Add deps: `grid_map_core`, `grid_map_ros`, `grid_map_costmap_2d` (rosdep-resolvable; already a workspace dep via camp) |
| `sea_surface_segmentation/test/test_occupancy_buffer.cpp` | New: log-odds + decay + `grid_map::move()` fractional-shift persistence tests |
| `sea_surface_segmentation/test/test_segments_projection.cpp` | Waterline tests (done) + contact/miss/occlusion + fusion cases |
| `sea_surface_segmentation/CMakeLists.txt` | `find_package(grid_map_*)` + link; register new test |
| `sea_surface_segmentation/README*` / params doc | Document sources, decay, thresholds, runtime-tunable params |
| seafloor / bizzy instance config *(follow-up PR — finalizes #19)* | The 4 `SeaSurfaceLayer` instances are **already live** in bizzy's local costmap via `unh_echoboats_project11/bizzyboat_project11/config/nav2_overlay.yaml` (re-enabled by seafloor **#18, closed** — running the pre-#19 **inverse** code). Migrate local: 4 instances → 1 multi-source `SeaSurfaceLayer`. Add `SeaSurfaceRelayLayer` to the **global** costmap (before `inflation_layer`) — gets buoys in front of the planner. Note IzzyBoat parity. |

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
| Layer publishes its contribution | new topic; `SeaSurfaceRelayLayer` consumes it in the **global** costmap (before inflation) | New scope (publish-and-relay) |
| Global costmap gains buoys | seafloor `nav2_params.yaml` **global_costmap** plugins (+relay layer); planner now routes around detections | Cross-repo (seafloor) |
| Producer coupled to `controller_server` | if it restarts, global loses buoys until republish; promote to standalone node if it bites | Accepted tradeoff |

## Open Questions

- **Decay model — DECIDED: log-odds** (2026-05-27). Tuning scalars live-reconfigurable (phase 6).
- **#19 closure / sequencing — DECIDED** (2026-05-27, review-plan): this perception PR is _Part of_ #19
  (capability); the seafloor config PR activates fusion, verifies the handoff, and finalizes #19.
- **#10 H tide z-drift — RESOLVED** (2026-05-27): non-issue for a floating boat (z=0 = water surface,
  tracked by `map_tide`); no `plane_z` work needed; leave a note on #10.
- **Default tuning** — decay half-life, hit/miss increments, lethal threshold: propose conservative
  defaults, tune on water (non-blocking). Revisit *after* the inverse-projection fix, not before.
- **Projection direction — DECIDED: inverse (cell→pixel)** (2026-05-27). The committed forward
  `project_observations` is a regression vs the pre-#19 inverse layer; offline A/B on the 2026-05-26
  bag confirmed inverse fills footprints where forward leaves speckle. Phase 3 restores inverse.
- **Buoys to the planner — DECIDED: publish-and-relay** (2026-05-27). Local `SeaSurfaceLayer` publishes
  its contribution; a thin `SeaSurfaceRelayLayer` feeds the global costmap (before inflation). Producer
  stays in-layer for now (coupled to `controller_server`); standalone node is a later option.
- **Global relay scope — DECIDED: all in #19** (2026-05-27, user). No separate issue; the relay layer +
  seafloor global config land under #19 alongside the inverse fix and local migration.

## Implementation Notes

- **grid_map CMake gotcha (phase 1):** `grid_map_core`'s extras inject
  `-DEIGEN_*_PLUGIN="grid_map_core/eigen_plugins/..."` via **global `add_definitions`** at
  `find_package` time, so *every* TU compiled afterward needs grid_map_core's include dir — not just
  targets that link `grid_map_core::grid_map_core` (else unrelated targets like
  `segments_to_pointcloud` fail with "FunctorsPlugin.hpp: No such file"). The legacy
  `${grid_map_core_INCLUDE_DIRS}` var is empty under the modern target export; pull the path from the
  imported target (`get_target_property(... INTERFACE_INCLUDE_DIRECTORIES)`) and `include_directories()`
  it globally. The layer-integration phase must keep this.
- **Free-space clearing (phase 3):** **water-as-free** — a cell that projects to a water pixel gets a
  `miss`. Conservative (only clears cells the camera *positively observes as water*, not whole rays),
  and in the inverse loop it falls out of the same per-cell classification as the hit/occlusion cases.
- **Layer verification (phase 3):** the buffer is unit-tested; the live layer wiring (TF→pose, rolling,
  projection, threshold-to-master) is verified offline by the `bag_to_costmap_video` tool. Today it
  renders a two-row mosaic — top: 4 segmentation tiles (port·fwd·stbd·aft); bottom: live boat costmap |
  OURS (inverse). An earlier four-panel version added a NEW-forward panel for the A/B that surfaced the
  forward-projection regression on the 2026-05-26 bag (episode #21); the forward panel has been dropped
  since the A/B is settled, but the diff lives in git history if it ever needs reproducing. The forward
  `project_observations` still exists in `segments_projection.hpp` for the reflex-pointcloud consumer;
  it will be removed from the layer's projection path in the phase-3 implementation commit. The layer
  itself has no unit test, so this offline render is the gate before the boat.
- **Inverse cost (phase 3):** iterating window cells × cameras is heavier than forward per-pixel; bound
  it with an early behind-camera reject + a per-camera FOV frustum cull. The buffer window (±half-extent)
  naturally caps range, so the far-horizon speckle ring cannot form.

## Estimated Scope

This perception PR (atomic commits per phase, each buildable + single-source-correct) + a dependent
`seafloor_echoboat_project11` config PR that **finalizes #19**. Phase 1 (grid_map-backed log-odds buffer
+ `move()`-rolling + fractional-shift test) is the core, independently reviewable piece — de-risked by
leaning on grid_map rather than hand-rolled buffer math. If the single perception PR proves unwieldy in
review, split phase 1 out first. Bookkeeping handled here; no per-ticket overhead for the user.

**Added 2026-05-27:** phase 3 now restores **inverse** projection (was a forward-projection regression),
and the **global-costmap propagation** (publish-and-relay: layer publisher + `SeaSurfaceRelayLayer`) is
new, cross-repo scope (with seafloor) **but all tracked under #19** — no separate issue. The
`bag_to_costmap_video` offline tool is the verification harness; it now ships the inverse-only mosaic
(forward A/B is settled and dropped from the tool — diff in git history). Sibling diagnostic tracked
separately as [#21](https://github.com/rolker/unh_marine_perception/issues/21).
