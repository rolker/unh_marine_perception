---
issue: 19
---

# Issue #19 — Re-architect SeaSurfaceLayer as a single multi-camera fused occupancy layer (temporal persistence + decay)

## Issue Review
**Status**: complete
**When**: 2026-05-27 10:37 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #19
**Comment**: https://github.com/rolker/unh_marine_perception/issues/19#issuecomment-4555511506
**Scope verdict**: needs-splitting

Issue is well-written as a design/tracking spec (evidence-backed motivation, nav2-idiomatic design,
acceptance criteria, surfaced decay-model decision) and is in the right repo. The implementation is
too large for one PR and overlaps the #10 umbrella (D, I, H, L, E/F) — both need handling before/at
plan-task. ROS 2 convention alignment (ADR-0008) is positive: adopts nav2 ObstacleLayer multi-source
idiom.

### Actions
- [ ] Cross-link #10 in the issue and declare which items #19 closes (D matchSize-nuke, I loop-inversion; address H map_tide z=0; satisfies L tests; folds E/F threading into shared-buffer design)
- [ ] plan-task: decompose into sequenced/stacked PRs — (a) decouple internal grid from rolling origin (#10 D, prerequisite for any persistence), (b) consolidate 4 instances → 1 multi-source single-buffer (behavior-preserving), (c) add persistence + decay, (d) loop-inversion (#10 I, likely folded into a)
- [ ] Make shared-buffer thread-safety an explicit design requirement (N camera + camera_info callbacks → one buffer); resolves #10 E/F
- [ ] Address #10 H (map_tide z=0 / vertical-drift assumption) in the world-frame buffer design
- [ ] Settle the decay-model open question (log-odds vs hit/miss+time-decay) before plan-task locks the design; record the choice in the issue
- [ ] Tests targeting persistence/decay, multi-camera fusion, and concurrency (satisfies #10 L)
- [ ] Track cross-repo config consequence: seafloor_echoboat_project11 nav2_params.yaml (4 blocks → 1 multi-source) + IzzyBoat parity (#120/#181)

## Plan Authored
**Status**: complete
**When**: 2026-05-27 11:24 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-19/plan.md` at `4cd2020`
**PR**: https://github.com/rolker/unh_marine_perception/pull/20 (`[PLAN]` prefix)
**Phases**: single PR, 8 phased atomic commits (+ 1 dependent seafloor config-migration PR)

Plan reconciles the review-issue findings: #10 items D/I/H/E/F/L folded into the rewrite; one
cohesive perception PR (back-compat single-source) so the code lands before the config flip;
log-odds chosen as the recommended decay model. fe337f6 (waterline patch) folds into phase 3.

### Open questions
- [ ] Decay/clearing model — log-odds (recommended) vs hit/miss+time-decay; confirm/veto at review-plan
- [ ] Sequencing — perception capability PR first (back-compat), then seafloor config-migration PR to activate cross-camera fusion? (recommended yes)
- [ ] Default tuning — decay half-life, hit/miss increments, lethal threshold (propose defaults, tune on water)

## Plan Review
**Status**: complete
**When**: 2026-05-27 11:35 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) (same agent identity — performed via fresh-context sub-agent for independence)

**Plan**: `.agent/work-plans/issue-19/plan.md` at `4179d4e`
**PR**: https://github.com/rolker/unh_marine_perception/pull/20
**Verdict**: changes-requested

Architecture (single fused world-frame log-odds buffer, mark-at-waterline, temporal decay) is sound;
ADR-0008 mechanism choices correct. Sub-agent verified global_costmap does NOT use the layer, so the
migration is local_costmap-only (plan scope correct). Four must-fixes below.

### Findings
- [ ] (must-fix) Float log-odds buffer can't reuse nav2 `updateOrigin()` (built for uint8 cost cells); phase 1 must spec the float-array shift + prior-fill on exposed cells + a fractional-shift drift test — the load-bearing prerequisite — plan.md phase 1
- [ ] (must-fix) `Closes #19` pairs with a single-source PR, but #19's signature handoff AC needs the multi-source shared buffer (activated only by the later seafloor config PR) → AC unverifiable in this PR; don't auto-close #19 or carry the handoff AC into the config PR — **surface to user** — plan.md Estimated Scope / Open Questions
- [ ] (must-fix) "fixes #10 H" overstated: waterline-contact addresses shadow-smear, NOT the `map_tide` z=0 vertical-drift (orthogonal; contact still back-projects to fixed plane_z=0). Downgrade to "partially addresses H"; parameterize plane_z from tide frame or defer — plan.md phase 3
- [ ] (must-fix) `project_obstacle_pixels` doesn't drop in (emits points for ALL obstacle pixels; no contact/water-miss/occlusion). Phase 3 needs a new helper (contact-only + free-space/miss raytrace camera→contact) and must confirm `segments_to_pointcloud.cpp` (other consumer) is unaffected — plan.md phase 3
- [ ] (suggestion) 128×96 mask quantization → contact-range uncertainty is range-dependent; decay/threshold defaults must tolerate a contact jittering across cells at long range (else re-creates flicker) — plan.md phase 3/tuning
- [ ] (suggestion) Phase-6 param callback: validate (reject NaN/negative/out-of-range, return successful=false), no re-lock — plan.md phase 6
- [ ] (suggestion) State a per-phase invariant: each commit leaves the layer buildable AND single-source-correct — plan.md Approach
- [ ] (suggestion) Reconsider `isClearable()` (hard-coded false today) under a decaying-occupancy model — plan.md phase 1/5

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 13:12 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-19 at `76c0c38`
**Mode**: pre-push
**Depth**: Standard (reason: ~390-line C++ change to a perception costmap-layer package)
**Must-fix**: 3 | **Suggestions**: 3

Phase 1 (grid_map occupancy buffer). cppcheck clean. Claude + Copilot adversarial both independently
flagged the lethal_threshold lower-bound gap (cross-model confirmation). Architecture/logic otherwise
confirmed correct by both (NaN-as-prior, decay vectorization, grid_map move semantics, row/col order,
body-pixel NO_INFORMATION safety).

### Findings
- [ ] (must-fix) `validate()` accepts non-positive `lethal_threshold` → 0 makes a single hit lethal (defeats flicker rejection); negative makes a water `miss` read lethal (safety inversion). Require `>0`. — `occupancy_buffer.hpp:114`
- [ ] (must-fix) `decay()` advances `last_decay_s_` before the `dt<=0` guard + uses `<0` as seed sentinel → backward-time over-decays next call; negative `now_s` re-seeds forever. Use `seeded_` flag, don't advance clock on non-positive dt. — `occupancy_buffer.hpp:69`
- [ ] (must-fix) Validation test only checks the upper threshold bound; add negative-threshold rejection + backward-time decay cases. — `test_occupancy_buffer.cpp`
- [ ] (suggestion) Bounds-guard `is_waterline_contact_pixel` at entry for future callers. — `segments_projection.hpp:64`
- [ ] (suggestion) Rename/recomment "FractionalShifts" test: grid_map snaps move() to integer cells, so it proves no-drift + exposed-unobserved, not sub-cell residual. — `test_occupancy_buffer.cpp:95`
- [ ] (suggestion) `setParams`/ctor trust caller to validate; phase-6 param callback is the enforced gate (deferred). — `occupancy_buffer.hpp:99`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 14:04 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested → addressed

**Branch**: feature/issue-19 at `3f618a1`
**Mode**: pre-push
**Depth**: Standard (reason: phase-3 layer rewrite — safety costmap layer, TF + projection + rolling)
**Must-fix**: 1 | **Suggestions**: 3

Phase 3 (layer consumes occupancy buffer). cppcheck clean (lone hit is a cross-TU false positive).
Claude + Copilot adversarial both confirmed the risky geometry CORRECT (TF cam→world direction
matches the reflex feed; project_observations contact/occlusion; rolling-vs-resize split = #10-D fix;
decay/threshold cycle; LETHAL-only master stamping). Both independently flagged the camera_model_ race
(cross-model) — fixed in `3f618a1`.

### Findings
- [x] (must-fix) camera_model_ read/deref outside lock vs locked write → torn read/UAF; fixed via immutable-snapshot (fresh model swap + shared_ptr copy under lock) — `sea_surface_layer.cpp` (3f618a1)
- [ ] (suggestion, phase 5) `current_` never set true → affects LayeredCostmap::isCurrent(); pre-existing, fold into phase-5 thread-safety/lifecycle — `sea_surface_layer.cpp`
- [ ] (suggestion) `maximum_range_` (100) advertises reach the buffer window can't fill; observations past the window are projected then dropped — clamp to half-extent or document — `sea_surface_layer.cpp`
- [ ] (suggestion) remaining camera_model_/count_x_ thread-safety hardening still owed to phase 5 (this fix covers camera_model_; count_x_ read in updateBounds still unlocked) — `sea_surface_layer.cpp`

## Offline Review: projection regression + global-costmap design
**Status**: investigation complete; plan revised
**When**: 2026-05-27 20:41 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Branch**: feature/issue-19 (local WIP: `bag_to_costmap_video` mosaic + inverse prototype + per-camera counters — not yet committed/reviewed)
**Artifacts**: `~/data/logs/analysis/2026-05-27/mosaic_ep21_fwd_vs_inv/` — mosaic on the 2026-05-26 bag, episode #21 (13:17:41 EDT, ~2.0 m/s approach → STOP)

Built an offline `bag_to_costmap_video` tool that replays recorded OAK segmentation through the phase-3
pipeline and renders a 4-panel mosaic (4-cam segmentation | live boat costmap | NEW forward | NEW inverse),
boat-centred and aligned in `map_tide`. Used it to A/B the projection direction on real water imagery.

### Findings
- [ ] (must-fix → plan) **Projection regression: forward, not inverse.** The committed phase-3
  `project_observations` is forward per-pixel (pixel → one world point → one cell), despite the plan's
  inverse / #10-I loop-inversion intent and the pre-#19 layer's inverse (cell→pixel) loop. Forward →
  gaps at range, lone jittery hits that never reach the 2-hit lethal threshold, and a far-horizon speckle
  "ring". Measured ep#21: **0.1 % lethal (forward) vs 3.2 % (inverse)** vs 6.8 % (live chart costmap);
  inverse fills a coherent footprint that tracks the live obstacle through the maneuver. **Plan phase 3
  updated to restore inverse projection** (keep contact-only; add an FOV frustum cull for cost).
- [ ] (architecture → plan) **Buoys never reach the planner.** The controller (`CrabbingPathFollower`)
  doesn't read the costmap; the global planner (`SmacPlannerHybrid`) plans on the global costmap, which
  has charts + inflation but no segmentation. So the local sea-surface layer is operator-display-only for
  planning. **Decision recorded:** publish-and-relay — `SeaSurfaceLayer` publishes its grid_map
  contribution; a new thin `SeaSurfaceRelayLayer` feeds the global costmap (before inflation), mirroring
  the `s57_grids`/`s57_layer` producer/consumer pattern. Projection runs once. New cross-repo scope.
- [x] (no-op) The "blank aft tile" in the first mosaic was a thumbnail misread, **not** a bug:
  per-camera counters show all four cameras fed the buffer (0 TF failures, ~4.4–4.7 M obs each). Kept the
  counters + tile-store-before-TF as defensive hardening in the tool.

### Open
- [x] Global relay scope — **decided: all in #19** (no separate issue, 2026-05-27).
- [ ] Commit/review the `bag_to_costmap_video` tool (mosaic + inverse prototype) — still local WIP.
- [ ] seafloor remaining: migrate local 4→1 multi-source (the #19 finalizer) + add the global relay layer.

**Correction (2026-05-27):** seafloor #18 is **closed/done** — it already re-enabled the layer as 4
direction-named instances live in `bizzyboat_project11/config/nav2_overlay.yaml`, running the **pre-#19
inverse** code. So segmentation IS deployed (local), and the LIVE panel in the A/B reflects inverse
segmentation (chart + inverse sea-surface + inflation), not charts alone. This reframes the regression:
the #19 forward `project_observations` regresses vs the **inverse layer currently running on bizzy**.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-27 23:04 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**Branch**: feature/issue-19 (pre-push delta only — vs `origin/feature/issue-19`; phases 1+3 already reviewed)
**Mode**: pre-push
**Depth**: Standard (reason: ~625 lines, 5 files, governance-touching plan + non-trivial C++ tool)
**Static**: cppcheck clean; xmllint clean
**Adversarial**: Claude + Copilot (both, cross-confirmed on findings 2 + 3 + 6)
**Must-fix**: 1 | **Suggestions**: 5

### Findings
- [x] (must-fix) Inverse path marks sky (blue-dominant) as `miss` — overclear in the OURS panel; classify water=miss / sky=skip / contact=hit. — `tools/bag_to_costmap_video.cpp:201` (addressed in `13742d5` tool commit)
- [x] (suggestion) No arg validation — `--res`/`--window-m`/`--render-dt`/`--max-range` accept ≤0 → divide-by-zero/invalid Mat. — `tools/bag_to_costmap_video.cpp:231` (addressed)
- [ ] (suggestion) `render_live` assumes identity grid origin orientation + `bizzy/map_tide` frame — warn if violated. — `tools/bag_to_costmap_video.cpp:129` (deferred, defensive)
- [ ] (suggestion) No staleness guard on `live_costmap`; once latched, stale grid renders forever. — `tools/bag_to_costmap_video.cpp:303` (deferred, defensive)
- [x] (suggestion) `obs_fed` counter semantics changed (inverse cells, not forward pixels); rename or legend. — `tools/bag_to_costmap_video.cpp:393` (renamed to `obs_inv_cells` / `cells_fed_to_buffer`)
- [x] (suggestion) Stale prose around the dropped forward A/B — plan + CMakeLists comment. — `plan.md`, `CMakeLists.txt:107` (addressed)

## Phase 3 Implemented (Pre-Push Review)
**Status**: complete (review); changes committed and pushed for overnight Copilot PR review
**When**: 2026-05-28 00:11 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approved-with-suggestions

**Branch**: feature/issue-19 (pre-push delta only — vs `origin/feature/issue-19`)
**Mode**: pre-push
**Depth**: Standard (~242 lines, 4 files, touches production layer code)
**Static**: cppcheck clean (one cross-TU false-positive filtered)
**Tests**: 54 / 54 passing (incl. 4 new for `project_observations_inverse`)
**Adversarial**: Claude (full report); Copilot **skipped — timed out at 300s** without producing findings. No cross-model confirmation this round.
**Must-fix**: 0 | **Suggestions**: 5

### Scope
- `src/segments_projection.hpp`: new shared helper `project_observations_inverse` (inverse cell→pixel; contact-only; sky-skip; range-gate; behind-camera early-out).
- `src/sea_surface_layer.cpp`: `segmentsCallback` switched from forward → inverse; iteration centred on camera-origin XY, half-extent = `maximum_range_`. `resolution_` snapshot added under `costmap_mutex_` (closes pre-existing race vs `matchSize()`).
- `test/test_segments_projection.cpp`: 4 new tests (all-water, all-sky-skipped, contact-vs-body, max-range).
- `tools/bag_to_costmap_video.cpp`: removed local copy; calls the shared helper. Smoke-render produced identical per-camera cell counts (5,848,135 / 6,404,148 / 5,306,146 / 6,096,452) before vs after the move.

### Findings (Claude Adversarial)
- [x] (suggestion, close-to-must-fix) **Test-coverage gap**: all 4 new tests are nadir + `cx=cy=cam_origin` + single-row stacks, so they don't exercise the production call shape (pitched cameras, `pc[2] ≤ 0` behind-camera reject, off-centre iteration, real occlusion). Add a pitched + off-centre integration test. — `test/test_segments_projection.cpp:528-588` (addressed `e263a21`)
- [ ] (suggestion) AABB half-extent = `maximum_range_` over-iterates by ~21% (corners exceed Euclidean gate); add a TODO at the call site for the eventual FOV cone cull. — `sea_surface_layer.cpp:222`
- [ ] (suggestion) Unthrottled WARN on TF / cv_bridge failures — use `RCLCPP_WARN_THROTTLE` and include source frame_id + stamp. — `sea_surface_layer.cpp:236`
- [x] (suggestion, nit) `>=` vs `==` in contact classification — `==` matches forward helper's exact semantics. — `segments_projection.hpp:389` (addressed `939fd24`)
- [x] (suggestion) Add `std::isfinite(uv.x/uv.y)` guard before `std::lround` int-narrowing; matches forward helper's defensive style. — `segments_projection.hpp:384` (addressed `939fd24`)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-28 10:42 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: approve

**Branch**: feature/issue-19 (pre-push delta only — 3 commits vs `origin/feature/issue-19`)
**Mode**: pre-push
**Depth**: Standard (~82 lines insertions, 3 files, addresses outstanding Copilot inline review comments)
**Tests**: 55 / 55 passing
**Adversarial**: Claude sub-agent (fresh-context); Copilot to re-review post-push
**Must-fix**: 0 | **Suggestions**: 1 (nit) | **Nits**: 0

### Scope
Addresses the 4 outstanding Copilot inline review comments on PR #20:

- `9ca9407` — `sea_surface_layer.cpp`: add missing `<chrono>`, `<functional>`, `<mutex>`; `test_segments_projection.cpp`: add missing `<utility>` for `std::pair`. Compile-on-libstdc++-by-transitive-include → make explicit.
- `939fd24` — `segments_projection.hpp::project_observations_inverse`: align contact classification with forward sibling (`v >= contact_row[u]` → `v == contact_row[u]`); add `std::isfinite(uv.x/uv.y)` guard before `std::lround`. Functionally equivalent against `is_waterline_contact_pixel` (the contact is the lowest obstacle pixel with water directly below OR an auto-contact at the image bottom; any obstacle pixel below the recorded contact would extend an unbroken column to the bottom and displace the recorded contact lower — `>` branch unreachable). Defensive consistency, not behavior change.
- `e263a21` — `test_segments_projection.cpp`: new `PitchedOffCentreRejectsBehindAndOccludesBody` test. Camera at world (5, 3, 1.5) pitched 30° down looking +x, iteration centred at the camera's XY so the back half is behind. Asserts (a) zero behind-camera observations (pc[2]<=0 gate), (b) ≥1 contact-row hit under pitched optics, (c) observations cluster around camera XY (off-centre iteration), (d) every hit lands at x < 7.30 (contact-row reach + margin; body row 6 would land at x ≈ 7.60).

### Findings (sub-agent)
- Correctness of `==` equivalence claim verified by tracing `is_waterline_contact_pixel` and the contact-finding loop. The `contact_row[u] == -1` (no contact in column) case is also preserved: old form gated explicitly on `>= 0`; new form fails `v == -1` because `v ≥ 0` after the bounds check.
- `isfinite` guard is genuinely defensive — `pc[2] > 0` plus a well-posed pinhole won't produce non-finite, but `lround` of non-finite is UB; cheap correctness improvement.
- Pitched test bounds verified independently (contact ray hits at x ≈ 7.09; body row 6 / principal-point ray at x ≈ 7.60; `x < 7.30` exclusion ≈ 0.30 m on body side). Fails cleanly if `==` regresses to `>=` and body pixels start projecting.
- All four added standard headers are actually used (`std::chrono::seconds`, `std::bind`/`placeholders`, `std::lock_guard<std::mutex>`, `std::pair`).

- [ ] (suggestion, nit) Pitched test uses `EXPECT_LT` inside a per-observation loop; on regression, every offending observation fires its own message. `ASSERT_LT` would fail-fast and quiet the output. Style preference only; not addressed pre-push. — `test/test_segments_projection.cpp:670`

### Follow-ups (next session)
- Triage Copilot's post-push re-review on PR #20.
- Phase 5 carry-ins: AABB FOV-cone TODO (`sea_surface_layer.cpp:222`), throttled WARN (`sea_surface_layer.cpp:236`), `current_` lifecycle, `maximum_range_` (100 m) vs buffer window mismatch, remaining `count_x_` race.
- Phase 4 (multi-source consolidation), 6 (live-tunable param callback), 8 (docs + dependent seafloor config PR — the #19 finalizer).
- New scope per offline review: `SeaSurfaceRelayLayer` global-costmap publish-and-relay (buoys don't reach the planner today — Smac sees only chart+inflation).
- Sibling diagnostic issue [#21](https://github.com/rolker/unh_marine_perception/issues/21) (camera↔TF motion-consistency tool) remains captured but unimplemented.
