---
issue: 17
---

# Issue #17 — Segmentation→sensor adapter for nav2 Collision Monitor (independent reflex safety feed)

## Issue Review
**Status**: complete
**When**: 2026-05-25 22:36 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Issue**: #17
**Comment**: https://github.com/rolker/unh_marine_perception/issues/17#issuecomment-4539339618
**Scope verdict**: well-scoped

### Actions
- [ ] In plan-task: acknowledge that `segments_to_pointcloud` already exists with map-frame output; decide extend-vs-sibling and justify, ensuring existing map-frame consumers (sea_surface_layer / costmap path) keep working.
- [ ] In plan-task: pin the robustness-ladder scope — is the coarse danger-region trigger in this PR or a follow-up?
- [ ] In plan-task: define test coverage using the May 21/22 `*_ffmpeg_seg` bags as regression fixtures and PR evidence.
- [ ] Default the reflex output frame to `bizzy/base_link_level`; rely on mru_transform (verified present in bizzy core launch) rather than subscribing to IMU; skip heave compensation.
- [ ] Coordinate the output topic name + `frame_id` handshake with `rolker/unh_echoboats_project11#170` once the plan is drafted.
- [ ] Consider an ADR in this repo capturing the reflex-safety-vs-costmap architecture choice (failure-stage independence) — non-obvious decision worth durable record.

## Plan Authored
**Status**: complete
**When**: 2026-05-25 22:50 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Plan**: `.agent/work-plans/issue-17/plan.md` at `0365a21`
**PR**: https://github.com/rolker/unh_marine_perception/pull/18 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] Bag-fixture size budget — **Resolved**: check in to git under `test/fixtures/`. No LFS, no external dir. Target ≪ 50 MB.
- [x] Hull-floor → waterline offset — **Resolved**: ship `projection_plane_z` with default 0.0; #170 absorbs the bias via polygon sizing. Escalate only if field tuning demands.
- [x] Rung-1 follow-up — **Resolved**: don't file now. Rung-2 in `base_link_level` should survive moderate extrinsic error; file rung-1 only if field testing surfaces a real gap.
- [x] Output topic name — **Resolved**: switch publisher from hardcoded `segmentation/pointcloud` to `~/pointcloud` (private namespace). Accepted cross-repo follow-ups: `unh_echoboats_project11` (izzy rviz + monitor) and `seafloor_echoboat_project11` (nav2 params). File these when #17 is reviewable so coordinated merge is possible.

## Plan Review
**Status**: complete
**When**: 2026-05-25 23:44 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context)) (fresh-context sub-agent)

**Plan**: `.agent/work-plans/issue-17/plan.md` at `a37bf6e`
**PR**: https://github.com/rolker/unh_marine_perception/pull/18
**Verdict**: approve-with-suggestions

### Findings
- [x] (must-fix) File targeting — Launch file added to change table + step 1; `name` + `target_frame` args specified (addressed in plan commit `fd98d47`).
- [x] (must-fix) ROS conventions — Output contract section added to step 1: `header.frame_id = target_frame`, `header.stamp = segments stamp`, rely on Collision Monitor `transform_tolerance` (addressed in plan commit `fd98d47`).
- [x] (suggestion) Consequences — Merge-ordering paragraph added to step 8: boat-config PRs first, then this PR; izzy nav2_params.yaml lines 158/207 cited (addressed in plan commit `fd98d47`).
- [x] (suggestion) ROS conventions — "Pre-existing non-fix items" subsection added: plain `Publisher`, missing `rclcpp_lifecycle` exec_depend, CMakeLists `target_link_libraries`/`ament_target_dependencies` mix (addressed in plan commit `fd98d47`).
- [x] (suggestion) File targeting — Step 5 specifies `mcap filter` recipe + four topics (`segmentation` Image, `CameraInfo`, `/tf`, `/tf_static`) (addressed in plan commit `fd98d47`).
- [x] (suggestion) Issue alignment — Step 5 explicit: "N-point threshold is a presence check, not a recall measurement" (addressed in plan commit `fd98d47`).
- [x] (suggestion) File targeting — CMakeLists row in change table notes "follow existing pattern (pre-existing mix; not converting in this PR)" (addressed in plan commit `fd98d47`).

## Implementation
**Status**: code complete; `/review-code` pending before push-to-review
**When**: 2026-05-26 00:57 -04:00
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Commits on `feature/issue-17`** (7 atomic commits since plan-review):
- `ff60582` — refactor: extract projection math into `segments_projection.hpp` (pure-logic helper in `src/`, mirroring `segments_apply.hpp` / `frame_id_resolver.hpp` pattern)
- `509f3a2` — feat: `target_frame` + `projection_plane_z` params; publisher switched to `~/pointcloud`
- `59c100d` — test: 11 GTests for the projection helper (mathematical cases + roll-honored)
- `ebdcd11` — feat(launch): `name` + `target_frame` launch args; defaults preserve legacy single-instance behavior
- `2a54671` — docs: `config/README.md` documents both operating modes
- `bf5c383` — test: launch_testing bag-replay regression + 3.8 MiB fixture from 2026-05-22 deployment
- `cc4b37d` — plan: align Files-to-Change with landed fixture layout (directory, not single .mcap)

**Test status**: 27/27 pass (13 pre-existing + 11 new GTest + 1 launch_test + 2 post-shutdown).

**Plan drifts captured inline during implementation**:
- Header lives in `src/` not `include/sea_surface_segmentation/` (matches existing pattern; see plan step 2 + Files-to-Change).
- Fixture is a directory (`*.mcap` + `metadata.yaml`), not a single file.
- `package.xml` test_depends enumerated.

**Diagnostic notes carried in commit messages** (worth surfacing for review-code):
- QoS mismatch caught during launch-test diagnosis (publisher best-effort vs default reliable subscriber → silent zero-message drop). Test now uses `qos_profile_sensor_data` explicitly.
- Launch test needs `use_sim_time=True` on the node + `--clock` on the bag player so the TF buffer accepts bag-era (May 2026) stamps instead of evicting them as stale relative to wall clock.

### Actions
- [ ] **`/review-code` pre-push** before merging or letting Copilot review. The skill catches static-analysis, governance, plan-drift, and adversarial findings while they're still cheap to fix locally — and matches the user's "internal review before Copilot" pattern.
- [ ] After review-code findings are addressed, the PR can be marked ready-for-review. Boat-config follow-up issues (per plan step 8: `unh_echoboats_project11` izzy rviz + monitor, `seafloor_echoboat_project11` nav2 params) should be filed at that point so coordinated merging is possible.
