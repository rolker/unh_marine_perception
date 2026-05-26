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
- [ ] Bag-fixture size budget — trimmed mcap slice must fit ≪ 50 MB; fall back to LFS or external test-data dir if it doesn't.
- [ ] Hull-floor → waterline offset — default `projection_plane_z=0.0` is a safety-polygon-scale approximation; #170 decides whether to set the param or add a `waterline_level` frame.
- [ ] File the rung-1 follow-up issue ("Coarse mask-fills-danger-region trigger") before this PR merges so the deferral is tracked.
- [ ] Confirm topic-name collision is solved by standard ROS remap in the boat launch (no node change needed).
