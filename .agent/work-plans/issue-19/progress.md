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
