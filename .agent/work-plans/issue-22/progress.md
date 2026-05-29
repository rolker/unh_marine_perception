---
issue: 22
---

# Issue #22 — sea_surface marking + costmap dynamics: accumulate per-pixel softmax log-odds (drop the argmax gate), graded cost ramp, bounded recovery

## Plan Authored
**Status**: complete
**When**: 2026-05-29 10:11 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-22/plan.md` at `0513a92`
**PR**: https://github.com/rolker/unh_marine_perception/pull/25 (`[PLAN]` prefix)
**Phases**: 2 stacked PRs (P1 graded evidence + asymmetric clamp + reflex gate; P2 graded cost ramp + relay gradient)

### Open questions
- [ ] PR split: P1 (graded evidence + asymmetric clamp + reflex gate) then P2 (graded ramp + relay), or single PR? P1 alone fixes the #186 bug.
- [ ] Reflex `τ` safety call — confidence before a faint obstacle triggers a Collision Monitor stop vs. only soft-biasing the route (surface to user).
- [ ] Numeric defaults (`clear_floor`, thresholds, `τ`) provisional pending #186-bag R-distribution tuning.
- [ ] Coordinate the `OccupancyObservation` struct change with the in-flight offline-core export (#23 / PR #24).
