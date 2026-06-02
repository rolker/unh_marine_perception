---
issue: 28
---

# Issue #28 — Segmentation output stamped with now() instead of frame capture time

## Plan Authored
**Status**: complete
**When**: 2026-06-02 11:52 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-28/plan.md` at `aa1e532`
**PR**: https://github.com/rolker/unh_marine_perception/pull/29 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] Anchoring strategy — RESOLVED: per-message re-anchor for all three stamping paths.
- [x] Camera-publisher frozen-anchor — RESOLVED: bundle into this PR (own commit), not a separate issue.
- [ ] Confirm this lands before the June 4 dev freeze (for June 15 survey).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-02 12:33 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved
**Branch**: feature/issue-28 at `c376154`
**Mode**: pre-push
**Depth**: Standard (reason: perception change feeding costmap/reflex; 2 packages)
**Must-fix**: 0 | **Suggestions**: 1 (addressed)

### Findings
- [x] (suggestion) segmentationCallback dereferenced a null-able NNData cast (pre-existing) — FIXED, added null guard matching sibling publishers — `sea_surface_segmentation.cpp:164`

### Adversarial verification
- Static: cpplint clean on changed files; uncrustify divergence is pre-existing base-style (local 0.78.1 version-drift false positive), not on added lines.
- Claude adversarial verified (incl. disassembling libdepthai_bridge.so): stamping algebra yields capture time (steady_base cancels), both converter paths honor setUpdateRosBaseTimeOnToRosMsg, init ordering safe (anchors set before addPublisherCallback), single-threaded per-camera queue (no data race), gtest proves stamp≠now() and isn't flaky.
