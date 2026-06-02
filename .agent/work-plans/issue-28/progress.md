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
