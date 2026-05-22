---
issue: 14
---

# Issue #14 — SeaSurfaceLayer segfault with multiple instances (distinct from #6, same symptom class)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-22 13:55
**By**: Claude Code Agent (Claude Opus 4.7)
**Verdict**: approved

**Branch**: `feature/issue-14` at `380aa6d`
**Mode**: pre-push
**Depth**: Standard (~300-line multi-file change with a fix + new module + tests)
**Must-fix**: 0 | **Suggestions**: 1 (declined with rationale)

### Findings
- [x] (must-fix) `GridEdgeBoundsDoesNotCrash` not a deterministic crash catch on small buffer — `test/test_segments_apply.cpp:55-71` — addressed by rename + clarifying comment + adding `HalfOpenBoundsRespectedOffOrigin` as load-bearing catch
- [x] (suggestion) Add off-origin window test — `test/test_segments_apply.cpp` — addressed by `HalfOpenBoundsRespectedOffOrigin`
- [x] (suggestion) Tighten test comments — `test/test_segments_apply.cpp` — addressed
- [ ] (suggestion declined) Add precondition asserts in `apply_segments_to_master` — `src/segments_apply.hpp:14-30` — nav2's layer code doesn't use them; contract enforced by `Layer::updateCosts` override signature + caller. Re-open as a follow-up issue if misuse becomes a recurring problem class.

## External Review
**Status**: complete
**When**: 2026-05-22 14:00
**By**: Claude Code Agent (Claude Opus 4.7)

**PR**: #16 — 1 review (Copilot bot), 0 valid, 0 false positives
**CI**: all-pass

### Actions
- [ ] Field verification on gabby (5+ cycle-the-launch with 4-layer config) before merge
- [ ] After merge: supersede `seafloor_echoboat_project11#21` workaround; close umbrella #10 items E/F as not-the-cause
