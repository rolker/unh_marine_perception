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
- [x] PR split → **single PR** (user direction, test window).
- [x] Reflex `τ` → **dropped**; CA/Collision-Monitor path left untouched (known-good, #186 bug is costmap-only).
- [ ] Numeric defaults provisional; all runtime-tunable — tune `obstacle_prob_min` first against the #186 bag / live segmentation.
- [ ] Coordinate the `OccupancyObservation` additive field with the in-flight offline-core export (#23 / PR #24).
- [ ] Planner-side `cost_penalty`/`CostCritic` config in the echoboats/seafloor nav2 repo to activate soft-cost routing (graded cost is inert for global routing until set).

## Implementation
**Status**: complete (build clean, 69 tests pass)
**When**: 2026-05-29 11:06 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

Graded softmax log-odds + graded cost ramp landed on `feature/issue-22`. Per-observation
evidence = prior-shifted capped logit of R (zero-crossing at `obstacle_prob_min`), asymmetric
accumulator clamp, `occupancyAt`→`occupancy_to_cost` ramp (−1 / 1..252 / LETHAL) in both the
layer and relay; all params runtime-reconfigurable. Reflex/CA path untouched. New
`cost_mapping.hpp`. Worked defaults: R=200→+0.85, R=120→+0.50, R=20→−0.85.

### Next
- [x] `/review-code` (pre-push) — done (see Local Review below).
- [ ] On-water tune; activate planner cost weighting (separate config repo).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-29 11:25 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: changes-requested → addressed
**Branch**: feature/issue-22 at `9ac617b`
**Mode**: pre-push
**Depth**: Deep (reason: safety-relevant costmap C++, concurrency/lifecycle, shared cross-repo API)
**Must-fix**: 2 (+1 promoted suggestion) | **Suggestions**: 2

### Findings
- [x] (must-fix) Layer + relay `updateCosts` overwrote master unconditionally — graded soft cost could downgrade another layer's INSCRIBED/LETHAL/unknown; switched to MAX-combine (updateWithMax semantics) — `sea_surface_layer.cpp:301`, `sea_surface_relay_layer.cpp:142` _(Copilot; Claude missed it)_
- [x] (must-fix) `obstacle_prob_min`/`max_evidence_step` validated at runtime but not at `onInitialize` — bad YAML → ±inf/NaN logit, silent layer disable; added init clamp+WARN — `sea_surface_layer.cpp:112` _(Claude + Copilot, independent)_
- [x] (promoted) `validate()` did not enforce `clear_floor <= free_threshold` → cleared water could publish positive cost; added guard — `occupancy_buffer.hpp:178` _(Copilot)_
- [ ] (suggestion) `bag_to_costmap_video` A/B tool still renders binary (logOdds vs lethal), not the graded ramp — won't visualize the new gradient; follow-up _(Claude)_
- [ ] (suggestion) cost ramp is coarse at the low end (occ=1→1, 2→4) — monotonic, below INSCRIBED; no action _(Claude)_

Static analysis: cpplint flags are pre-existing house style (passes repo pre-commit). Cross-model adversarial agreed on the init-validation gap; Copilot uniquely caught the cost-downgrade regression.
