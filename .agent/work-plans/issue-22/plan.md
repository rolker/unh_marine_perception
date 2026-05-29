# Plan: sea_surface marking + costmap dynamics — accumulate per-pixel softmax log-odds

## Issue

https://github.com/rolker/unh_marine_perception/issues/22

## Context

The segmentation mask is a per-pixel **softmax** scaled to 255
(`sea_surface_segmentation.cpp:124–131`): `R=255·P(obstacle)`, `G=255·P(water)`,
`B=255·P(sky)`, `R+G+B=255`. The costmap discards this graded confidence: a strict
argmax gate (`segments_projection.hpp:44`) feeds a flat `hit(+0.85)`/`miss(−0.40)`
into the log-odds buffer. Marginal-red obstacles are scored as confident water and
*erode* accumulated evidence (never register — the #186 operator bug), and the
symmetric ±5 clamp over-damps recovery once a cell has saturated to water.

Fix: stop collapsing the softmax. Feed the buffer the real per-pixel occupancy
log-odds `Δ = clamp(log(R/(255−R)))`, bound recovery with an asymmetric clamp, and
(phase 2) emit a graded cost ramp instead of binary lethal.

## Approach

**Phase 1 — graded evidence + bounded recovery (safety/correctness, PR1)**

1. **`OccupancyObservation` carries evidence** — add a `double log_odds` field
   (keep `bool obstacle` for back-compat with the #23 offline core). Additive only.
2. **Compute per-pixel log-odds** in `project_observations_inverse` (and forward
   `project_observations`): for the waterline-contact and water cells, set
   `log_odds = clamp(log((R+ε)/(255−R+ε)))` with ε guarding `R∈{0,255}` → ±∞.
   Drop the argmax-only classification as the vote source.
3. **Buffer applies the increment** — `OccupancyBuffer::apply(pos, obs.log_odds)`
   instead of fixed hit/miss; `sea_surface_layer.cpp:408–414` passes it through.
4. **Asymmetric clamp** — split `clamp` into `obstacle_ceiling` (+5) and
   `clear_floor` (−2) in `OccupancyParams` + `validate()`; recovery cleared→lethal
   drops ~8→~4 frames.
5. **Reflex-gate consistency** — `segments_to_pointcloud` (reflex feed) thresholds
   `P(obs) = R/255 > τ` instead of strict argmax, so faint obstacles reach the live
   Collision Monitor too.
6. **Tests** — `test_occupancy_buffer` (asymmetric clamp, increment apply, tail
   clamp); `test_segments_projection` (log-odds at known RGB, ε guard, water
   negative).

**Phase 2 — graded cost ramp + relay gradient (planning, PR2)**

7. **Cost ramp** — map accumulated log-odds → cost in `updateCosts`:
   `≤clear_thresh`→0, ramp `1…252`, `≥lethal_thresh`→254, NaN→untouched.
8. **Relay full gradient** — `sea_surface_relay_layer.cpp` forwards the whole
   `1…254` range, not only `≥100`.
9. **Config + docs** — declare new params; update `config/README.md` + `config/`.
10. **Tests** — ramp boundaries; relay carries intermediate costs.

## Files to Change

| File | Change |
|------|--------|
| `src/segments_projection.hpp` | `OccupancyObservation.log_odds`; compute `log(R/(255−R))` in both projections |
| `src/occupancy_buffer.hpp` | apply arbitrary increment; asymmetric clamp params + `validate()`; (P2) `costAt()` ramp |
| `src/sea_surface_layer.cpp` | pass `obs.log_odds` to buffer; (P2) emit cost ramp; param declarations |
| `src/sea_surface_relay_layer.cpp` | (P2) forward full `1…254` gradient |
| `src/segments_to_pointcloud.cpp` (+`-reflex`) | reflex gate `P(obs)>τ` |
| `config/` + `config/README.md` | new params (`clear_floor`, `obstacle_ceiling`, `clear_thresh`, `lethal_thresh`, `reflex_tau`) |
| `test/test_occupancy_buffer.cpp`, `test/test_segments_projection.cpp` | new cases above |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Relay gradient, `OccupancyObservation` consumers (#23 offline core), config docs, tests all in plan |
| Only what's needed | Two-timescale rebuild explicitly rejected (reflex monitor owns dynamics); graded output is P2, not gold-plating |
| Test what breaks | Log-odds tails (±∞ guard), asymmetric clamp, ramp boundaries, water-negative all get unit tests |
| Improve incrementally | Stacked PRs: P1 fixes the operator bug + safety; P2 adds planning gradient |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 ROS2 conventions | Yes | New params declared/validated; costmap value semantics (252 vs 253 inscribed / 254 lethal) respected |
| 0013 progress.md vocabulary | Yes | Plan-authored entry appended |
| 0011 field mode | No | Repo origin is github.com (dev mode) |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `OccupancyObservation` struct | #23 offline bag→costmap core (consumes it — PR #24 open) | Yes — additive field; coordinate sequencing |
| flat hit/miss → increment | any test asserting fixed ±0.85/−0.40 | Yes — Phase 1 tests |
| binary lethal → ramp | relay (`≥100`), global planner cost weights, inflation ordering | Yes — Phase 2 + tuning note |

## Open Questions

- **PR split** — P1 (graded evidence + asymmetric clamp + reflex gate) then P2
  (graded ramp + relay)? Or single PR? P1 alone fixes the #186 bug.
- **Reflex `τ`** — safety call: how confident before a faint obstacle triggers a
  Collision Monitor stop vs. only soft-biasing the route? (surface to user)
- **Numeric defaults** — `clear_floor`, thresholds, `τ` are provisional pending the
  #186-bag R-distribution tuning; ship defaults that preserve current behavior at
  the margins, then tune.
- **#23 coordination** — sequence the `OccupancyObservation` change relative to the
  in-flight offline-core export (PR #24).

## Estimated Scope

Two stacked PRs (P1 safety/correctness, P2 planning gradient). P1 ~moderate; P2
touches relay + planner tuning.
