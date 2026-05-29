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
log-odds (prior-shifted at `obstacle_prob_min`; see Implementation Notes), bound
recovery with an asymmetric clamp, and emit a graded cost ramp instead of binary
lethal. Single PR; all knobs runtime-tunable.

## Approach (single PR — implemented)

Per the 2026-05-29 discussion: **one PR**, **graded cost output** (not binary),
**all knobs runtime-tunable** for on-water tuning, and the **reflex/Collision-Monitor
path left untouched** (it is known-good; the #186 bug is costmap-only).

1. **Graded per-observation evidence** — `OccupancyObservation` gains a signed
   `double log_odds` (additive; keeps `obstacle` for the #23 shared API). A new
   `pixel_log_odds(R, obstacle_prob_min, max_evidence_step)` returns a
   **prior-shifted, capped logit** of the red channel:
   `clamp(log((R+ε)/(255−R+ε)) − log(prob_min/(1−prob_min)), ±max_step)`.
   The prior shift puts the zero-crossing at `obstacle_prob_min` (not 0.5), so a
   buoy where water is marginally more likely (`P_obs > prob_min`) still banks
   **positive** evidence — the actual #186 fix. Defaults: R=200→+0.85, R=120→+0.50,
   R=20→−0.85.
2. **Graded gate (costmap path only)** — `project_observations_inverse` replaces
   the argmax `is_obstacle_pixel`/`is_waterline_contact_pixel` with
   `is_obstacle_ish(px) = R ≥ obstacle_prob_min·255` (same waterline-contact
   geometry), and emits `log_odds` for contact (+) and green-water (−) cells; sky
   /ambiguous still skipped.
3. **Buffer accumulates the signed increment** — `OccupancyBuffer::accumulate(pos,
   log_odds)` (replaces `hit()/miss()`), clamped to **asymmetric**
   `[clear_floor=−2, obstacle_clamp=+5]` (recovery cleared→lethal ~8→~4 frames).
4. **Graded occupancy + cost** — `occupancyAt()` maps log-odds to `−1` (no opinion:
   unobserved or ≤`free_threshold`) / `1…99` ramp / `100` (≥`lethal_threshold`).
   New `cost_mapping.hpp::occupancy_to_cost()` maps that to nav2 cost: `−1`
   (untouched) / soft `1…252` / `LETHAL=254` (stays below `INSCRIBED=253`).
5. **Graded output everywhere** — layer `updateCosts` stamps the ramp; the layer
   publishes the graded occupancy grid; `SeaSurfaceRelayLayer` forwards the full
   gradient (not just `≥100`). Both still only **ADD** (no-opinion never clears).
6. **All params runtime-tunable** — `obstacle_prob_min`, `max_evidence_step`,
   `obstacle_clamp`, `clear_floor`, `free_threshold`, `lethal_threshold`,
   `decay_half_life_s` validate-then-apply via `onParametersSet`.
7. **Tests** — buffer accumulate/`occupancyAt`/`validate`; `cost_mapping`
   boundaries; graded accumulator expectations; projection log-odds signs.
   **69 tests pass.**

## Files Changed

| File | Change |
|------|--------|
| `include/.../segments_projection.hpp` | `OccupancyObservation.log_odds`; `pixel_log_odds()`; graded gate in `project_observations_inverse` (argmax predicates kept for the reflex/forward paths) |
| `include/.../occupancy_buffer.hpp` | `accumulate()`; asymmetric clamp params; `occupancyAt()`; reworked `validate()`; snapshot ctor for off-lock publish |
| `include/.../cost_mapping.hpp` (NEW) | `occupancy_to_cost()` — the one nav2-cost mapping, shared by layer + relay |
| `include/.../occupancy_accumulator.hpp` | `AccumulateParams` gains `obstacle_prob_min`/`max_evidence_step`; uses `accumulate()` |
| `src/sea_surface_layer.cpp` | new param block + live snapshot; graded `updateCosts`; graded publish; `onParametersSet` |
| `src/sea_surface_relay_layer.cpp` | forward the full graded gradient via `occupancy_to_cost` |
| `config/README.md` | param table + behavior for both layers |
| `CMakeLists.txt`, `test/*` | `cost_mapping` link; updated/added tests |

_Not changed (intentionally): `src/segments_to_pointcloud.cpp` and the argmax
predicates — the reflex Collision-Monitor path is known-good and out of scope._

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Relay gradient, `OccupancyObservation` consumers (#23 offline core), config docs, tests all in plan |
| Only what's needed | Two-timescale rebuild rejected (reflex owns dynamics); reflex-gate change dropped (CA known-good); graded output kept (user wants varying cost) |
| Test what breaks | Log-odds tails (ε guard), asymmetric clamp, ramp boundaries, water-negative, cost-mapping boundaries — all unit-tested (69 pass) |
| Improve incrementally | Single PR by user direction (test window); knobs runtime-tunable so behavior is refined on-water rather than across PRs |

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
| flat hit/miss → increment | tests asserting fixed ±0.85/−0.40 | Yes — accumulator/buffer tests updated to graded values |
| binary lethal → graded ramp | relay (was `≥100`), global planner cost weights, inflation ordering | Relay + layer emit the gradient; **planner `cost_penalty`/`CostCritic` live in the echoboats/seafloor nav2 config repo — soft costs are inert for routing until set there** (see Open Questions) |

## Open Questions

- **Planner cost weighting (already wired — tune, don't enable)** — verified
  against `seafloor_echoboat_project11/.../nav2_params.base.yaml`: the global
  `SmacPlannerHybrid` already has `cost_penalty: 10.0`, so the relayed graded
  gradient **does** bias global routing today (no config change needed to
  activate). The controller is `CrabbingPathFollower` (PID path tracker) which
  ignores costmap cost entirely — so avoidance is planner-side + the reflex
  Collision Monitor, not the controller. Real follow-up is *tuning*: (1)
  `cost_penalty: 10` is aggressive and may deflect routes around low-confidence
  soft cells; (2) `inflation_radius: 150` m means every now-lethal cell (incl.
  newly-marking weak buoys) creates a 150 m standoff bubble — validate routing
  isn't over-conservative in a buoy field.
- **On-water tuning** — defaults (`obstacle_prob_min=0.35`, `max_evidence_step=0.85`,
  `clear_floor=−2`, `free_threshold=0`, `lethal_threshold=1`) are provisional; all
  are runtime-reconfigurable. Tune `obstacle_prob_min` first (dim-buoy sensitivity)
  against the #186 bag / live segmentation.
- **#23 coordination** — `OccupancyObservation` gained a field (additive); coordinate
  with the in-flight offline-core export (PR #24) on merge order.

## Estimated Scope

Single PR (#25), implemented. Follow-up: on-water tuning of `cost_penalty` /
`inflation_radius` vs. the new gradient in the echoboats/seafloor nav2 config
(tuning, not enabling — routing already honors costs).

## Implementation Notes

- **Prior-shifted logit, not plain `log(R/(255−R))`.** The original plan/issue used
  the unshifted logit (zero-crossing at P=0.5). That would still drop a buoy whose
  pixels are red-present-but-water-dominant (`G > R`, i.e. `P_obs < 0.5`) — the
  exact #186 case. Shifting the zero-crossing to `obstacle_prob_min` lets such
  pixels bank positive evidence, and makes `obstacle_prob_min` the headline
  on-water sensitivity knob. `max_evidence_step` caps a single frame (flicker
  rejection; ~2 frames to lethal at defaults).
- **Off-lock graded publish** uses a read-only `OccupancyBuffer` snapshot ctor so
  `occupancyAt` remains the single source of the ramp (no duplicate mapping).
