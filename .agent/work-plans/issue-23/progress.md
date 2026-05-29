---
issue: 23
---

# Issue #23 — Export sea_surface algorithm core (headers + bag→costmap driver) for reuse by offline tools

## Issue Review
**Status**: complete
**When**: 2026-05-28 22:00 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Issue**: #23
**Comment**: https://github.com/rolker/unh_marine_perception/issues/23#issuecomment-4569823420
**Scope verdict**: well-scoped

### Actions
- [ ] Make the extracted bag→costmap driver unit test a firm scope item (no test exercises the driver today; it's inline in `bag_to_costmap_video.cpp`'s `main`). Promoting it to public API warrants coverage — treat "if not already covered" as firm.
- [ ] After moving the three headers to `include/sea_surface_segmentation/`, update the `target_include_directories(... ${CMAKE_CURRENT_SOURCE_DIR}/src)` lines for `test_segments_apply`, `test_segments_projection`, `test_occupancy_buffer`, `bag_to_costmap_video`, and the `sea_surface_layer` lib to the public include dir. Leave `test_frame_id_resolver` on `src/` (that header is not moving).
- [ ] Add `install(DIRECTORY include/ ...)` + `ament_export_include_directories(include)` so headers actually install (ADR-0008).
- [ ] Fix the stale CMake comments that call the three headers "private to the layer / lives next to its .cpp in src/".
- [ ] Keep `frame_id_resolver.hpp` private — verified it is not a transitive dependency of the export set; no action beyond confirming.
- [ ] Verified no out-of-package consumers of the three headers (workspace-wide grep) — promotion to public API is safe; plan-task need not re-litigate.

## Plan Authored
**Status**: complete
**When**: 2026-05-28 22:21 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-23/plan.md` at `e74d78b`
**PR**: https://github.com/rolker/unh_marine_perception/pull/24 (`[PLAN]` prefix)
**Phases**: single

### Open questions
- [x] Driver boundary — **resolved with Roland (2026-05-28): pure `cv::Mat mask_rgb8` + resolved geometry in; decode + TF per-caller.** Also corrected the segmentation-is-ffmpeg premise (segmentation is raw rgb8 `Image`; ffmpeg is the camera `image_raw` stream, display-only). Follow-up: fix that bullet in `marine_perception_tools#1`.
- [x] Header/function name — **resolved: `occupancy_accumulator.hpp` / `accumulate_frame()`** (accumulates into the `OccupancyBuffer`, not a nav2 costmap); projection/window knobs grouped into `AccumulateParams`.

## Plan Review
**Status**: complete
**When**: 2026-05-28 22:47 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) — fresh-context sub-agent (`Plan` type); same agent identity as author, run with isolated context for independence

**Plan**: `.agent/work-plans/issue-23/plan.md` at `2838864`
**PR**: https://github.com/rolker/unh_marine_perception/pull/24
**Verdict**: changes-requested → findings folded into plan (commit follows this entry)

### Findings
- [x] (must-fix) CMake export omits OpenCV — `segments_projection.hpp` directly `#include <opencv2/core.hpp>` and exposes `cv::Mat`/`cv::Vec3b` publicly; add `OpenCV` to `ament_export_dependencies`. — `plan.md` step 5
- [x] (must-fix) grid_map_core `-DEIGEN_*_PLUGIN` global-include hack (`CMakeLists.txt:21-27`) follows `occupancy_buffer.hpp` downstream; a header-only consumer hits the same plugin-header failure. Plan must verify `ament_export_dependencies(grid_map_core)` suffices for a clean external build (smoke-test consumer) or document the workaround. — `plan.md` step 5 + new verification step
- [x] (must-fix) Driver test must cover `move()`/`decay()` ordering, not just project/hit-miss — `decay()` is wall-clock + first-call-seeded, `move()` rolls the window; a single-frame test leaves the move→decay→ingest sequence (the part a refactor reorders) unverified. — `plan.md` step 6
- [x] (fix) Plan wrongly claimed all three headers are `namespace sea_surface_segmentation`; `segments_apply.hpp` is `namespace sea_surface_layer` (a nav2 `Costmap2D` bridge). Corrected so the implementer doesn't "fix" the namespace and break the layer + its test. — `plan.md` Context + step 1
- [x] (suggestion) `ament_export_targets(export_sea_surface_layer HAS_LIBRARY_TARGET)` — the `EXPORT` set is installed but never ament-exported (pre-existing). Not required for header-only consumption (the tuner links nothing from this pkg); noted as optional completeness. — `plan.md` step 5
- [x] (suggestion) `AccumulateParams` defaults must mirror `project_observations_inverse`'s (`plane_z=0.0`, `min_grazing_angle_deg=0.0`) or the struct owns them and the function drops its defaults — avoid two drifting default sources. — `plan.md` step 3
- [x] Prior review-issue action items (4) all reflected in the plan; the export-completeness gap (above) was missed by review-issue and is now closed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-05-29 06:47 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (1 must-fix + 3 suggestions found and fixed in-session before push)

**Branch**: feature/issue-23 — reviewed `4b6ff31`, fixes in `05b6674`
**Mode**: pre-push
**Depth**: Standard (reason: ~440-line change, new public API export consumed cross-repo + build-config)
**Must-fix**: 1 | **Suggestions**: 3
**Specialists**: static analysis (ament_cpplint; cppcheck skipped — slow-version guard), governance, plan-drift, Claude adversarial (fresh-context), Copilot adversarial (ran but could not resolve file paths → no usable findings)

### Findings
- [x] (must-fix) `ament_export_dependencies(OpenCV)` exported a dep not declared in `package.xml`, yet the exported headers directly `#include <opencv2/core.hpp>` — added `<depend>libopencv-dev</depend>`. — `package.xml`
- [x] (suggestion) move→decay ordering not actually locked (move/decay commute on the overlap); added `DecayPrecedesFreshIngest` to lock decay-before-ingest. — `test/test_occupancy_accumulator.cpp`
- [x] (suggestion) `accumulate_frame` return-count doc overstated "applied" — reworded (counts produced obs incl. out-of-window, matching the tool's original tally). — `occupancy_accumulator.hpp`
- [x] (suggestion) dropped the dead `EXPORT export_sea_surface_layer` keyword (we deliberately don't `ament_export_targets` the layer). — `CMakeLists.txt`
- [x] Static analysis: cpplint `legal/copyright` + `runtime/int (long)` + `line_length`/`runtime/string` are all pre-existing or non-package-conventions (existing headers carry no copyright; existing code uses `long`; none of the line-length/string hits are in new files) — non-actionable.
- [x] Plan drift: none — plan synced with the two implementation pivots (no `ament_export_targets`; global `include_directories(include)`).
- [x] Behavioral equivalence with the original inline driver verified exact (incl. per-camera count semantics); `AccumulateParams` aggregate-init correct.

Post-fix: 67 tests, 0 failures; downstream consumer build still clean.

## Integrated Review
**Status**: complete
**When**: 2026-05-29 08:45 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #24 at `edb2cf6`
**Sources**: 2 (Copilot review @ `edb2cf6`, Local Review (Pre-Push) @ `4b6ff31`→`05b6674`); CI rollup
**Cross-source confirmations**: 0
**CI**: all-pass (copilot-pull-request-reviewer: success)

### Findings
- (none) Copilot reviewed 13/15 files and generated no inline comments. All four pre-push local-review findings (1 must-fix + 3 suggestions) were fixed before push and re-verified against current code at `edb2cf6`: `libopencv-dev` depend present (`package.xml:21`), `DecayPrecedesFreshIngest` test present, accumulator return-doc reworded, dead `export_sea_surface_layer` CMake keyword absent.

### False positives
- (none)
