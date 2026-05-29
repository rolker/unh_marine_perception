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
