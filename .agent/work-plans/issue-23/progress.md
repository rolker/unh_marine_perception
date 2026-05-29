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
- [ ] Driver extraction boundary: shared `accumulate_frame()` takes a decoded `cv::Mat mask_rgb8` (decode per-tool) vs. a `sensor_msgs::Image` — recommend cv::Mat; confirm before implementing.
- [ ] Header/function name: `costmap_accumulator.hpp` / `accumulate_frame()` vs. the issue's "bag→costmap driver" wording (processes one frame, not a bag).
