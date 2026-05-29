# Plan: Export sea_surface algorithm core (headers + bag→costmap driver) for reuse by offline tools

## Issue

https://github.com/rolker/unh_marine_perception/issues/23

## Context

`sea_surface_segmentation` keeps its algorithm core in private headers under
`src/` (`occupancy_buffer.hpp`, `segments_projection.hpp`, `segments_apply.hpp`),
reached only via per-target `target_include_directories(... /src)`. They already
live in `namespace sea_surface_segmentation`. The per-frame bag→costmap logic
(re-centre → decay → `project_observations_inverse` → `hit`/`miss`) is inlined in
`tools/bag_to_costmap_video.cpp`'s `main` (~L347–364). The forthcoming
`sea_surface_tuner` (`marine_perception_tools`, ui_ws) must run the **real**
algorithm without dragging in Qt or the GUI. This is a mechanical export +
extract-function refactor — **no algorithm change**. Verified: no out-of-package
consumer `#include`s these headers today (workspace-wide grep), and the package
currently has **no** `ament_export_include_directories`/`ament_export_dependencies`.

## Approach

1. **Move the three core headers** to `include/sea_surface_segmentation/`
   (`occupancy_buffer.hpp`, `segments_projection.hpp`, `segments_apply.hpp`).
   `frame_id_resolver.hpp` stays in `src/` — verified it is not included by any
   of the three, nor by the driver.
2. **Rewrite the 8 include sites** from bare `"occupancy_buffer.hpp"` to the
   ROS 2-conventional `"sea_surface_segmentation/occupancy_buffer.hpp"` (etc.):
   `sea_surface_layer.cpp` (×2), `segments_to_pointcloud.cpp`,
   `bag_to_costmap_video.cpp` (×2), `test_occupancy_buffer.cpp`,
   `test_segments_projection.cpp`, `test_segments_apply.cpp`.
3. **Extract the per-frame driver** into a new header-only
   `include/sea_surface_segmentation/costmap_accumulator.hpp` — an `inline`
   function `accumulate_frame(OccupancyBuffer&, const cv::Mat& mask_rgb8,
   camera_model, camera_origin, rotation, bx, by, stamp_s, max_range, res,
   half_extent, …)` doing move/decay/project/hit-miss. Boundary = **decoded
   `cv::Mat` in** (decode stays per-tool; see Open Questions). Header-only keeps
   parity with the other core headers — no new library target.
4. **Repoint `bag_to_costmap_video.cpp`** to call `accumulate_frame(...)` so the
   exporter and the future tuner share one path (drift prevention).
5. **CMake export plumbing**: add `install(DIRECTORY include/ DESTINATION
   include)`, `ament_export_include_directories(include)`, and
   `ament_export_dependencies(grid_map_core image_geometry nav2_costmap_2d)` so a
   downstream package gets the headers + transitive deps. Update the four
   `target_include_directories(... /src)` lines that feed the moved headers
   (`bag_to_costmap_video`, `test_segments_apply`, `test_segments_projection`,
   `test_occupancy_buffer`) to use `include/`; leave `test_frame_id_resolver` on
   `/src`. Fix the now-stale "private to the layer / lives next to its .cpp in
   src/" comments.
6. **Add a unit test for the extracted driver** (`test/test_costmap_accumulator.cpp`):
   feed a tiny synthetic mask + camera model + identity-ish pose into
   `accumulate_frame` and assert expected cells flip to obstacle/free in the
   buffer. Wire it into `BUILD_TESTING` like the other gtests.
7. **Build + test**: `./sensors_ws/build.sh sea_surface_segmentation` then
   `./sensors_ws/test.sh sea_surface_segmentation`; confirm the 3 pre-existing
   unit tests + plugin-load + launch test still pass and the new test passes.

## Files to Change

| File | Change |
|------|--------|
| `src/{occupancy_buffer,segments_projection,segments_apply}.hpp` | Move → `include/sea_surface_segmentation/` |
| `include/sea_surface_segmentation/costmap_accumulator.hpp` | New: inline `accumulate_frame()` extracted from the tool |
| `tools/bag_to_costmap_video.cpp` | Replace inline per-frame loop with `accumulate_frame()` call; fix 2 includes |
| `src/sea_surface_layer.cpp`, `src/segments_to_pointcloud.cpp` | Fix include paths (3 total) |
| `test/test_{occupancy_buffer,segments_projection,segments_apply}.cpp` | Fix include paths |
| `test/test_costmap_accumulator.cpp` | New unit test for the driver |
| `CMakeLists.txt` | `install(DIRECTORY include/)`, `ament_export_include_directories`/`_dependencies`, repoint test/tool include dirs, new gtest, fix stale comments |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Exports exactly the 3 core headers + 1 shared driver; `frame_id_resolver.hpp` confirmed not needed → stays private. |
| DRY / single source of truth | Exporter and tuner share `accumulate_frame()` — the explicit drift-prevention goal. |
| Test what breaks | New driver test makes the extracted code a firm regression target (none exists today); existing tests must still pass. |
| A change includes its consequences | CMake export plumbing + stale-comment fixes land in the same PR; no package README exists to update. |
| Improve incrementally | Single small PR, no behaviour change. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — Follow ROS 2 conventions | Yes | `include/<pkg>/` layout + `ament_export_include_directories` + `pkg/header.hpp` include spelling are the conventions; install the headers so the export is real. |
| 0002 — Worktree isolation | Yes | Work on `feature/issue-23` layer worktree (`sensors_ws` / `unh_marine_perception`). |

(No imagery-pipeline or sensor-driver ADR exists — `docs/decisions/` runs 0001–0014.)

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Header locations (private→public API) | Include sites (8), CMake include dirs (4 targets), stale CMake comments | Yes (steps 2, 5) |
| Add reusable public API (`accumulate_frame`) | Unit test for it | Yes (step 6) |
| Package now exports headers | `ament_export_include_directories`/`_dependencies` + `install(DIRECTORY include/)` | Yes (step 5) |
| Downstream `marine_perception_tools#1` | Can `find_package(sea_surface_segmentation)` + include the core after this lands | Out of scope (separate repo); unblocked by this PR |

## Open Questions

- **Driver extraction boundary.** The issue lists "decode rgb8" as part of the
  driver, but the tool decodes via `cv_bridge` (sensor_msgs::Image) while the
  tuner decodes ffmpeg (`marine_perception_tools#1`). Recommend the shared
  `accumulate_frame()` take an already-decoded **`cv::Mat mask_rgb8`** and keep
  decode per-tool — maximises reuse given the differing decode paths. Confirm
  before implementing (vs. a sensor_msgs::Image-in boundary).
- **Header name.** `costmap_accumulator.hpp` / `accumulate_frame()` vs. the
  issue's "bag→costmap driver" wording (the function processes one frame, not a
  bag). Minor; will use the per-frame name unless you prefer otherwise.

## Estimated Scope

Single PR.
