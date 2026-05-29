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
   `include/sea_surface_segmentation/occupancy_accumulator.hpp` — an `inline`
   function `accumulate_frame(OccupancyBuffer&, const cv::Mat& mask_rgb8,
   camera_model, camera_origin, rotation, boat_x, boat_y, stamp_s, AccumulateParams)`
   doing move/decay/project/hit-miss. The projection/window knobs (`max_range`,
   `res`, `half_extent`, `plane_z`, `min_grazing_angle_deg`) are grouped into an
   `AccumulateParams` struct so the call site isn't an 11-arg call. Boundary =
   **decoded `cv::Mat mask_rgb8` + already-resolved geometry in** — decode
   (`cv_bridge` for both tools; the projected segmentation is a raw rgb8 `Image`
   in either case) and TF lookup stay per-caller, keeping the exported header
   free of `cv_bridge` / `sensor_msgs` / `tf2`. Header-only keeps parity with the
   other core headers — no new library target.
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
6. **Add a unit test for the extracted driver** (`test/test_occupancy_accumulator.cpp`):
   feed a tiny synthetic `cv::Mat` mask + camera model + a hand-built rotation
   into `accumulate_frame` and assert expected cells flip to obstacle/free in the
   buffer (no `Image` message or TF buffer fixture needed — that's the payoff of
   the pure boundary). Wire it into `BUILD_TESTING` like the other gtests.
7. **Build + test**: `./sensors_ws/build.sh sea_surface_segmentation` then
   `./sensors_ws/test.sh sea_surface_segmentation`; confirm the 3 pre-existing
   unit tests + plugin-load + launch test still pass and the new test passes.

## Files to Change

| File | Change |
|------|--------|
| `src/{occupancy_buffer,segments_projection,segments_apply}.hpp` | Move → `include/sea_surface_segmentation/` |
| `include/sea_surface_segmentation/occupancy_accumulator.hpp` | New: inline `accumulate_frame()` + `AccumulateParams` extracted from the tool |
| `tools/bag_to_costmap_video.cpp` | Replace inline per-frame loop with `accumulate_frame()` call; fix 2 includes |
| `src/sea_surface_layer.cpp`, `src/segments_to_pointcloud.cpp` | Fix include paths (3 total) |
| `test/test_{occupancy_buffer,segments_projection,segments_apply}.cpp` | Fix include paths |
| `test/test_occupancy_accumulator.cpp` | New unit test for the driver |
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

## Decisions (resolved with Roland, 2026-05-28)

- **Driver boundary → pure `cv::Mat` + geometry in.** `accumulate_frame()` takes
  an already-decoded `cv::Mat mask_rgb8` and already-resolved geometry; decode
  and TF lookup stay per-caller. Exported header carries no `cv_bridge` /
  `sensor_msgs` / `tf2` dependency. Rationale: minimal coupling + a pure,
  trivially unit-testable function. (The "decode rgb8" in the issue means
  "operates on rgb8 layout," not "performs the decode.")
- **Segmentation is raw `Image`, not ffmpeg.** Correcting a premise in
  `marine_perception_tools#1`'s open items: the segmentation projected onto the
  costmap is a **raw rgb8 `sensor_msgs::Image`** (both tools `cv_bridge`-decode
  it identically). The ffmpeg encoding is on the **camera `image_raw` stream**,
  which the tuner decodes for **display only — never projected**. So there is no
  decode-path mismatch on the projection boundary, and #1 needs no ffmpeg decode
  for projection. **Follow-up:** fix that bullet in #1 when its planning starts.
- **Name → `occupancy_accumulator.hpp` / `accumulate_frame()`.** Precise: it
  accumulates one frame into the `OccupancyBuffer` (not a nav2 `Costmap2D` — that
  is `segments_apply`). Projection/window knobs grouped into `AccumulateParams`.

## Open Questions

- None — decisions above resolve the plan; ready for review-plan / implementation.

## Estimated Scope

Single PR.
