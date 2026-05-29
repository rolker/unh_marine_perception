# Plan: Export sea_surface algorithm core (headers + bag→costmap driver) for reuse by offline tools

## Issue

https://github.com/rolker/unh_marine_perception/issues/23

## Context

`sea_surface_segmentation` keeps its algorithm core in private headers under
`src/` (`occupancy_buffer.hpp`, `segments_projection.hpp`, `segments_apply.hpp`),
reached only via per-target `target_include_directories(... /src)`.
`occupancy_buffer.hpp` and `segments_projection.hpp` are in `namespace
sea_surface_segmentation`; `segments_apply.hpp` is in `namespace
sea_surface_layer` (it's the nav2 `Costmap2D` bridge, not part of the projection
core) — the move preserves each header's existing namespace; do **not** unify
them (would break `sea_surface_layer.cpp` + `test_segments_apply.cpp`). The
per-frame bag→costmap logic
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
   `AccumulateParams` struct so the call site isn't an 11-arg call. The struct's
   defaults must mirror `project_observations_inverse`'s (`plane_z=0.0`,
   `min_grazing_angle_deg=0.0`) — or the struct owns them and the function keeps
   none — so there is a single source of default values, not two that can drift.
   Boundary =
   **decoded `cv::Mat mask_rgb8` + already-resolved geometry in** — decode
   (`cv_bridge` for both tools; the projected segmentation is a raw rgb8 `Image`
   in either case) and TF lookup stay per-caller, keeping the exported header
   free of `cv_bridge` / `sensor_msgs` / `tf2`. Header-only keeps parity with the
   other core headers — no new library target.
4. **Repoint `bag_to_costmap_video.cpp`** to call `accumulate_frame(...)` so the
   exporter and the future tuner share one path (drift prevention).
5. **CMake export plumbing**: add `install(DIRECTORY include/ DESTINATION
   include)`, `ament_export_include_directories(include)`, and
   `ament_export_dependencies(grid_map_core image_geometry nav2_costmap_2d
   OpenCV)` — **OpenCV is a public dep**: `segments_projection.hpp` directly
   `#include <opencv2/core.hpp>` and exposes `cv::Mat`/`cv::Vec3b` in its API, and
   `accumulate_frame` takes `cv::Mat`. Also add `ament_export_targets(
   export_sea_surface_layer HAS_LIBRARY_TARGET)` — the `EXPORT` set is installed
   today (`CMakeLists.txt:104-108`) but never ament-exported; not required for the
   tuner's header-only consumption (it links nothing from this pkg) but closes a
   pre-existing gap cheaply. Update the four `target_include_directories(... /src)`
   lines that feed the moved headers (`bag_to_costmap_video`, `test_segments_apply`,
   `test_segments_projection`, `test_occupancy_buffer`) to use `include/`; leave
   `test_frame_id_resolver` on `/src`. Fix the now-stale "private to the layer /
   lives next to its .cpp in src/" comments.

   **grid_map_core compile fragility (verify downstream):** `CMakeLists.txt:21-27`
   documents that grid_map_core's extras inject `-DEIGEN_*_PLUGIN` globally, so
   every TU needs grid_map_core's include dir to find the plugin headers — the
   package works around it with a global `include_directories(${grid_map_core_INC})`.
   Because `occupancy_buffer.hpp` includes `grid_map_core`, an external header-only
   consumer of the export inherits this fragility. Confirm
   `ament_export_dependencies(grid_map_core)` delivers the needed global
   define+include ordering to a downstream `find_package(sea_surface_segmentation)`
   build; if it does not, document the required downstream workaround in the
   package's exported-API notes. This is the single biggest risk to the "external
   repo reuses the REAL algorithm" goal — verify it explicitly (step 8), don't
   assume the in-package build passing means downstream will.
6. **Add a unit test for the extracted driver** (`test/test_occupancy_accumulator.cpp`):
   feed a tiny synthetic `cv::Mat` mask + camera model + a hand-built rotation
   into `accumulate_frame` and assert expected cells flip to obstacle/free in the
   buffer (no `Image` message or TF buffer fixture needed — that's the payoff of
   the pure boundary). **Cover the full move→decay→ingest sequence**, not just
   project/hit-miss: `accumulate_frame` calls `buffer.move(pos)` then
   `buffer.decay(stamp_s)` before projecting, and `decay()` is wall-clock-ish /
   first-call-seeded while `move()` rolls the window (NaN-fills new cells) — the
   ordering a refactor is most likely to break. Add ≥2-frame cases asserting (a) a
   `move()` window shift preserves overlapping evidence at its world position and
   (b) `decay()` between frames attenuates a prior hit. Wire into `BUILD_TESTING`
   like the other gtests.
7. **Build + test**: `./sensors_ws/build.sh sea_surface_segmentation` then
   `./sensors_ws/test.sh sea_surface_segmentation`; confirm the 3 pre-existing
   unit tests + plugin-load + launch test still pass and the new test passes.
8. **Verify downstream consumability** (closes the grid_map_core fragility risk):
   stand up a throwaway minimal consumer package that does
   `find_package(sea_surface_segmentation)` and `#include
   "sea_surface_segmentation/occupancy_buffer.hpp"` in one TU, and confirm it
   compiles+links against the installed export — i.e. the `-DEIGEN_*_PLUGIN`
   ordering and exported deps actually reach a downstream build. If it fails,
   capture the required workaround in the package's exported-API notes before
   marking #23 done. (This is what `marine_perception_tools#1` will rely on.)

## Files to Change

| File | Change |
|------|--------|
| `src/{occupancy_buffer,segments_projection,segments_apply}.hpp` | Move → `include/sea_surface_segmentation/` |
| `include/sea_surface_segmentation/occupancy_accumulator.hpp` | New: inline `accumulate_frame()` + `AccumulateParams` extracted from the tool |
| `tools/bag_to_costmap_video.cpp` | Replace inline per-frame loop with `accumulate_frame()` call; fix 2 includes |
| `src/sea_surface_layer.cpp`, `src/segments_to_pointcloud.cpp` | Fix include paths (3 total) |
| `test/test_{occupancy_buffer,segments_projection,segments_apply}.cpp` | Fix include paths |
| `test/test_occupancy_accumulator.cpp` | New unit test for the driver |
| `CMakeLists.txt` | `install(DIRECTORY include/)`, `ament_export_include_directories`, `ament_export_dependencies(grid_map_core image_geometry nav2_costmap_2d OpenCV)`, `ament_export_targets(export_sea_surface_layer HAS_LIBRARY_TARGET)`, repoint test/tool include dirs, new gtest, fix stale comments |

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
