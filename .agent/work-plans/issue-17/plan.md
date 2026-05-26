# Plan: Segmentation→sensor adapter for nav2 Collision Monitor

## Issue

https://github.com/rolker/unh_marine_perception/issues/17

## Context

`sea_surface_segmentation/src/segments_to_pointcloud.cpp` already does
flat-water homography from a segmentation mask and publishes
`segmentation/pointcloud`. Today the projection plane is `map_frame`
(default `"map"`; bizzy/izzy override to `bizzy/map`) and the cloud is
stamped in that frame — appropriate for the WIP costmap path, **wrong**
for a reflex Collision Monitor feed because it inherits all the map TF
drift the reflex layer is supposed to skip.

This PR adds a second mode to the same node so a launch-file remap can
run a parallel instance whose output is failure-stage-independent of map
TF *and* of boat pitch/roll. `mru_transform` (verified present in the
bizzy core launch) publishes `bizzy/base_link_level`, which is exactly
that frame.

Companion config in `rolker/unh_echoboats_project11#170` will instantiate
the second instance and wire it into `nav2_collision_monitor`.

## Approach

1. **Add a `target_frame` parameter** (`std::string`, default empty).
   When non-empty it overrides `map_frame` for both the projection
   lookup and the output stamp. `map_frame` stays as-is so existing
   sea_surface_layer / costmap launches keep working untouched. Treat
   this as an addition, not a rename — the cross-repo blast radius of a
   rename outweighs the cleanup benefit here.
2. **Extract the projection math into a pure-logic header**
   (`include/sea_surface_segmentation/segments_projection.hpp`):
   `project_obstacle_pixels(image, camera_model, cam_to_target, plane_z) →
   std::vector<cv::Point3f>`. Mirrors the pattern of `segments_apply.hpp`
   and `frame_id_resolver.hpp`; makes the math GTest-able without ROS or
   TF in the loop. The node becomes a thin shell that does the TF
   lookup and message I/O.
3. **Add an optional `projection_plane_z` parameter** (`double`,
   default `0.0`). The bizzy URDF anchors `base_link` at the hull-floor
   center screw hole — not the waterline. The hull-floor-to-waterline
   offset is small (≪ Collision Monitor slowdown polygon scale), so the
   default of 0.0 is the right starting point; the parameter exists for
   tuning if field data shows it matters.
4. **Add a GTest** in `test/test_segments_projection.cpp` for the new
   header: synthetic single-pixel masks at known image coordinates with
   a synthetic pinhole `CameraInfo` and identity / pitched / rolled
   camera-to-target transforms; assert resulting 3D points within tight
   tolerance. Specifically include a rolled-camera case proving roll is
   correctly rotated out when the target frame is heading-only.
5. **Add a launch_testing integration test** in
   `test/test_segments_to_pointcloud_bag.py` that replays a small
   pre-extracted slice of `~/data/logs/bizzy_images/bag_2026-05-22T20.00.02_ffmpeg_seg`
   (segmentation Image + camera_info + TF) against an instance
   configured with `target_frame=bizzy/base_link_level` and asserts the
   resulting `segmentation/pointcloud` contains ≥N points in the forward
   danger sector (`x ∈ [0, 10] m`, `|y| ≤ 3 m`) during the obstacle
   approach window. The slice is checked into `test/fixtures/` as a
   trimmed mcap (≪ 50 MB target — trim ruthlessly).
6. **Verify the obstacle-pixel heuristic** against the bag. Current
   code uses `R-dominant` (rgb8 channel 0 > channels 1 and 2). The
   integration test will catch a mismatch deterministically; only widen
   the heuristic if the test demands it. Do not pre-emptively change it.
7. **Document the new params** in `sea_surface_segmentation/config/README.md`
   (one line each on `target_frame`, `projection_plane_z`) and add a
   short "Reflex safety mode" subsection explaining the
   failure-stage-independence rationale at a single paragraph's depth.
   No new ADR in this PR — the cross-package safety-architecture ADR is
   better placed in the boat config repo where the reflex layer is
   actually wired up.

The coarse danger-region trigger (rung 1 of the issue's robustness
ladder) is **deferred to a follow-up issue**. Rung 2 (metric homography
in a stabilized frame) is what this PR delivers; rung 1 is independent
work that doesn't block the field-ready safety reflex. Filing the
follow-up is in Open Questions.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/segments_to_pointcloud.cpp` | Add `target_frame`, `projection_plane_z` params; delegate projection to new header; preserve `map_frame` behavior |
| `sea_surface_segmentation/include/sea_surface_segmentation/segments_projection.hpp` | **new** — pure-logic projection helper |
| `sea_surface_segmentation/CMakeLists.txt` | Add header install; add new test target |
| `sea_surface_segmentation/test/test_segments_projection.cpp` | **new** — unit tests for the projection helper |
| `sea_surface_segmentation/test/test_segments_to_pointcloud_bag.py` | **new** — launch_testing bag-replay integration test |
| `sea_surface_segmentation/test/fixtures/issue17_obstacle_approach.mcap` | **new** — trimmed slice from 2026-05-22 deployment bag |
| `sea_surface_segmentation/config/README.md` | Document `target_frame`, `projection_plane_z`, reflex-mode use |
| `sea_surface_segmentation/package.xml` | Add any new test deps (ros2_bag, launch_testing) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | New param is additive; existing `map_frame` users untouched. Doc update + tests in the same PR. |
| Capture decisions, not just implementations | "Why not rename `map_frame`" recorded in step 1; "Why no heave compensation" carried from issue-review; ADR deferred to boat-config repo. |
| Test what breaks | Bag-replay test asserts the adapter *would have* produced danger-sector points during the 2026-05-22 obstacle approach — direct regression evidence for the deployments that motivated #17. |
| Only what's needed | Single optional param + one helper extraction; no rung-1 trigger, no LaserScan/Range output, no heave compensation, no URDF changes. |
| Improve incrementally | Existing map-frame mode preserved; reflex mode opt-in via param. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Working in `layers/worktrees/issue-unh_marine_perception-17`. |
| 0008 — ROS 2 conventions | Yes | New params use snake_case; new helper header under `include/<pkg>/`; launch_testing for integration; sensor_msgs types unchanged. |
| 0013 — `progress.md` vocabulary | Yes | `## Plan Authored` entry appended in step 8 of the skill. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `segments_to_pointcloud` parameter surface | `config/README.md` | Yes (step 7) |
| `segments_to_pointcloud` parameter surface | Downstream `unh_echoboats_project11#170` (will use `target_frame: bizzy/base_link_level`) | Out of scope — handshake recorded in #170 |
| `sea_surface_segmentation/include/` contents | `CMakeLists.txt` (install rule) | Yes (step 2) |
| Add new test executables | `CMakeLists.txt` + `package.xml` test deps | Yes |

## Open Questions

- **Bag-fixture size budget.** Trimmed mcap slice needs to be small enough to check into the repo (target ≪ 50 MB). If the smallest viable obstacle-approach window exceeds the budget, switch to Git LFS or an external test-data dir. Decide during step 5.
- **Hull-floor → waterline offset.** Default `projection_plane_z = 0.0` is fine for safety-polygon scales, but if Collision Monitor tuning in #170 wants the bias removed, we'll either set this param in the bizzy config or add a static `bizzy/waterline_level` frame. Decision lives in #170, not here.
- **Coarse danger-region trigger follow-up issue.** Should be filed before this PR merges so the rung-1 deferral is tracked. Title sketch: "Coarse mask-fills-danger-region trigger for Collision Monitor (rung 1)" — file under `unh_marine_perception`.
- **Output topic name.** Current code hardcodes `segmentation/pointcloud`. Two parallel instances will share the topic unless the reflex-mode launch in #170 uses ROS remap (the standard path). No node change needed — confirming.

## Estimated Scope

Single PR. Implementation steps 1–3 are ~half a day; tests + docs are the bulk of the work. Bag-fixture trimming is the variable. Plan stays under the 80-line ceiling.
