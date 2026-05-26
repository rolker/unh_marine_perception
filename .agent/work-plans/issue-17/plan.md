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
   **Also**: switch the publisher topic from the hardcoded
   `segmentation/pointcloud` to `~/pointcloud` (private namespace,
   scoped by node name) so two parallel instances auto-isolate without
   relying on launch-file remap discipline. This intentionally breaks
   the current default-topic convention; downstream consumers are
   covered by step 8.

   **Output contract** (load-bearing for the reflex-safety claim):
   when `target_frame` is non-empty, the published cloud is stamped
   with `header.frame_id = target_frame` and `header.stamp =
   segments_msg->header.stamp`. The TF lookup uses the same segments
   stamp; Collision Monitor's `transform_tolerance` covers the small
   age between segments and current TF.

   **Launch surface**: update `segments_to_pointcloud_launch.py` to
   accept `name` and `target_frame` as launch arguments (defaults:
   `segments_to_pointcloud`, empty). The reflex-mode instance is
   launched in #170 by passing `name='segments_to_pointcloud_reflex'`,
   `target_frame='bizzy/base_link_level'`. Without these args, current
   includers (izzy's `oak1_launch.py`) keep the legacy single-instance
   behavior.
2. **Extract the projection math into a pure-logic header**
   (`src/segments_projection.hpp` — private to the executable,
   matching the file-locality pattern of `segments_apply.hpp` and
   `frame_id_resolver.hpp`, which also live in `src/`):
   `project_obstacle_pixels(mask_rgb8, camera_model, camera_origin,
   rotation_cam_to_target, plane_z) → std::vector<ProjectedPoint>`.
   Pure logic — no ROS, no TF, no PCL. The node still does the TF
   lookup (extracting translation + 3×3 rotation from the
   `TransformStamped`) and PointCloud2 assembly; the helper owns the
   geometry. `is_obstacle_pixel(cv::Vec3b)` is its own inline so
   tests can pin the heuristic deterministically.
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
   against an instance configured with `target_frame=bizzy/base_link_level`
   and asserts the resulting `~/pointcloud` contains ≥N points in the
   forward danger sector (`x ∈ [0, 10] m`, `|y| ≤ 3 m`) during the
   obstacle approach window. The slice is checked into `test/fixtures/`
   as a trimmed mcap (≪ 50 MB target — trim ruthlessly). Trim with
   `mcap filter --include-topic …` keeping exactly four streams: the
   segmentation `Image`, its `CameraInfo`, `/tf`, and `/tf_static`. The
   N-point threshold is a **presence check, not a recall measurement**
   — recall characterization is out of scope for this PR.
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

8. **File the cross-repo consumer-update follow-ups.** Two repos have
   consumers of the legacy `segmentation/pointcloud` topic name that
   will need to migrate to the new `~/pointcloud` convention:
   - `rolker/unh_echoboats_project11`: izzy's rviz config
     (`izzyboat_project11/config/izzyboat.rviz`) and izzy's diagnostic
     monitor (`izzyboat_project11/config/izzyboat.yaml`).
   - `rolker/seafloor_echoboat_project11`: nav2 params
     (`echoboat_project11/config/nav2_params.yaml`, two costmap layer
     entries).

   Bizzy doesn't currently run `segments_to_pointcloud` at all
   (`bizzyboat_project11/launch/oak_cameras_launch.py` starts only the
   segmenter executable), so the June 4 safety-reflex critical path is
   unaffected by this migration. File the two follow-up issues at the
   time this PR is reviewable so coordinated merging can happen with a
   minimal regression window for izzy's costmap.

   **Merge ordering**: keep this PR in draft until both follow-up PRs
   are open and reviewable. Merge same-day, boat-config PRs first, then
   this one — that ordering ensures izzy never has a publisher with no
   subscriber (the reverse would leave izzy's costmap silently empty).
   izzy's costmap subscribes via `seafloor_echoboat_project11/
   echoboat_project11/config/nav2_params.yaml` lines 158 and 207.

The coarse danger-region trigger (rung 1 of the issue's robustness
ladder) is **not being filed as a follow-up issue at this point**.
Decision: rung 2 in `base_link_level` should already survive moderate
extrinsic error; rung-1's redundancy buy is against camera-mount-error,
mru_transform fault, and a hypothetical projection bug. We'll only file
rung-1 if field testing of rung-2 surfaces a real gap. Documented here
so the decision is durable.

### Pre-existing non-fix items (out of scope for this PR)

Logged so review-code / Copilot doesn't flag them as introduced
regressions:

- The node uses plain `rclcpp::Publisher`, not `rclcpp_lifecycle::LifecyclePublisher`.
  `on_activate`/`on_deactivate` lifecycle transitions are currently
  no-ops; publication happens regardless of activation state.
- `package.xml` doesn't declare `<exec_depend>rclcpp_lifecycle</exec_depend>`
  even though `CMakeLists.txt` links the library. Will add a `test_depend`
  for any new test deps, but the lifecycle exec_depend gap is pre-existing.
- `CMakeLists.txt` has a pre-existing mix of `target_link_libraries` and
  `ament_target_dependencies` on the `segments_to_pointcloud` target. New
  test executables follow the existing pattern; conversion to modern
  CMake targets is a separate cleanup.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/segments_to_pointcloud.cpp` | Add `target_frame`, `projection_plane_z` params; switch publisher topic to `~/pointcloud`; delegate projection to new header; preserve `map_frame` behavior |
| `sea_surface_segmentation/launch/segments_to_pointcloud_launch.py` | Surface `name` and `target_frame` launch args (defaults preserve legacy single-instance behavior) |
| `sea_surface_segmentation/src/segments_projection.hpp` | **new** — pure-logic projection helper (private to the executable, like the existing `segments_apply.hpp` and `frame_id_resolver.hpp`) |
| `sea_surface_segmentation/CMakeLists.txt` | Add new test targets reaching into `src/` for the private header (mirrors how `test_frame_id_resolver` is set up). Follow existing `target_link_libraries` + `ament_target_dependencies` pattern (pre-existing mix; not converting in this PR) |
| `sea_surface_segmentation/test/test_segments_projection.cpp` | **new** — unit tests for the projection helper |
| `sea_surface_segmentation/test/test_segments_to_pointcloud_bag.py` | **new** — launch_testing bag-replay integration test |
| `sea_surface_segmentation/test/fixtures/issue17_obstacle_approach/` | **new** — trimmed slice from 2026-05-22 deployment bag (directory with `*_0.mcap` + `metadata.yaml`; ros2 bag needs both). Installed under `share/` so the test can resolve it via `get_package_share_directory`. |
| `sea_surface_segmentation/config/README.md` | Document `target_frame`, `projection_plane_z`, reflex-mode use |
| `sea_surface_segmentation/package.xml` | Add test_depend entries: `launch_testing_ament_cmake`, `launch_testing`, `launch_testing_ros`, `ros2bag`, `rosbag2_storage_mcap`, `sensor_msgs_py` |

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
| `segments_to_pointcloud` publisher topic (`segmentation/pointcloud` → `~/pointcloud`) | `unh_echoboats_project11` (izzy rviz + monitor) and `seafloor_echoboat_project11` (nav2 params) | Out of scope — follow-up issues filed per Approach step 8 |
| `segments_to_pointcloud_launch.py` surface | `unh_echoboats_project11/izzyboat_project11/launch/oak1_launch.py` (current includer; should keep working with the new defaults but verify) | Yes — quick verification noted in step 1 |
| New header in `src/` | `CMakeLists.txt` — none (private header, no install rule needed, matching existing pattern) | Yes (step 2) |
| Add new test executables | `CMakeLists.txt` + `package.xml` test deps | Yes |

## Open Questions

None remaining for this plan. The four open questions raised during
plan drafting were resolved in discussion:

- **Bag-fixture size budget.** Decision: check in to git under
  `test/fixtures/`. Target ≪ 50 MB after trim. No LFS, no external
  dir — keep CI / fresh clones self-sufficient.
- **Hull-floor → waterline offset.** Decision: ship the
  `projection_plane_z` param with default 0.0. Treat the bias as part
  of #170's polygon-sizing budget. Escalate to a measured value or a
  `waterline_level` static TF only if field tuning demands it.
- **Rung-1 coarse trigger follow-up issue.** Decision: don't file
  now. Rung-2 in `base_link_level` should survive moderate extrinsic
  error; rung-1's redundancy buy is conditional. File it only if
  field evidence shows rung-2 misses something rung-1 would catch.
- **Output topic name.** Decision: switch publisher from
  `segmentation/pointcloud` to `~/pointcloud` (private namespace).
  Accept the coordinated cross-repo consumer update (Approach step 8).

## Estimated Scope

Single PR. Implementation steps 1–3 are ~half a day; tests + docs are the bulk of the work. Bag-fixture trimming is the variable. Plan stays under the 80-line ceiling.
