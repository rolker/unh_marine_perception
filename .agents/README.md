# Agent Guide: unh_marine_perception

> Camera-based marine perception for UNH autonomous survey boats: OAK/DepthAI
> camera support (`depthai_marine`) and sea-surface segmentation feeding the
> nav2 costmap and the collision-monitor reflex (`sea_surface_segmentation`).

## Workflow

GitHub-origin repo: `rolker/unh_marine_perception`. **Default branch is
`jazzy`** (not `main`). All changes go through worktree + PR per the workspace
rules; a `gitcloud` field remote also exists (field mode applies only when a
clone's *origin* is non-GitHub).

**When this repo is checked out as part of a
[ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace)**,
workflow rules are defined in the workspace `AGENTS.md`. To determine the
active mode, run from the workspace root:

```bash
.agent/scripts/field_mode.sh --describe layers/main/sensors_ws/src/unh_marine_perception
```

CI (`.github/workflows/ci.yml`) runs colcon build + test on `ros:jazzy-ros-core`
with a source-sibling clone of `rolker/marine_control` (a dependency not in
rosdep). Pre-commit (`.pre-commit-config.yaml`) covers whitespace/YAML/XML/
CMake lint; black/flake8 are deliberately off pending license-header cleanup.

## Safety Context

This perception stack feeds two independent obstacle paths on a live
autonomous boat (BizzyBoat): the nav2 costmap (planner avoidance) and the
`nav2_collision_monitor` reflex (emergency stop). Consequences verified in
source:

- **Timestamps are load-bearing.** Segmentation frames are stamped from the
  OAK *device capture time*, not `now()` (`segmentation_stamp.hpp`, issue #28)
  — `now()` discarded ~123.5 ms of pipeline latency and made TF resolve stale
  camera poses, so close obstacles never marked.
- **Frames are load-bearing.** The reflex feed projects into a
  failure-stage-independent heading-only frame (`target_frame`, e.g.
  `bizzy/base_link_level`), not `map`. `SeaSurfaceRelayLayer` drops grids whose
  `frame_id` mismatches its costmap's global frame rather than stamp LETHAL at
  wrong world positions.
- **Operator controls are capped.** `obstacle_prob_min` on the reflex node is
  clamped to [0, 0.95] by its on-set-parameters callback so a remote
  marine_control change can never blind the reflex feed (ADR-0003 D8.3).
- Both costmap layers only **ADD** cost (max-combine; occupancy ≤ 0 leaves the
  master cell untouched) and report `isClearable() == false`.

## Package Inventory

| Package | Language | Description |
|---------|----------|-------------|
| `depthai_marine` | C++ (ament_cmake) | Library + tools for Luxonis OAK cameras: `CameraBase` (connection retry, pipeline, publishers), raw `Image` publishing, opt-in on-device H.265/H.264 encoding published as `ffmpeg_image_transport_msgs/FFMPEGPacket`. Executables: `wide_stereo`, `list_devices`, `measure_timing`, `display_time`. |
| `sea_surface_segmentation` | C++ (ament_cmake) | On-device NN water/sky/obstacle segmentation (`sea_surface_segmentation` node), reflex pointcloud adapter (`segments_to_pointcloud` lifecycle node), two `nav2_costmap_2d` plugins (`SeaSurfaceLayer`, `SeaSurfaceRelayLayer`), exported header-only algorithm core, and the offline `bag_to_costmap_video` review tool. |

## Repository Layout

```
unh_marine_perception/
├── depthai_marine/
│   ├── include/depthai_marine/   # camera_base, image_publisher, ffmpeg_publisher (exported lib)
│   ├── src/                      # lib + list_devices, display_time, measure_timing, wide_stereo
│   ├── docs/                     # API.md, h265_transport.md
│   └── test/test_h265_params.cpp
├── sea_surface_segmentation/
│   ├── include/sea_surface_segmentation/  # EXPORTED header-only core: occupancy_buffer,
│   │                                      # segments_projection, segments_apply,
│   │                                      # occupancy_accumulator, cost_mapping (#23)
│   ├── src/                      # nodes + plugins; private hdrs frame_id_resolver, segmentation_stamp
│   ├── costmap_plugins.xml       # pluginlib export (nav2_costmap_2d::Layer)
│   ├── tools/bag_to_costmap_video.cpp
│   ├── config/README.md          # detailed node/plugin parameter reference
│   └── test/                     # 7 gtests + 2 launch_testing + fixtures/issue17_obstacle_approach/
├── benchmarks/h265_transport/    # JPEG-vs-H.265 bandwidth/quality study (#2), not built by colcon
└── docs/perception_capability_report.md  # plain-language capability snapshot + figures
```

## Architecture Overview

Data flow (all topic names verified in source):

1. `sea_surface_segmentation` node (plain `rclcpp::Node`, name
   `sea_surface_segmentation_node`, MultiThreadedExecutor) drives N OAK
   cameras (`camera_ids`/`camera_names`/`frame_ids` params). Per camera it
   publishes `<name>/segmentation` (`sensor_msgs/Image`, rgb8 128×96 — per-pixel
   softmax as R=obstacle, G/B = other classes) plus
   `<name>/segmentation/camera_info` with intrinsics scaled for the
   anisotropic 16:9 preview → 512×384 → 128×96 NN stretch. Stamps use device
   capture time. Via `CameraParams`, the same node can also publish per-camera
   video (`<name>/image_raw`) and H.265 (`<name>/image_raw/ffmpeg`).
2. `SeaSurfaceLayer` (costmap plugin, typically local_costmap) fuses one or
   more segmentation sources into a shared rolling log-odds `OccupancyBuffer`
   (decay half-life, waterline-contact hits, water misses; inverse cell→pixel
   projection) and stamps a graded cost (soft 1..252, LETHAL at threshold).
   With `published_topic` set it republishes a graded `nav_msgs/OccupancyGrid`
   (QoS depth 1, transient_local) that `SeaSurfaceRelayLayer` (typically
   global_costmap) stamps into a second costmap — projection runs once.
3. `segments_to_pointcloud` (lifecycle node, SingleThreadedExecutor — the
   single thread is the marine_control threading contract; do not switch)
   subscribes `segmentation` + `segmentation/camera_info` (SensorDataQoS) and
   publishes `~/pointcloud` (`PointCloud2`, SensorDataQoS) in `map_frame` or
   `target_frame` for the Collision Monitor reflex. Health on `/diagnostics`
   (task "obstacle projection feed", WARN-only severity). While active it
   exposes `obstacle_prob_min` on `~/control/state` / `~/control/change` as
   marine_control device "Reflex Obstacle Filter" (unh_marine_autonomy#140 /
   ADR-0003), built in `on_activate`, torn down in `on_deactivate`.

The header-only core under `include/` is exported (`ament_export_*`) so
offline tools (`bag_to_costmap_video` here, `sea_surface_tuner` in
marine_perception_tools) compute costmaps with the real algorithm. The
`sea_surface_layer` plugin library is deliberately NOT target-exported.

## Key Files to Read First

1. `sea_surface_segmentation/config/README.md` — full, source-accurate
   parameter tables for the layer/relay plugins and node modes.
2. `sea_surface_segmentation/src/segments_to_pointcloud.cpp` — reflex adapter:
   lifecycle, param validation, marine_control wiring, diagnostics.
3. `sea_surface_segmentation/src/sea_surface_layer.cpp` — the costmap layer:
   multi-source config, locking pattern, live-tunable param callback.
4. `sea_surface_segmentation/include/sea_surface_segmentation/occupancy_buffer.hpp`
   and `segments_projection.hpp` — the algorithm core + parameter semantics.
5. `depthai_marine/include/depthai_marine/camera_base.hpp` +
   `docs/h265_transport.md` — camera/encoder knobs and the FFMPEGPacket
   contract.

## Build & Test

Lives in the workspace's `sensors_ws` layer. From a worktree:
`./sensors_ws/build.sh` / `./sensors_ws/test.sh`, or:

```bash
cd layers/main/sensors_ws
colcon build --symlink-install --packages-up-to sea_surface_segmentation
source ../../../.agent/scripts/setup.bash && colcon test --packages-select depthai_marine sea_surface_segmentation && colcon test-result --verbose
```

**No OAK hardware is needed for any test** (per-test rationale is commented in
`sea_surface_segmentation/CMakeLists.txt`); everything runs in plain
`colcon test` and in CI:

- gtest, `depthai_marine`: `test_h265_params` (profile parsing/encoding
  mapping, `CameraParams` setter validation).
- gtest, `sea_surface_segmentation`: `test_frame_id_resolver`,
  `test_segmentation_stamp`, `test_segments_apply`,
  `test_segments_projection`, `test_occupancy_buffer`,
  `test_occupancy_accumulator`, plus `test_layer_plugin_loading` — loads both
  plugins via pluginlib AND calls `Layer::initialize` against a real
  LayeredCostmap + LifecycleNode (catches declaration-order bugs that would
  abort layer loading at deployment).
- launch_testing (bag replay): `test_segments_to_pointcloud_bag.py` replays
  the in-repo fixture `test/fixtures/issue17_obstacle_approach` — a 20 s MCAP
  slice of the 2026-05-22 deployment (100 forward-OAK segmentation frames +
  camera_info + /tf + /tf_static) — through `segments_to_pointcloud` in
  reflex mode (`target_frame=bizzy/base_link_level`, `use_sim_time`) and
  asserts ≥100 obstacle points in the forward danger sector AND that the
  cloud is stamped only in the target frame (the #17 deployment-evidence
  regression).
- launch_testing (control): `test_segments_to_pointcloud_control.py` activates
  the node and asserts the marine_control channel advertises
  `obstacle_prob_min` with 0–0.95 bounds, applies in-range changes, and
  rejects out-of-range ones (needs `marine_control_interfaces`).

## Cross-Layer Dependencies

| Package | Depends On | Where | What It Uses |
|---------|-----------|-------|--------------|
| `sea_surface_segmentation` | `marine_control` (+ `marine_control_interfaces` for tests) | source sibling (CI clones it; not in rosdep) | `marine_control/control_server.hpp` device-control channel |
| `sea_surface_segmentation` | `depthai_marine` | this repo | `CameraBase`, publishers |
| both | `depthai`, `depthai_bridge` | rosdep | device API, `BridgePublisher`, `ImageConverter` time helpers |

## Common Pitfalls

- `wide_stereo`'s `main()` names its node `"measure_timing"`
  (`depthai_marine/src/wide_stereo.cpp`) — a copy-paste quirk; the launch file
  overrides the name to `wide_stereo`.
- `depthai_marine/launch/measure_timing_launch.py` references NN blobs under
  `config/` that are not in the repo, and `detection_visualizer_launch.py`
  needs an external `detection_visualizer` package — both are legacy dev
  tooling, not deployed paths.
- Image `image_raw` uses `depthai_bridge::BridgePublisher`'s internal RELIABLE
  QoS while the FFMPEG topic uses SensorDataQoS (BEST_EFFORT) — intentional
  asymmetry, documented in `docs/h265_transport.md`.
- Subscribers to `segmentation` / `~/pointcloud` must use sensor-data QoS; a
  RELIABLE subscriber silently receives nothing (the bag test comments this).
- In `SeaSurfaceLayer`, topic/source-list params are configure-time only; the
  on-set callback rejects them with "restart the costmap to apply". Register
  order of `declareParameter` vs the callback matters (see the CMake-adjacent
  comment block in `onInitialize`).
- The pre-commit large-file hook (500 kB) already tracks the bag fixture;
  discuss before adding new large binaries.
