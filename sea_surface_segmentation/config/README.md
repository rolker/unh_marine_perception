# sea_surface_segmentation configuration

This directory holds configuration for the `sea_surface_segmentation` package.

## `segments_to_pointcloud` node

Projects forward-camera obstacle segmentation masks onto a horizontal
plane in a chosen target frame and publishes the intersection points as
a `sensor_msgs/PointCloud2` on `~/pointcloud` (the node's private
namespace, so two parallel instances auto-isolate by node name).

### Parameters

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `map_frame` | string | `"map"` | Legacy projection frame. Used as the projection target when `target_frame` is empty. |
| `target_frame` | string | `""` | When non-empty, overrides `map_frame` for both the TF lookup and the output `header.frame_id`. |
| `projection_plane_z` | double | `0.0` | z-coordinate of the projection plane in the active target frame. |

### Operating modes

**Legacy map-frame mode** (current sea_surface_layer / costmap path):
- Leave `target_frame` empty. The node uses `map_frame` (typically
  `<robot>/map`) and stamps the output cloud in that frame.
- The projection plane is the world-frame z=0 surface. Output points
  inherit any map TF drift downstream consumers must tolerate.

**Reflex safety mode** (for `nav2_collision_monitor`):
- Set `target_frame` to a heading-only frame such as
  `<robot>/base_link_level` (published by `mru_transform`).
- This skips the world-frame TF accumulation that the WIP
  segmentation→costmap pipeline depends on. mru_transform zeros boat
  pitch and roll into the frame definition, so the resulting cloud is
  immune to attitude noise as well as to map drift.
- Collision Monitor receives points in a frame whose only TF chain to
  base_link is heading; its `transform_tolerance` covers the small TF
  age between segments stamp and current TF.
- `projection_plane_z` stays at 0.0 by default. The bizzy URDF anchors
  `base_link` at the hull-floor screw hole rather than the waterline,
  so projected ranges carry a small, bounded near-horizon bias that
  the Collision Monitor slowdown polygon absorbs.

### Health monitoring

The node publishes a `diagnostic_msgs/DiagnosticArray` on `/diagnostics`
(task name `obstacle projection feed`), picked up by the operator-station
annunciator. This makes a silently-degraded reflex feed visible instead of
showing up only as an empty cloud. Reported fields include `mode`,
`projection_frame`, `camera_info_received`, `segmentation_frames_received`,
`clouds_published`, `tf_lookup_failures`, `nonfinite_points_dropped`, and
the age of the last frame/publish.

Status levels (intentionally conservative — `WARN`, never `ERROR`, so a
transient TF gap or a not-yet-calibrated camera doesn't raise a hard alarm;
escalation thresholds belong on the annunciator side):

- `OK` — projecting obstacles (frames in, clouds out).
- `WARN` — `waiting for camera_info`, `no segmentation frames received
  yet`, or `segmentation arriving but projection failing` (the TF lookup
  for the projection frame is failing, so the obstacle stream is currently
  dead).

### Launch

`launch/segments_to_pointcloud_launch.py` exposes two launch
arguments:

- `name` (default `segments_to_pointcloud`) — node name. Override when
  running a reflex-mode instance alongside the legacy one in the same
  namespace.
- `target_frame` (default empty) — passed through to the
  `target_frame` parameter above.

Example reflex instance (boat-config side):

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare('sea_surface_segmentation'),
        '/launch/segments_to_pointcloud_launch.py',
    ]),
    launch_arguments={
        'name': 'segments_to_pointcloud_reflex',
        'target_frame': 'bizzy/base_link_level',
    }.items(),
)
```

## `sea_surface_layer::SeaSurfaceLayer` (costmap_2d plugin)

A `nav2_costmap_2d::Layer` plugin that fuses one or more camera
segmentation streams into a single persistent, decaying log-odds
occupancy buffer in the costmap's global frame, and stamps cells whose
accumulated evidence has crossed the lethal threshold into the master
costmap. Replaces the pre-#19 per-camera layer instances (each with a
private buffer wiped every frame), giving cross-camera fusion and
short-term obstacle memory through FOV handoffs.

### Sources

The layer accepts one OR more camera sources. Multi-source is
configured via `observation_sources` (a list of source names); each
named source declares its own segmentation + camera_info topics:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["chart_layer", "sea_surface_layer", "inflation_layer"]
      sea_surface_layer:
        plugin: "sea_surface_layer::SeaSurfaceLayer"
        observation_sources: ["forward", "port", "starboard", "aft"]
        forward:
          segmentation_topic: bizzy/sensors/cameras/oak_forward/segmentation
          camera_info_topic:  bizzy/sensors/cameras/oak_forward/segmentation/camera_info
        port:
          segmentation_topic: bizzy/sensors/cameras/oak_port/segmentation
          camera_info_topic:  bizzy/sensors/cameras/oak_port/segmentation/camera_info
        # ... starboard, aft analogously
        maximum_range: 150.0
        published_topic: /sea_surface/lethal_grid  # for SeaSurfaceRelayLayer
```

When `observation_sources` is empty (default), the layer falls back to
top-level `segmentation_topic` / `camera_info_topic` as a single source
named `"default"` — pre-#19 single-camera configs work unchanged.

All sources feed the same shared occupancy buffer in the costmap's
global frame, so an obstacle in two cameras' FOV overlap accumulates
evidence from every camera that sees it (faster threshold crossing,
fewer flicker losses) and persists across FOV handoffs.

### Parameters

| Parameter | Type | Default | Live-tunable | Purpose |
|---|---|---|---|---|
| `observation_sources` | string[] | `[]` | no | Named sources. Empty falls back to single-source legacy mode. |
| `<source>.segmentation_topic` | string | `""` | no | Per-source obstacle-mask topic. |
| `<source>.camera_info_topic` | string | `""` | no | Per-source intrinsics topic. |
| `segmentation_topic` | string | `"segmentation"` | no | Legacy single-source (when `observation_sources` is empty). |
| `camera_info_topic` | string | `"camera_info"` | no | Legacy single-source. |
| `maximum_range` | double | `100.0` | **yes** | Projection AABB half-extent per camera (m). |
| `min_grazing_angle_deg` | double | `0.0` | **yes** | Reject rays that hit the water plane at less than this angle from horizontal. `0` = filter off. Bounds horizontal reach to ≈ `camera_height / tan(angle)`; cuts long-range noise where small pitch uncertainty produces large ground error. |
| `published_topic` | string | `""` | no | When non-empty, republishes the lethal mask on this topic for `SeaSurfaceRelayLayer`. Empty = disabled. |
| `hit_log_odds` | double | `0.85` | **yes** | Log-odds added on a waterline-contact observation. |
| `miss_log_odds` | double | `-0.40` | **yes** | Log-odds added on a water (free-space) observation; negative. |
| `clamp` | double | `5.0` | **yes** | Symmetric log-odds bound — keeps evidence revisable, prevents saturation. |
| `lethal_threshold` | double | `1.0` | **yes** | Log-odds at or above which a cell stamps `LETHAL_OBSTACLE`. Must be in `(0, clamp]`. |
| `decay_half_life_s` | double | `30.0` | **yes** | Unobserved evidence halves every this many wall-clock seconds. |

**Live-tunable**: change via `ros2 param set <costmap_node>
<layer_name>.<param> <value>`. The validating
`OnSetParametersCallbackHandle` rejects NaN / out-of-range /
safety-inverting values (e.g. `lethal_threshold <= 0`, negative
`maximum_range`) with `successful=false` and a human-readable reason —
the safety layer's interpretation cannot be silently poisoned by a
fat-fingered set. Configure-time params (topics, `observation_sources`,
`published_topic`) are rejected explicitly with a "restart the costmap
to apply" message so the operator isn't silently misled.

### Behavior notes

- The layer projects pixel→world via the cell→pixel inverse direction
  (`project_observations_inverse` in `segments_projection.hpp`): iterate
  the world cells the camera can reach, classify each by the pixel it
  covers. Waterline contact → hit (evidence ↑); water → miss
  (evidence ↓); occluded body / sky pixels → skipped (no update). Only
  the waterline contact gets back-projected onto the water plane (z=0);
  above-water body pixels would project past the real obstacle as a
  false "shadow" of lethal cells out to maximum_range.
- The buffer rolls with the parent costmap via `grid_map::move()` — a
  pure circular-buffer index shift that preserves accumulated evidence
  at its world position through sub-cell origin shifts (no drift to
  neighboring cells).
- `isClearable()` is false: the layer clears itself via water-miss
  observations + time-based decay, so nav2 clear-costmap behaviors
  don't need to (and shouldn't) wipe it.

## `sea_surface_layer::SeaSurfaceRelayLayer` (costmap_2d plugin)

A thin counterpart to `SeaSurfaceLayer` for the case where the same
lethal cells should reach a second costmap (typically the global
costmap, so `SmacPlannerHybrid` plans around buoys). The relay
subscribes to a `nav_msgs::msg::OccupancyGrid` published by a peer
`SeaSurfaceLayer` (running in another costmap, with `published_topic`
set), and stamps `LETHAL_OBSTACLE` into its own master grid where the
published cell is at or above `100`. The segmentation projection runs
only once — in the producer.

### Configuration

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["chart_layer", "sea_surface_relay", "inflation_layer"]
      sea_surface_relay:
        plugin: "sea_surface_layer::SeaSurfaceRelayLayer"
        topic: /sea_surface/lethal_grid
```

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `topic` | string | `"sea_surface/lethal_grid"` | OccupancyGrid topic published by the peer `SeaSurfaceLayer`. |

### Behavior notes

- Only cells published as `100` (lethal) stamp into the master; `-1`
  ("no opinion") cells leave the master untouched, so the relay never
  inadvertently clears another layer's marks (the global costmap's
  chart + inflation contributions stay authoritative; sea-surface only
  ADDS).
- Cell-lookup is by world position, so the producer's costmap and the
  consumer's can have different pose / resolution / dimensions and the
  lethal cells still land at the correct world location.
- The relay drops messages whose `header.frame_id` doesn't match the
  consumer's `getGlobalFrameID()` — stamping LETHAL at the wrong world
  position is a safety risk on an obstacle layer.
- `isClearable()` is false (the producer owns clearing via decay).
