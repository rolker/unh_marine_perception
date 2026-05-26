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
