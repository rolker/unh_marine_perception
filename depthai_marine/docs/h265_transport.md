# On-device H.265 Transport

`depthai_marine::CameraBase` can publish an on-device H.265 (or H.264) bitstream
produced by the OAK camera's Myriad X `VideoEncoder`. The encoded stream goes
out as an `ffmpeg_image_transport_msgs::msg::FFMPEGPacket` on
`<camera>/image_raw/ffmpeg`, sidestepping the bandwidth cost of the raw NV12 →
JPEG path for over-the-air (LTE / Starlink / `udp_bridge`) operation.

Opt-in — default behavior is unchanged.

## Topic contract

| Topic | Type | QoS | When |
|---|---|---|---|
| `<camera>/image_raw/ffmpeg` | `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` | `rclcpp::SensorDataQoS()` (BEST_EFFORT, depth 5) | Published when `h265_enable = true`. |

Conversion from `dai::EncodedFrame` (the output of `dai::node::VideoEncoder::out`)
to `FFMPEGPacket` is handled by `dai::ros::ImageConverter::toRosFFMPEGPacket` —
the same `ImageConverter` class that produces the sibling `sensor_msgs/Image`.
This is important: `ImageConverter` captures a steady-clock → ROS-time base
offset at construction, so both topics' `header.stamp` land in the same ROS
time domain. Downstream consumers doing cross-stream sync (bag replay,
`message_filters::TimeSynchronizer`, operator dashboards) see matched
timestamps for each physical frame.

Field mapping on each `FFMPEGPacket` (filled in by `toRosFFMPEGPacket`):

- `header.stamp` — ROS time, base-offset from `EncodedFrame::getTimestamp()`.
- `header.frame_id` — the `frame_id` passed to `FFMPEGPublisher` (same as the
  sibling `sensor_msgs/Image`'s `frame_id`). When `CameraBase::initialize()`
  constructs the publisher, this defaults to `<label>_optical_frame` to match
  the URDF / `image_geometry` convention; callers can override by passing an
  explicit `frame_id` to `initialize()`.
- `width` / `height` — from `EncodedFrame::getWidth()` / `getHeight()`.
- `encoding` — `"hevc"` for H.265 profiles, `"h264"` for H.264 profiles;
  configured via `ImageConverter::setFFMPEGEncoding` at `FFMPEGPublisher`
  construction. Canonical codec names that `ffmpeg_encoder_decoder::Decoder`
  uses for `findDecoders()` lookup.
- `pts` — derived by `ImageConverter` from the EncodedFrame timestamp.
- `flags` — keyframe bit derived from `EncodedFrame::getFrameType()` (no
  hand-rolled NAL inspection).
- `data` — encoder bitstream bytes, Annex-B framed.

## QoS rationale

`SensorDataQoS` is the `image_transport` convention and what
`ffmpeg_image_transport` subscribers expect for auto-discovery. For live video
over lossy links (LTE, `udp_bridge`), BEST_EFFORT is the correct choice:
dropping a stale frame is strictly better than publisher backpressure when the
network can't keep up. This publisher is intentionally asymmetric with the
sibling `image_raw` topic (which uses `depthai_bridge::BridgePublisher`'s
internal RELIABLE QoS); consolidating the raw path to `SensorDataQoS` is a
separate cleanup.

## ROS params (on the owning node)

| Param | Type | Default | Purpose |
|---|---|---|---|
| `h265_enable` | bool | `false` | Opt-in. When `true`, the VideoEncoder branch is added to the pipeline and `FFMPEGPublisher` publishes on `<camera>/image_raw/ffmpeg`. |
| `h265_bitrate_kbps` | int | `4000` | CBR target bitrate. Feeds `VideoEncoder::setBitrateKbps`. Calibrate per platform; starting point is the PR #3 recommendation. |
| `h265_keyframe_frequency_frames` | int | `30` | Keyframe every Nth frame. At 5 FPS that's every 6 seconds. |
| `h265_profile` | string | `"H265_MAIN"` | `H265_MAIN`, `H264_MAIN`, `H264_BASELINE`, or `H264_HIGH`. Passed through `CameraBase::parseProfile`. |
| `video_width` | int | `1280` | ISP output width — feeds both the encoder (`camera_->video`) and preview downscale. Increasing it gives preview more source detail to downscale from at the cost of more on-device memory. |
| `video_height` | int | `720` | ISP output height. |

The existing `enable_video`, `preview_width`, `preview_height`, and `fps`
params continue to work and retain their defaults.

## Running modes

`enable_video` (existing) and `h265_enable` (new) compose orthogonally:

| `enable_video` | `h265_enable` | Outcome |
|---|---|---|
| `true` | `false` | Today's behavior: raw `image_raw` + JPEG via `image_transport`. |
| `true` | `true` | **Alongside**: raw + JPEG + H.265. Best for rollout — operator can A/B and fall back to JPEG if the H.265 path misbehaves. Costs two XLinkOut streams on the OAK. |
| `false` | `true` | **Replace**: H.265 only. Minimum device-to-host traffic. Preview is still available on the device for NN tapping (e.g. `sea_surface_segmentation`). |
| `false` | `false` | Neither — valid for NN-only nodes. |

Rollout order (per `unh_echoboats_project11#78`): enable `alongside` first,
then flip `enable_video=false` once the operator-station decoder is verified.

## Launch recipe (alongside mode)

For `wide_stereo`:

```python
Node(
    package='depthai_marine',
    executable='wide_stereo',
    parameters=[{
        'right_camera_id': '...',
        'left_camera_id':  '...',
        'h265_enable': True,
        'h265_bitrate_kbps': 4000,
        'h265_keyframe_frequency_frames': 30,
        'h265_profile': 'H265_MAIN',
        'video_width': 1920,
        'video_height': 1080,
    }],
)
```

For `sea_surface_segmentation`, the same four `h265_*` and two `video_*`
params are declared on the segmentation node and propagate to every camera
via `CameraParams`.

## Subscribing

Standard `ffmpeg_image_transport` pattern. The `encoding` string drives the
decoder selection:

```python
Node(
    package='image_transport',
    executable='republish',
    arguments=['ffmpeg', 'raw'],
    remappings=[
        ('in/ffmpeg', '<camera>/image_raw/ffmpeg'),
        ('out',       '<camera>/image_raw/decoded'),
    ],
)
```

## Hardware considerations

The `camera_->video` → `VideoEncoder` → `XLinkOut("ffmpeg")` branch runs in
parallel with the existing `camera_->preview` → `XLinkOut("camera")` branch
when `enable_video=true` and `h265_enable=true`. On multi-camera platforms,
confirm USB / PoE bandwidth headroom during calibration
(`unh_echoboats_project11#78`); if contention emerges, go straight to replace
mode.
