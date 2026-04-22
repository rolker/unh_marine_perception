# Plan: On-device H.265 encoding in depthai_marine for OAK cameras

## Issue

https://github.com/rolker/unh_marine_perception/issues/4

## Context

Follow-up to #2 / PR #3 (software H.265 benchmark harness, merged). This plan
adds an on-device (Myriad X) `VideoEncoder` branch to `depthai_marine`'s
`CameraBase` pipeline so OAK cameras produce an H.265 bitstream directly and
publish it as `ffmpeg_image_transport_msgs::msg::FFMPEGPacket`. The goal is
the best operator-facing video at the lowest over-the-air bandwidth.

Scope is **split** per the review on the issue and confirmation from the user:

- **This PR (code-only, `unh_marine_perception`)**: `CameraBase` H.265 branch,
  new `H265Publisher`, ROS params (including tunable video-side resolution),
  `package.xml`/CMake deps, non-hardware tests, short docs.
- **Out of this PR (hardware-gated, tracked on `unh_echoboats_project11#78`)**:
  izzyboat/bizzyboat launch + `udp_bridge` config updates, bag-based
  HW-vs-SW calibration writeup, platform rollout.

Investigation on `depthai_ros_driver` (jazzy, v2.12.2) confirmed the driver
already exposes `i_low_bandwidth_profile: H265_MAIN` for plain-streaming
cameras, but its `Segmentation` NN family is hardcoded to DeepLab argmax —
incompatible with `sea_surface_segmentation`'s 3-class softmax-as-RGB output
that `sea_surface_layer` / `segments_to_pointcloud` consume. So the
`depthai_marine` H.265 branch is still needed for segmentation cameras; the
driver shortcut covers only detection/streaming cameras and is handled on the
platform side.

## Design Decisions (resolved with user)

1. **Encoder source stream**: `camera_->video` (NV12 ISP output), not
   `preview`. The encoder runs on a separate tap from the preview→host path,
   so the two streams are independent.
2. **Video/ISP resolution is tunable** via new ROS params (`video_width`,
   `video_height`), backing `camera_->setSize()`. Default preserves today's
   behavior (1280×720); operators who want higher-quality OTH frames bump to
   1920×1080 or sensor-native in their launch YAML.
3. **PTS derivation**: `dai::ImgFrame::getTimestamp()` converted to
   microseconds (`AV_TIME_BASE`-native). Same source as the sibling
   `sensor_msgs/Image`'s `header.stamp`, so the two time signals on the
   packet remain consistent.
4. **Alongside vs. replace via orthogonal flags, no mode enum**: existing
   `enable_video` controls the raw-preview host publication; new `h265_enable`
   controls the H.265 path. All four combinations are valid:

   | `enable_video` | `h265_enable` | Outcome |
   |---|---|---|
   | true | false | Today's behavior (raw + JPEG via image_transport) |
   | true | true | Alongside (maximum rollback safety) |
   | false | true | H.265 only (minimum bandwidth) |
   | false | false | Neither (valid for NN-only nodes) |

   Platform rollout (#78) enables alongside first, then flips
   `enable_video=false` once H.265 is proven on the operator side.

## Approach

1. **Extend `CameraBase` public surface** — add opt-in params and setters so
   subclasses (`MainCamera`/`SecondaryCamera` in `wide_stereo`,
   `SegmentorCamera` in `sea_surface_segmentation`) can forward ROS params
   through:
   - `h265_enable` (bool, default false)
   - `h265_bitrate_kbps` (int, default 4000 — per PR #3 recommendation)
   - `h265_keyframe_frequency_frames` (int, default 30)
   - `h265_profile` (string, default `H265_MAIN`; also accept `H264_MAIN`,
     `H264_BASELINE`, `H264_HIGH`)
   - `video_width` (int, default 1280 — preserves today's `setSize`)
   - `video_height` (int, default 720)
2. **Add the VideoEncoder branch in `getPipeline()`** — replace the hardcoded
   `camera_->setSize(1280, 720)` with `setSize(video_width_, video_height_)`.
   When `h265_enable_` is true, create `dai::node::VideoEncoder` linked from
   `camera_->video` (independent of the existing `camera_->preview` path, so
   both streams can run simultaneously when `enable_video_` is also true).
   Create a second `XLinkOut` stream named `"h265"` fed by the encoder's
   `bitstream` output. No change to the preview/JPEG path.
3. **Add `H265Publisher`** — new class in `depthai_marine` analogous to
   `ImagePublisher` but publishing
   `ffmpeg_image_transport_msgs::msg::FFMPEGPacket` on
   `<topic>/image_raw/ffmpeg`. Each `dai::ImgFrame` from the encoder queue
   becomes one FFMPEGPacket:
   - `data` = encoder bitstream bytes (NAL units as emitted by the encoder)
   - `encoding` = `hevc` for H.265 profiles, `h264` for H.264 profiles
   - `pts` / `dts` = `duration_cast<microseconds>(frame->getTimestamp().time_since_epoch()).count()`
     — derived from `getTimestamp()`, not `getSequenceNum()`, so it stays
     consistent with `header.stamp`
   - `width` / `height` = `video_width_` / `video_height_`
   - `header.stamp` = ROS-converted device timestamp (matches the sibling
     `sensor_msgs::Image`)
   - `flags` = keyframe bit from `dai::ImgFrame` metadata
   Instantiated in `CameraBase::initialize()` alongside `camera_publisher_`
   when `h265_enable_` is true.
4. **Wire params through callers** — in `wide_stereo.cpp` and
   `sea_surface_segmentation.cpp`, declare the six new ROS params and
   forward to the camera subclass constructors via the existing setter
   pattern (`enableVideo`, `setFps`, etc.).
5. **CMake / package.xml** — add `ffmpeg_image_transport_msgs` dependency;
   link against it in `depthai_marine_lib`. `ffmpeg_image_transport` was
   already declared as `exec_depend` from PR #3.
6. **Non-hardware tests** — add a GTest target that:
   - Parses each supported `h265_profile` string into the correct
     `dai::VideoEncoderProperties::Profile` enum.
   - Validates param bounds (bitrate > 0, keyframe_freq > 0,
     `video_width`/`video_height` > 0 and even).
   - Optionally asserts the pipeline graph can be constructed with
     `h265_enable=true` without a connected device (pipeline build is a pure
     C++ operation; `dai::Device` construction is not needed). If DepthAI's
     graph validation requires a device, skip this sub-test and document why.
7. **Docs** — short `depthai_marine/docs/h265_transport.md` capturing:
   - Topic contract: `<camera>/image_raw/ffmpeg`, `FFMPEGPacket.encoding`
     string values, keyframe/GOP semantics, PTS derivation (microseconds
     from device timestamp).
   - Param reference (all six).
   - Enable / alongside / replace recipes for launch files, with caveat
     about subscribers needing `ffmpeg_image_transport` plugin.
   - Note that changing `video_width`/`video_height` affects the ISP source
     resolution that both `video` (encoder) and `preview` (raw/JPEG) derive
     from — increasing it gives preview more source detail to downscale
     from; decreasing it reduces both paths' quality.
   Link from `depthai_marine/docs/API.md` (or extend it).

## Files to Change

| File | Change |
|------|--------|
| `depthai_marine/include/depthai_marine/camera_base.hpp` | Add H.265 and video-size member vars and setter methods. |
| `depthai_marine/src/camera_base.cpp` | Replace hardcoded 1280×720 in `setSize`; add `VideoEncoder` + `XLinkOut("h265")` branch in `getPipeline()`; instantiate `H265Publisher` in `initialize()`. |
| `depthai_marine/include/depthai_marine/h265_publisher.hpp` | New: publisher class consuming `dai::ImgFrame` bitstream, emitting `FFMPEGPacket` with `getTimestamp()`-derived PTS. |
| `depthai_marine/src/h265_publisher.cpp` | New: implementation. |
| `depthai_marine/src/wide_stereo.cpp` | Declare 6 new ROS params and forward to `MainCamera` / `SecondaryCamera`. |
| `sea_surface_segmentation/src/sea_surface_segmentation.cpp` | Declare 6 new ROS params and forward to `SegmentorCamera` constructor. |
| `depthai_marine/package.xml` | Add `<depend>ffmpeg_image_transport_msgs</depend>`. |
| `depthai_marine/CMakeLists.txt` | `find_package(ffmpeg_image_transport_msgs)`, link into `depthai_marine_lib`, add test target. |
| `depthai_marine/test/test_h265_params.cpp` | New: GTest covering profile parsing, bounds, and pipeline-graph construction. |
| `depthai_marine/docs/h265_transport.md` | New: topic contract, param reference, launch recipes. |
| `depthai_marine/docs/API.md` | Link to new h265_transport doc; add the new topic. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | All knobs are ROS params; `h265_enable=false` and default video size unchanged mean zero out-of-the-box behavior change. Rollout is diffable via launch YAML. |
| Only what's needed | Single opt-in branch, reuses existing `BridgePublisher`/ImageConverter patterns. No mode enum — orthogonal `enable_video` + `h265_enable` flags cover alongside/replace without new abstractions. |
| Improve incrementally | PR is code-only; platform rollout + calibration split into `unh_echoboats_project11#78`. |
| Test what breaks | Profile-string → enum parsing and param bounds are the obvious regression risks; covered by unit test. Hardware-in-the-loop tests correctly deferred. |
| A change includes its consequences | New topic (`image_raw/ffmpeg`) and new deps covered in-PR. Downstream consumers (operator station, `udp_bridge`) enumerated on the platform issue, not this one. |
| Capture decisions, not just implementations | Wire-format contract lives in `docs/h265_transport.md` so operator-station authors don't have to reverse-engineer it. The four design decisions above are captured in this plan itself. |
| Workspace vs project separation | All changes stay in `unh_marine_perception`; workspace repo untouched. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Layer worktree created at `layers/worktrees/issue-unh_marine_perception-4/`; branch `feature/issue-4` off `jazzy`. |
| 0003 — Project-agnostic workspace | No | Changes are project-local. |
| 0008 — Follow ROS 2 conventions | Yes | Topic suffix `ffmpeg` matches the `image_transport` plugin name; codec identifier lives in `FFMPEGPacket.encoding` per the `ffmpeg_image_transport` contract. No deviation ADR needed. `package.xml` dep tags follow REP-140. |
| 0009 — Python package management | No | C++ change. Benchmarks subtree already has its own `requirements.txt` and is untouched. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `depthai_marine/package.xml` | `CMakeLists.txt` link list + `find_package` | Yes |
| Public `CameraBase` surface (new params) | All subclasses: `MainCamera`/`SecondaryCamera` in `wide_stereo`, `SegmentorCamera` in `sea_surface_segmentation` | Yes |
| New topic `image_raw/ffmpeg` | Operator-station consumers (bag recorder, dashboard, `udp_bridge`) | Out of scope for this PR — tracked on `unh_echoboats_project11#78` |
| API of `depthai_marine` | `depthai_marine/docs/API.md` + new `h265_transport.md` | Yes |

**Workspace grep completed** (per memory's API-change feedback): the only in-workspace consumers of `CameraBase` are `wide_stereo.cpp` and `sea_surface_segmentation.cpp`, both in this same repo and both covered above.

## Deferred to Hardware Calibration (#78)

These questions cannot be answered without an OAK on the bench or platform:

- **Simultaneous-stream bandwidth**: whether `enable_video=true, h265_enable=true` on 4 cameras at full FPS sustains both streams without Myriad X or USB/PoE contention. If it bites, platform rollout skips the alongside step and goes straight to replace mode.
- **Device-clock monotonicity**: the plan uses `getTimestamp()` for PTS without guarding against clock regressions. If field data shows non-monotonic frames, a follow-up adds a "last-pts, advance on regression" shim.
- **Actual vs. target bitrate**: the hypothesis that HW rate control hits target more precisely than libx265 ultrafast (which under-delivered by ~17× in PR #3). Calibration on a bag of bizzyboat footage will answer this; results feed back into default `h265_bitrate_kbps` tuning.

## Estimated Scope

Single PR. Roughly:

- ~150 lines new C++ (`h265_publisher.{hpp,cpp}` + `camera_base` extension)
- ~40 lines param/forwarding updates across both callers
- ~80 lines of test
- ~100 lines of docs

Fits comfortably in one review.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.7 (1M context)`
