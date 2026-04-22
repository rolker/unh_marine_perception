# Plan: On-device H.265 encoding in depthai_marine for OAK cameras

## Issue

https://github.com/rolker/unh_marine_perception/issues/4

## Context

Follow-up to #2 / PR #3 (software H.265 benchmark harness, merged). This plan
adds an on-device (Myriad X) `VideoEncoder` branch to `depthai_marine`'s
`CameraBase` pipeline so OAK cameras produce an H.265 bitstream directly and
publish it as `ffmpeg_image_transport_msgs::msg::FFMPEGPacket`. The JPEG path
stays in place for rollout safety.

Scope is **split** per the review on the issue and confirmation from the user:

- **This PR (code-only, `unh_marine_perception`)**: `CameraBase` H.265 branch,
  new `H265Publisher`, ROS params, `package.xml`/CMake deps, non-hardware
  tests, short docs.
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

## Approach

1. **Extend `CameraBase` public surface** — add opt-in H.265 params and
   setters so subclasses (`wide_stereo`'s `MainCamera`, `SegmentorCamera`)
   can forward ROS params through:
   - `h265_enable` (bool, default false)
   - `h265_bitrate_kbps` (int, default 4000)
   - `h265_keyframe_frequency_frames` (int, default 30)
   - `h265_profile` (string, default `H265_MAIN`; also accept `H264_MAIN`,
     `H264_BASELINE`, `H264_HIGH`)
2. **Add the VideoEncoder branch in `getPipeline()`** — when `h265_enable_`
   is true, create `dai::node::VideoEncoder` linked from `camera_->video`
   (separate from the existing `camera_->preview` path so both streams run
   simultaneously). Create a second `XLinkOut` stream named `"h265"` fed by
   the encoder's `bitstream` output. No change to the preview/JPEG path.
3. **Add `H265Publisher`** — new class in `depthai_marine` analogous to
   `ImagePublisher` but publishing `ffmpeg_image_transport_msgs::msg::FFMPEGPacket`
   on `<topic>/image_raw/ffmpeg`. Each `dai::ImgFrame` from the encoder queue
   becomes one FFMPEGPacket:
   - `data` = encoder bitstream bytes (NAL units as emitted by the encoder)
   - `encoding` = `hevc` for H.265 profiles, `h264` for H.264
   - `pts` / `dts` = derived from `dai::ImgFrame::getSequenceNum()` and
     `getTimestamp()` (NTP-consistent clock)
   - `width` / `height` = configured encoder frame size
   - `header.stamp` = ROS-converted frame timestamp (match the
     `sensor_msgs::Image` sibling)
   Instantiated in `CameraBase::initialize()` alongside `camera_publisher_`
   when `h265_enable_` is true.
4. **Wire params through `wide_stereo`** — declare the four params on the
   node, pass into `MainCamera` / `SecondaryCamera` via the existing
   setter pattern (`enableVideo`, `setFps`, etc.). Same wiring in
   `sea_surface_segmentation` (in the `SegmentorCamera` constructor).
5. **CMake / package.xml** — add `ffmpeg_image_transport_msgs` dependency;
   link against it in `depthai_marine_lib`. `ffmpeg_image_transport` becomes
   a genuine runtime dep (already declared as `exec_depend` from PR #3).
6. **Non-hardware tests** — add a GTest target that:
   - Parses each supported `h265_profile` string into the correct
     `dai::VideoEncoderProperties::Profile` enum.
   - Validates param bounds (bitrate > 0, keyframe_freq > 0).
   - Optionally asserts the pipeline graph can be constructed with
     `h265_enable=true` without a connected device (pipeline build is a pure
     C++ operation; `dai::Device` construction is not needed). If DepthAI's
     graph validation requires a device, skip this sub-test and document why.
7. **Docs** — short `depthai_marine/docs/h265_transport.md` capturing:
   - Topic contract: `<camera>/image_raw/ffmpeg`, `FFMPEGPacket.encoding =
     "hevc"`, keyframe/GOP semantics, PTS derivation.
   - Param reference.
   - "Enable" one-liner for launch files, with caveat about subscribers
     using `ffmpeg_image_transport` plugin.
   Link from `depthai_marine/docs/API.md` (or extend it).

## Files to Change

| File | Change |
|------|--------|
| `depthai_marine/include/depthai_marine/camera_base.hpp` | Add H.265 member vars (`h265_enable_`, `h265_bitrate_kbps_`, `h265_keyframe_frequency_frames_`, `h265_profile_`) and setter methods. |
| `depthai_marine/src/camera_base.cpp` | Add `VideoEncoder` + `XLinkOut("h265")` branch in `getPipeline()`; instantiate `H265Publisher` in `initialize()`. |
| `depthai_marine/include/depthai_marine/h265_publisher.hpp` | New file: publisher class consuming `dai::ImgFrame` bitstream, emitting `FFMPEGPacket`. |
| `depthai_marine/src/h265_publisher.cpp` | New file: implementation. |
| `depthai_marine/src/wide_stereo.cpp` | Declare 4 new ROS params and forward to `MainCamera` / `SecondaryCamera`. |
| `sea_surface_segmentation/src/sea_surface_segmentation.cpp` | Declare 4 new ROS params and forward to `SegmentorCamera` constructor. |
| `depthai_marine/package.xml` | Add `<depend>ffmpeg_image_transport_msgs</depend>`. Promote `ffmpeg_image_transport` from `exec_depend` to `exec_depend` (keep). |
| `depthai_marine/CMakeLists.txt` | `find_package(ffmpeg_image_transport_msgs)`, link into `depthai_marine_lib`, add test target. |
| `depthai_marine/test/test_h265_params.cpp` | New: GTest covering profile parsing and pipeline-graph construction. |
| `depthai_marine/docs/h265_transport.md` | New: topic contract, param reference, enable snippet. |
| `depthai_marine/docs/API.md` | Link to the new h265_transport doc; add the new topic. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | All knobs are ROS params; `h265_enable=false` by default; JPEG path untouched. Behavior is easy to diff. |
| Only what's needed | Single opt-in branch, reuses existing `BridgePublisher`/ImageConverter patterns. No new abstraction layers; `H265Publisher` mirrors `ImagePublisher` 1:1. |
| Improve incrementally | PR is code-only; platform rollout + calibration is split into `unh_echoboats_project11#78`. |
| Test what breaks | Profile-string → enum parsing is the obvious regression risk; covered by unit test. Hardware-in-the-loop tests correctly deferred to the platform issue. |
| A change includes its consequences | New topic (`image_raw/ffmpeg`) and new deps covered in-PR. Downstream consumers (operator station, `udp_bridge`) enumerated in the platform issue, not this one. |
| Capture decisions, not just implementations | Wire-format contract lives in `docs/h265_transport.md` so operator-station authors don't have to reverse-engineer it. |
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

**Workspace grep completed** (per the feedback in memory on API-change caution): the only in-workspace consumers of `CameraBase` are `wide_stereo.cpp` and `sea_surface_segmentation.cpp`, both in this same repo and both covered above.

## Open Questions

1. **Encoder source stream** — `dai::node::Camera` has both `preview` (resized, currently used by `BridgePublisher`) and `video` (full-resolution NV12). Plan assumes `video` is the right input for `VideoEncoder` and that both can be tapped concurrently on the same `Camera` node in DepthAI 2.x. If hardware testing shows contention, the fallback is to route `preview`→`VideoEncoder` and accept the lower resolution. Confirm during hardware calibration (tracked in `unh_echoboats_project11#78`).
2. **PTS derivation** — derive from `dai::ImgFrame::getSequenceNum()` (monotonic at camera FPS) or from `getTimestamp()` (wall-clock)? `ffmpeg_image_transport`'s decoder accepts either; using `getTimestamp()` preserves sensor-time semantics across the bridge but requires consistent conversion. Current intent: PTS = `getTimestamp()` in encoder ticks, matching the ROS `header.stamp`.
3. **Whether to publish `FFMPEGPacket` *instead of* or *alongside* the sibling `sensor_msgs/Image`** — plan says alongside during rollout, which means the Myriad X runs the encoder in parallel with the preview stream. If hardware benchmarking shows this is too expensive, a follow-up can add a mode where `h265_enable=true` replaces the JPEG path entirely. Not proposing that now.

## Estimated Scope

Single PR. Roughly:

- ~150 lines new C++ (`h265_publisher.{hpp,cpp}` + `camera_base` extension)
- ~30 lines param/forwarding updates in each caller
- ~80 lines of test
- ~100 lines of docs

Fits comfortably in one review.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.7 (1M context)`
