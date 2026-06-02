# Plan: Segmentation output stamped with now() instead of frame capture time

## Issue

https://github.com/rolker/unh_marine_perception/issues/28

## Context

`SegmentorCamera::segmentationCallback` builds its `Image` manually and stamps it
with wall-clock `now()` (`sea_surface_segmentation.cpp:162`), discarding the
source frame's device capture time. Measured offset: **+123.5 ms, constant**
(fixed pipeline latency, not jitter). Every consumer that does a TF lookup at that
stamp — `SeaSurfaceLayer`, `segments_to_pointcloud`, offline `sea_surface_tuner` —
resolves the camera pose 123.5 ms stale, which on a turning vehicle rotates the
back-projection enough that a clearly-segmented close buoy walks several costmap
cells/frame and never accumulates. This is the pose-timing half of the #26/#27
small-buoy marking gap; not fixable in the projection algorithm.

The callback already holds `in_det` (`dai::NNData`), which carries the device
capture timestamp via `in_det->getTimestamp()` — the same source the camera
publishers use. The established conversion pattern is `depthai_marine`'s
`measure_timing.cpp:256-257`:
`msg.header.stamp = dai::ros::getFrameTime(ros_base_time_, steady_base_time_, det->getTimestamp());`

**Verified during planning:**
- `CameraBase` does **not** hold `ros_base_time_`/`steady_base_time_` (they live
  inside the publishers' `ImageConverter`s). So `SegmentorCamera` must add its own.
- `BridgePublisher::publishHelper` copies the image stamp onto camera_info
  (`/opt/ros/jazzy/include/depthai_bridge/BridgePublisher.hpp:245`:
  `localCameraInfo.header.stamp = currMsg.header.stamp`). **So fixing the Image
  stamp auto-propagates to `segmentation/camera_info`.** The `segmentation/compressed`
  sibling is an image_transport republish of the same Image → inherits the stamp too.
- The depthai_bridge `ImageConverter` honors `setUpdateRosBaseTimeOnToRosMsg(true)`
  in **both** consumer paths — `toRosMsgRawPtr` (used by `ImagePublisher`'s
  `toRosMsg`) and `toRosFFMPEGPacket` (used by `FFMPEGPublisher`) each call
  `updateRosBaseTime()` when the flag is set (verified in upstream
  `depthai_bridge/src/ImageConverter.cpp`). So the camera-publisher frozen-anchor
  fix is a clean one-liner per publisher.

**Scope decision (Roland, 2026-06-02): bundle the frozen-anchor fix.** The camera
publishers (`ImagePublisher`/`FFMPEGPublisher` in `depthai_marine`) freeze their
ROS↔steady anchor at converter construction and never re-sync — the same class of
defect, latent in this run. Leaving wrong behavior wrong isn't justified by "blast
radius": correcting it is a bug fix, not a risky behavior change. So this PR also
flips both converters to per-message re-anchor. With the segmentation node *also*
re-anchoring (below), all three stamping paths use one consistent strategy, so the
`sea_surface_tuner` δ-sweep that correlates seg-vs-ffmpeg stays ≈0 even under clock
slew (the earlier seg-vs-camera divergence concern is thereby removed, not just
deferred).

## Approach

1. **Add time anchors to `SegmentorCamera`** — members `rclcpp::Time ros_base_time_`,
   `std::chrono::time_point<std::chrono::steady_clock> steady_base_time_`, and
   `int64_t total_ns_change_ = 0`, captured at construction
   (`ros_base_time_ = node->get_clock()->now(); steady_base_time_ = std::chrono::steady_clock::now();`),
   mirroring `measure_timing.cpp:130-131`.
2. **Extract a pure stamping helper** for testability — `src/segmentation_stamp.hpp`
   (header-only, sibling of `frame_id_resolver.hpp`):
   `rclcpp::Time deviceFrameStamp(rclcpp::Time & ros_base, steady_tp & steady_base, int64_t & total_ns_change, steady_tp device_tstamp)`
   that re-anchors (`dai::ros::updateBaseTime`) then returns `dai::ros::getFrameTime(...)`.
   Taking the already-extracted `time_point` (not the `NNData`) keeps it free of any
   device dependency so it unit-tests without hardware.
3. **Replace line 162** with
   `image_message.header.stamp = deviceFrameStamp(ros_base_time_, steady_base_time_, total_ns_change_, in_det->getTimestamp());`
4. **Enable the per-message re-anchor** (`updateBaseTime`, inside the helper) so the
   segmentation stamp tracks the live ROS clock that `/tf` uses (robust to clock
   slew). This is the chosen anchoring strategy (resolves former Open Question 1).
5. **Fix the camera-publisher frozen anchor (bundled).** Add
   `image_converter_->setUpdateRosBaseTimeOnToRosMsg(true);` in
   `depthai_marine/src/image_publisher.cpp` (after the converter is built, line 14)
   and `converter_->setUpdateRosBaseTimeOnToRosMsg(true);` in
   `depthai_marine/src/ffmpeg_publisher.cpp` (after line 25). One line each; both
   converter paths honor the flag (verified against the jazzy branch source — see
   Context). This is its **own commit** (`depthai_marine`), separate from the
   segmentation-package commit, to keep each logical change atomic within the one PR.
6. **Add `test/test_segmentation_stamp.cpp`** (gtest): feed synthetic anchors + a
   device timestamp offset by a known Δ, assert the returned stamp equals the
   expected capture-derived ROS time and is **not** ≈ `now()`. Register in
   `CMakeLists.txt` (`ament_add_gtest`), link `depthai_bridge` for `getFrameTime`.
   (The camera-publisher one-liners are flag flips on the upstream converter — not
   meaningfully unit-testable without hardware; covered by the offline bag re-run.)
7. **Manual/offline verification** (cannot run on-device in CI): re-run
   `sea_surface_tuner` on `bag_2026-05-29T15.56.42_ffmpeg_seg` — port horizon
   overlay should sit on the true horizon through the pier-departure turn, the
   ~t6 s yellow buoy should mark, and the δ-sweep best alignment should move from
   δ≈−123 ms to δ≈0.

## Files to Change

| File | Change |
|------|--------|
| `sea_surface_segmentation/src/sea_surface_segmentation.cpp` | Add `ros_base_time_`/`steady_base_time_`/`total_ns_change_` members + init; replace `now()` stamp at line 162 with `deviceFrameStamp(...)` |
| `sea_surface_segmentation/src/segmentation_stamp.hpp` (new) | Pure helper wrapping `dai::ros::updateBaseTime` + `getFrameTime` |
| `sea_surface_segmentation/test/test_segmentation_stamp.cpp` (new) | gtest: stamp derives from device tstamp, not `now()` |
| `sea_surface_segmentation/CMakeLists.txt` | Register the new gtest |
| `depthai_marine/src/image_publisher.cpp` | `image_converter_->setUpdateRosBaseTimeOnToRosMsg(true)` — fix frozen anchor |
| `depthai_marine/src/ffmpeg_publisher.cpp` | `converter_->setUpdateRosBaseTimeOnToRosMsg(true)` — fix frozen anchor |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Quality Standard — fix completely + test | Fixes the root cause (not the projection symptom); adds a deterministic CI test; siblings (camera_info/compressed) verified to auto-propagate, so none left stale. |
| Robustness for open-water autonomy | Correct pose timing is what lets the reflex/costmap actually mark close obstacles — directly safety-relevant for the buoy-miss class. |
| Verify docs/claims against source | BridgePublisher stamp-copy and CameraBase anchor-absence were verified in source during planning, not assumed. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 Follow ROS 2 conventions | Yes | Using the sensor's hardware capture timestamp in `header.stamp` (not host arrival time) is the ROS convention; mirrors the existing `depthai_marine` publishers. |
| ADR-0002 Worktree isolation | Yes | Work on `feature/issue-28`; this plan is the first commit. |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| Segmentation stamp (publish-time → capture-time) | `SeaSurfaceLayer`, `segments_to_pointcloud`, `sea_surface_tuner` all consume this stamp | No change needed — they *want* capture time; the fix is transparent and corrective |
| Stamp now ~123 ms earlier | TF lookups happen at an older time | Within tf buffer history and well within consumers' tolerances (costmap `transform_tolerance` 0.2 s, Collision Monitor `source_timeout` 1.0 s) — no lookup failures |
| `recv − stamp` latency metric grows ~123 ms | Any diagnostic that alarms on stamp age | Expected/correct (stamp is now the true capture time); no code asserts on this |
| camera_info / compressed siblings | Must carry corrected stamp | Auto-propagate (BridgePublisher:245 + image_transport) — verify in the tuner re-run |
| Camera-publisher re-anchor (`depthai_marine`) | Affects **all** consumers of the raw / ffmpeg-h265 / camera_info topics across every platform using `depthai_marine` (all OAKs, not just segmentation) | Correctness fix (stamps track live clock); in normal NTP-locked operation (no slew) stamps are unchanged. Verify raw/h265 stamps still look correct in the bag re-run |

## Open Questions

- ~~**Anchoring strategy?**~~ **Resolved (Roland, 2026-06-02):** per-message
  re-anchor (`updateBaseTime`) for all three stamping paths.
- ~~**Camera-publisher frozen-anchor: defer?**~~ **Resolved (Roland, 2026-06-02):
  bundle it.** Fixing wrong behavior is a bug fix, not a risky change — both
  `depthai_marine` converters get the re-anchor flag in this PR (own commit).
- **Freeze timing.** June 4 dev freeze; small high-value change for the June 15
  survey. Confirm this lands before the freeze.

## Estimated Scope

Single PR, **two atomic commits**: (1) segmentation stamp fix + pure helper + gtest
(`sea_surface_segmentation`); (2) camera-publisher re-anchor flag
(`depthai_marine`). On-device / tuner verification is a manual follow-up, not a CI
gate.
