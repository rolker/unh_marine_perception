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
  Net: the fix is the **one** Image stamp.

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
   stamp tracks the live ROS clock that `/tf` uses — see Open Question 1 for the
   anchoring-strategy trade-off and the deferred camera-publisher frozen-anchor.
5. **Add `test/test_segmentation_stamp.cpp`** (gtest): feed synthetic anchors + a
   device timestamp offset by a known Δ, assert the returned stamp equals the
   expected capture-derived ROS time and is **not** ≈ `now()`. Register in
   `CMakeLists.txt` (`ament_add_gtest`), link `depthai_bridge` for `getFrameTime`.
6. **Manual/offline verification** (cannot run on-device in CI): re-run
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

## Open Questions

- **Anchoring strategy (recommend: re-anchor).** Per-message `updateBaseTime`
  re-syncs the anchor to the live ROS clock each frame, so the projection-critical
  segmentation stamp tracks the same clock `/tf` uses (robust to NTP slew). The
  alternative — freeze the anchor at construction to exactly match the camera
  publishers — keeps seg-vs-camera-raw stamps identical but reintroduces a slow
  stale-pose drift under clock slew (the camera raw isn't projected, so this matters
  less). The run showed no slew (camera/tf flat to 0.3 ms over 1021 s), so both fix
  the 123 ms bug; re-anchor is the safer default. **Confirm.**
- **Camera-publisher frozen-anchor follow-up.** The issue flags a *separate* latent
  bug: `ImagePublisher`/`FFMPEGPublisher` converters freeze their anchor at
  construction and never re-sync. It did not fire in this run. Out of scope here
  (touches `depthai_marine`, expands the diff near the freeze) — **surface as a new
  issue** rather than silently bundling or dropping it. Agreed to defer?
- **Freeze timing.** June 4 dev freeze; small high-value change for the June 15
  survey. Confirm this lands before the freeze.

## Estimated Scope

Single PR (one stamp fix + pure helper + one gtest). On-device / tuner verification
is a manual follow-up, not a CI gate.
