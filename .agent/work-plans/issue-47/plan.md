# Plan: Dynamic h265_bitrate_kbps with automatic pipeline restart

## Issue

https://github.com/rolker/unh_marine_perception/issues/47

## Context

`CameraBase` reads `h265_bitrate_kbps` once at startup and bakes it into the
DepthAI pipeline at `dai::Device` construction. `dai::node::VideoEncoder` (RVC2
/ depthai-v2) has no runtime bitrate control, so live reconfiguration requires
tearing down and rebuilding the pipeline. The issue proposes making
`h265_bitrate_kbps` a dynamic ROS 2 parameter with an automatic deferred
restart (~3–6 s outage per camera).

**Teardown safety confirmed:**
- `FFMPEGPublisher::~FFMPEGPublisher()` removes its queue callback — safe.
- `BridgePublisher::~BridgePublisher()` calls `_readingThread.join()`.
  `ImagePublisher` uses `addPublisherCallback()` (not `startPublisherThread()`),
  so the thread is never started; join is a no-op. Safe to reset mid-flight.

**SeaSurfaceLayer gap confirmed safe:** the occupancy buffer decay half-life
defaults to 30 s. A 6 s restart gap causes ≤ 15% log-odds decay — existing
obstacles do not disappear during the restart window.

## Approach

1. **Store connection identity in `CameraBase`** — persist `id_`, `label_`,
   `resolved_frame_id_` in `initialize()` so `restartPipeline()` can reuse
   them. (Named `resolved_frame_id_`, storing the post-default-resolution
   value, to avoid shadowing `SegmentorCamera`'s existing private `frame_id_`
   — `sea_surface_segmentation.cpp:205`.)

2. **Add virtual restart hooks to `CameraBase`** — `virtual void
   onBeforeRestart()` and `virtual void onAfterRestart()` with empty default
   implementations, called inside `restartPipeline()`.

3. **Implement `restartPipeline()` in `CameraBase`** — under `restart_mutex_`,
   in this order (queues must be released before `device_` is destroyed):
   `onBeforeRestart()` → reset `ffmpeg_publisher_` / `camera_publisher_` →
   reset `device_` → rebuild via `getPipeline()` + the shared connect helper
   (extracted from `initialize()`, 5×2s retry) → reconstruct publishers using
   stored identity → `onAfterRestart()`. **Connect failure must not throw into
   the executor**: catch it, log `RCLCPP_ERROR`, and schedule a retry via the
   restart timer (10 s) so a transiently-absent camera recovers when it
   returns. Log `RCLCPP_WARN` at start naming camera, old bitrate → new
   bitrate; log `RCLCPP_INFO` on successful reconnect.

4. **Opt-in dynamic-bitrate registration, decoupled from `initialize()`** —
   new public `CameraBase::enableDynamicBitrate()` registers an
   `add_on_set_parameters_callback` for `h265_bitrate_kbps` (registration
   needs only `node_`, so it is testable device-free). Validation lives in a
   static `CameraBase::validateBitrateKbps(int)` shared by the callback and
   `setH265BitrateKbps`. On accept: update `h265_bitrate_kbps_`; when
   `h265_enable_` is false, store only (INFO log, no restart — the value
   applies if H.265 is enabled later); otherwise cancel any pending restart
   timer and schedule a one-shot 100 ms timer (coalesces rapid sets) that
   calls protected `virtual doRestart()` (default: `restartPipeline()`; the
   virtual is the device-free test seam for coalescing). **Adopters opt in
   explicitly**: `sea_surface_segmentation` calls it; `wide_stereo` does NOT
   (see step 5b).

5. **Override hooks in `SegmentorCamera`** — `onBeforeRestart()` resets
   `segmentation_queue_` and `segmentation_publisher_`; `onAfterRestart()`
   re-fetches the NN output queue from the new `device_` and re-registers the
   BridgePublisher callback.

5b. **`wide_stereo` is excluded from dynamic restart** (decision at the plan
   checkpoint, 2026-08-05): it hosts two devices on one node with a
   cross-device left→right forwarding queue (`wide_stereo.cpp:130`) that a
   device rebuild would silently invalidate. It keeps today's read-once
   behavior by simply not calling `enableDynamicBitrate()`. Document the
   exclusion in `docs/h265_transport.md`.

6. **Expose the param to the operator station via `marine_control`**
   (scope addition, user request 2026-08-05): `udp_bridge` carries only
   pub/sub, so `ros2 param set` is invisible topside. In the
   `SeaSurfaceSegmentation` node: declare `h265_bitrate_kbps` with an
   `IntegerRange` descriptor (100–10000 kbps, step 100 — UI/operational
   bounds), construct a `marine_control::ControlServer`, and
   `bind_parameter("h265_bitrate_kbps", "kbps", "video")`. The change path
   goes through the normal parameter machinery, so the deferred restart fires
   identically to `ros2 param set`. In-repo precedent:
   `segments_to_pointcloud.cpp`. Add the `marine_control` dependency to
   `sea_surface_segmentation`. (Bridge topic wiring on the platform is
   `unh_echoboats_project11` config — ADR-0003 D7 — a follow-up there, out of
   scope here.)

7. **Add unit test `test_h265_bitrate_dynamic.cpp`** — device-free, in the
   spirit of `test_h265_params.cpp` (which deliberately never calls
   `initialize()`): (a) `validateBitrateKbps` / callback rejects ≤ 0 values
   via the `rcl_interfaces` API; (b) accepts valid values; (c) rapid
   successive sets coalesce to a single restart — observable by overriding
   the `doRestart()` seam with a counter, never touching a device; (d) with
   `h265_enable=false`, a set stores the value but schedules no restart.
   Register test in `CMakeLists.txt`.

8. **Update `docs/h265_transport.md`** — add a "Dynamic bitrate" section:
   hardware constraint (no live encoder dial on RVC2/depthai-v2), deferred-restart
   semantics, expected ~3–6 s outage, failure/retry behavior, SeaSurfaceLayer
   gap behavior, operator UX (`ros2 param set /oak_forward h265_bitrate_kbps
   300` boat-side; marine_control panel topside), the `wide_stereo` exclusion,
   note that all cameras in a node restart together. Update params table to
   label `h265_bitrate_kbps` as dynamic.

## Files to Change

| File | Change |
|------|--------|
| `depthai_marine/include/depthai_marine/camera_base.hpp` | Add `id_`, `label_`, `resolved_frame_id_` members; `restartPipeline()`; `enableDynamicBitrate()`; `validateBitrateKbps()`; virtual hooks + `doRestart()` seam; `restart_mutex_`; `pending_restart_timer_`; param callback handle |
| `depthai_marine/src/camera_base.cpp` | Extract shared connect+publisher-build helper; implement `restartPipeline()` (no-throw, retry-on-failure) and `enableDynamicBitrate()` |
| `sea_surface_segmentation/src/sea_surface_segmentation.cpp` | Override `onBeforeRestart()` / `onAfterRestart()` in `SegmentorCamera`; call `enableDynamicBitrate()`; `IntegerRange` descriptor on `h265_bitrate_kbps`; `ControlServer` + `bind_parameter` |
| `sea_surface_segmentation/package.xml` | Add `marine_control` dependency |
| `sea_surface_segmentation/CMakeLists.txt` | Link `marine_control` |
| `depthai_marine/test/test_h265_bitrate_dynamic.cpp` | New: param callback validation + coalescing tests (device-free via `doRestart()` seam) |
| `depthai_marine/CMakeLists.txt` | Register `test_h265_bitrate_dynamic` test target |
| `depthai_marine/docs/h265_transport.md` | Add "Dynamic bitrate" section; mark `h265_bitrate_kbps` as dynamic in params table; document `wide_stereo` exclusion |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Explicit operator UX (`ros2 param set`); WARN + INFO log trail names camera and bitrates; outage documented |
| Enforcement over documentation | `restart_mutex_` enforces restart serialization; param callback enforces > 0 via `SetParametersResult` |
| Capture decisions, not just implementations | RVC2 hardware constraint + deferred-restart design captured in `docs/h265_transport.md` |
| A change includes its consequences | Teardown safety verified; NN queue teardown handled via virtual hooks; SeaSurfaceLayer gap confirmed safe; doc updated same PR |
| Only what's needed | Only `h265_bitrate_kbps` is dynamic; GOP gating and platform config are out of scope |
| Test what breaks | Device-free param validation tests; hardware-in-loop restart test documented as manual (no `dai::Device` mock available) |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 — Adopt ADRs | Watch | Hardware constraint and deferred-restart design captured in `docs/h265_transport.md`; new ADR not warranted given existing doc scope |
| ADR-0008 — Follow ROS 2 Conventions | Yes | `add_on_set_parameters_callback` + `rcl_interfaces::msg::SetParametersResult` pattern per ROS 2 dynamic parameter convention |
| ADR-0013 — progress.md | Yes | This plan + `progress.md` entry |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `h265_bitrate_kbps` to dynamic param | `docs/h265_transport.md` params table | Yes — step 8 |
| `CameraBase::initialize()` stores identity | No signature change; subclass constructors unaffected | Yes — verified |
| Add restart path | `SegmentorCamera` must release NN queue before `device_` reset (order enforced in step 3) | Yes — steps 3, 5 |
| Device rebuild invalidates cross-device queues | `wide_stereo`'s left→right forwarding would break — excluded from dynamic restart (step 5b) | Yes — step 5b |
| Param exposed via marine_control | Platform bridge wiring (`unh_echoboats_project11`, ADR-0003 D7) | Follow-up issue there, noted in PR |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `depthai_marine/docs/h265_transport.md`
  — add dynamic bitrate section; update params table to mark `h265_bitrate_kbps`
  as dynamic with restart semantics.
- **Agent-instruction candidates** (proposals only): None — the deferred-restart
  pattern is hardware-specific (DepthAI RVC2), not a broadly reusable workspace
  pattern worth adding to `.agent/knowledge/`.

## Open Questions

- [x] `wide_stereo` two-cameras-one-node topology — **resolved at the plan
  checkpoint (2026-08-05): excluded from dynamic restart** (step 5b). It keeps
  read-once bitrate; per-camera params not needed.

## Estimated Scope

Single PR. ~350–450 lines changed/added across 9 files.
