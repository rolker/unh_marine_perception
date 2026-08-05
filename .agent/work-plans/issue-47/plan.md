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
   `frame_id_` in `initialize()` so `restartPipeline()` can reuse them.

2. **Add virtual restart hooks to `CameraBase`** — `virtual void
   onBeforeRestart()` and `virtual void onAfterRestart()` with empty default
   implementations, called inside `restartPipeline()`.

3. **Implement `restartPipeline()` in `CameraBase`** — under `restart_mutex_`:
   reset `ffmpeg_publisher_`, `camera_publisher_`, and `device_`; call
   `onBeforeRestart()`; rebuild via `getPipeline()` + existing 5×2s retry loop;
   reconstruct publishers using stored identity; call `onAfterRestart()`. Log
   `RCLCPP_WARN` at start naming camera, old bitrate → new bitrate; log
   `RCLCPP_INFO` on successful reconnect.

4. **Register dynamic param callback in `CameraBase::initialize()`** — use
   `add_on_set_parameters_callback` for `h265_bitrate_kbps`. Validate > 0;
   return reject result otherwise. On accept: update `h265_bitrate_kbps_`,
   cancel any pending restart timer, schedule a one-shot 100 ms timer that
   calls `restartPipeline()` on the executor thread. `restart_mutex_` inside
   the timer serializes rapid `param set` calls.

5. **Override hooks in `SegmentorCamera`** — `onBeforeRestart()` resets
   `segmentation_queue_` and `segmentation_publisher_`; `onAfterRestart()`
   re-fetches the NN output queue from the new `device_` and re-registers the
   BridgePublisher callback.

6. **Add unit test `test_h265_bitrate_dynamic.cpp`** — device-free, in the
   spirit of `test_h265_params.cpp`: (a) param callback rejects ≤ 0 values via
   `rcl_interfaces` API; (b) accepts valid values; (c) rapid successive sets
   result in a single restart (timer coalescing, observable via counter mock or
   test with short timer durations). Register test in `CMakeLists.txt`.

7. **Update `docs/h265_transport.md`** — add a "Dynamic bitrate" section:
   hardware constraint (no live encoder dial on RVC2/depthai-v2), deferred-restart
   semantics, expected ~3–6 s outage, SeaSurfaceLayer gap behavior,
   operator UX example (`ros2 param set /oak_forward h265_bitrate_kbps 300`),
   note that all cameras in a node restart together. Update params table to
   label `h265_bitrate_kbps` as dynamic.

## Files to Change

| File | Change |
|------|--------|
| `depthai_marine/include/depthai_marine/camera_base.hpp` | Add `id_`, `label_`, `frame_id_` members; `restartPipeline()`; virtual hooks; `restart_mutex_`; `pending_restart_timer_`; param callback handle |
| `depthai_marine/src/camera_base.cpp` | Implement `restartPipeline()` and param callback registration in `initialize()` |
| `sea_surface_segmentation/src/sea_surface_segmentation.cpp` | Override `onBeforeRestart()` / `onAfterRestart()` in `SegmentorCamera` |
| `depthai_marine/test/test_h265_bitrate_dynamic.cpp` | New: param callback validation tests (device-free) |
| `depthai_marine/CMakeLists.txt` | Register `test_h265_bitrate_dynamic` test target |
| `depthai_marine/docs/h265_transport.md` | Add "Dynamic bitrate" section; mark `h265_bitrate_kbps` as dynamic in params table |

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
| `h265_bitrate_kbps` to dynamic param | `docs/h265_transport.md` params table | Yes — step 7 |
| `CameraBase::initialize()` stores identity | No signature change; subclass constructors unaffected | Yes — verified |
| Add restart path | `SegmentorCamera` must release NN queue before `device_` reset | Yes — step 5 |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `depthai_marine/docs/h265_transport.md`
  — add dynamic bitrate section; update params table to mark `h265_bitrate_kbps`
  as dynamic with restart semantics.
- **Agent-instruction candidates** (proposals only): None — the deferred-restart
  pattern is hardware-specific (DepthAI RVC2), not a broadly reusable workspace
  pattern worth adding to `.agent/knowledge/`.

## Open Questions

- [ ] `wide_stereo` hosts two cameras on one node: a single `h265_bitrate_kbps`
  param change restarts both streams simultaneously. Confirm this is acceptable,
  or add per-camera bitrate params (`right_h265_bitrate_kbps` /
  `left_h265_bitrate_kbps`) if independent tuning is needed.

## Estimated Scope

Single PR. ~250–350 lines changed/added across 6 files.
