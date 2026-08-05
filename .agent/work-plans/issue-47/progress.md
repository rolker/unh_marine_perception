---
issue: 47
---

# Issue #47 — Dynamic h265_bitrate_kbps with automatic pipeline restart

## Issue Review
**Status**: complete
**When**: 2026-08-05 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #47
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Summary

The issue proposes making `h265_bitrate_kbps` a dynamic ROS 2 parameter on nodes that own OAK cameras. Because `dai::node::VideoEncoder` (DepthAI v2 / RVC2) has no runtime bitrate control, the only achievable mechanism is an in-place pipeline restart: tear down publishers and `device_`, rebuild via `CameraBase::getPipeline()`, reconnect using the existing 5×2s retry loop. The per-camera outage is ~3–6 s; each camera is its own process so only the changed stream blanks.

### Scope Assessment

**Well-scoped?** Yes — the issue has a clear design (deferred restart via one-shot timer, serialized for MultiThreadedExecutor), explicit out-of-scope items (GOP gating, platform config), and explicit deliverables (param validation, unit tests, `docs/h265_transport.md` update). The two-package surface area (`depthai_marine` + `sea_surface_segmentation`) is bounded.

**Right repo?** Yes — `unh_marine_perception`. No workspace infrastructure changes required.

**Dependencies?** None identified. The existing reconnection retry loop and `CameraBase::getPipeline()` path are already in place.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Operator UX is explicit (`ros2 param set`); restart is documented with expected ~3–6 s outage per camera. |
| Enforcement over documentation | Watch | Concurrent-restart serialization must be enforced with a mutex, not just documented. The deferred-restart pattern needs a clear guard. |
| Capture decisions, not just implementations | Action needed | The hardware constraint (no live encoder dial on RVC2/depthai-v2) and the deferred-restart design choice are significant. They should be captured in `docs/h265_transport.md` or a lightweight ADR, not just in the commit message. |
| A change includes its consequences | Action needed | `ImagePublisher`'s `BridgePublisher` owns a thread with no explicit destructor — teardown safety must be verified and documented before trusting the restart path. `SegmentorCamera`'s `segmentation_queue_` (NN output) is outside `CameraBase`; subclasses must participate in teardown. The SeaSurfaceLayer stale-data question is flagged "confirm" in the issue — it must be confirmed, not deferred, since the costmap feeding collision-avoidance is safety-relevant. |
| Only what's needed | OK | Out-of-scope items are correctly deferred. |
| Improve incrementally | OK | This is a single, bounded feature. Each camera being its own process limits blast radius. |
| Test what breaks | Action needed | Unit tests (device-free) cover param validation plumbing but not restart-under-concurrency. The restart safety properties need at minimum a mock-device integration test or a documented manual procedure. |
| Workspace vs. project separation | OK | Entirely in `unh_marine_perception`. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 — Adopt ADRs | Watch | The deferred-restart design (hardware constraint + chosen approach) is non-obvious enough to warrant a lightweight ADR or thorough `docs/h265_transport.md` section; implementor's call whether a new ADR is warranted vs. a doc section. |
| ADR-0008 — Follow ROS 2 Conventions | Yes | Dynamic parameters must use `add_on_set_parameters_callback` / `rcl_interfaces::msg::SetParametersResult`; issue correctly calls this out. |
| ADR-0013 — progress.md entry-type vocabulary | Yes | This entry. |

### Consequences

- **`docs/h265_transport.md`** must be updated in the same PR: add a section on dynamic bitrate, the restart semantics, the ~3–6 s outage, and any caveats for `sea_surface_segmentation`.
- **`ImagePublisher` destructor safety**: `BridgePublisher` spawns a thread (`addPublisherCallback()`). If it does not join cleanly when the `shared_ptr` is reset, there is a use-after-free during restart. This must be verified against the installed `depthai_bridge` version before merging — the issue flags it but does not resolve it.
- **`SegmentorCamera` subclass teardown**: `SegmentorCamera` holds `segmentation_queue_` (NN output queue), which is not tracked by `CameraBase`. A `CameraBase`-level restart method cannot clean this up. The subclass must override the restart hook (or `CameraBase` must provide a virtual teardown extension point) so all queues are released before `device_` is destroyed.
- **SeaSurfaceLayer stale-data**: During the ~3–6 s restart window the segmentation topic goes dark. The occupancy buffer's log-odds decay must be slow enough that obstacles accumulated before the restart do not fully decay during the gap. The issue says "confirm" — this confirmation is a hard prerequisite for the safety claim, not a follow-up.
- **Parameter documentation**: `h265_bitrate_kbps` will change from a static to a dynamic parameter; the `docs/h265_transport.md` params table and any launch-file comments referencing it as baked-in must be updated in the same PR.

### Recommendations

- Before implementing, run `BridgePublisher` teardown under valgrind or with ASAN on the existing code path to confirm no thread-join issue; document the result in the PR.
- Consider whether `CameraBase` should expose a `virtual void onBeforeRestart()` / `virtual void onAfterRestart()` hook so `SegmentorCamera` can participate cleanly without duplicating the restart sequence.
- The one-shot timer approach is correct for deferral; make the serialization mutex a `std::mutex` member of `CameraBase` (or the owning node class) and lock it in both the parameter callback and the timer callback to prevent concurrent restarts if the user fires rapid `param set` calls.
- Add a `RCLCPP_WARN` at the start of the restart (naming the camera, old bitrate, new bitrate) and a `RCLCPP_INFO` on successful reconnect so operators have a clear log trail.

### Actions
- [ ] Capture the hardware constraint (no live encoder dial on RVC2/depthai-v2) and deferred-restart design decision in `docs/h265_transport.md` (or a new ADR).
- [ ] Verify `ImagePublisher`/`BridgePublisher` thread teardown safety; resolve before merging.
- [ ] Ensure `SegmentorCamera` participates in teardown (add virtual hook or subclass override for `segmentation_queue_`).
- [ ] Confirm SeaSurfaceLayer stale-data / occupancy-decay behavior during the restart window; document result in the PR.
- [ ] Add integration or mock-device test covering restart-under-concurrent-callbacks (unit test alone is insufficient for this safety property).

## Plan Authored
**Status**: complete
**When**: 2026-08-05 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-47/plan.md` at `29ba53b`
**Branch**: feature/issue-47 at `29ba53b`
**Phases**: single

### Open questions
- [ ] `wide_stereo` hosts two cameras on one node — single `h265_bitrate_kbps` param change restarts both streams simultaneously. Confirm this is acceptable, or add per-camera bitrate params if independent tuning is needed.
