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

## Plan Review
**Status**: complete
**When**: 2026-08-05 14:23 +00:00
**By**: Claude Code Agent (Claude Opus)
<!-- Independent review: the `## Plan Authored` entry shares this workspace's
single agent name ("Claude Code Agent") but was authored by a different model
(Claude Sonnet) in a separate context. This is an Opus fresh-context review, so
no author-self-review annotation is applied. -->

**Plan**: `.agent/work-plans/issue-47/plan.md` at `29ba53b`
**PR**: PR-less (`--issue 47` dispatch; branch `feature/issue-47`)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Teardown order inverted: step 3 resets `device_` **before** calling `onBeforeRestart()`, but the subclass hook must release its NN queue (`segmentation_queue_`) before `device_` is destroyed — exactly what the Issue Review required ("all queues released before `device_` destroyed"). Reorder to: `onBeforeRestart()` → reset base publishers → reset `device_`. — `plan.md:35`
- [ ] (must-fix) `wide_stereo` consequence unaddressed: `wide_stereo.cpp` runs two `CameraBase` subclasses (`MainCamera`/`SecondaryCamera`) on one node and pipes left→right frames via `MainCamera::getLeftImageInQueue()` (`wide_stereo.cpp:130`). Registering the param callback in `CameraBase::initialize()` gives both cameras automatic restart, but rebuilding `device_` invalidates that cross-device forwarding queue and silently breaks stereo sync. Neither `wide_stereo.cpp` nor this consequence is in the plan. Add restart hooks to re-wire forwarding (or exclude wide_stereo from dynamic restart), and add `wide_stereo.cpp` to the file list. — `plan.md:67`
- [ ] (must-fix) Restart reuses `initialize()`'s retry loop, which **throws** on connect failure (`camera_base.cpp:58-61`). Called from a one-shot timer on the executor, that exception propagates into the executor and can kill the node — a transient reconnect miss during a live bitrate change would be worse than the ~6 s outage it replaces. `restartPipeline()` must catch connect failure (log `RCLCPP_ERROR`, leave the camera down / schedule a retry) rather than throw. — `plan.md:35`
- [ ] (must-fix) Device-free param-callback test not achievable as written: step 4 registers the callback inside `initialize()`, which needs a live device (5×2 s retry, then throw). Existing device-free tests deliberately never call `initialize()` (`test_h265_params.cpp`). Decouple registration/validation from device connection (e.g. a static validator or a pre-connect registration method) so step 6's tests can run device-free. — `plan.md:42`
- [ ] (suggestion) `frame_id_` member shadow: `SegmentorCamera` already declares `private: std::string frame_id_;` (`sea_surface_segmentation.cpp:205`). Adding a protected `frame_id_` to `CameraBase` (step 1) creates two members of the same name. Name the base member distinctly (e.g. `resolved_frame_id_`) and store the **resolved** id (`label + "_optical_frame"`, `camera_base.cpp:70`) so restart's publishers match `initialize()`'s. — `plan.md:29`
- [ ] (suggestion) DRY: `restartPipeline()` duplicates `initialize()`'s retry loop + publisher construction (`camera_base.cpp:41-85`). Extract a shared private helper called by both so they can't drift. — `plan.md:35`
- [ ] (suggestion) Gate restart on `h265_enable_`: a bitrate change on a node with `h265_enable=false` would restart the whole pipeline (blanking video + NN) for no encoder benefit. Skip the restart (or the callback registration) when H.265 is disabled. — `plan.md:42`
- [ ] (suggestion) Coalescing test needs a seam: verifying "rapid sets → single restart" device-free requires the timer cancel/reschedule bookkeeping to be separable from the device-touching `restartPipeline()`. State that seam explicitly in step 6 rather than "counter mock." — `plan.md:56`
- [ ] (suggestion) ROS-convention nuance: `add_on_set_parameters_callback` is the pre-set *validation* hook; reacting to the accepted value is conventionally done via `add_post_set_parameters_callback`. Reading the new value from the callback's parameter vector (as planned) is acceptable, but note the pre/post distinction. — `plan.md:42`

### Summary

The plan is well-structured, correctly identifies the RVC2 no-live-dial constraint, and its deferred-restart-via-timer design is sound. But four must-fix issues block implementation: the teardown ordering is inverted relative to the Issue Review's stated safety requirement; the `wide_stereo` two-cameras-on-one-node topology (with its left→right stereo forwarding queue) is a real consequence the plan neither lists nor guards; reusing `initialize()`'s throwing retry loop can kill the node from a timer callback; and the device-free param-callback test can't run because registration is buried inside the device-dependent `initialize()`. All are concrete and addressable inline.

### Recommended Actions
- [ ] Reorder `restartPipeline()`: `onBeforeRestart()` → reset base publishers → reset `device_` → rebuild → reconstruct → `onAfterRestart()`.
- [ ] Add `wide_stereo.cpp` to the plan: either re-wire `getLeftImageInQueue()` forwarding after restart via hooks, or exclude wide_stereo from dynamic restart; capture the stereo-forwarding consequence in the Consequences table.
- [ ] Make `restartPipeline()` catch connect failure instead of throwing into the executor.
- [ ] Decouple param-callback registration/validation from `initialize()` so step 6's tests are genuinely device-free; state the coalescing test seam.
- [ ] (nice-to-have) Rename the base `frame_id_` to avoid shadowing; factor the shared connect+publisher-build helper; gate restart on `h265_enable_`.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-05 15:21 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-47 at `51fe64d`
**Mode**: pre-push
**Depth**: Deep (reason: concurrency + device-lifecycle teardown/rebuild, cross-package depthai_marine + sea_surface_segmentation + marine_control)
**Must-fix**: 1 | **Suggestions**: 5
**Round**: 1 | **Ship**: continue — one genuine use-after-free on the restart path warrants a fix + re-read

### Findings
- [x] (must-fix) Use-after-free on restart teardown: `BridgePublisher` (ImagePublisher `camera_publisher_` + `segmentation_publisher_`) never removes its DepthAI queue callback in `~BridgePublisher`, so resetting the publishers while `device_` still streams leaves a dangling `daiCallback` firing on the XLink thread until `device_.reset()`. Fix: quiesce device/queues before destroying those publishers, or add a removeCallback path like FFMPEGPublisher — `depthai_marine/src/camera_base.cpp:107-109`, `sea_surface_segmentation/src/sea_surface_segmentation.cpp:68-73`
- [x] (suggestion) Coalescing doesn't cover a differing-value set that arrives while a restart is already in progress → a second multi-second outage — `depthai_marine/src/camera_base.cpp:187-209`
- [x] (suggestion) `h265_enable_` read non-atomically in the apply callback while `h265_bitrate_kbps_` is atomic; safe today but asymmetric — `depthai_marine/src/camera_base.cpp:198`
- [x] (suggestion) Tests never exercise the production IntegerRange (100–10000); only validateBitrateKbps (>0) — add boundary/rejection cases — `depthai_marine/test/test_h265_bitrate_dynamic.cpp:49`
- [x] (suggestion) Layered bounds inconsistent (validateBitrateKbps 1..INT_MAX vs descriptor 100..10000) and reason string says only "must be > 0" — `depthai_marine/src/camera_base.cpp:164-166`
- [x] (suggestion) Plan/code drift: IntegerRange.step plan=100 vs code=1 — reconcile plan or code — `sea_surface_segmentation/src/sea_surface_segmentation.cpp:291`

### Notes
- Static analysis (ament_cpplint) produced only advisories (line-length on new lines; missing copyright on new test file) — none enforced by this repo's toolchain (packages have no ament_cpplint target; pre-commit checks whitespace/yaml/xml/cmake only) and the copyright omission matches the sibling test_h265_params.cpp. Not elevated.
- Local Adversarial skipped: no Ollama server at http://localhost:11434.
- Copilot Adversarial off (default). Two disjoint-lens Claude adversarial passes both independently confirmed the must-fix (cross-pass confirmed).
- Governance: compliant (ADR-0008 dynamic-param conventions, ADR-0013 progress vocabulary, marine_control project device-control ADR); docs + dependency land in-PR.

## Implementation
**Status**: complete
**When**: 2026-08-05 15:44 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-47 at `e6d7871`
**Addressed**: Local Review (Pre-Push) 2026-08-05 15:21 +00:00 at `51fe64d` (1 must-fix + 5 suggestions, all unchecked)
**Commits**: `a216124`, `b6b616c`, `f73baeb`, `e6d7871`

### Actions
- [x] (must-fix) Restart-path use-after-free: `restartPipeline()` now calls `device_->close()` before any publisher teardown, stopping the DepthAI output-queue reading threads so a `BridgePublisher` callback (which `~BridgePublisher` never removes) cannot fire into a half-destroyed publisher. This protects both `camera_publisher_` and `SegmentorCamera::segmentation_publisher_` (whose `onBeforeRestart()` teardown now also runs quiesced) — `depthai_marine/src/camera_base.cpp` (`a216124`)
- [x] (suggestion) Coalescing across an in-progress restart: track `applied_bitrate_kbps_` (the bitrate baked into the running pipeline, seeded in `initialize()`, updated on each successful restart) and short-circuit a restart that would re-apply it — so a set absorbed early by an in-progress restart's `getPipeline()` no longer triggers a second outage via its own scheduled timer. `target` is captured before `getPipeline()` reads the atomic, so a late set is never dropped (at worst one extra, correct restart) — `depthai_marine/src/camera_base.cpp` (`a216124`)
- [x] (suggestion) `h265_enable_` made `std::atomic<bool>` for symmetry with the atomic `h265_bitrate_kbps_` — `depthai_marine/include/depthai_marine/camera_base.hpp` (`b6b616c`)
- [x] (suggestion) Tests now exercise the production IntegerRange: the device-free tests declare `h265_bitrate_kbps` with the same descriptor (100–10000, step 100) and add rejection/boundary cases (99, 100, 10000, 10001) — `depthai_marine/test/test_h265_bitrate_dynamic.cpp` (`e6d7871`)
- [x] (suggestion) Layered bounds reconciled: `CameraBase::kH265Bitrate{Min,Max}Kbps` (100/10000) single-source `validateBitrateKbps`, the descriptor, and the callback/setter messages, which now report the real range — `depthai_marine/src/camera_base.cpp`, `sea_surface_segmentation/src/sea_surface_segmentation.cpp` (`f73baeb`)
- [x] (suggestion) Plan/code step drift fixed: descriptor `step` 1 → 100 (operator-UI granularity per the plan), docs updated — `sea_surface_segmentation/src/sea_surface_segmentation.cpp`, `depthai_marine/docs/h265_transport.md` (`f73baeb`)

### Notes
- Commit grouping (4 commits for 6 findings): the must-fix (UAF) and the coalescing suggestion are physically interleaved in `restartPipeline()` and shipped together (`a216124`); the two descriptor-consistency findings (bounds single-sourcing + step) both edit the same IntegerRange block and shipped together (`f73baeb`). No finding was deferred — all six are fixed-and-checked.
- Verification: `depthai_marine` built clean; `test_h265_bitrate_dynamic` (8 tests, incl. 2 new range tests) and `test_h265_params` (15 tests) pass. The `sea_surface_segmentation` node target compiles (its `marine_control` dep built from `layers/main/core_ws`); the package's full build is blocked only by an unrelated missing system dep (`libpcap.so` `-dev` symlink) needed by the separate `segments_to_pointcloud` target, pre-existing and untouched by this change.

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 47 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-08-05 16:03 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-47 at `26582c9`
**Mode**: pre-push
**Depth**: Deep (reason: device-lifecycle teardown/rebuild under MultiThreadedExecutor; cross-package depthai_marine + sea_surface_segmentation + marine_control)
**Must-fix**: 0 | **Suggestions**: 4
**Round**: 2 | **Ship**: recommended — no must-fix; Round-1's 1 must-fix + 5 suggestions all addressed and verified, code is byte-identical to the implementer's verified-build SHA e6d7871

### Findings
- [ ] (suggestion) h265_bitrate_kbps now carries IntegerRange [100,10000] step 100: out-of-range OR off-grid startup overrides now abort the node at declare_parameter (behavioral change); verify cross-repo platform configs are in-range + step-100-aligned. Single-sourcing covers min/max but not step — `sea_surface_segmentation/src/sea_surface_segmentation.cpp:284-297`
- [ ] (suggestion) Multi-camera-per-node: enableDynamicBitrate() registers validate+apply callbacks per camera on the shared node → one param set fans out to N restarts + N redundant validators; each restart blocks an executor thread up to 10s. Latent (1 cam/node today). Register node-level callbacks once / dedicated callback group — `sea_surface_segmentation/src/sea_surface_segmentation.cpp:335-352`
- [ ] (suggestion) Coalescing precision: applied_bitrate_kbps_=target may not equal the value getPipeline() bakes if a set lands between line 106 and the atomic read at line 276 — benign (<=1 extra correct restart, never a drop) but the "skips redundant restart" claim is imprecise — `depthai_marine/src/camera_base.cpp:106,173`
- [ ] (suggestion) Defensive: ~CameraBase doesn't cancel pending_restart_timer_ or reset the param-callback handles; unreachable in current main() but makes the class safe for reuse if a camera is destroyed while spinning — `depthai_marine/src/camera_base.cpp:252`

### Notes
- Two disjoint-lens Claude adversarial passes: Lens A found no must-fix; Lens B raised four "must-fix" claims, none survived verification — the restart_mutex_/timer_mutex_ AB/BA deadlock is a FALSE POSITIVE (the timer lambda's inner scope releases timer_mutex_ before doRestart() acquires restart_mutex_, camera_base.cpp:189-199); the residual-UAF is Round-1's accepted close-first guard (ImagePublisher destroys its BridgePublisher before its queue; the only stronger fix lives in upstream depthai_bridge); the shutdown-timer UAF is unreachable in the actual main(); the N-callback fan-out is intended. No cross-pass-confirmed must-fix.
- Static analysis (ament_cpplint): advisory line-length (>100) on new lines only; the two other hits (camera_base.hpp:37 explicit-ctor, sea_surface_segmentation.cpp:80 redundant-virtual) are on untouched lines and skipped. Not repo-enforced (no cpplint target; pre-commit checks whitespace/yaml/xml/cmake) — not elevated, consistent with Round 1.
- Local Adversarial skipped: no Ollama server at http://localhost:11434. Copilot off (default).
- Governance compliant: ADR-0008 (pre/post-set callbacks + IntegerRange descriptor + SetParametersResult), ADR-0003 (marine_control ControlServer wired: find_package + ament_target_dependencies + package.xml depend), ADR-0013. Docs (docs/h265_transport.md section Dynamic bitrate) + marine_control dependency land in-PR. Plan followed with safety improvements (device_->close() first; validate/apply split).
- Build/test: code at HEAD identical to verified SHA e6d7871 (only progress.md changed since); implementer reported depthai_marine clean, test_h265_bitrate_dynamic (8) + test_h265_params (15) passing.

### Next step
Lifecycle: **Local Review (Pre-Push)** -> push / open PR -> **triage-reviews**. Verdict is approved (no must-fix); the 4 suggestions can be applied or tracked, with Suggestion 1 (cross-repo param-range compatibility) worth confirming before deploy.

## Integrated Review
**Status**: complete
**When**: 2026-08-05 12:45 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #48 at `1350fd6`
**Sources**: 3 (Copilot R1 @ `1350fd6`, Local Review (Pre-Push) R2 @ `e6d7871`, CI rollup)
**Cross-source confirmations**: 1
**CI**: all-pass

### Findings
- [ ] (cross-confirmed: Copilot + Local Review R2 sugg-3, trivial) Comment above `applied_bitrate_kbps_ = target` overstates the coalescing guarantee — a set landing between the `target` capture and `getPipeline()`'s atomic read yields one redundant extra restart, not "one more (correct) restart"; reword the comment to match actual behavior (conservative: redundant outage possible, silent drop impossible) — `depthai_marine/src/camera_base.cpp:169-173`

### False positives
- (none)
