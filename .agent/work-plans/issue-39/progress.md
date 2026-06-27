---
issue: 39
---

# Issue #39 — Wire reflex node (segments_to_pointcloud) into marine_control device control

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-27 16:46 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved

**Branch**: feature/issue-39 at `d356da8`
**Mode**: pre-push
**Depth**: Standard (reason: safety-critical lifecycle node + ~256 lines across 4 files)
**Must-fix**: 0 | **Suggestions**: 5 (all addressed)
**Round**: 1 | **Ship**: recommended — no must-fix; both adversarial lenses converged, all suggestions applied.

Two disjoint-lens Claude adversarial passes (Lens A logic, Lens B systemic/safety)
cross-confirmed each other. No must-fix findings. The 0.95 cap is double-guarded
(FloatingPointRange descriptor + on-set callback) with no write path around the
validated `set_parameters` route — confirmed sound by both passes.

### Findings
- [x] (suggestion) on-set callback wrote cached floor during validation — split into validate (on-set) + commit (post-set) so a rejected batched set can't desync cache — `segments_to_pointcloud.cpp:107`
- [x] (suggestion) SingleThreadedExecutor is load-bearing for the ControlServer threading contract — documented in `main()` — `segments_to_pointcloud.cpp:393`
- [x] (suggestion) rejection test could pass vacuously (dropped vs rejected) — added in-range sentinel after the out-of-range request + max-observed tracking — `test_segments_to_pointcloud_control.py:test_out_of_range_change_is_rejected`
- [x] (suggestion) test imports marine_control_interfaces directly but only had transitive dep — added `<test_depend>` — `package.xml:38`
- [x] (suggestion) docstring numbering implied execution order — noted tests are self-contained / alphabetical — `test_segments_to_pointcloud_control.py`
- [ ] (deferred) `on_error` override does not reset the control server on the active→ErrorProcessing path — latent only (no callback triggers the error transition for this node); not addressed.
