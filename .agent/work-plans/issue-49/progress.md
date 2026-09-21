---
issue: 49
---

# Issue #49 — Offline WaSR-T vs eWaSR comparison on recorded glint/reflection footage (evidence for #42 option choice)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-21 12:52 -04:00
**By**: Claude Code Agent (Claude Fable 5.1)
**Verdict**: approved

**Branch**: feature/issue-49 at `af69afe`
**Mode**: pre-push
**Depth**: Standard (reason: ~1300-line additive benchmark tooling + committed results in a project repo; no runtime node code)
**Must-fix**: 3 | **Suggestions**: 9
**Round**: 1 | **Ship**: recommended — all 3 must-fixes were labelling/guard defects fixed in-branch before push; numbers unchanged on re-run

### Findings
- [x] (must-fix) metrics/README called the 0.7/0.8/0.95 sweep "what the field used"; the reflex ran 0.60 — relabelled, 0.6 added — `metrics.py:16`
- [x] (must-fix) reflex_replay counted frames without /tf as "no trigger" — now excluded from denominators, reported — `reflex_replay.py:179`
- [x] (must-fix) no stamp consistency check between frames.npz and probs_*.npz — stamp_ns saved + enforced — `run_models.py`, `metrics.py`, `reflex_replay.py`
- [x] (suggestion) extractor did not detect a missing keyframe in the GOP lead-in — now errors — `extract_window.py`
- [x] (suggestion) redundant WaSR-T probe forward pass — removed — `run_models.py:107`
- [x] (suggestion) nearest-sample TF vs tf2 interpolation undocumented — documented + median age reported — `reflex_replay.py`
- [x] (suggestion) WaSR-T repo commit and torch versions unpinned — pinned — `README.md`, `requirements.txt`
- [x] (suggestion) confidence-floor date/commit uncited — cites echoboats 4ce5086 — `results/README.md:18`
- [x] (suggestion) Lightning alias is process-global — noted — `run_models.py:79`
- [x] (suggestion) unquoted window name in sed — guarded — `collect_results.sh:20`
- [x] (suggestion) benchmark missing from the repo layout tree — added — `.agents/README.md:76`
- [x] (suggestion) 59% should be 58% — fixed — `results/README.md`
- [x] (static) two unused imports — removed — `extract_window.py`
