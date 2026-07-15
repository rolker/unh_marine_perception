# AGENTS.md — unh_marine_perception

Instructions for AI agents working in this repository — including **GitHub
Copilot code review**, which reads this file when reviewing PRs. Coding
agents: the deep guide (packages, layout, pitfalls) is
[`.agents/README.md`](.agents/README.md); read it before making changes.

## Workspace Rules

This repo is developed inside a [ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace).
The workspace root `AGENTS.md` carries the full shared rules (worktree
isolation, issue-first policy, commit conventions, AI signatures). This file
**references** those rules and adds repo-specific context only — it must
never restate or fork them.

## Quality Standard

<!-- Standalone excerpt of the workspace AGENTS.md § Quality Standard. It is
     intentionally condensed for repos reviewed without workspace context; when
     the workspace § Quality Standard changes materially, re-sync this excerpt
     (the drift ADR-0017 acknowledges). -->

This is software for autonomous robot boats operating on open water.
Robustness is not optional.

- Fix bugs completely: add the test, handle the edge case, check the
  lifecycle transition.
- Concerns about error handling, silent failures, stale data, or missing
  validation are not nits — flag them unless the failure mode genuinely
  cannot occur. "Config is under our control" and "pathological input" are
  not blanket dismissals; field configs change under pressure.
- A change includes its consequences: tests, documentation, and dependent
  references update in the same PR.

## Reviewing PRs

- If the PR carries a work plan (`.agent/work-plans/issue-<N>/plan.md` or a
  plan in the PR body), the plan is kept **in sync with the implementation
  as it evolves** — an implementation that matches the current plan text is
  not "plan drift", even if the plan changed after the PR opened.
- Verify claims against source: parameters, topics, services, and message
  types in docs must match the code.

## Review Context — unh_marine_perception

- Safety-relevant perception for a live autonomous boat: sea-surface
  segmentation feeds obstacle costmap layers and the collision-avoidance
  reflex. Stale detections, wrong frames, or bad timestamps are safety
  problems, not cosmetic ones — timestamp/frame handling deserves close
  review.
- Two packages: `depthai_marine` (OAK/DepthAI camera support, H.265 via
  ffmpeg_image_transport) and `sea_surface_segmentation` (segmentation
  nodes + a nav2_costmap_2d plugin; lifecycle nodes).
- Tests need no camera hardware: gtest suites plus launch_testing
  integrations that replay the in-repo bag fixture
  (`sea_surface_segmentation/test/fixtures/`). New perception behavior
  should extend that bag-replay pattern rather than requiring hardware.
- Default branch: `jazzy`.
