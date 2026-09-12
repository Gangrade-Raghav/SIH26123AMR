# Antigravity Workflow

This project is intentionally structured around Antigravity's agentic workflow.

## Why this structure

Official Antigravity guidance recommends:
- explore → plan → execute,
- explicit verification loops,
- workspace rules,
- AGENTS.md/GEMINI.md project guidance,
- reusable skills,
- artifacts for plans/results,
- parallel subagents for independent work.

## Primary agent

The primary agent owns:
- architecture,
- integration,
- milestone execution,
- regression checks,
- final verification.

## Parallel subagents

Use subagents for independent investigations such as:

### Agent A — ROS/Gazebo
Inspect simulation integration and robot stack.

### Agent B — MAPF
Investigate planner implementation and benchmark interfaces.

### Agent C — Distributed systems
Inspect Zenoh configuration, communication abstractions, and fault injection.

### Agent D — Testing
Build unit/integration/regression tests.

### Agent E — Research
Verify literature assumptions and maintain references.

Subagents must return artifacts/results to the primary agent. They must not independently redefine the architecture.

## Artifact expectations

For meaningful work, produce:
- implementation plan,
- changed-file summary,
- test evidence,
- benchmark report where applicable.

## Agent interruption

If the agent starts moving in the wrong direction, stop early rather than letting it accumulate changes.

Use Git and Antigravity session controls to recover to a known-good state.

## Experiment branching

Use separate branches/worktrees for speculative algorithm experiments where practical.

Do not destabilize the main integration branch to test an uncertain idea.

## One milestone at a time

The primary agent must not start the next architectural milestone until the current milestone passes its acceptance gate.
