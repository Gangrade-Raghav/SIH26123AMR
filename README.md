# Decentralized AMR Fleet Coordination — Antigravity Project

This repository is the implementation control plane for the research-grade decentralized AMR fleet coordination project.

## Stack

- Ubuntu Linux
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic
- Zenoh / `rmw_zenoh_cpp`
- C++ and Python
- Git
- Antigravity as the primary autonomous implementation agent

## Core research architecture

```text
Task Generator
      ↓
CBBA / ACBBA
      ↓
RHCR / GD-RHCR
      ↓
PIBT fallback
      ↓
Reservation + WFG
      ↓
Local Safety Controller / NH-ORCA
      ↓
Robot Controller
```

Zenoh is the communication substrate for distributed ROS 2 components.

## Operating principle

Build the system incrementally, but create the final architectural interfaces early.

Every milestone must have:
1. a defined acceptance criterion,
2. automated verification,
3. reproducible configuration,
4. measurable metrics,
5. a recorded result,
6. no regression against previous milestones.

Do not implement the entire architecture in one pass.

See:
- `PRD.md`
- `AGENTS.md`
- `.agents/rules/`
- `.agents/skills/`
- `docs/architecture.md`
- `docs/research-plan.md`
- `docs/experiment-protocol.md`
- `docs/antigravity-workflow.md`
