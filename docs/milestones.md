# Implementation Milestones

## M0 — Environment + repository
- Verify ROS 2 Jazzy
- Verify Gazebo Harmonic
- Verify Zenoh
- Create repository
- Create package skeleton
- Establish CI/test commands

## M1 — Single AMR
- Spawn
- sensors
- odometry
- command interface
- navigation smoke test

## M2 — Parameterized multi-robot simulation
- 2, then 5, then 10 robots
- namespaces
- TF isolation
- automated spawning

## M3 — Task system
- task generator
- task message
- centralized baseline allocator

## M4 — CBBA
- decentralized allocation
- convergence tests
- allocation metrics

## M5 — Zenoh
- rmw_zenoh execution
- communication health metrics
- reconnection test

## M6 — PIBT
- graph representation
- planner
- deterministic tests
- large-scale simulator

## M7 — RHCR
- rolling horizon
- integration with task lifecycle
- RHCR vs PIBT benchmark

## M8 — WFG
- dependency graph
- cycle detection
- victim/recovery policy
- deliberate deadlock tests

## M9 — Fault injection
- loss
- jitter
- outage
- reproducible sweeps

## M10 — Compute-aware adaptation
- pressure metrics
- RHCR/GD-RHCR → PIBT switching
- recovery policy
- benchmark

## M11 — GD-RHCR
- research implementation
- comparison with RHCR

## M12 — NH-ORCA
- local controller benchmark

## M13 — MAPF-LNS2
- optional optimization layer
- safe plan handoff

## M14 — Large benchmark suite
- automated experiment matrix
- result database
- plots
- reproducibility report

## M15 — Research packaging
- architecture report
- experiment report
- limitations
- final claims
