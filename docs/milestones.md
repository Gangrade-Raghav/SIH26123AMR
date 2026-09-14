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

## M6 — Multi-Agent Coordination & Deadlock Recovery [APPROVED]
- Rolling-Horizon Conflict Resolution (RHCR)
- Space-Time Reservation Tables
- Priority Inheritance Backtracking (PIBT) local coordination
- Wait-For-Graph (WFG) cycle detection & deterministic recovery
- Decoupled 10 Hz reactive LiDAR safety controller

## M7 — Communication Resilience & Fault Injection [APPROVED]
- Communication impairment model (packet loss, latency, jitter, partitions)
- Stale-state management & timeout-based eviction
- Path authority protocol under network partitions
- Degradation sweeps in Gazebo Harmonic

## M8A — Experimental Benchmark & Workload Scaling [APPROVED]
- Deterministic workload generation (15, 30, 50, 100 tasks)
- Bounded-horizon benchmark protocol (60s, 120s, 200s, 360s)
- Host resource instrumentation (CPU, RAM) & publication-grade plotting
- Task accounting invariants and safety verification

## M8B — Adaptive Compute & Degradation-Aware Coordination [APPROVED - FROZEN BASELINE]
- Discrete compute modes: LOW, NORMAL, HIGH
- Deterministic priority-ordered state machine (Comm > CPU > Contention > Recovery)
- Asymmetric hysteresis, minimum dwell time (3.0s), and sample confirmation (K=3)
- Dynamic runtime timer adjustment in ROS 2 Jazzy
- 12 real Gazebo Harmonic trials evaluating compute reallocation under stress

## M9 — Large-Scale Warehouse & Scenario Expansion [IN DESIGN - PHASE 1 COMPLETE]
- Progressive environment tiers: M9-V1 (Expanded), M9-V2 (Congested), M9-V3 (Hard/Chokepoints), M9-V4 (Research Stress)
- Progressive fleet scaling: 5, 10, 15, 20+ AMRs
- Scenario matrix covering Scenarios A through J (corridors, intersections, chokepoints, hotspots, asymmetric demand)
- Physical feasibility validation (passing bays, corridor widths)
- M8B frozen baseline evaluation and limitation discovery

## M10 — Advanced Coordination & Optimization [FUTURE]
- GD-RHCR guidance graphs & topological flow control
- MAPF-LNS2 large neighborhood search optimization
- Multi-lane highway routing rules

## M11 — Local Controller Benchmarking [FUTURE]
- Non-holonomic ORCA (NH-ORCA) integration
- Continuous local velocity obstacle benchmarking

## M12 — Final Research Packaging & Multi-Seed Benchmarks [FUTURE]
- Automated multi-seed experiment campaigns (n >= 5)
- Cross-milestone synthesis & publication report

