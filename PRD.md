# Product Requirements Document — NRDAS AMR Fleet

## 1. Product / Research Goal

Build and experimentally evaluate a fault-aware decentralized multi-robot AMR fleet coordination system for warehouse-style environments.

The project is not claiming that a single algorithm solves all fleet coordination problems. The research contribution is the system-level integration and controlled evaluation of:

- decentralized task allocation,
- lifelong multi-agent path finding,
- fast fallback planning,
- distributed deadlock detection/recovery,
- communication degradation,
- compute-aware adaptation,
- realistic ROS 2/Gazebo integration.

## 2. Research Questions

### RQ1 — Decentralization
How does decentralized task allocation affect throughput, planning latency, and robustness compared with a centralized baseline?

### RQ2 — Lifelong MAPF
How do RHCR, GD-RHCR, and PIBT behave at increasing fleet scales and traffic densities?

### RQ3 — Communication degradation
How does packet loss, latency/jitter, and temporary communication outage affect coordination quality and safety?

### RQ4 — Deadlock recovery
Can localized wait-for-graph reasoning detect and recover from cyclic resource dependencies without a centralized deadlock manager?

### RQ5 — Compute-aware planning
Can a fleet maintain acceptable performance by switching from computationally expensive planning to a fast fallback when compute pressure rises?

### RQ6 — System-level interaction
Which combinations of these mechanisms provide the best trade-off between safety, throughput, latency, communication cost, and compute cost?

## 3. Functional Requirements

### FR-01: Fleet simulation
The system shall spawn a configurable number of namespaced AMRs from a single launch/configuration interface.

### FR-02: Robot isolation
Each robot shall have isolated ROS namespaces and TF trees.

### FR-03: Task generation
The system shall generate reproducible pickup/dropoff tasks with priority and timing metadata.

### FR-04: Pluggable task allocation
The system shall expose a common task allocator interface supporting at minimum:
- centralized baseline,
- CBBA,
- ACBBA where implemented.

### FR-05: Pluggable planning
The system shall expose a common planning interface supporting:
- RHCR,
- GD-RHCR experimental implementation,
- PIBT fallback.

### FR-06: Deadlock handling
The system shall maintain localized dependency information and support WFG cycle detection and recovery.

### FR-07: Communication abstraction
Distributed components shall communicate through ROS 2 interfaces while permitting Zenoh middleware execution.

### FR-08: Fault injection
The experiment framework shall support configurable:
- packet loss,
- latency/jitter,
- communication outages.

### FR-09: Compute adaptation
The planner shall support policy-driven switching between normal and fallback planning based on measurable compute pressure.

### FR-10: Benchmarking
Every experiment shall save:
- configuration,
- seed,
- metrics,
- logs,
- software revision,
- environment information.

## 4. Non-Functional Requirements

- Reproducible experiments.
- No hard-coded robot count.
- No algorithm-specific assumptions in core fleet interfaces.
- Safety-critical execution path must not depend on the optimization layer.
- Every major component must have automated tests.
- Performance claims must be backed by measurements.
- Large-agent scalability must be demonstrated in a lightweight MAPF simulator, not inferred from Gazebo.

## 5. Explicit Non-Goals

The initial release shall NOT claim:
- collision-free operation under arbitrary network failures,
- universal scalability to arbitrary warehouse topologies,
- physical 500-robot Gazebo validation,
- guaranteed thermal behavior without hardware measurement,
- novelty merely from combining existing algorithms.

## 6. Target Backends

### Gazebo backend
Use for realistic ROS 2, sensor, controller, navigation, and multi-robot integration.

Target scale: approximately 5–50 robots depending on hardware.

### Lightweight MAPF backend
Use for large-scale algorithmic evaluation.

Target scale: 500–10,000+ abstract agents where computationally feasible.

## 7. Acceptance Gates

A milestone is complete only when:
- implementation exists,
- automated tests pass,
- a reproducible demo passes,
- metrics are collected,
- documentation is updated,
- Git working tree is understandable,
- no unexplained regressions remain.

## 8. Definition of Done

A feature is DONE only when an independent engineer can:
1. understand why it exists,
2. reproduce it,
3. run its tests,
4. inspect its configuration,
5. understand its failure modes,
6. compare its performance using recorded metrics.
