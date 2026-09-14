# Research Plan

## Central hypothesis

A decentralized AMR fleet can maintain useful throughput and safety under communication and compute degradation by combining decentralized task allocation, lifelong planning, fast fallback planning, localized deadlock recovery, and explicit fault injection.

This is a hypothesis to test, not a guaranteed result.

## Baselines

### B0 — Centralized baseline
Central task allocation + centralized planning using the same map/workload.

### B1 — Decentralized basic
Zenoh + simple distributed state + PIBT.

### B2 — Proposed core
Zenoh + CBBA/ACBBA + RHCR + PIBT fallback + WFG.

### B3 — Advanced
B2 + GD-RHCR.

### B4 — Optional
B3 + NH-ORCA.

### B5 — Optional
B3/B4 + MAPF-LNS2.

## Main independent variables

- fleet size,
- traffic density,
- workload intensity,
- map topology,
- packet loss,
- latency/jitter,
- outage duration,
- compute pressure,
- planner configuration.

## Main dependent variables

### Safety
- collisions,
- near collisions,
- minimum separation,
- emergency stops.

### Efficiency
- throughput,
- flowtime,
- makespan,
- path length,
- idle time.

### Planning
- P50/P95/P99 planning latency,
- replans,
- failures,
- fallback frequency.

### Distributed communication
- bytes/sec/robot,
- end-to-end message latency,
- discovery/reconnection time,
- stale-state events.

### Deadlock
- deadlock count,
- detection latency,
- recovery latency,
- unnecessary recoveries.

### Compute
- CPU,
- GPU,
- RAM,
- temperature,
- power where available.

## Novelty discipline

The project should not claim that the combination is novel merely because it combines known algorithms.

Potential contribution areas to investigate experimentally:
1. adaptive planner switching under compute pressure,
2. interaction between communication degradation and decentralized task allocation,
3. versioned localized WFG recovery under stale distributed state,
4. system-level benchmark methodology,
5. empirically supported integration of the above mechanisms.

A stronger novelty claim should only be made after literature review and experimental evidence.

---

## M9 Large-Scale Environment Expansion Strategy

Milestone M9 broadens the experimental scope of the project from the initial $16\,\text{m} \times 16\,\text{m}$ single-room benchmark into scalable, topologically complex warehouse and factory layouts:

### 1. Controlled Baseline
The complete M8B architecture (CBBA + RHCR + Reservations + PIBT + WFG + Comm Resilience + Adaptive Compute) serves as the frozen reference baseline. No algorithms are modified prior to testing on the expanded topologies:
$$\text{Baseline} = \mathcal{A}_{\text{M8B}} + \text{Tier}(\text{M9-V1} \to \text{M9-V4})$$

### 2. Experimental Independent Variables
- **Environment Topology**: M9-V1 (Expanded $32\times 32$), M9-V2 (Congested $32\times 32$), M9-V3 (Hard/Chokepoints $48\times 32$), M9-V4 (Research Stress $60\times 40$ & Abstract $120\times 80$).
- **Fleet Scale**: 5 AMRs (controlled baseline comparison), 10 AMRs (moderate congestion), 15 AMRs (high spatial/compute density), 20+ AMRs (abstract MAPF limits).
- **Traffic Regimes**: Scenarios A through J (low/moderate/severe congestion, bidirectional corridors, chokepoint queuing, intersection clashes, charging pad contention, task hotspots, asymmetric demand, communication degradation during transit).

### 3. Failure Classification Framework
To maintain scientific integrity, failure events (deadlocks, safety aborts, timeouts) must be explicitly classified into five distinct categories:
1. **Environment Difficulty**: Topological constraint requiring specific maneuver (e.g. narrow corridor requiring yield into passing bay).
2. **System Scaling Limitation**: Degradation caused by network message count or ROS 2 discovery overhead across large fleets.
3. **Algorithmic Limitation**: Inability of decentralized greedy/horizon planning to resolve complex multi-agent deadlock without global coordination.
4. **Implementation Limitation**: Bug or race condition in message handling, state transitions, or coordinate transformations.
5. **Simulator / Resource Limitation**: Host workstation CPU starvation or Gazebo physics real-time factor (RTF) collapse.

