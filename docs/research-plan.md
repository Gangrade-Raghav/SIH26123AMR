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
