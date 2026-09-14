# Experiment Protocol

## Experiment hierarchy

### Tier A — Unit / algorithmic
Tiny deterministic graphs and task sets.

### Tier B — Large-scale MAPF
500–10,000+ abstract agents where computationally feasible.

### Tier C — Gazebo integration
5–50 robots depending on machine capacity.

### Tier D — Hardware
Only if physical hardware becomes available.

## Network scenarios

Baseline:
- 0% loss
- nominal latency

Stress:
- 5% loss
- 10% loss
- 15% loss
- 50 ms jitter
- 100 ms jitter
- 150 ms jitter
- 1 s outage
- 3 s outage

These are controlled stress scenarios, not claims about typical real-world warehouse conditions.

## Compute scenarios

- unconstrained,
- controlled CPU pressure,
- planner-latency pressure,
- hardware-specific thermal/power measurements when available.

Do not infer Jetson thermal behavior from a desktop simulation.

## Agent counts

Algorithmic:
- 10
- 50
- 150
- 500
- 1000+
- larger only when useful

Gazebo:
- start small and increase until the machine becomes the limiting factor.

## Repetitions

Unless an experiment is deterministic, use multiple random seeds.

Store every seed.

## Required run metadata

```text
experiment_id
timestamp
git_commit
software_environment
map
robot_count
task_count
task_seed
network_config
compute_config
planner_config
allocator_config
```

## Result integrity

Raw data is immutable.

Derived plots must reference raw data.

Failed runs must remain recorded with a failure classification.

---

## M9 Large-Scale Warehouse & Scenario Protocol

Milestone M9 introduces structured experimental protocols for large-scale environment and scenario expansion:

### 1. Progressive Tier Testing Order
Experiments must progress strictly through the environment hierarchy:
$$\text{M9-V1 (Expanded Baseline)} \longrightarrow \text{M9-V2 (Congested)} \longrightarrow \text{M9-V3 (Hard/Chokepoints)} \longrightarrow \text{M9-V4 (Research Stress)}$$
Testing higher tiers is contingent on verifying basic navigation and allocation sanity in earlier tiers.

### 2. Physical Feasibility Constraints
- **Wide Corridors ($\ge 2.4\,\text{m}$)**: Standard two-way passing clearance ($\ge 1.4\,\text{m}$).
- **Narrow Corridors ($1.6\,\text{m}-1.8\,\text{m}$)**: Tight two-way passing clearance ($0.6\,\text{m}-0.8\,\text{m}$), requiring cautious tracking and PIBT priority deconfliction.
- **Single-Lane Corridors ($1.1\,\text{m}$)**: Simultaneous passing is physically impossible. All single-lane corridors MUST feature physical passing bays ($2.5\,\text{m} \times 2.0\,\text{m}$) situated at entrances and midpoints. Opposing AMRs entering a corridor without passing bays is an impossible geometry and will NOT be benchmarked.

### 3. Progressive Fleet Scaling Protocol
- Gazebo Harmonic validation: 5 AMRs (baseline control), 10 AMRs (moderate congestion), 15 AMRs (high density).
- Abstract discrete MAPF simulation: 20, 50, and 100+ AMRs in kinematic grid simulation to isolate algorithmic convergence from physics engine solver limits.

### 4. Pilot Campaign Requirement
Before executing the full scenario matrix (A through J), an initial pilot experiment ($n=1$, seed 42, 5 AMRs, 15 tasks, 120s horizon) on Tier M9-V1 must verify clean world loading, obstacle parsing, route completion, and metric logging.

