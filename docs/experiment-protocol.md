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
