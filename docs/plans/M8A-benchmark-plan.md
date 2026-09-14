# M8A Implementation Plan: Experimental Benchmark & Workload Scaling

## 1. Executive Summary & Objective

Milestone M8A builds an empirical, reproducible experimental benchmark infrastructure for the decentralized 5-AMR fleet system developed in M0–M7. The objective is to make the multi-agent system systematically measurable under scaled workloads (15, 30, 50, and 100 tasks) and multi-trial horizons without modifying the core planning algorithms (CBBA, RHCR, PIBT, WFG) or introducing adaptive-compute mechanisms.

M8A establishes the experimental foundation and statistical baseline required before subsequent milestones (e.g., M8B adaptive compute) can evaluate performance gains with statistical rigor.

---

## 2. Architectural Boundary & Constraints

- **Preservation of M0–M7 Guarantees**:
  - Decentralized consensus (CBBA bundle building and consensus).
  - Rolling-horizon collision avoidance (RHCR, space-time reservations, PIBT, WFG deadlock recovery).
  - Safety filter (local reactive braking $\le 0.40\,\text{m}$ and hard emergency threshold at $0.35\,\text{m}$).
  - Communication degradation layer (M7 impairment models).
  - Existing results (`gazebo_m7_results.json`) remain untouched.
- **Strict Scope Exclusions**:
  - No adaptive compute mechanisms.
  - No thermal throttling or CPU-triggered planner switching.
  - No alternative collision avoidance algorithms (no NH-ORCA, VO, MAPF-LNS2).
  - No machine learning models.
  - No synthetic formulas or fabricated performance metrics.

---

## 3. Workload Scaling Design

Four canonical workload configurations are defined under `config/workloads/`:

| Workload ID | Task Count ($N$) | Calibrated Horizon ($H$) | Max Bundle Capacity ($B$) | Seed | Obstacle Bounds |
| :--- | :---: | :---: | :---: | :---: | :---: |
| `workload_15_tasks.yaml` | 15 | 60.0 s | 4 | 101 | Free grid cells ($x \in [1, 19], y \in [1, 14]$) |
| `workload_30_tasks.yaml` | 30 | 120.0 s | 8 | 102 | Free grid cells ($x \in [1, 19], y \in [1, 14]$) |
| `workload_50_tasks.yaml` | 50 | 200.0 s | 12 | 103 | Free grid cells ($x \in [1, 19], y \in [1, 14]$) |
| `workload_100_tasks.yaml` | 100 | 360.0 s | 24 | 104 | Free grid cells ($x \in [1, 19], y \in [1, 14]$) |

### Workload Generator Invariants:
1. **Obstacle Collision Invariant**: All pickup and dropoff coordinates are validated against the static warehouse obstacle map (`warehouse_grid_small`). No coordinate lies on shelf racks or boundary walls.
2. **Uniqueness**: Every task ID is guaranteed unique within the workload file.
3. **Reproducibility**: Coordinates and priority assignments are deterministically generated from fixed random seeds.

---

## 4. Benchmark Architecture & Components

### 4.1 Benchmark Domain Model (`amr_fleet_core.benchmark_manager`)
- **`BenchmarkConfig`**: Immutable configuration dataclass defining workload path, trial counts, base seed, horizon, and communication profile.
- **`ExperimentMetadata`**: Audit metadata capturing experiment ID, UTC timestamp, Unix epoch, Git commit SHA, ROS 2 distribution, fleet size, map ID, and explicit termination reason.
- **`TerminationReason`** Enum:
  - `ALL_TASKS_COMPLETED`: Fleet successfully picked up and dropped off all $N$ tasks.
  - `HORIZON_REACHED`: Trial terminated upon reaching the experimental time horizon $H$.
  - `SAFETY_ABORT`: Inter-robot distance violated the emergency safety threshold ($< 0.35\,\text{m}$).
  - `EXPERIMENT_ERROR`: Unhandled exception or node communication failure.
- **`StandardMetrics`**: Schema-compliant dataclass covering task completion, throughput, makespan, planning latency percentiles (mean, p50, p95, p99), CBBA convergence, coordination events (conflicts, deadlocks), safety interventions, and host compute utilization (CPU %, RAM MB).
- **`StatisticalAggregator`**: Computes sample statistics across repeated trials ($n$, mean, median, sample standard deviation with $ddof=1$, min, max, p50, p95 for $n \ge 5$, and 95% Student's t confidence intervals for $n \ge 3$).

### 4.2 Benchmark Runner CLI (`scripts/run_benchmark.py`)
- Standardized command-line interface supporting `--workload`, `--trials`, `--seed`, `--horizon`, `--communication`, `--raw-dir`, `--agg-dir`, and `--dry-run`.
- Pre-flight hardware resource verification (`psutil` checking available RAM $\ge 2000\,\text{MB}$).
- Sequential execution of trials with automated process teardown and lifecycle auditing.
- Subscription to empirical ROS 2 topics (`/tasks/all`, `/tasks/events`, `/{robot_id}/rolling_plan`, `/{robot_id}/bundle`, `/{robot_id}/odom`, `/{robot_id}/scan`, `/fleet/conflicts`, `/fleet/deadlocks`).
- Persistence of raw trial records (`results/m8a/raw/{experiment_id}.json`) and aggregated summaries (`results/m8a/aggregated/{benchmark_id}.json`).

### 4.3 Analytical Visualization (`scripts/plot_m8a_benchmarks.py`)
- Generates publication-grade 7-panel scaling figures (`docs/images/m8a_workload_scaling.png`) plotting:
  1. Workload Size vs Fleet Throughput (tasks/min)
  2. Workload Size vs Task Completion Rate (%)
  3. Workload Size vs Workload Makespan (s)
  4. Workload Size vs Planning Latency (Mean & P95 ms)
  5. Workload Size vs Compute Utilization (CPU % & RAM MB)
  6. Workload Size vs Conflicts & Deadlocks Resolved
  7. Workload Size vs Minimum Inter-Robot Distance & Safety Interventions

---

## 5. Verification Plan

1. **Automated Unit Testing**:
   - `test_m8a_benchmark.py`: 10 test cases verifying workload YAML loading, coordinate validity, seed determinism, dataclass schema serialization, termination reasons, task accounting invariance ($N_{\text{completed}} + N_{\text{remaining}} \equiv N_{\text{generated}}$), statistical aggregator math, and confidence interval suppression for $n < 3$.
   - Full workspace regression passing with 0 failures, 0 errors.
2. **Empirical Gazebo Validation**:
   - Scenario A: Workload 15 tasks $\times$ 1 trial ($H=60.0\,\text{s}$, seed 42).
   - Scenario B: Workload 30 tasks $\times$ 1 trial ($H=120.0\,\text{s}$, seed 42).
   - Scenario C: Reproducibility check repeating Scenario A under identical seed and configuration.
3. **Artifact Generation & Archival**:
   - Save all empirical trial JSONs under `results/m8a/`.
   - Generate analytical plots under `docs/images/`.
   - Document complete checkpoint in `docs/checkpoints/M8A.md`.
