# M8A Research Methodology: Fleet Workload Scaling & Experimental Benchmarking

## 1. Scientific Motivation

Multi-agent coordination in warehouse robotics exhibits complex non-linear scaling behaviors as task density, reservation density, and planning frequency increase. Prior to introducing optimization algorithms or adaptive-compute mechanisms (such as those planned for M8B), an empirical baseline must be established.

Milestone M8A provides:
1. **Reproducible Workload Configurations**: Deterministic task distributions scaled across 15, 30, 50, and 100 tasks.
2. **Explicit Bounded-Horizon Protocol**: Disentangling workload saturation from execution progress through calibrated horizons and explicit termination criteria.
3. **Rigorous Statistical Treatment**: Proper sample variance, percentile tracking, and confidence intervals without synthetic estimates.
4. **Host Resource Profiling**: Synchronous tracking of CPU and memory utilization to understand computational overhead as fleet coordination demands scale.

---

## 2. Experimental Design & Protocol

### 2.1 Environmental Control & Repeatability
- **Simulation Platform**: Gazebo Harmonic (`gz sim`) coupled with ROS 2 Jazzy via `ros_gz_bridge`.
- **Fleet Specification**: 5 differential-drive Autonomous Mobile Robots (`amr_0` through `amr_4`), each equipped with a 2D planar LiDAR and wheel odometry.
- **Warehouse Topology**: `warehouse_grid_small` ($20 \times 15$ discrete grid, cell size $1.0\,\text{m}$), containing 4 static shelf rack obstacles.
- **Hardware Isolation**: All benchmark trials are executed strictly sequentially on dedicated host hardware (16 logical cores, 32 GB RAM) to eliminate inter-process contention, hyperthreading jitter, and CPU thermal throttling.

### 2.2 Workload Scaling & Calibrated Horizons
Workload size directly affects path reservation density and task auction duration. In a physical warehouse simulation, AMRs travel at bounded velocities ($v_{\max} = 0.5\,\text{m/s}$). Consequently, the time required to complete a given number of tasks scales with workload magnitude:

$$\text{Horizon}(N) = \max\left(60.0, \, 3.6 \times N\right) \quad [\text{seconds}]$$

- **15 Tasks**: $H = 60.0\,\text{s}$, Max Bundle Capacity = 4
- **30 Tasks**: $H = 120.0\,\text{s}$, Max Bundle Capacity = 8
- **50 Tasks**: $H = 200.0\,\text{s}$, Max Bundle Capacity = 12
- **100 Tasks**: $H = 360.0\,\text{s}$, Max Bundle Capacity = 24

### 2.3 Task Generation & Spatial Invariants
Tasks are defined by coordinates $(x_{\text{pickup}}, y_{\text{pickup}}) \to (x_{\text{dropoff}}, y_{\text{dropoff}})$ and priority values $p \in [1, 5]$.
To guarantee physical validity:
1. **Rejection Sampling**: Any coordinate falling within $0.5\,\text{m}$ of a static shelf rack or outside the warehouse walls ($x \in [1, 19], y \in [1, 14]$) is rejected and re-sampled.
2. **Deterministic Pseudorandom Seeds**: Fixed seeds (101, 102, 103, 104) ensure identical task sequences across test runs.
3. **Task ID Invariant**: Every task is assigned a unique identifier `task_001` through `task_N`.

---

## 3. Data Collection Architecture

Telemetry is collected by a non-intrusive auditor node (`BenchmarkAuditor`) subscribing directly to live ROS 2 topics:

```
+-------------------------------------------------------------+
|                     Gazebo Harmonic                         |
+-------------------------------------------------------------+
       |                        |                       |
  /{amr_i}/odom           /{amr_i}/scan         /tasks/events
       |                        |                       |
       v                        v                       v
+-------------------------------------------------------------+
|                BenchmarkAuditor (ROS 2 Node)                |
|  - Spatial Safety Monitor: Inter-robot Euclidean distance   |
|  - Task Lifecycle Auditor: Pickups, dropoffs, timestamps    |
|  - Coordination Auditor: CBBA convergence, conflicts, WFG  |
|  - Host Resource Profiler: psutil CPU %, RAM MB             |
+-------------------------------------------------------------+
       |
       v
+-------------------------------------------------------------+
|      Standardized JSON Schema (schema_version: "m8a.v1")    |
|   raw: results/m8a/raw/{experiment_id}.json                 |
|   aggregated: results/m8a/aggregated/{benchmark_id}.json    |
+-------------------------------------------------------------+
```

### 3.1 Termination Reasons
Every trial must terminate with an explicit, mutually exclusive reason:
- **`ALL_TASKS_COMPLETED`**: Reached when $\text{count}(\text{COMPLETED}) = N_{\text{generated}}$.
- **`HORIZON_REACHED`**: Reached when $t_{\text{elapsed}} \ge H$ while tasks remain uncompleted.
- **`SAFETY_ABORT`**: Reached immediately if $\min_{i \neq j} d(r_i, r_j) < 0.35\,\text{m}$.
- **`EXPERIMENT_ERROR`**: Node crash, discovery timeout, or fatal exception.

---

## 4. Statistical Aggregation

For repeated trials ($k = 1, \dots, n$):
- **Sample Mean**: $\bar{x} = \frac{1}{n} \sum_{k=1}^n x_k$
- **Sample Standard Deviation** (unbiased, $ddof = 1$):
  $$s = \sqrt{\frac{1}{n - 1} \sum_{k=1}^n (x_k - \bar{x})^2} \quad (\text{for } n > 1)$$
- **Percentiles**: $p_{50}$ (median), $p_{95}$ (computed for $n \ge 5$ using nearest-rank ceiling).
- **95% Confidence Interval** (Student's t-distribution):
  $$\text{CI}_{95\%} = \left[ \bar{x} - t_{0.025, \, n-1} \frac{s}{\sqrt{n}}, \; \bar{x} + t_{0.025, \, n-1} \frac{s}{\sqrt{n}} \right]$$
  *Confidence intervals are reported only when $n \ge 3$ and $s > 0$. For $n < 3$, they are strictly suppressed (`null`) to prevent false claims of statistical significance.*
