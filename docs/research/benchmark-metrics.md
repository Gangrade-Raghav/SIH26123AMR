# M8A Research Metrics Specification: Benchmark Telemetry & Standard Definitions

## 1. Metric Taxonomy & Formal Definitions

Every M8A trial produces a standardized record matching schema `m8a.v1`. All metrics are derived from empirical ROS 2 topic subscriptions and OS resource samplers.

---

## 2. Task Lifecycle & Performance Metrics

| Metric Key | Unit | Formal Definition / Calculation Source |
| :--- | :---: | :--- |
| `generated_tasks` | count | Total count of tasks published on `/tasks/all` at initialization ($N$). |
| `assigned_tasks` | count | Count of unique tasks that transitioned to `ASSIGNED` via CBBA consensus. |
| `completed_tasks` | count | Count of unique tasks that received a valid `COMPLETED` event from `/tasks/events`. |
| `failed_tasks` | count | Count of tasks explicitly abandoned or transitioned to `FAILED`. In baseline M8A, this is 0. |
| `remaining_tasks` | count | $N_{\text{generated}} - N_{\text{completed}}$. Enforced invariant: $N_{\text{completed}} + N_{\text{remaining}} \equiv N_{\text{generated}}$. |
| `completion_rate_pct` | % | $\frac{N_{\text{completed}}}{N_{\text{generated}}} \times 100\%$. |
| `throughput_tasks_per_min` | tasks/min | $\frac{N_{\text{completed}}}{\Delta t_{\text{actual}} / 60.0}$, where $\Delta t_{\text{actual}} = t_{\text{mission\_end}} - t_{\text{mission\_start}}$. |
| `makespan_sec` | seconds | $t_{\text{final\_completed\_event}} - t_{\text{mission\_start}}$ if $N_{\text{completed}} > 0$; otherwise $\Delta t_{\text{actual}}$. |

---

## 3. Planning & Replanning Metrics

| Metric Key | Unit | Formal Definition / Calculation Source |
| :--- | :---: | :--- |
| `planning_cycles` | count | Number of plan generation iterations executed across all AMRs. |
| `replan_count` | count | Fleet-wide sum of discrete replanning events: $\sum_{i=0}^4 (R_{i, \text{final}} - R_{i, \text{initial}})$, where $R_i$ is reported by `amr_i/rolling_plan:replan_count`. |
| `planning_latency_mean_ms`| ms | Arithmetic mean of individual rolling-horizon A* planning latencies: $\frac{1}{M} \sum_{m=1}^M \tau_m$. |
| `planning_latency_p50_ms` | ms | Median (50th percentile) planning latency. |
| `planning_latency_p95_ms` | ms | 95th percentile planning latency. |
| `planning_latency_p99_ms` | ms | 99th percentile planning latency. |

---

## 4. Auction & Coordination Metrics

| Metric Key | Unit | Formal Definition / Calculation Source |
| :--- | :---: | :--- |
| `cbba_convergence_time_ms`| ms | $(t_{\text{consensus}} - t_{\text{mission\_start}}) \times 1000.0$, where $t_{\text{consensus}}$ is the timestamp when all 5 AMR bundles publish `is_converged = True`. |
| `allocation_changes` | count | Number of times a task bid was outbid by another AMR after initial assignment. |
| `unassigned_tasks` | count | $\max(0, N_{\text{generated}} - N_{\text{assigned}})$. |
| `conflicts_detected` | count | Total space-time conflict events published on `/fleet/conflicts`. |
| `conflicts_resolved` | count | Total conflicts resolved by PIBT priority arbitration without collision. |
| `deadlocks_detected` | count | Total wait-for-graph cycle events published on `/fleet/deadlocks`. |
| `deadlocks_recovered` | count | Total deadlocks resolved via yield-delay or alternative path replanning. |

---

## 5. Spatial Safety & Physical Interaction Metrics

| Metric Key | Unit | Formal Definition / Calculation Source |
| :--- | :---: | :--- |
| `minimum_inter_robot_distance_m` | meters | $\min_{t} \min_{i \neq j} \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|_2$, computed from odometry topics `/{amr_i}/odom`. |
| `collision_contact_events` | count | Number of instances where inter-robot Euclidean distance dropped below the physical robot footprint boundary ($< 0.35\,\text{m}$). |
| `safety_brake_interventions` | count | Number of 2D planar LiDAR scan points on `/{amr_i}/scan` detecting an obstacle within the reactive brake zone ($\le 0.40\,\text{m}$). |

---

## 6. Host Hardware Utilization Metrics

| Metric Key | Unit | Formal Definition / Calculation Source |
| :--- | :---: | :--- |
| `cpu_utilization_mean_pct` | % | Temporal average of overall host CPU percentage sampled at $0.5\,\text{Hz}$ via `psutil.cpu_percent()`. |
| `cpu_utilization_max_pct` | % | Maximum instantaneous host CPU percentage observed during the trial. |
| `ram_utilization_mean_mb` | MB | Temporal average of host memory used sampled via `psutil.virtual_memory().used / 1e6`. |
| `ram_utilization_max_mb` | MB | Peak host memory used observed during the trial. |

---

## 7. JSON Schema Specification (`m8a.v1`)

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "M8A_Benchmark_Trial_Schema",
  "type": "object",
  "required": ["schema_version", "experiment_id", "metadata", "metrics"],
  "properties": {
    "schema_version": { "type": "string", "enum": ["m8a.v1"] },
    "experiment_id": { "type": "string" },
    "metadata": {
      "type": "object",
      "required": [
        "experiment_id",
        "timestamp_utc",
        "git_commit",
        "fleet_size",
        "workload_size",
        "trial_id",
        "seed",
        "mission_horizon_sec",
        "actual_duration_sec",
        "communication_profile",
        "termination_reason"
      ]
    },
    "metrics": {
      "type": "object",
      "required": [
        "generated_tasks",
        "assigned_tasks",
        "completed_tasks",
        "remaining_tasks",
        "completion_rate_pct",
        "throughput_tasks_per_min",
        "makespan_sec",
        "replan_count",
        "minimum_inter_robot_distance_m",
        "collision_contact_events"
      ]
    }
  }
}
```
