# NRDAS Final Experimental Results Package

This directory contains the final, frozen, and canonical benchmark results for the **Autonomous Mobile Robot (AMR) Decentralized Fleet Coordination System** developed under the NRDAS initiative.

---

## 1. Directory Structure

```
results/final/
├── canonical/          # Exact copies of the approved canonical raw JSON trial outputs
├── aggregated/         # Exact copies of the aggregated benchmark JSON summaries
├── comparisons/        # Cross-scenario analytical summaries and Markdown tables
├── figures/            # High-resolution, publication-grade analytical figures (PNG)
├── tables/             # Structured CSV tables exported directly from trial telemetry
├── FINAL_EXPERIMENT_MATRIX.md  # Comprehensive multi-milestone empirical matrix
└── README.md           # This document
```

---

## 2. Canonical Artifact Inventory

| Milestone | Scenario Identifier | World | Fleet | Workload | Horizon | Raw Telemetry Artifact | Aggregated Summary |
| :--- | :--- | :--- | :---: | :---: | :---: | :--- | :--- |
| **M7** | Comm Resilience | `warehouse_small` | 5 | 15 | 60 s | `canonical/m7_gazebo_raw.json` | `docs/images/m7_degradation_curves.png` |
| **M8A** | Workload Scaling | `warehouse_small` | 5 | 30 | 60 s | `canonical/m8a_w30_canonical_raw.json` | `aggregated/m8a_w30_aggregated.json` |
| **M8B** | Adaptive Compute | `warehouse_small` | 5 | 30 | 60 s | `canonical/m8b_w30_adaptive_raw.json` | `aggregated/m8b_w30_adaptive_aggregated.json` |
| **M9-V1** | Expanded Footprint | `warehouse_m9_v1` | 5 | 15 | 120 s | `canonical/m9_v1_canonical_raw.json` | `aggregated/m9_v1_aggregated.json` |
| **M9-V2** | Congested Baseline | `warehouse_m9_v2` | 10 | 30 | 180 s | `canonical/m9_v2_canonical_raw.json` | `aggregated/m9_v2_aggregated.json` |
| **M9-V3-D** | Dynamic Task Arrival | `warehouse_m9_v2` | 10 | 30 | 180 s | `canonical/m9_v3_d_canonical_raw.json` | `aggregated/m9_v3_d_aggregated.json` |
| **M9-V3-A** | Aisle Blockage | `warehouse_m9_v2` | 10 | 30 | 180 s | `canonical/m9_v3_a_canonical_raw.json` | `aggregated/m9_v3_a_aggregated.json` |
| **M9-V3-E** | Combined Stress | `warehouse_m9_v2` | 10 | 30 | 180 s | `canonical/m9_v3_e_canonical_raw.json` | `aggregated/m9_v3_e_aggregated.json` |

---

## 3. High-Resolution Figure Gallery

The figures in `figures/` were generated directly from empirical telemetry via `scripts/plot_final_results.py`:

1. **`final_m9_cross_scenario_comparison.png`**:
   6-panel comparison of replans, dynamic CBBA latency, packet loss, separation distance, safety interventions, and safety compliance across M9 scenarios.
2. **`final_m9_v3_e_stress_timeline.png`**:
   Multi-event dynamic timeline illustrating task release ($t=45\,\text{s}$), corridor obstruction ($t=45\to90\,\text{s}$), network loss ($t=75\to120\,\text{s}$), concurrent stress window ($t\in[75, 90]\,\text{s}$), and fleet replanning response.
3. **`final_task_accounting_lifecycle.png`**:
   Rigorous accounting validation verifying the lifecycle invariant across all mutually exclusive task states ($\sum S_i = 30$) and remaining task definition.
4. **`final_planning_latency_and_safety.png`**:
   Latency percentiles (Mean, P50, P95, P99) and per-robot replan distribution in the 10-AMR fleet.
5. **`m7_degradation_curves.png`**:
   Empirical communication degradation curves across latency, packet loss, and link outage profiles.
6. **`m8a_workload_scaling.png`**:
   Scalability analysis comparing 15-task vs. 30-task workloads under baseline coordination.
7. **`m8b_adaptive_compute_scaling.png`**:
   Computational efficiency and safety stability across `LOW`, `NORMAL`, `HIGH`, and `ADAPTIVE` compute modes.

---

## 4. Key Empirical Findings

1. **Zero Physical Collisions**:
   Across all canonical runs in Gazebo Harmonic, **zero physical contacts** were detected by the physics engine, and **zero OBB bounding box overlapping samples** occurred.
2. **Communication Resilience**:
   During the $45\,\text{s}$ high packet loss regime ($p_{\text{loss}} = 0.35$), the fleet autonomously tolerated $5,642$ dropped packets ($8.58\%$ overall trial loss) by pruning stale peer reservations without safety aborts or deadlocks.
3. **Dynamic Topology Adaptation**:
   When Aisle 1 South was blocked in Gazebo Harmonic for $45\,\text{s}$, local `GridWorld`s updated within $150\,\text{ms}$, rerouting robots through adjacent corridors without centralized intervention.
4. **Decentralized Allocation Stability**:
   CBBA bundle allocation achieved initial consensus in $14.0\,\text{ms}$ and converged on $15$ dynamically injected tasks in $167.6\,\text{ms}$ while AMRs were actively executing prior assignments.

---

## 5. Experimental Limitations & Research Integrity

- **Sample Size ($n=1$)**:
  The canonical runs represent deterministic executions under fixed pseudorandom seed 42. No statistical claims regarding variance, distribution bounds, or general performance distributions can be made.
- **Metric Definitions**:
  `OBB chassis-overlap samples` represent a geometric SAT/OBB proximity proxy and must not be confused with Gazebo physics contact sensor ground truth.
- **Horizon & Throughput**:
  Within the expanded $32\,\text{m} \times 32\,\text{m}$ warehouse footprint and $0.5\,\text{m/s}$ speed limits, transit times between perimeter racks and the central delivery hub exceed $120\,\text{s}$. Tasks in progress at $180\,\text{s}$ are correctly accounted as `IN_PROGRESS` rather than `COMPLETED`.
