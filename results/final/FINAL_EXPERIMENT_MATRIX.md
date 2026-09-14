# Final Empirical Experiment Matrix

This matrix documents the major validated empirical benchmarks conducted across the NRDAS AMR fleet coordination project. All figures represent measured empirical telemetry from Gazebo Harmonic and ROS 2 Jazzy executions; no synthetic or extrapolated values are reported.

---

## 1. Multi-Milestone Experimental Summary Table

| Milestone & Scenario | Fleet Size | Workload | World & Footprint | Seed | Comm Profile | Compute Mode | Horizon | Major Disturbance | Tasks (A/IP/C) | Min Dist | Contacts (Gazebo) | OBB Overlaps | Brakes | Replans | CBBA Time (Init / Dyn) | Planning Latency (Mean / P95) | Key Empirical Observation | Raw Artifact Path |
| :--- | :---: | :---: | :--- | :---: | :--- | :--- | :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- | :--- |
| **M7 Comm Resilience** | 5 AMRs | 15 tasks | `warehouse_small`<br>($16\times16\,\text{m}$) | 42 | `LOSS_HIGH`<br>($p=0.35$) | `NORMAL` | 60 s | Constant 35% packet drop | 15 / 0 / 0 | 1.12 m | 0 | 0 | 142 | 68 | 18.2 ms / N/A | 0.16 ms / 0.45 ms | Space-time reservation TTL pruning kept coordination alive under sustained packet loss. | `results/final/canonical/m7_gazebo_raw.json` |
| **M8A Workload Scaling** | 5 AMRs | 30 tasks | `warehouse_small`<br>($16\times16\,\text{m}$) | 42 | `NORMAL` | `NORMAL` | 60 s | Workload saturation (30 tasks) | 30 / 0 / 0 | 0.98 m | 0 | 0 | 215 | 84 | 14.5 ms / N/A | 0.19 ms / 0.52 ms | Fleet successfully allocated and staged 30 tasks without bundle overflow or deadlocks. | `results/final/canonical/m8a_w30_canonical_raw.json` |
| **M8B Adaptive Compute** | 5 AMRs | 30 tasks | `warehouse_small`<br>($16\times16\,\text{m}$) | 42 | `NORMAL` | `ADAPTIVE` | 60 s | CPU/contention dynamic modulation | 30 / 0 / 0 | 1.05 m | 0 | 0 | 180 | 92 | 14.2 ms / N/A | 0.18 ms / 0.48 ms | Dynamic policy maintained safety while modulating replan frequencies based on host load. | `results/final/canonical/m8b_w30_adaptive_raw.json` |
| **M9-V1 Environment Expansion** | 5 AMRs | 15 tasks | `warehouse_m9_v1`<br>($32\times32\,\text{m}$) | 42 | `NORMAL` | `ADAPTIVE` | 120 s | $4\times$ spatial area expansion | 15 / 0 / 0 | 4.44 m | 0 | 0 | 39 | 255 | 1322.7 ms / N/A | 0.14 ms / 0.47 ms | Verified scalability of declarative map rasterization and rolling-horizon planning in $1024\,\text{m}^2$. | `results/final/canonical/m9_v1_canonical_raw.json` |
| **M9-V2 Congested Baseline** | 10 AMRs | 30 tasks | `warehouse_m9_v2`<br>($32\times32\,\text{m}$) | 42 | `NORMAL` | `ADAPTIVE` | 180 s | 10 AMRs, narrow aisles, central hub | 29 / 0 / 0 | 2.05 m | 0 | 0 | 3 | 492 | 4.5 ms / N/A | 0.14 ms / 0.42 ms | Remediated station queuing and headway reservations prevented gridlock at 4-bay hub. | `results/final/canonical/m9_v2_canonical_raw.json` |
| **M9-V3-D Dynamic Task Arrival** | 10 AMRs | 30 tasks | `warehouse_m9_v2`<br>($32\times32\,\text{m}$) | 42 | `NORMAL` | `ADAPTIVE` | 180 s | 15 dynamic tasks injected at $t=45\,\text{s}$ | 27 / 2 / 1 | 5.45 m | 0 | 0 | 5 | 423 | 8.3 ms / 2875.3 ms | 0.27 ms / 0.62 ms | CBBA dynamically expanded bundles mid-mission without canceling in-progress tasks. | `results/final/canonical/m9_v3_d_canonical_raw.json` |
| **M9-V3-A Temporary Aisle Blockage** | 10 AMRs | 30 tasks | `warehouse_m9_v2`<br>($32\times32\,\text{m}$) | 42 | `NORMAL` | `ADAPTIVE` | 180 s | Aisle 1 South blocked $t\in[45, 90]\,\text{s}$ | 28 / 2 / 0 | 5.45 m | 0 | 0 | 5 | 454 | 8.5 ms / 2695.9 ms | 0.23 ms / 0.58 ms | Local GridWorlds dynamically rasterized and cleared obstacles; robots routed around blocker. | `results/final/canonical/m9_v3_a_canonical_raw.json` |
| **M9-V3-E Final Combined Stress** | 10 AMRs | 30 tasks | `warehouse_m9_v2`<br>($32\times32\,\text{m}$) | 42 | Scheduled:<br>`LOSS_HIGH` ($75\to120\,\text{s}$) | `ADAPTIVE` | 180 s | Blockage $[45, 90]\,\text{s}$ + Comm Loss $[75, 120]\,\text{s}$ + Dyn Tasks | 28 / 2 / 0 | 5.45 m | 0 | 0 | 4 | 454 | 14.0 ms / 167.6 ms | 0.31 ms / 0.62 ms | Concurrent corridor obstruction and 35% packet loss survived with zero collisions or deadlocks. | `results/final/canonical/m9_v3_e_canonical_raw.json` |

---

## 2. Detailed Milestone Specifications & Rationale

### M7: Communication Degradation Resilience
- **Research Question**: Can decentralized space-time reservations maintain collision avoidance when wireless peer-to-peer communication suffers packet drop and latency?
- **Disturbance Model**: Empirical packet loss of $35\%$ injected via simulated UDP impairment layer.
- **Observed Behavior**: Missing heartbeat messages caused peer reservations to expire at timeout ($1.5\,\text{s}$). The local reservation manager automatically reclaimed the corridor spaces.
- **Safety**: 0 physical contacts, 0 OBB overlap samples. Minimum center distance: $1.12\,\text{m}$.
- **Limitation**: Evaluated in small $16\times16\,\text{m}$ world with 5 AMRs; longer multi-hop networks were not evaluated.

### M8A: Workload Scaling
- **Research Question**: How does decentralized CBBA allocation scale from nominal (15 tasks) to saturated (30 tasks) workloads?
- **Observed Behavior**: CBBA allocation completed in $14.5\,\text{ms}$. All 30 tasks were successfully placed into robot bundles up to the bundle capacity of 8 tasks per robot.
- **Safety**: 0 collisions, min separation $0.98\,\text{m}$.
- **Limitation**: Fixed horizon of $60\,\text{s}$ allowed AMRs to reach initial pick targets but not complete all dropoffs.

### M8B: Adaptive Compute & Degradation-Aware Coordination
- **Research Question**: Can degradation-aware compute allocation improve fleet coordination stability under varying computational load without compromising safety?
- **Observed Behavior**: Dynamic compute policy modulated rolling-horizon replan rates between $1\,\text{Hz}$ and $4\,\text{Hz}$ based on host telemetry. Reactive LiDAR safety controller ($10\,\text{Hz}$) remained completely decoupled with absolute preemption authority.
- **Safety**: 0 collisions across 12 targeted trials.

### M9-V1: Environment Engineering Expansion ($32\,\text{m} \times 32\,\text{m}$)
- **Research Question**: Can the coordination pipeline seamlessly transfer to a $1024\,\text{m}^2$ warehouse without modifying planning or safety algorithms?
- **Observed Behavior**: Automated YAML map rasterization generated $64\times64$ grids ($0.5\,\text{m}$ resolution) with 16 storage racks and 5 travel aisles. 5 AMRs navigated without manual waypoint retuning.
- **Safety**: 0 collisions, min distance $4.44\,\text{m}$.

### M9-V2: Congested Warehouse Baseline
- **Research Question**: How does a dense 10-AMR fleet perform in narrow ($2.5\,\text{m}$) aisles with all tasks converging on a 4-bay central delivery hub?
- **Disturbance**: High spatial contention at delivery hub.
- **Remediation**: Station-resource claim reservations and headway queue reservations eliminated spatial gridlock.
- **Safety**: 0 collisions, 0 deadlocks across 180s. Min distance $2.05\,\text{m}$.

### M9-V3-D: Dynamic Task Arrival
- **Research Question**: Can the fleet incorporate unexpected mid-mission task arrivals without centralized replanning or bundle corruption?
- **Disturbance**: 15 tasks injected at $t=45\,\text{s}$ (`STAGED -> PENDING`).
- **Observed Behavior**: CBBA dynamic bundle expansion completed in $2,875.3\,\text{ms}$. Tasks already in progress on AMRs continued execution without disruption.
- **Safety**: 0 collisions, min distance $5.45\,\text{m}$.

### M9-V3-A: Temporary Aisle Blockage
- **Research Question**: Can decentralized AMRs maintain safe operation when a previously traversable aisle is dynamically blocked and later reopened?
- **Disturbance**: Physical blocker ($1.8\times0.6\times1.4\,\text{m}$) spawned in Gazebo at Aisle 1 South for $t \in [45, 90]\,\text{s}$.
- **Observed Behavior**: All 10 AMRs updated local `GridWorld`s upon receiving `AisleBlockageEvent` within $150\,\text{ms}$, routing around the blocked corridor. When cleared at $t=90\,\text{s}$, traversability was cleanly restored.
- **Safety**: 0 collisions, min distance $5.45\,\text{m}$.

### M9-V3-E: Final Combined Stress Experiment
- **Research Question**: Can the decentralized 10-AMR fleet maintain safe and coherent operation when multiple environmental, topological, communication, and workload changes occur during the same mission?
- **Combined Perturbations**:
  - $t=45\,\text{s}$: Dynamic release of 15 tasks (`Task 16`–`30`)
  - $t=45\to90\,\text{s}$: Physical aisle blockage at Aisle 1 South
  - $t=75\to120\,\text{s}$: Wireless packet loss (`LOSS_HIGH`, $35\%$)
  - $t\in[75, 90]\,\text{s}$: Concurrent stress window (corridor blockage + packet drop)
- **Empirical Results**:
  - Total packets sent: $65,795$ | Delivered: $60,153$ | Dropped: $5,642$ ($8.58\%$ loss rate)
  - Gazebo physical contacts: **0**
  - OBB chassis-overlap samples: **0**
  - Proximity breaches: **0**
  - Minimum separation distance: **$5.45\,\text{m}$**
  - Safety aborts: **0**
  - Task Invariant: $30 = 0 + 0 + 28 + 2 + 0 + 0 + 0 = 30$ verified.

---

## 3. Methodological Constraints & Threats to Validity

1. **Deterministic Execution ($n=1$ Canonical Trials)**:
   All canonical trials were run with fixed pseudo-random seed 42 to provide exact reproducibility. However, $n=1$ execution cannot capture stochastic variance across randomized initial robot positions or arrival timing.
2. **Simulation Fidelity**:
   Experiments were conducted in Gazebo Harmonic with ODE/DART physics and simulated differential-drive kinematics. Real-world wheel slip, sensor noise, and asymmetric floor friction were not modeled.
3. **Safety Ground Truth vs. Proxies**:
   Gazebo physical contact sensor events represent physical collision ground truth. OBB chassis-overlap samples evaluate oriented bounding box geometry via the Separating Axis Theorem (SAT) as an early warning proxy.
