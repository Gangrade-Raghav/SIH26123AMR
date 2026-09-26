# NRDAS-FR // Milestone 3: Adversarial Environment & Collision Resilience

**Project**: NRDAS-FR (NRDAS v2) Fault-Resilient Decentralized AMR Fleet Coordination  
**Milestone**: Milestone 3 — Adversarial Environment & Collision Resilience  
**Date**: September 2026  
**Status**: All defined M3 software/integration validation scenarios passed.  

---

## 1. Research Question & Core Objective

In automated warehousing and industrial logistics, Autonomous Mobile Robots (AMRs) do not operate in static or benign conditions. Real-world facilities exhibit sudden environmental disruptions:
- Unscheduled dynamic aisle blockages (fallen cartons, maintenance barriers, forklift crossings).
- Unheralded physical obstacles detected solely via onboard ranging sensors.
- High-contention bottleneck intersections and opposing narrow-corridor entries.
- Stranded robot chassis failures occurring directly inside critical choke points.

Milestone 3 extends NRDAS-FR into adversarial environmental conditions to resolve the core research question:
> **"Can a decentralized AMR fleet maintain strict safety envelopes, resolve localized spatio-temporal path and reservation contention without deadlock or physical collisions, and autonomously recover delivery throughput in the presence of dynamic obstacles and adversarial bottlenecks without modifying canonical baseline algorithms?"**

### Strict Preservation of Frozen Baselines
Per project specification, all canonical baseline algorithms and benchmarks remain strictly untouched:
- **NRDAS v1 Core**: Decentralized CBBA task allocation, A* single-agent pathfinding, Rolling-Horizon Collision Resolution (RHCR), PIBT-style local fallback, Wait-For-Graph (WFG) cycle detection, and Space-Time Reservations.
- **Milestone Baselines**: M7 Communication Degradation Benchmarks, M8A Workload Scaling, M8B Adaptive Compute Policy, M9 Benchmarks, M1/M1.1 Node Failure Recovery, and M2 Network Resilience.
- Zero modifications were made to frozen algorithmic logic.

---

## 2. Safety Hierarchy & Perception Decoupling

### 2.1 Formal Safety Authority
When dynamic obstacles or contention arise, local decision-making is strictly governed by the established NRDAS-FR safety hierarchy:

$$\text{Local Reactive LiDAR Safety (0.28m)} > \text{Space-Time Corridor Reservations} > \text{Fault Recovery} > \text{Local Planning} > \text{CBBA Task Allocation}$$

```
+-------------------------------------------------------------+
| 1. LOCAL LIDAR SAFETY (Physical Reactive Emergency Brake)   |
|    - Experimental/Design Threshold: d_crit < 0.28 m         |
|      (Empirical design threshold; not optimal/universal)    |
|    - Action: Immediate zero-velocity override (v = 0.0 m/s) |
|    - Operates completely independent of network & planning  |
+-------------------------------------------------------------+
                              v
+-------------------------------------------------------------+
| 2. SPACE-TIME CORRIDOR RESERVATIONS & PIBT LOCAL RESOLVER   |
|    - Evaluates footprint-aware clearance (0.65m x 0.45m)    |
|    - Enforces space-time exclusivity: at most 1 AMR / cell  |
|    - PIBT priority push-and-yield on localized contention   |
+-------------------------------------------------------------+
                              v
+-------------------------------------------------------------+
| 3. FAULT DETECTOR & ENVIRONMENT ORACLE RECOVERY             |
|    - Classifies physical obstacle vs. layout blockage       |
|    - Invalidates reservations; releases dead robot claims   |
+-------------------------------------------------------------+
                              v
+-------------------------------------------------------------+
| 4. ROLLING-HORIZON / DYNAMIC A* LOCAL PLANNER               |
|    - Withdraws obstructed cells from traversability graph   |
|    - Computes collision-free detours around obstructions   |
+-------------------------------------------------------------+
                              v
+-------------------------------------------------------------+
| 5. DECENTRALIZED CBBA TASK RE-AUCTION                       |
|    - Reclaims orphaned tasks from choke-point failures     |
|    - Guarantees zero duplicate task ownership (I_uniq)      |
+-------------------------------------------------------------+
```

### 2.2 Strict Separation of Detection Horizon vs. Reactive Braking Threshold
A fundamental contribution of Milestone 3 is the architectural separation between early sensory awareness and terminal reactive braking:
1. **Obstacle Detection Horizon ($d_{\text{det}} \le 1.5\text{ m}$)**:
   - Onboard LiDAR scan sweeps forward within a $24^\circ$ angular arc.
   - When range readings reflect obstacles at $d \le 1.5\text{ m}$ intersecting the planned trajectory, the AMR proactively initiates candidate sidestep clearance evaluation and dynamic graph replanning.
   - Prevents premature emergency stops while maintaining continuous transit.
2. **Reactive Safety Braking Threshold ($d_{\text{brake}} < 0.28\text{ m}$)**:
   - Experimental/design reactive safety threshold derived from kinematic deceleration profiles at nominal transit speed ($0.4\text{ m/s}$ with stopping distance $< 0.25\text{ m}$).
   - Immediately clamps drive velocities to zero ($v_{\text{cmd}} = 0.0\text{ m/s}$) if an obstacle enters this boundary, ensuring zero physical contact under all circumstances.

### 2.3 Onboard Perception vs. Environment Oracle Decoupling
NRDAS-FR enforces rigorous architectural decoupling between sensory perception and global layout updates:
- **Environment Oracle (`AisleBlockageEvent` on `/environment/aisle_blockages`)**: Models scheduled facility layout modifications (e.g. maintenance closures, closed fire doors). It emits layout events that trigger space-time graph withdrawal and reservation invalidation without triggering onboard proximity emergency brakes.
- **Onboard Sensor Perception (`LaserScan` processed by `LocalObstacleDetector`)**: Models unexpected dynamic physical objects (e.g. dropped boxes, stray equipment). In scenario **M3-B2**, zero oracle events are emitted; the entire detection, avoidance evaluation, graph withdrawal, and replanning cycle is driven purely by onboard LiDAR data.

---

## 3. Footprint-Aware Candidate Sidestep Clearance

### 3.1 Kinematic Footprint Geometry
Standard grid-based MAPF planners frequently make point-mass assumptions, treating an open $0.5\text{ m} \times 0.5\text{ m}$ grid cell as sufficient clearance. In contrast, NRDAS-FR coordinates industrial AMRs with physical dimensions:
$$\text{Chassis Length: } L = 0.65\text{ m}, \quad \text{Chassis Width: } W = 0.45\text{ m}$$

On a grid with resolution $\Delta g = 0.5\text{ m}$, an AMR centered in a cell extends longitudinally beyond the cell boundaries:
$$\text{Front / Rear Overhang: } \Delta L = \frac{0.65 - 0.50}{2} = 0.075\text{ m}$$

```
              Direction of Motion (Ahead) --->
       +--------------------+--------------------+
       |                    |  [Overhang Cell]   |
       |  (Candidate Cell)  |                    |
       |     +--------------+-----+              |
       |     | AMR Chassis  |     |              |
       |     | 0.65m x 0.45m|     |              |
       |     +--------------+-----+              |
       |                    |                    |
       +--------------------+--------------------+
```

### 3.2 Clearance Invariants
Before selecting a 1-step lateral sidestep cell $c_{\text{cand}}$, `LocalObstacleDetector.check_footprint_clearance` evaluates:
1. **Static Map Traversability**: $c_{\text{cand}}$ and the longitudinal overhang cell $c_{\text{overhang}}$ must be statically open in `GridWorld`.
2. **Sensor Occupancy**: Neither $c_{\text{cand}}$ nor $c_{\text{overhang}}$ may contain sensor-detected obstacle points.
3. **Space-Time Reservation Exclusivity**:
   $$\forall \tau \in \{t_{\text{current}}, t_{\text{current}} + 1\}, \quad \text{is\_reserved}(c_{\text{cand}}, \tau) = \text{False} \land \text{is\_reserved}(c_{\text{overhang}}, \tau) = \text{False}$$

If any constraint fails, the sidestep is rejected, and the robot transitions safely to `LOCAL_SAFETY_HOLD` while requesting a global A* replan.

---

## 4. Adversarial Injector Isolation & Integrity

### 4.1 Non-Invasive Test Injector
The adversarial test injector (`AdversarialConflictInjector`) is strictly isolated:
- It creates controlled collision-risk conditions (`SAME_CELL`, `OPPOSING_CORRIDOR`, `CROSSING_TRAJECTORIES`, `RESERVATION_CONFLICT`, `HIGH_CONTENTION_INTERSECTION`).
- It contains **zero recovery logic**, zero planning routines, and zero task allocation methods.
- It never disables safety systems, never bypasses reservation tables, and never commands physical contact.

### 4.2 Non-Corrupting Reservation Conflict Injection & Unambiguous Ownership Validation (M3-C4)
In scenario **M3-C4**, the injector attempts a synthetic duplicate write to an active space-time reservation $(c, \tau)$ owned by robot $R_a$:

#### Semantic Disambiguation of Reservation State
In prior iterations, validation tested `is_reserved(cell, step, robot_id='amr_1')`, which returned `True` (meaning reserved by a robot other than `amr_1`). To eliminate semantic ambiguity, `SpaceTimeReservationTable` was augmented with definitive ownership inquiry methods:
- `get_authoritative_owners(cell, time_step) -> Set[str]`: Returns the set of robots possessing authoritative reservations for that cell and time step.
- `get_owner(cell, time_step) -> Optional[str]`: Returns the unique authoritative owner ID, or `None`.
- `is_owner(cell, time_step, robot_id) -> bool`: Explicit boolean predicate verifying whether `robot_id` holds the authoritative reservation.

#### Exact Validation Invariants:
1. **Authoritative Exclusivity**:
   $$\text{authoritative\_owners}(c, \tau) = \{ \text{'amr\_0'} \}$$
2. **Contender Exclusion**:
   $$\text{'amr\_1'} \notin \text{authoritative\_owners}(c, \tau) \quad \land \quad \text{is\_owner}(c, \tau, \text{'amr\_1'}) = \text{False}$$
3. **Cardinality Invariant**:
   $$\|\text{authoritative\_owners}(c, \tau)\| = 1$$
4. **Duplicate Rejection & Non-Corruption**:
   $$\text{duplicate\_write\_accepted} = \text{False}, \quad \text{table\_corruption} = \text{False}$$

The synthetic write is rejected; the internal state remains uncorrupted, valid, and consistent.

---

## 5. Comprehensive Scenario Validation Matrix (All 9 Scenarios)

All 9 scenarios were executed using `scripts/validate_m3_adversarial_scenarios.py`. Every scenario passed with zero geometric overlap proxy violations.

| Scenario ID | Scenario Name | Primary Challenge | Resolving Mechanism | Geometric Overlap Proxy Count | Status |
| :--- | :--- | :--- | :--- | :---: | :---: |
| **M3-A** | Dynamic Aisle Blockage (Oracle) | Scheduled corridor closure (3 cells) | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-B** | Sensor-Visible Obstacle (Synthetic Fixture) | Discrete 8-timestamp breakdown | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-B2** | Sensor-Only Dynamic Obstacle | Zero oracle events; 100% sensor-driven | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-C1** | Same-Cell Contention | 2 AMRs contest cell $(5, 5)$ at $t=2$ | SpaceTimeReservationTable + PIBT | 0 | ✅ PASS |
| **M3-C2** | Opposing Corridor Entry | Narrow 1-cell corridor head-to-head | SpaceTimeReservationTable + PIBT | 0 | ✅ PASS |
| **M3-C3** | Crossing Trajectories | Orthogonal intersection contention at $(6, 6)$ | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-C4** | Injected Reservation Conflict | Synthetic adversarial duplicate write | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-C5** | High-Contention Intersection | 3 AMRs converge on $(7, 7)$ at $t=3$ | PIBT Local Planner | 0 | ✅ PASS |
| **M3-H** | Choke-Point Robot Failure | AMR dies at critical intersection $(7, 7)$ | SingleAgentAStar + ResTable | 0 | ✅ PASS |

### Execution Classification & Provenance Disclosures
- **Scenario Execution Classification**: All scenarios are evaluated at the `INTEGRATION / SIMULATION` level.
- **PIBT Execution Provenance**: Scenarios invoking PIBT (M3-C1, M3-C2, M3-C5) are classified as `PLANNER_LEVEL_PIBT_INTEGRATION` (direct programmatic invocation of `PIBTLocalPlanner.plan_step_with_telemetry`; not full multi-threaded ROS 2 runtime node execution).
- **Geometric Overlap Proxy Grounding**: All contact detection is evaluated as a `GEOMETRIC OVERLAP PROXY` using a $0.45\text{ m}$ chassis radius ($0.8\text{ m}$ center-to-center clearance threshold) and 2D Oriented Bounding Box (OBB) Separating Axis Theorem (SAT). **Zero proxy overlaps in integration/simulation scenarios do not constitute physical collision experiments; physical contact bumpers and dynamic rigid-body contacts are not simulated in Gazebo.**

---

## 6. Detailed Scenario Highlights

### 6.1 M3-B: Discrete 8-Timestamp Breakdown (Synthetic Timing Fixture)
> [!NOTE]
> **Provenance Disclosure**: Scenario M3-B evaluates a **SYNTHETIC TIMING FIXTURE (Instrumentation / Sequence Verification Only)**. Recorded timestamps validate strictly monotonic stage sequencing and telemetry data collection pipelines. They do **not** represent empirical hardware or physical sensor/planner latency measurements.

The synthetic fixture validates strictly monotonic execution ordering:
1. $t_{\text{inject}} = 100.000\text{s}$: Dynamic obstacle inserted into physical world representation.
2. $t_{\text{observed}} = 100.042\text{s}$: Synthesized LiDAR scan registers obstacle in forward arc ($d = 0.90\text{ m}$).
3. $t_{\text{safety\_eval}} = 100.050\text{s}$: Proximity evaluated; reactive brake armed.
4. $t_{\text{graph\_withdrawn}} = 100.055\text{s}$: Obstructed cells removed from local traversability graph.
5. $t_{\text{res\_invalidated}} = 100.059\text{s}$: Active reservations for obstructed cells revoked.
6. $t_{\text{replan\_start}} = 100.061\text{s}$: Single-agent A* initiated for collision-free detour.
7. $t_{\text{replan\_done}} = 100.076\text{s}$: Valid alternative route synthesized ($15.0\text{ ms}$ fixture duration).
8. $t_{\text{resumed}} = 100.084\text{s}$: Motion execution resumed along detour.

$$\text{All intervals satisfy } t_{k+1} > t_k, \quad \text{Geometric Overlap Proxy Count} = 0$$

### 6.2 M3-B2: Sensor-Only Dynamic Obstacle (Zero Oracle Event)
- **Pipeline Terminology**: Evaluates the full sequence:
  $$\text{SYNTHESIZED LASERSCAN INPUT} \to \text{SENSOR PIPELINE INTEGRATION} \to \text{LOCAL RECOVERY} \to \text{REPLANNING}$$
- **Sensor Test Classification**: `SENSOR_PIPELINE_INTEGRATION` (Synthesized LaserScan range array processed by `LocalObstacleDetector`; not Gazebo physical LiDAR simulation).
- **Oracle Isolation**: Oracle event log count = exactly **0** (`oracle_event_count: 0`).
- At $t = 150.0\text{s}$, `amr_0` receives a 180-ray synthesized `LaserScan`; ray 90 reflects $0.90\text{ m}$.
- `LocalObstacleDetector` maps ray hit to world $(1.90, 1.00) \to$ grid cell $(3, 4)$.
- Evaluates footprint-aware sidestep; lateral cells constrained; triggers local graph update.
- Reservation for cell $(3, 4)$ invalidated; A* synthesizes 9-cell detour avoiding $(3, 4)$.
- Geometric Overlap Proxy Count = **0**.

### 6.3 M3-C5: PIBT High-Contention Intersection Telemetry
Three AMRs (`amr_0`, `amr_1`, `amr_2`) converge simultaneously on intersection cell $(7, 7)$ at timestep $t=2$. Telemetry is captured directly from `plan_step_with_telemetry` on `PIBTLocalPlanner`:
```json
{
  "pibt_invoked": true,
  "trigger": "HIGH_CONTENTION_INTERSECTION_ARBITRATION",
  "robots_involved": ["amr_0", "amr_1", "amr_2"],
  "action": "PRIORITIZED_PUSH_AND_YIELD",
  "result": "SUCCESS",
  "duration_ms": 0.049,
  "execution_level": "PLANNER_LEVEL_PIBT_INTEGRATION",
  "assigned_moves": {
    "amr_0": [7, 7],
    "amr_1": [7, 6],
    "amr_2": [8, 7]
  }
}
```
Highest-priority AMR (`amr_0`, priority 3) secures $(7, 7)$; peers yield to adjacent holding cells. Geometric Overlap Proxy Count = **0**.

### 6.4 M3-H: Problematic-Location Failure at Choke Point
`amr_1` suffers fail-stop while traversing bottleneck intersection $(7, 7)$:
1. Reservations held by `amr_1` are purged from `SpaceTimeReservationTable`.
2. An inflated **$0.8\text{ m}$ experimental keep-out envelope** (spanning $5 \times 5 = 25$ cells at $0.5\text{ m}$ resolution) is added to the shared obstacle layer.
3. Peer `amr_0` routes around the entire 25-cell envelope without entering the choke point ($0.28\text{ ms}$ replan).
4. Task `T_M3H_CRITICAL` is transitioned from `IN_PROGRESS` to `PENDING` via CAS invariant.
5. CBBA re-auctions `T_M3H_CRITICAL`; `amr_0` acquires task without duplicate ownership.
6. Geometric Overlap Proxy Count = **0**.

---

## 7. Formal Research Invariants Verification

| Invariant | Formal Definition | Target Value | Empirical Result | Status |
| :--- | :--- | :--- | :--- | :--- |
| **$I_{\text{uniq}}$ (Task Uniqueness)** | $\sum_{r} \mathbb{I}[T_k \in \mathcal{B}_r] \le 1$ | 0 duplicates | 0 duplicate assignments observed across evaluated scenarios | **VERIFIED** |
| **$I_{\text{reserv}}$ (Exclusivity)** | $\left\| \{ R_i \mid (c, \tau) \in \mathcal{S}_i \} \right\| \le 1$ | Max 1 robot / cell / step | Zero multi-owner conflicts granted | **VERIFIED** |
| **$I_{\text{clear}}$ (Chassis Clearance)** | $\| p_a(t) - p_f \| \ge 0.45\text{m}$ | Buffer $\ge 0.45\text{m}$ | $0.8\text{m}$ keep-out envelope enforced | **VERIFIED** |
| **$I_{\text{syn\_integ}}$ (Table Integrity)**| Table uncorrupted on synthetic conflict | 0 corruptions | Authoritative owner == `{'amr_0'}`, contender denied | **VERIFIED** |
| **$I_{\text{sensor\_oracle}}$ (Isolation)**| Sensor replans without oracle event | 0 oracle dependencies | Scenario M3-B2 recovered with 0 oracle events | **VERIFIED** |
| **$I_{\text{adv\_clear}}$ (Proxy Overlap)** | $\text{Proxy\_Overlap}(R_i(t), R_j(t)) = \text{False}$ | 0 proxy overlaps | 0 geometric overlap proxy violations observed | **VERIFIED** |

---

## 8. Definition of Done (DoD) Checklist

- [x] **Adversarial Test Injector**: Implemented in `amr_fleet_core/adversarial_injector.py` with zero recovery logic.
- [x] **Local Obstacle Detector**: Implemented in `amr_fleet_core/local_obstacle_detector.py` with $1.5\text{ m}$ detection horizon, $0.28\text{ m}$ reactive braking, and footprint-aware clearance.
- [x] **Reservation Table Spatial Invalidation & Unambiguous Ownership**: Implemented `invalidate_cells`, `get_authoritative_owners`, `get_owner`, and `is_owner`.
- [x] **Full Test Suite Passing**: 13/13 M3 unit tests, 31/31 M1/M2 tests, and 276/276 total workspace tests passing.
- [x] **Scenario Validation Harness**: All defined M3 software/integration validation scenarios passed in `scripts/validate_m3_adversarial_scenarios.py`.
- [x] **Resilience Dashboard**: Port 8081 updated with M3 controls, 9-stage recovery stepper, and REST APIs.
- [x] **Code Quality**: 100% compliant with `ament_flake8` and `ament_pep257`.
- [x] **Evidence Generated**: JSON and Markdown reports persisted in `docs/evidence/` with strict provenance disclosures.
- [x] **🛑 Human Checkpoint**: Work stopped at Milestone 3 completion. Milestone 4 not started.
