# M9-V2 REMEDIATION DESIGN SPECIFICATION
## Comprehensive Architectural & Algorithmic Defense-in-Depth for Congested Environments

**Document ID**: M9-V2-DES-001  
**Status**: APPROVED FOR IMPLEMENTATION  
**Target Milestone**: Milestone 9 (Congested Warehouse M9-V2)  
**Baseline Reference**: M8B (Frozen Baseline), M9-V1 (Frozen Baseline), M9-V2 Pilot 1 & Pilot 2 (Canonical Failure Baselines)  

---

## 1. Executive Summary & Problem Formulation

In Milestone M9-V2 Pilot 2 (10 AMRs, 30 tasks, seed 42, `NORMAL` communication, `ADAPTIVE` compute, 180 s horizon), a safety abort occurred at simulation time $T = 149.34\,\text{s}$ at the Central Delivery Hub:
- **Minimum Inter-Robot Center Distance**: $0.341\,\text{m}$ (breached the inviolable $0.350\,\text{m}$ safety threshold).
- **Proximity Breaches**: 1
- **OBB Chassis-Overlap Proxy Samples**: 394 samples (indicative of geometric bounding-box intersection during queue compression).
- **Location**: Central Delivery Bay 1 $(14.5, 14.5)$ along the single-track northern approach corridor ($X=29$).
- **Involved Agents**: `amr_3` (stationary lead vehicle at $(14.75, 14.94)$) and `amr_4` (approaching follower vehicle compressed to $(14.75, 15.28)$).

The comprehensive pre-remediation failure investigation established that this failure was caused by an unfortunate conjunction of four distinct architectural issues across different layers of the software stack:
1. **Waypoint / Task-Goal Completion Mismatch**: Continuous station coordinates $(14.5, 14.5)$ are offset by $0.354\,\text{m}$ from the discrete grid-cell center $(14.75, 14.75)$. Waypoint exhaustion occurred at $(14.75, 14.94)$ where distance to station was $0.506\,\text{m} > 0.500\,\text{m}$ (goal tolerance), stalling `amr_3` indefinitely in the entry lane.
2. **Discrete Reservation Footprint Absence**: `SpaceTimeReservationTable` models robots as 0D point particles. On a $0.50\,\text{m}$ grid, adjacent cells $(29, 29)$ and $(29, 30)$ have a nominal separation of only $0.50\,\text{m}$, whereas the physical AMR chassis length is $0.65\,\text{m}$ (yielding a nominal bumper clearance of $-0.15\,\text{m}$). The reservation table allowed `amr_4` to reserve and occupy $(29, 30)$ directly behind `amr_3`.
3. **LiDAR Sub-0.15m Blind Spot**: The 2D LiDAR has `range_min = 0.15 m` and is mounted $0.22\,\text{m}$ forward of center. When the bumper separation between vehicles dropped below $0.15\,\text{m}$, Gazebo rays returned non-finite values (`inf`) which `_handle_scan()` discarded, deactivating reactive safety braking.
4. **Unmanaged Shared Station Hub Capacity**: Central Delivery Bay 1 was targeted by 9 out of 30 tasks without queue-holding sidings or bay locking, creating an uncontrolled single-file convoy.

This document defines the 4-Pillar Remediation Design to eliminate these failure modes permanently while maintaining all baseline invariants.

---

## 2. Invariants and Architectural Boundaries

The remediation strictly complies with all project guidelines:
1. **Frozen Baseline Preservation**: CBBA allocation logic, RHCR rolling horizon $(h=10, w=4, \text{replan\_rate}=2.0\,\text{Hz})$, PIBT core arbitration principles, WFG cycle detection, and M8B adaptive compute policy remain conceptually intact.
2. **Safety Threshold Invariant**: The $0.350\,\text{m}$ center-to-center minimum distance threshold is an absolute safety invariant. It is **NOT** modified, relaxed, or tuned downwards.
3. **No Artificial Goal Tolerance Inflation**: The $0.500\,\text{m}$ task-goal completion tolerance is **NOT** expanded.
4. **Warehouse Geometry Preservation**: The physical layout of `warehouse_m9_v2` (including the 3.8 m North Cross-Aisle and 1.8 m interior aisles) is preserved without widening central corridors.
5. **Sensor Invariants**: No synthetic or fictitious sensors are introduced. Robots operate strictly with 2D LiDAR, wheel odometry, and inter-agent ROS 2 network messaging.
6. **Workload Invariants**: Fleet size (10 AMRs), workload (30 tasks), seed (42), and evaluation horizon (180 s) remain strictly identical to canonical Pilot 2.

---

## 3. The 4 Remediation Pillars

```
┌────────────────────────────────────────────────────────────────────────┐
│                        4-PILLAR DEFENSE-IN-DEPTH                       │
├────────────────────────────────────────────────────────────────────────┤
│ Pillar A: Goal Harmonization    │ Pillar B: Headway-Aware Reservation  │
│ Terminal waypoint explicitly    │ Space-time table & coordination      │
│ targets continuous sub-goal     │ enforce k >= 2 longitudinal headway  │
│ coordinates (dist <= 0.25 m).   │ (center dist >= 1.0 m, clear >= 0.35)│
├─────────────────────────────────┼──────────────────────────────────────┤
│ Pillar C: Local Safety Backstop │ Pillar D: Shared Station Management  │
│ Deterministic odometric peer-   │ Decentralized bay locking & staging  │
│ distance brake (D_stop = 0.70 m)│ in holding cell (29, 33) prevents    │
│ covering LiDAR blind spot.      │ corridor compression at hub.         │
└─────────────────────────────────┘──────────────────────────────────────┘
```

### Pillar A: Waypoint / Task-Goal Harmonization
- **Component**: `RollingHorizonPlanner` (`rh_planner.py`), `RollingHorizonPlannerNode` (`rh_node.py`).
- **Mechanism**: When `replan()` converts `grid_path` into `world_path`, the terminal waypoint of `world_path` is explicitly set to the continuous sub-goal coordinates `(self.current_goal[0], self.current_goal[1])` instead of the cell center `to_world(grid_path[-1])`.
- **Single-Cell Plan Handling**: When `start_grid == goal_grid` and the robot has not yet satisfied `check_subgoal_arrival()`, `world_path` is constructed as `[self.current_position, (self.current_goal[0], self.current_goal[1])]`.
- **Mathematical Guarantee**: Intermediate waypoint arrival radius is $0.250\,\text{m}$. When the robot arrives at the final waypoint, its distance to `self.current_goal` is $\le 0.250\,\text{m} < 0.500\,\text{m}$. Therefore, `check_subgoal_arrival()` evaluates to `True` deterministically, triggering task completion without stalling.

### Pillar B: Physical Footprint / Headway-Aware Reservation
- **Component**: `SpaceTimeReservationTable` (`reservation_table.py`), `RollingHorizonPlannerNode` (`rh_node.py`).
- **Mechanism**: Enforce a minimum longitudinal headway of $k_{\text{headway}} \ge 2$ discrete cells along the direction of travel during reservation checks.
- **Formulation**: If robot $A$ attempts to reserve vertex $C_{\text{target}}$ at $t+1$ moving from $C_{\text{curr}}$, let $\mathbf{v} = C_{\text{target}} - C_{\text{curr}}$. A headway conflict is declared if any peer robot $B$ occupies or reserves cell $C_{\text{ahead}} = C_{\text{target}} + \mathbf{v}$ at $t+1$. In addition, a follower cannot enter an adjacent cell within Manhattan distance 1 directly behind a lead vehicle that is stationary or occupying the corridor ahead.
- **Mathematical Guarantee**: Minimum center-to-center discrete separation for collinear queuing is $2 \times 0.50\,\text{m} = 1.00\,\text{m}$. Given chassis length $L = 0.65\,\text{m}$, nominal bumper clearance is $1.00 - 0.65 = +0.35\,\text{m}$. Under empirical tracking noise ($\pm 0.19\,\text{m}$), center separation is $\ge 0.81\,\text{m} \gg 0.350\,\text{m}$, mathematically preventing proximity breaches.

### Pillar C: Deterministic Local Safety Backstop
- **Component**: `RollingHorizonPlannerNode` (`rh_node.py`) motion control loop (`_control_loop()`).
- **Mechanism**: Independent of LiDAR return validity, the motion controller computes real-time relative Cartesian coordinates $(d_\parallel, d_\perp)$ in the robot body frame against all known peer robots:
  $$d_\parallel = (x_p - x_r) \cos\theta_r + (y_p - y_r) \sin\theta_r$$
  $$d_\perp = -(x_p - x_r) \sin\theta_r + (y_p - y_r) \cos\theta_r$$
- **Trigger Envelope**:
  1. $0 < d_\parallel \le D_{\text{stop}} = 0.700\,\text{m}$ (longitudinal stopping distance)
  2. $|d_\perp| \le W_{\text{corridor}} / 2 = 0.400\,\text{m}$ (lateral corridor half-width)
- **Response**: When triggered, `cmd.linear.x` is immediately forced to $0.0\,\text{m/s}$ (emergency reactive brake).
- **Mathematical Guarantee**: At $d_\parallel = 0.700\,\text{m}$, center distance is $0.700\,\text{m} > 0.350\,\text{m}$ and bumper-to-bumper clearance is $+0.050\,\text{m}$ ($+5\,\text{cm}$ positive clearance). This acts as a deterministic fail-safe against the sub-$0.15\,\text{m}$ LiDAR blind spot.

### Pillar D: Shared Station Capacity Management & Staging Buffers
- **Component**: `RollingHorizonPlannerNode` (`rh_node.py`).
- **Mechanism**:
  1. **Bay Claim**: When an AMR enters the discrete bay cell $(29, 29)$ in `TRANSIT_TO_DROPOFF` or delivery state, it claims ownership of Bay 1.
  2. **Holding Siding Staging**: Any following AMR targeting Bay 1 checks bay availability. If Bay 1 is claimed by a lead vehicle, the follower halts at designated holding cell $(29, 33)$ ($x = 14.75\,\text{m}, y = 16.75\,\text{m}$) outside the terminal approach chute.
  3. **Release & Handover**: Upon delivery completion and bay vacation by the lead AMR, the bay lock is released, allowing the staged follower to advance safely into the bay.
- **Mathematical Guarantee**: The distance between holding cell $(29, 33)$ and bay cell $(29, 29)$ is $2.00\,\text{m}$. Nominal clearance is $1.35\,\text{m}$, entirely eliminating queue accumulation in the single-track chute.

---

## 4. Instrumentation Terminology Correction (Pillar E / Benchmark)
- **Component**: `run_benchmark.py`, `benchmark_manager.py`.
- **Correction**: Distinguish and report separately:
  1. `min_center_to_center_dist`: Continuous center distance (inviolable threshold $0.350\,\text{m}$).
  2. `proximity_breaches`: Count of center distance samples $< 0.350\,\text{m}$.
  3. `obb_chassis_overlap_samples`: 2D Oriented Bounding Box geometric intersection proxy via Separating Axis Theorem (SAT).
  4. `safety_brake_interventions`: Reactive brake events triggered by LiDAR or deterministic peer backstop.
  5. `safety_aborts`: Simulation terminations triggered by safety threshold breach.
- Backward compatibility aliases (`physical_gazebo_contacts` and `collision_events`) are retained.

---

## 5. Implementation Phases & Verification Strategy

| Phase | Description | Key Modules | Validation Criterion |
| :--- | :--- | :--- | :--- |
| **Phase 1** | Remediation Design Specification | `docs/plans/M9-V2-remediation-design.md` | Approved by design |
| **Phase 2** | Waypoint / Goal Harmonization | `rh_planner.py`, `rh_node.py` | Unit test suite: `test_m9_v2_waypoint_consistency.py` |
| **Phase 3** | Headway-Aware Reservation ($k \ge 2$) | `reservation_table.py`, `rh_node.py` | Unit test suite: `test_m9_v2_headway_reservation.py` |
| **Phase 4** | Local Deterministic Safety Backstop | `rh_node.py` | Unit test suite: `test_m9_v2_safety_backstop.py` |
| **Phase 5** | Shared Station Capacity & Staging | `rh_node.py`, `reservation_table.py` | Unit test suite: `test_m9_v2_station_resource.py` |
| **Phase 6** | Benchmark Instrumentation Update | `run_benchmark.py`, `benchmark_manager.py` | Metric reporting verification |
| **Phase 7** | Full Regression Suite | All packages | 170 existing + new unit tests 100% passing |
| **Phase 8** | Controlled Validation (Tests A–E) + Integrated Pilot | Live Gazebo Harmonic simulation | 0 breaches, 0 aborts, $d_{\min} \ge 0.350\,\text{m}$ |
