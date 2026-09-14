# Milestone M6 Implementation Plan: Multi-Agent Collision Avoidance, Reservations & Deadlock Handling

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Status**: COMPLETED & VERIFIED  
**Milestone**: M6 — Multi-Agent Collision Avoidance, Reservations & Deadlock Handling  
**Target Environment**: ROS 2 Jazzy, Gazebo Harmonic 8.11.0 / 8.15.0, Ubuntu 24.04 LTS  

---

## 1. Executive Objective

The objective of Milestone M6 is to extend the Milestone M5 single-agent rolling-horizon planning layer into a genuine, decentralized multi-agent path coordination layer.

While **M4** answers *"Which robot gets which tasks?"* and **M5** answers *"What route/schedule does each robot compute for its current sub-goal over horizon $h$ and execution window $w$?"*, **M6** answers:
> *"How do multiple autonomous mobile robots sharing the same physical warehouse floor detect path conflicts, negotiate space-time occupancy, prioritize movement via Priority Inheritance with Backtracking (PIBT), detect persistent deadlocks using Wait-For Graphs (WFG), and deterministically recover without physical collision?"*

```
CBBA Task Allocation (M4)
          ↓
Assigned Bundles [t_1, t_2, ..., t_k]
          ↓
Rolling-Horizon Task Planner (M5)
(Single-Agent A* Path, Sub-goal State Machine, Horizon h, Window w)
          ↓
M6 Multi-Agent Path Coordination Layer
  ├── Space-Time Reservation Table (Vertex & Edge Reservations)
  ├── Conflict Detection Engine (Vertex, Edge-Swap, Waiting Conflicts)
  ├── PIBT Priority Coordination (Deterministic Priorities, Inheritance, Backtracking)
  └── WFG Deadlock Detector & Deterministic Recovery (Tarjan SCC, Lateral Sidestepping)
          ↓
Safety Arbitrator & LiDAR Forward Brake (< 0.65m)
          ↓
Differential-Drive Velocity Commands (`/cmd_vel`)
          ↓
Gazebo Harmonic Fleet Execution (amr_0 ... amr_4)
```

The goal of M6 is not to claim impossible universal collision-free operation under all adversarial conditions, but to build a mathematically grounded, measurable, and testable multi-agent coordination layer and experimentally characterize its real-world behavior in Gazebo simulation.

---

## 2. Invariants, Scope Boundaries & Non-Goals

1. **Strict M5 Preservation & Wrapping**:
   - The M5 single-agent planner (`RollingHorizonPlanner`, `SingleAgentAStar`, `TaskSequencer`) remains intact.
   - M6 wraps M5 via `MultiAgentCoordinator`, filtering proposed trajectories through the reservation table, conflict detector, and PIBT coordinator before sending commands to differential drive controllers.
2. **Decentralized Execution Architecture**:
   - There is **no centralized node** deciding or controlling robot movements.
   - Each robot runs an independent `rh_node` in its own namespace (`/amr_0` ... `/amr_4`), maintaining a local `SpaceTimeReservationTable` and local `WaitForGraph`.
   - Coordination state is communicated peer-to-peer across standardized ROS 2 topics (`/fleet/reservations`, `/fleet/conflicts`, `/fleet/deadlocks`).
3. **Strict Safety Authority Hierarchy**:
   The robot motion pipeline enforces the following immutable hierarchy of control:
   $$\text{Emergency Stop} > \text{LiDAR Forward Safety Brake } (< 0.65\,\text{m}) > \text{PIBT / Reservations} > \text{RHCR Planning} > \text{CBBA Bundles}$$
   If an unmodeled obstacle or coordination delay causes inter-robot distance to drop below $0.65\,\text{m}$, the LiDAR safety brake halts forward velocity unconditionally.
4. **Strict Scope Exclusions (Deferred to M7+)**:
   - **DO NOT** implement communication fault injection, packet loss simulation, or Zenoh QoS degradation (Milestone M9).
   - **DO NOT** implement Non-Holonomic ORCA (NH-ORCA) continuous velocity obstacles.
   - **DO NOT** implement centralized MAPF solvers or MAPF-LNS2.
   - **DO NOT** implement heterogeneous kinematic models or large-scale benchmarking.

---

## 3. Mathematical & Algorithmic Foundations

### 3.1. Discrete Space-Time Representation & Reservations

Let the warehouse floor be discretized into grid cells $v = (x, y) \in \mathbb{Z}^2$ with resolution $\Delta s = 0.5\,\text{m}$ and discrete time steps $t \in \{0, 1, \dots, H\}$ where $\Delta t = 1.0\,\text{s}$.

A space-time reservation $R$ represents exclusive occupancy rights:
- **Vertex Reservation**:
  $$R_v = (v, t, r) \in \mathcal{V} \times \mathcal{T} \times \mathcal{R}$$
  where robot $r$ claims vertex $v$ at time step $t$.
- **Edge Reservation**:
  $$R_e = (u, v, t, r) \in \mathcal{V} \times \mathcal{V} \times \mathcal{T} \times \mathcal{R}$$
  where robot $r$ claims the transition from cell $u$ at time $t$ to cell $v$ at time $t+1$.

#### Conflict Definitions:
1. **Vertex Conflict**:
   $$\exists r_1 \neq r_2, \quad (v, t, r_1) \in \mathcal{R}_V \land (v, t, r_2) \in \mathcal{R}_V$$
   Two robots attempt to occupy the same spatial vertex at the same time step.
2. **Edge-Swap Conflict**:
   $$\exists r_1 \neq r_2, \quad (u, v, t, r_1) \in \mathcal{R}_E \land (v, u, t, r_2) \in \mathcal{R}_E$$
   Robot $r_1$ attempts to traverse $u \to v$ while robot $r_2$ simultaneously attempts to traverse $v \to u$. Both robots would collide head-on midway across the edge.
3. **Waiting Conflict**:
   Robot $r_1$ holds position $(u, u, t, r_1)$ while robot $r_2$ attempts to enter $u$ at time $t+1$.

### 3.2. Priority-Inheritance with Backtracking (PIBT)

PIBT (Okumura et al., 2019) coordinates multi-agent movements one time step at a time without requiring exponential joint space-time search:
1. **Priority Ordering**: Deterministic priority $\rho_i$ is assigned to each agent:
   $$\rho_i = (\text{task\_priority}_i \times 1000) + \text{dist\_to\_goal}_i - (\text{wait\_steps}_i \times 50) + \text{tie\_breaker}(i)$$
   Higher $\rho_i$ gives earlier reservation rights.
2. **Candidate Generation**: Agent $i$ generates candidate moves ordered by heuristic distance to goal: $C_i = [c_1, c_2, \dots, c_k]$.
3. **Priority Inheritance Push**: If agent $i$'s desired cell $v^*$ is currently occupied by lower-priority agent $j$, agent $i$ "pushes" agent $j$ to find an alternative cell. Agent $j$ temporarily inherits agent $i$'s priority level during recursive resolution.
4. **Backtracking**: If agent $j$ cannot find any valid cell, the push fails, and agent $i$ backtracks to evaluate candidate $c_2$, or yields by holding position.

### 3.3. Wait-For Graph (WFG) & Persistent Deadlock Detection

A directed Wait-For Graph $\mathcal{G}_{\text{WFG}} = (\mathcal{V}_{\text{WFG}}, \mathcal{E}_{\text{WFG}})$ models dependency relationships:
- Vertices $\mathcal{V}_{\text{WFG}} = \{r_1, r_2, \dots, r_n\}$ correspond to robots.
- A directed edge $(r_i \to r_j) \in \mathcal{E}_{\text{WFG}}$ exists if robot $r_i$ is waiting for a cell currently occupied or reserved by robot $r_j$.

#### Cycle Detection & Persistence Filtering:
1. **Cycle Detection**: Tarjan's strongly connected components (SCC) algorithm finds directed cycles $\mathcal{C} = (r_1 \to r_2 \to \dots \to r_k \to r_1)$ where $|\mathcal{C}| \ge 2$.
2. **Transient vs. Persistent Distinction**:
   - A transient wait occurs when robot $r_i$ pauses for $r_j$ to clear an intersection, resolving naturally in $< 1.5\,\text{s}$.
   - A **deadlock** is declared if and only if:
     $$\Delta t_{\text{cycle}} \ge t_{\text{persist}} \; (1.5\,\text{s}) \quad \land \quad \text{stall\_cycles} \ge N_{\text{stall}} \; (3 \text{ cycles})$$

### 3.4. Deterministic Deadlock Recovery

When a persistent cycle is detected:
1. **Victim Selection**: The lowest-priority participant $r_v \in \mathcal{C}$ with an available lateral escape cell is selected as the victim.
2. **Escape Cell Evaluation**: Lateral cells orthogonal to the blocked corridor direction are evaluated for traversability and reservation freedom.
3. **Lateral Sidestep**: Robot $r_v$ relinquishes its forward reservation, reserves the lateral escape cell, and executes a 1-step sidestep. This breaks the cycle, allowing higher-priority robots to traverse the corridor unobstructed.

---

## 4. Implementation Breakdown

### 4.1. ROS 2 Interface Messages (`amr_fleet_msgs`)
- `SpaceTimeReservation.msg`: Robot ID, cell coordinates $(x, y)$, time step $t$, duration, reservation type (`VERTEX` / `EDGE`), and target cell for edges.
- `ConflictReport.msg`: Detected conflict type (`VERTEX`, `EDGE_SWAP`, `WAITING`), participant robot IDs, cell coordinates, time step, and timestamp.
- `DeadlockEvent.msg`: Cycle participants list, persistence duration, stall cycle count, selected victim robot ID, recovery action, and timestamp.
- `CoordinationStatus.msg`: Robot ID, coordination state (`NOMINAL`, `WAITING`, `YIELDING`, `SIDESTEPPING`, `DEADLOCKED`), active priority score, waiting-on robot ID, and active reservation count.

### 4.2. Algorithmic Coordination Engine (`amr_fleet_core`)
- `coordination_models.py`: Data models, dataclasses, and enums (`ConflictType`, `CoordinationState`, `Reservation`, `Conflict`, `DeadlockRecord`).
- `reservation_table.py`: `SpaceTimeReservationTable` supporting vertex and edge reservations, ownership queries, collision checking, and rolling-horizon expiry (`release_time_before`).
- `conflict_detector.py`: `ConflictDetector` identifying vertex overlaps, bidirectional edge swaps, and waiting occupancy conflicts.
- `pibt_planner.py`: `PIBTLocalPlanner` implementing candidate generation, deterministic priorities, priority inheritance push, and recursive backtracking.
- `wfg_deadlock.py`: `WaitForGraph` tracking directed wait edges and Tarjan's SCC cycle detection; `DeadlockDetector` enforcing persistence thresholds.
- `deadlock_recovery.py`: `DeadlockRecoveryManager` implementing victim selection, orthogonal lateral cell search, and sidestep commands.
- `multi_agent_coordinator.py`: High-level wrapper coordinating `RollingHorizonPlanner`, reservations, conflicts, PIBT, and deadlock recovery.

### 4.3. ROS 2 Decentralized Node Integration (`rh_node.py`)
- Independent instance per robot (`/amr_0` ... `/amr_4`).
- Maintains local `SpaceTimeReservationTable` synchronized via `/fleet/reservations`.
- Publishes detected conflicts to `/fleet/conflicts` and deadlocks to `/fleet/deadlocks`.
- Publishes per-robot status to `/{robot_id}/coordination_status`.
- LiDAR safety brake integration: halts robot forward motion if obstacle detected within $0.65\,\text{m}$.

### 4.4. Launch & Dashboard (`amr_fleet_bringup`, `fleet_dashboard.py`)
- `m6_coordination_fleet.launch.py`: Brings up Gazebo Harmonic warehouse simulation with 5 AMRs, launching decentralized `rh_node` instances with active M6 coordination.
- `fleet_dashboard.py`: Industrial Brutalist web dashboard on port 8080 displaying:
  - Live 2D floorplan with rolling-horizon paths and active reservations.
  - Conflict log and active deadlock alerts.
  - Per-robot coordination states, priorities, and waiting dependencies.
  - Real-time minimum inter-robot distance gauge.

---

## 5. Verification & Experimental Protocol

### 5.1. Automated Test Suites
1. **Workspace Test Suite**:
   ```bash
   colcon test --packages-select amr_fleet_msgs amr_fleet_core amr_fleet_sim amr_fleet_bringup
   colcon test-result --all --verbose
   ```
   Requires 100% pass rate with 0 errors and 0 failures.
2. **M6 Algorithmic Verification Suite**:
   ```bash
   python3 scripts/verify_m6_coordination.py
   ```
   Verifies 7 sections: reservations, conflict detection, PIBT coordination, WFG cycle detection, deadlock recovery, coordinator integration, and bitwise determinism.
3. **Linter Compliance**:
   ```bash
   ament_flake8 src/amr_fleet_core src/amr_fleet_bringup
   ```
   Requires 0 lint errors.

### 5.2. Gazebo Conflict Scenarios (5 AMRs)
`scripts/demonstrate_real_gazebo_m6.py` evaluates 5 distinct scenarios:
- **Scenario A (No-Conflict Parallel Motion)**: Two AMRs moving in adjacent parallel lanes. Expectation: 0 conflicts, continuous motion.
- **Scenario B (Vertex Conflict Resolution)**: Two AMRs converging on the same 4-way intersection. Expectation: Priority arbitration yields lower-priority robot; higher-priority robot passes cleanly.
- **Scenario C (Edge-Swap Prevention)**: Two AMRs head-on in a single-lane corridor. Expectation: Edge swap rejected; lower-priority robot stops/yields, preventing head-on collision.
- **Scenario D (Temporary Wait Without Deadlock)**: Crossing paths causing transient wait ($< 1.5\,\text{s}$). Expectation: Transient wait cleared without false-positive deadlock event.
- **Scenario E (Deadlock Cycle & Deterministic Recovery)**: Two AMRs mutual head-to-head deadlock. Expectation: WFG detects cycle after $1.5\,\text{s}$; victim selected; lateral sidestep executed; corridor cleared.

---

## 6. Definition of Done Checklist

- [x] Space-time discrete representation implemented
- [x] Reservation table implemented with vertex/edge support
- [x] Vertex conflict detection implemented
- [x] Edge-swap conflict detection implemented
- [x] PIBT local coordination implemented
- [x] Deterministic priorities implemented
- [x] Wait-For Graph implemented
- [x] Persistent deadlock detection implemented
- [x] Deterministic recovery implemented
- [x] ROS 2 integration complete
- [x] M5 integration complete (wrapped cleanly)
- [x] Real 5+ AMR Gazebo validation complete
- [x] Vertex conflict experiment complete
- [x] Edge-swap experiment complete
- [x] Deadlock/recovery experiment complete
- [x] Collision/contact measurements collected
- [x] Dashboard integration complete
- [x] Meaningful automated tests pass (114/114)
- [x] M0–M5 regression passes
- [x] Documentation complete
- [x] Git status reviewed
- [x] Checkpoint report complete
