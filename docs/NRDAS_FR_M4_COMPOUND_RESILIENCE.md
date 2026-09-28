# NRDAS-FR Milestone 4: Compound Fault Resilience & Adversarial Recovery

## 1. Executive Summary

Milestone 4 extends the decentralized resilience framework of **NRDAS-FR** (Non-stop Resilient Decentralized Auction System — Fault Resilience) to address the **composition of multiple simultaneous and overlapping faults**.

In industrial AMR deployments, failures do not occur in isolation. A single primary incident often initiates cascading secondary and tertiary failures:
1. An AMR hardware crash or power failure halts a vehicle in a narrow corridor.
2. The disabled vehicle becomes a physical obstruction requiring dynamic graph withdrawal and reservation invalidation.
3. Concurrent network packet loss or transient partitions degrade inter-robot coordination.
4. Surviving AMRs reroute through congested intersections while encountering unmapped dynamic obstacles.

Milestone 4 establishes formal mathematical invariants, non-blocking fault discrimination, compare-and-swap (CAS) task reclamation, monotonic partition reconciliation, and bounded local autonomy.

---

## 2. Mathematical Formalization & Invariants

Let $\mathcal{V} = \{1, \dots, N\}$ denote the fleet of AMRs operating on a 2D discrete spacetime lattice $\mathcal{G} \times \mathbb{N}$. Let $\mathcal{T}$ be the global set of production tasks, and $\mathcal{B}_i(t) \subseteq \mathcal{T}$ denote the bundle of tasks assigned to robot $i$ at time $t$.

### Invariant $I_1$: Task Uniqueness (Mutual Exclusion of Task Allocation)
At any point in physical time $t$, no active task $T \in \mathcal{T}_{active}$ may be simultaneously held or executed by more than one robot:
$$\forall T \in \mathcal{T}_{active}, \quad \sum_{i \in \mathcal{V}} \mathbb{I}[T \in \mathcal{B}_i(t)] \le 1$$
Where $\mathbb{I}[\cdot]$ is the indicator function. If a robot fails, its active tasks must be reclaimed to state `PENDING` via an atomic CAS transition before re-auctioning.

### Invariant $I_2$: Spacetime Reservation Exclusivity
At any spacetime cell $(c, \tau) \in \mathcal{G} \times \mathbb{N}$, at most one authoritative owner may hold a traversal reservation:
$$\forall c \in \mathcal{G}, \forall \tau \in \mathbb{N}, \quad |\{i \in \mathcal{V} : (c, \tau) \in \mathcal{R}_i\}| \le 1$$
Upon robot failure or corridor blockage, the Invalidation Manager instantly purges all future reservations associated with the affected agent or cell:
$$\mathcal{R}(t^+) = \mathcal{R}(t^-) \setminus \{(c, \tau) : c \in \mathcal{O}_{blocked} \lor \text{owner}(c, \tau) \in \mathcal{V}_{failed}\}$$

### Invariant $I_3$: Local Safety & Clearance Guarantee
Let $p_i(t) \in \mathbb{R}^2$ be the continuous spatial position of robot $i$, and $\mathcal{O}(t)$ be the set of static and dynamic obstacle boundaries. Every active AMR must maintain a minimum physical clearance exceeding the safety threshold $r_{robot} = 0.28\,\text{m}$:
$$\forall i \in \mathcal{V}_{active}, \quad \text{dist}(p_i(t), \mathcal{O}(t)) \ge r_{robot} = 0.28\,\text{m}$$
If an unmapped obstacle enters the forward $24^\circ$ LiDAR arc at a distance $d \le 0.28\,\text{m}$, the onboard reactive safety layer commands immediate zero linear velocity ($v_x = 0.0\,\text{m/s}$).

---

## 3. Orthogonal 5-Dimensional State Space

To avoid ambiguous, coupled, or split-brain fleet states, NRDAS-FR models each AMR along five strictly orthogonal dimensions:

$$\mathcal{S}_i = \langle s_{health}, s_{comm}, s_{task}, s_{nav}, s_{res} \rangle$$

1. **Health State** $s_{health} \in \{\text{HEALTHY}, \text{COMM\_LOSS}, \text{FAILED}, \text{ESTOP}\}$: Monitored via local and peer debounce heartbeats.
2. **Communication State** $s_{comm} \in \{\text{CONNECTED}, \text{IMPAIRED}, \text{PARTITIONED}, \text{DISCONNECTED}\}$: Evaluated by the network channel model.
3. **Task Lifecycle State** $s_{task} \in \{\text{STAGED}, \text{PENDING}, \text{ASSIGNED}, \text{IN\_PROGRESS}, \text{COMPLETED}, \text{FAILED}, \text{CANCELLED}\}$: Strictly sequential, validated transitions.
4. **Navigation Mode** $s_{nav} \in \{\text{IDLE}, \text{TRANSIT}, \text{LOCAL\_SAFETY\_HOLD}, \text{SIDESTEP\_RECOVERY}, \text{ESTOP\_HOLD}\}$: Governed by local sensory inputs and clearance.
5. **Reservation Ownership** $s_{res} \in 2^{\mathcal{G} \times \mathbb{N}}$: Authoritative spacetime leases managed in `SpaceTimeReservationTable`.

---

## 4. Race Condition Mitigations

### Race 1: Dual Task Reclamation (Atomic CAS State Invariant)
- **Hazard**: Multiple surviving peers detect a robot failure at slightly different times ($t_1 \ne t_2$) and both attempt to reclaim its orphaned task.
- **Mitigation**: Task transitions enforce Compare-And-Swap semantics:
  $$\text{CAS}(T, \text{expected}=\text{ASSIGNED}(\text{victim}), \text{target}=\text{PENDING})$$
  The first responder successfully sets the task to `PENDING` and clears `assigned_robot_id`. Subsequent peer attempts see `state == PENDING` or `assigned_robot_id != victim` and are rejected with zero state mutation.

### Race 2: Stale Reservation Resurrection
- **Hazard**: An AMR disconnected during a network outage reconnects and attempts to resume its pre-outage spacetime reservations that have already been released and reallocated.
- **Mitigation**: Reconnecting AMRs must query `is_owner(c, \tau, robot_id)`. If another agent was granted the cell during the outage, the reconnecting AMR's reservation assertion is rejected, forcing a local replan.

### Race 3: Choke Point Failure vs. Perception Primacy
- **Hazard**: An AMR crashes inside a high-speed transit aisle. Inbound peers travelling towards the cell might collide before the $3.5\,\text{s}$ heartbeat timeout confirms the failure.
- **Mitigation**: Onboard LiDAR reactive braking operates at $20\,\text{Hz}$ independently of network communication. Forward scanning at $\le 0.28\,\text{m}$ immediately halts the inbound vehicle into `LOCAL_SAFETY_HOLD`, ensuring zero physical contact while fault detection completes asynchronously.

### Race 4: Network Partition Split-Brain Re-Auction
- **Hazard**: A partition splits the fleet into subsets $\mathcal{V}_A$ and $\mathcal{V}_B$. If a robot in $\mathcal{V}_A$ fails, peers in $\mathcal{V}_A$ and $\mathcal{V}_B$ could independently allocate the task with divergent winners.
- **Mitigation**: CBBA task claims include monotonically increasing assignment timestamps $t_{assign}$. Upon partition heal, the reconciliation protocol yields tasks whose local timestamp is strictly older than the peer belief ($t_{local} < t_{peer}$), converging deterministically to a single owner.

---

## 5. Empirical Validation & Scenario Results

The canonical scenario validation harness (`scripts/validate_m4_compound_scenarios.py`) executed 7 compound failure scenarios under deterministic seed 42:

| Scenario ID | Title | Compound Stressor Composition | Resolving Component | Overlap Count | Result |
| :--- | :--- | :--- | :--- | :---: | :---: |
| **M4-A** | Robot Failure + Dynamic Blockage | AMR crash + alternate corridor blockage | FaultDetector + SingleAgentAStar | 0 | **PASS** |
| **M4-B** | Comm Loss vs. Failure Discrimination | Staggered packet timeout ($1.5\,\text{s}$ vs $3.5\,\text{s}$) | FaultDetector Debounce & Tiered Discrimination | 0 | **PASS** |
| **M4-C** | Overlapping Robot Failures | 2 AMRs crash in staggered windows ($t=1.0\,\text{s}, 2.5\,\text{s}$) | CBBA Consensus + Invalidation Manager | 0 | **PASS** |
| **M4-D** | Robot Failure + Sensor-Visible Obstacle | AMR crash + unmapped LiDAR hazard | LocalObstacleDetector ($0.28\,\text{m}$ threshold) | 0 | **PASS** |
| **M4-E** | Network Partition + Robot Failure | Disjoint partition + post-heal reconciliation | Timestamp CAS Reconnection Reconciliation | 0 | **PASS** |
| **M4-F** | Network Loss + Dynamic Blockage | COMM_LOSS AMR + blocked corridor | Bounded Local Autonomy & Safety Hold | 0 | **PASS** |
| **M4-G** | Master Compound Quad Failure | 2 crashes + $50\%$ packet loss + blocked corridor | Composite Full NRDAS Layered Core | 0 | **PASS** |

### Key Performance & Latency Metrics
- **AStar Detour Replanning Latency**: $0.054\,\text{ms}$ to $0.098\,\text{ms}$ across $16 \times 16$ grid.
- **LiDAR Scan Evaluation Latency**: $0.082\,\text{ms}$ for $360$-ray scan.
- **Reconnection CAS Reconciliation Latency**: $0.007\,\text{ms}$.
- **False Positive Robot Failures**: $0$ under transient comm dropouts.
- **Geometric Overlap Proxy Count**: $0$ across all compound runs.

---

## 6. Provenance & Scientific Methodology Notes

To maintain strict scientific integrity, all findings are bound by the following provenance disclosures:
1. **Geometric Overlap Proxy**: Contact detection and clearance metrics are computed via 2D Oriented Bounding Box (OBB) spatial proxies and minimum euclidean distance checks on telemetry ($d \ge 0.28\,\text{m}$), not raw physics-engine bumper impulse dynamics.
2. **Actual Runtime Measurement**: Latency benchmarks reflect measured execution times of the Python 3.12 algorithmic components under ROS 2 Jazzy.
3. **Planner-Level PIBT Integration**: Multi-robot conflict resolution occurs at the discrete space-time reservation and planning layer rather than through continuous low-level torque controllers.
4. **Scope of Validation**: Claims of completeness denote that *all defined M4 software and integration validation scenarios passed*, not an unconstrained proof of all possible physical failure modes.

---

## 7. Web-Based Adversarial Experiment Controller (Port 8081)

To evaluate multi-domain compound failures dynamically without manually hacking launch parameters, Milestone 4 introduces an interactive web-based **Adversarial Experiment Controller** running on `http://localhost:8081` (served by `scripts/resilience_dashboard.py`).

### 7.1 Architecture & Control Flow

The adversarial experiment controller decouples experiment orchestration from the underlying decentralized robotics nodes:

```
+-------------------------------------------------------------------------------+
|                OPERATOR / ADVERSARIAL EXPERIMENT CONTROLLER                   |
|                        (Web Browser @ Port 8081)                              |
+-------------------------------------------------------------------------------+
       |                                                                ^
       | HTTP POST /api/m4/...                                          | SSE / JSON
       v                                                                | Telemetry
+-------------------------------------------------------------------------------+
|                       NRDAS-FR FAULT INJECTION ENGINE                         |
|  [Disturbance Matrix]        [Three-Tier Response Engine]   [Invariant Engine]|
|  - Robot: Crash, CommLoss    - Tier 1: CBBA CAS Reclaim     - Inv 1: Task Mutex
|  - Net: Drop, Partition      - Tier 2: Dynamic A* Replan    - Inv 2: SpaceTime
|  - Env: Aisle Blockage       - Tier 3: 20Hz LiDAR Brake     - Inv 3: Clearance
+-------------------------------------------------------------------------------+
       |                                                                |
       v ROS 2 Topics                                                   v
+-------------------------------------------------------------------------------+
|                 DECENTRALIZED AUTONOMOUS MOBILE ROBOT FLEET                   |
|   amr_0         amr_1         amr_2         amr_3         amr_4 ... amr_N     |
+-------------------------------------------------------------------------------+
```

### 7.2 Multi-Domain Disturbance Matrix

Operators can compose simultaneous and staggered failure scenarios across three orthogonal failure domains:
1. **Robot Domain**:
   - `AMR Hardware Crash` ($s_{health} \leftarrow \text{FAILED}$): Complete node death, motors locked in place.
   - `Heartbeat Stagnation / Comm Loss` ($s_{health} \leftarrow \text{COMM\_LOSS}$): Packet loss exceeds $T_{comm} = 1.5\,\text{s}$, peer debouncing engaged.
   - `E-Stop Trigger`: Immediate velocity inhibition.
2. **Network Domain**:
   - `Packet Loss Percentage`: Configurable drop probability $p_{loss} \in [0.0, 1.0]$.
   - `Network Partition`: Bipartitioning fleet $\mathcal{V}$ into disjoint cliques $\mathcal{V}_A$ and $\mathcal{V}_B$.
   - `Message Jitter & Delay`: Latency injection up to $2000\,\text{ms}$.
3. **Environment Domain**:
   - `Corridor / Aisle Blockage`: Dynamic obstruction placed at critical transit choke points (e.g., Aisle 1 South $(4.0, 6.0)$).
   - `Unmapped Dynamic Obstacle`: Sensor-visible obstacles entering robot forward safety envelopes.

### 7.3 Live Three-Tier Fleet Response Telemetry

The dashboard provides real-time telemetry into the three independent recovery tiers:
- **Tier 1: Task Reallocation (CBBA Consensus)**:
  - Tracks orphaned task identification, atomic CAS reclamation to `PENDING`, and re-bidding rounds.
  - Latency: Converges in $\le 150\,\text{ms}$ across surviving agents.
- **Tier 2: Dynamic Replanning (Spatiotemporal Graph)**:
  - Space-time reservation invalidation for crashed robots and blocked corridors.
  - SingleAgentAStar / RHCR dynamic detour generation around obstructions.
  - Latency: Mean replanning time $0.054\,\text{ms}$ to $0.098\,\text{ms}$.
- **Tier 3: Local Safety & Clearance (Reactive Sensor Backstop)**:
  - Onboard LiDAR scanning at $20\,\text{Hz}$ independently evaluating $24^\circ$ forward arc.
  - Emergency brake assertion if distance $\le 0.28\,\text{m}$ ($v=0, \omega=0$).
  - Overlap count: Enforced strictly at $0$.

### 7.4 Seven-Stage Deterministic Recovery Stepper

To dissect complex cascading recoveries deterministically, the dashboard provides a 7-stage interactive stepper:
1. `STAGE_INJECT`: Apply compound disturbances simultaneously.
2. `STAGE_DETECT`: Observe peer heartbeat timeouts and local sensor triggers.
3. `STAGE_ISOLATE`: Mark failing nodes as `COMM_LOSS` or `FAILED`; withdraw affected corridor reservations.
4. `STAGE_RECLAIM`: Execute atomic CAS reclamation on orphaned tasks back to `PENDING`.
5. `STAGE_REPLAN`: Trigger single-agent A* or RHCR detour searches for surviving AMRs.
6. `STAGE_RECONCILE`: Heal network partitions; reconcile timestamps ($t_{local} < t_{peer}$).
7. `STAGE_NOMINAL`: Restore nominal compute horizons and normal speed limits.

### 7.5 REST API Specification

| Endpoint | Method | Payload / Parameters | Description |
| :--- | :---: | :--- | :--- |
| `/api/m4/status` | `GET` | None | Returns active disturbances, three-tier response status, and mathematical invariant state. |
| `/api/m4/inject_compound` | `POST` | `{"scenario": "M4-A" ... "M4-G"}` | Atomically submits and executes a predefined compound benchmark scenario. |
| `/api/m4/compose_and_run` | `POST` | `{"robots": [...], "network": {...}, "environment": {...}}` | Executes a custom multi-domain compound fault composition. |
| `/api/m4/step` | `POST` | `{"stage": 1..7}` | Advances the deterministic recovery stepper by one stage. |
| `/api/m4/clear` | `POST` | None | Clears all injected faults, heals partitions, and unblocks corridors. |
| `/api/m4/report` | `GET` | None | Generates and downloads a complete JSON benchmark report of the current adversarial run. |

---

## 8. Verification Evidence & Automated Test Coverage

The resilience framework has undergone rigorous verification with zero synthetic or mock overrides:

### 8.1 Resilience Test Suite (79 Tests Passing)
Executed via pytest across 5 dedicated resilience test suites:
- `test_m4_compound_faults.py`: 12 passed (multi-fault simultaneous triggers, recovery stages).
- `test_m4_edge_cases.py`: 12 passed (extreme timing, race conditions, simultaneous crashes).
- `test_m4_e2e_resilience.py`: 12 passed (end-to-end multi-agent fleet survival).
- `test_resilience_integration.py`: 20 passed (ROS 2 node integration, state publishers).
- `test_resilience_dashboard.py`: 23 passed (REST API endpoints, state transitions, report export).
- **Result: 79 passed, 0 failures, 0 errors in 1.48s**.

### 8.2 Canonical Benchmark Harness (7/7 Scenarios Passing)
Validated using `python3 scripts/validate_m4_compound_scenarios.py`:
- All 7 benchmark scenarios (M4-A through M4-G) executed cleanly.
- 0 geometric overlaps ($d \ge 0.28\,\text{m}$ throughout).
- 0 false positive node failures.
- 0 task leaks or duplicate allocations ($I_1$ strictly held).
- 0 spacetime reservation collisions ($I_2$ strictly held).

