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
