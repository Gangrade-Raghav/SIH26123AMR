# NRDAS-FR // Milestone 2: Network Failure, Local Autonomy & Reconnection

**Project**: NRDAS-FR (NRDAS v2) Fault-Resilient Decentralized AMR Fleet Coordination  
**Milestone**: Milestone 2 — Network Failure, Local Autonomy & Reconnection  
**Date**: September 2026  
**Status**: COMPLETE (100% Tests, Invariants & Scenarios Verified)

---

## 1. Research Question & Core Objective

In decentralized Autonomous Mobile Robot (AMR) fleets operating in dynamic industrial environments, wireless communication channels frequently degrade due to multipath fading, structural occlusion (racks, machinery), antenna shadowing, and radio frequency interference.

Milestone 2 addresses the fundamental research question:
> **"Can an individual AMR remain locally safe and operational during temporary loss of fleet communication, and can it consistently rejoin the decentralized fleet after communication is restored without human intervention, duplicate task execution, or safety corridor collisions?"**

### Primary Technical Axiom: Communication Failure $\neq$ Physical Robot Failure
A critical flaw in naive fault-tolerance models is conflating **heartbeat silence** with **robot death**. If an AMR is declared failed whenever packets drop:
1. Active tasks are prematurely revoked and reassigned, leading to catastrophic **duplicate task execution** ($I_{\text{uniq}}$ violation).
2. Space-time corridor reservations are prematurely freed, allowing peer robots into the moving robot's occupied zone, causing collisions.
3. The disconnected robot is falsely treated as a static obstacle, introducing ghost obstacles into the navigation map.

NRDAS-FR Milestone 2 strictly decouples communication degradation from physical failure through multi-tier debouncing, local autonomy permissions, and post-reconnection consensus reconciliation.

---

## 2. Safety Hierarchy & Local Autonomy Rules

During communication outages, an AMR cannot query remote coordinators or negotiate new space-time reservations. Safe operation is enforced by a strict, immutable safety hierarchy:

$$\text{Local LiDAR Safety (0.28m experimental/design threshold)} > \text{Space-Time Corridor Reservations} > \text{Fault Recovery} > \text{Local Planning} > \text{CBBA Task Allocation}$$

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
| 2. SPACE-TIME CORRIDOR RESERVATIONS                         |
|    - Valid while t <= t_expiry                              |
|    - If reservation expires: LOCAL_SAFETY_HOLD (v = 0.0 m/s)|
|    - Strict prohibition of unreserved blind exploration     |
+-------------------------------------------------------------+
                              v
+-------------------------------------------------------------+
| 3. FAULT DETECTOR & RECOVERY CONTROLLER                     |
|    - Classifies COMM_LOSS vs FAILED                         |
|    - Enforces local autonomy policy and recovery debouncing |
+-------------------------------------------------------------+
                              v
+-------------------------------------------------------------+
| 4. ROLLING-HORIZON / A* LOCAL PLANNER                       |
|    - Plans within pre-reserved space-time corridors         |
+-------------------------------------------------------------+
                              v
+-------------------------------------------------------------+
| 5. DECENTRALIZED CBBA TASK ALLOCATION                       |
|    - Bidding suspended during COMM_LOSS                     |
|    - Reconnection reconciliation executed on restoration    |
+-------------------------------------------------------------+
```

### Local Autonomy Execution Policy (Bounded Local Autonomy)
Under NRDAS-FR, communication loss does **not** imply unrestricted autonomous navigation. Local autonomy is strictly bounded by prior space-time reservations and local safety constraints:
- **Communication loss does not imply unrestricted autonomous navigation**: An AMR disconnected from the fleet network cannot plan arbitrary paths or assume uncoordinated spatial regions are clear.
- **Valid reservations may permit continued local execution**: An AMR that entered `COMM_LOSS` while in `IN_PROGRESS` state is permitted to continue along its pre-negotiated space-time corridor at nominal speed ($v \le 0.4\text{ m/s}$), provided its reservations remain valid ($t \le t_{\text{res\_expiry}}$) and local LiDAR detects no obstacles within the $0.28\text{ m}$ experimental/design safety threshold.
- **Expired reservations force `LOCAL_SAFETY_HOLD`**: Because new space-time cells cannot be safely reserved without network consensus, when an AMR exhausts its reserved horizon, it must immediately transition to `LOCAL_SAFETY_HOLD` ($v = 0.0\text{ m/s}$). It retains its stopped position and holds its terminal reservation cell.
- **Unreserved blind exploratory navigation is prohibited**: Under no circumstances may a disconnected robot explore unreserved cells without active coordination.
- **In `ASSIGNED` State (Pre-Departure)**: If an outage occurs before corridor departure, the AMR holds position at the start cell. It does not embark on uncoordinated paths until network connectivity is confirmed and reservations are locked.

---

## 3. Consensus Reconciliation on Reconnection

When network connectivity is restored after an extended outage, the rejoining AMR and the fleet must reconcile state asynchronously without centralized master control.

### Mathematical Reconnection Protocol
Let $R_i$ be the reconnecting robot with bundle $\mathcal{B}_i$ and local timestamp vector $s_i$. Let $R_j$ be a peer broadcasting its winning robot vector $z_j$, timestamp vector $s_j$, and winning bid vector $y_j$.

For each task $T_k \in \mathcal{B}_i$:
$$\text{If } z_j(T_k) \neq R_i \land s_j(T_k) > s_i(T_k) \implies \begin{cases} \mathcal{B}_i \leftarrow \mathcal{B}_i \setminus \{T_k\} \\ \mathcal{P}_i \leftarrow \mathcal{P}_i \setminus \{T_k\} \\ z_i(T_k) \leftarrow z_j(T_k) \\ y_i(T_k) \leftarrow y_j(T_k) \\ s_i(T_k) \leftarrow s_j(T_k) \end{cases}$$

- **Strict Newer-Timestamp Yield**: If the peer fleet declared $R_i$ failed during an extended outage ($>3.5\text{s}$) and autonomously re-auctioned $T_k$, the peer timestamp satisfies $s_j(T_k) > s_i(T_k)$. $R_i$ yields the task instantly, clearing it from its bundle and waypoints.
- **Stale Protection**: If a peer emits an older stale bid ($s_j(T_k) \le s_i(T_k)$), $R_i$ retains the task.
- **Single Task Ownership Evaluation ($I_{\text{uniq}}$)**:
  No duplicate task ownership was observed, and the implemented reconciliation invariant prevents duplicate ownership across the evaluated stale-state/reconnection cases. (Universal mathematical theorem is not claimed).
  $$\forall T_k \in \mathcal{T}, \quad \sum_{R_r \in \mathcal{R}} \mathbb{I}[T_k \in \mathcal{B}_r] \le 1$$

---

## 4. Scenario Validation & Execution Classification (M2-A through M2-G, M2-B2)

Validation was executed using `scripts/validate_m2_network_scenarios.py`. All 8 scenarios passed 100% with all invariants verified.

Every scenario is classified by its concrete execution level:
- **`INTEGRATION / SIMULATION`**: Executed on the multi-agent software simulation testbed combining discrete kinematics, simulated clock, ROS nodes, and algorithmic agents.

| Scenario | Execution Level | Title | Outcome | Findings & Invariant Verification |
| :--- | :---: | :--- | :---: | :--- |
| **M2-A** | `INTEGRATION / SIMULATION` | Short Outage ($\le 2$s) in Active Navigation | **PASSED** | Duration 1.5s. Robot entered `COMM_LOSS`, continued reserved motion safely. Peers refrained from false failure declaration ($1.5\text{s} < 3.5\text{s}$). Reconnected seamlessly to `HEALTHY`. |
| **M2-B** | `INTEGRATION / SIMULATION` | Outage in `ASSIGNED` State | **PASSED** | AMR held position at origin ($v=0.0\text{ m/s}$). Zero uncoordinated exploratory movements. Zero spurious reclamation by peers. |
| **M2-B2** | `INTEGRATION / SIMULATION` | COMM_LOSS during Active Reserved Navigation | **PASSED** | AMR moving at $0.4\text{ m/s} \to$ `COMM_LOSS` injected $\to$ continues along valid reservation at $(4, 2) \to$ local safety buffer remains active ($1.2\text{m} > 0.28\text{m}$ experimental/design threshold) $\to$ network restored $\to$ reconciliation retains task without duplicates. |
| **M2-C** | `INTEGRATION / SIMULATION` | Outage in `IN_PROGRESS` State | **PASSED** | Navigated reserved cells 1 & 2 at $0.4\text{ m/s}$. Upon reaching step 3 where reservation ended, immediately engaged `LOCAL_SAFETY_HOLD` ($v=0.0\text{ m/s}$). |
| **M2-D** | `INTEGRATION / SIMULATION` | Extended Outage ($>3.5$s) & Autonomous Reclaim | **PASSED** | Outage 5.0s. At 3.8s, peers debounced silence to `FAILED`. CBBA beliefs purged, task reclaimed via CAS, reallocated to `amr_0` with timestamp $s=4.0\text{s}$. |
| **M2-E** | `INTEGRATION / SIMULATION` | Post-Reconnection Reconciliation ($I_{\text{uniq}}$) | **PASSED** | Reconnected `amr_1` received peer broadcast ($s=4.5\text{s} > 1.0\text{s}$). Yielded task from bundle and path. Dual ownership = 0; reconciliation invariant prevents duplicate ownership across evaluated cases. |
| **M2-F** | `INTEGRATION / SIMULATION` | High Packet Loss ($p=0.35$) Resilience | **PASSED** | 1000 messages evaluated. Configured loss $p=0.35$, measured loss rate **$34.4\%$** ($344/1000$). 2-sample debouncing prevented any false failure declarations (0 false failures). |
| **M2-G** | `INTEGRATION / SIMULATION` | Bipartite Partition & Re-merge | **PASSED** | Partition isolated `{amr_0, amr_1}` from `{amr_2}`. Cross-talk dropped. During partition: **GLOBAL CONSENSUS = NOT AVAILABLE** (intra-partition local operations only; cross-partition consensus unavailable). After partition recovery: state reconciliation and convergence were observed upon link restoration, with 0 conflicts. |

### Configured vs. Measured Packet Loss Telemetry
In Scenario M2-F, stochastic loss telemetry explicitly separates the configuration baseline from empirical measurements:
- **Execution Level**: `INTEGRATION / SIMULATION`
- **Configured Profile**: `LOSS_HIGH` ($p = 0.35$, zero latency/jitter)
- **Total Packets Transmitted**: 1000
- **Delivered Packets**: 656
- **Dropped Packets**: 344
- **Measured Packet Loss Rate**: $34.40\%$ ($\Delta = 0.60\%$ against nominal)
- **False Failures Observed**: **0**

---

## 5. Dedicated Resilience Testbed & UI Enhancements

The standalone Resilience Test Dashboard (`scripts/resilience_dashboard.py` on port 8081) was upgraded to support Milestone 2:

1. **Network Impairment Console (Column 1)**:
   - Configurable network impairment presets: `NORMAL`, `LOW_LATENCY`, `HIGH_LATENCY`, `JITTER`, `LOSS_LOW`, `LOSS_HIGH`, `BURST_LOSS`, `OUTAGE`, `OUTAGE_RECOVERY`, `PARTITION`.
   - Sliders for custom latency (0–1000 ms), jitter (0–200 ms), packet loss (0–100%), burst loss, and partition isolation.
   - REST Endpoints: `POST /api/network/impairment` and `POST /api/network/reconnect`.
2. **Network Recovery Pipeline Stepper (Column 2)**:
   - Dynamic mode switcher: **M1 Robot Failure (7 stages)** vs. **M2 Network Recovery (9 stages)**.
   - Stage progression: `NOMINAL` $\to$ `COMM_LOSS_DETECTED` $\to$ `LOCAL_AUTONOMY_ACTIVE` $\to$ `RESERVATION_EXPIRY_HOLD` $\to$ `PEER_FAILURE_DEBOUNCE` $\to$ `AUTONOMOUS_RECLAIM` $\to$ `NETWORK_RESTORED` $\to$ `RECONCILIATION_YIELD` $\to$ `FLEET_RECONVERGED`.
   - Real-time invariant monitoring: `false_failure_rejection`, `reconnection_zero_duplication`, `reservation_hold_on_expiry`.
3. **Live Network Telemetry & M2 Scenarios (Column 3)**:
   - Per-robot network status badge: `ONLINE` (green), `DEGRADED` (yellow), `COMM_LOSS` (orange), `OFFLINE` (red).
   - Live telemetry counters: Messages Sent, Messages Delivered, Messages Dropped, Empirical Loss Rate, P50/P95 Latency, Jitter, Burst Events.
    - Scenario launcher with one-click triggers for `M2-A` through `M2-G` and `M2-B2`.
4. **Architectural Non-Invasiveness & Zero SPOF**:
   - The dashboard remains a purely external visualization and fault-injection tool.
   - **Zero recovery logic** exists in the dashboard process. All fault detection, state transitions, local autonomy holds, and reconciliation run autonomously within individual decentralized AMR nodes.

---

## 6. Verification and Regression Summary

- **M2 Network Resilience Tests**: 12/12 passing (`test_m2_network_resilience.py`).
- **Resilience Dashboard Tests**: 12/12 passing (`test_resilience_dashboard.py`).
- **M2 Scenarios Validation**: 8/8 passing (`validate_m2_network_scenarios.py`).
- **Total Test Suite**: 256/256 tests passing across `amr_fleet_core`.
- **Static Analysis & Linters**: Zero flake8 errors, zero pep257 docstring errors.
- **NRDAS v1 Canonical Preservation**: All frozen baselines (A*, CBBA core, RHCR, PIBT, M7-M9 benchmarks) remain 100% intact and untouched.
