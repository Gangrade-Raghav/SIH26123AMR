# NRDAS-FR Milestone 2 Network Resilience Validation Evidence

**Execution Timestamp**: 2026-09-23T22:27:54.089967
**Overall Outcome**: **100% PASSED**
**Scenarios Evaluated**: 8 / 8

---

## Executive Summary Matrix
| Scenario ID | Execution Level | Scenario Description | Status | Key Invariant |
| :--- | :---: | :--- | :---: | :--- |
| **M2-A** | `INTEGRATION / SIMULATION` | Short Network Outage ($\le 2$s) during Active Navigation | `PASSED` | False failure rejected, local autonomy active |
| **M2-B** | `INTEGRATION / SIMULATION` | Network Outage in ASSIGNED State (Pre-Departure Hold) | `PASSED` | Pre-departure safe hold, zero uncoordinated motion |
| **M2-B2** | `INTEGRATION / SIMULATION` | COMM_LOSS during Active Reserved Navigation | `PASSED` | AMR moving $\to$ COMM_LOSS $\to$ valid path $\to$ safe $\to$ reconciled |
| **M2-C** | `INTEGRATION / SIMULATION` | Network Outage in IN_PROGRESS State (Local Autonomy & Hold) | `PASSED` | Navigates reserved corridor, enters safe hold on reservation expiry |
| **M2-D** | `INTEGRATION / SIMULATION` | Extended Outage ($>3.5$s) with Autonomous M1 Reclamation | `PASSED` | Silence debounced to FAILED at 3.5s, autonomous CAS re-auction |
| **M2-E** | `INTEGRATION / SIMULATION` | Post-Reconnection State Reconciliation & Invariant $I_{{\text{{uniq}}}}$ | `PASSED` | Strict newer-timestamp yield; no duplicate ownership observed |
| **M2-F** | `INTEGRATION / SIMULATION` | High Packet Loss ($p=0.35$) Resilience with Debounced Filtering | `PASSED` | Measured loss 34.4%, 0 false failures |
| **M2-G** | `INTEGRATION / SIMULATION` | Bipartite Network Partition & Re-merge Convergence | `PASSED` | Partition: consensus unavailable; post-merge convergence observed |

---

## Technical Invariant Evaluation Details

### Invariant 1: Communication Failure $\neq$ Robot Failure
Under scenarios M2-A, M2-B, and M2-B2, network outages lasting up to 2.0s ($\le$ the 3.5s failure timeout) caused the AMR and its peers to enter `COMM_LOSS`. Peers refrained from triggering M1 task reclamation or obstacle insertion. Upon restoration, normal `HEALTHY` operation resumed with zero disruption.

### Invariant 2: Bounded Local Autonomy & LOCAL_SAFETY_HOLD
Under scenarios M2-B2 and M2-C, local autonomy is explicitly bounded:
- Communication loss does not imply unrestricted autonomous navigation.
- Valid space-time corridor reservations permit continued local execution.
- Expired reservations force `LOCAL_SAFETY_HOLD` ($v = 0.0\text{ m/s}$).
- Unreserved blind exploratory navigation is strictly prohibited.

### Invariant 3: Single Task Ownership Invariant ($I_{{\text{{uniq}}}}$)
Under scenarios M2-D, M2-E, and M2-B2, task reallocation and reconciliation were evaluated.
> **Verified Finding**: No duplicate task ownership was observed, and the implemented reconciliation invariant prevents duplicate ownership across the evaluated stale-state/reconnection cases.

### Invariant 4: Stochastic Loss Debouncing ($p=0.35$)
Under scenario M2-F, 1000 messages were evaluated under the `LOSS_HIGH` profile with independent packet loss probability $p=0.35$. The measured packet loss was 34.40%. Despite substantial loss, confirmation debouncing (`confirmation_samples=2`) ensured zero false failure triggers.

### Invariant 5: Partition Consensus Semantics (Scenario M2-G)
Under scenario M2-G, network split behavior was evaluated:
- **During partition**: `GLOBAL CONSENSUS = NOT AVAILABLE`. Intra-partition communication continued locally, but cross-partition consensus was strictly unavailable.
- **After partition recovery**: Full state reconciliation and convergence were observed upon link restoration, with zero duplicate task ownership.

---

## Ground Truth, Safety Thresholds & Execution Classification
- **Execution Level**: All M2 scenarios (M2-A through M2-G) were evaluated at the **`INTEGRATION / SIMULATION`** level using the multi-robot software simulation testbed.
- **Local Safety Threshold ($0.28\text{{m}}$)**: $0.28\text{{m}}$ is utilized as an **experimental/design safety threshold**; it is not characterized as optimal or universal.
- **Contact Detection Proxy**: 2D OBB Geometric Proxy (Separating Axis Theorem on live odometry poses) with $0.45\text{{m}}$ buffer.
- **Safety Hierarchy**: Local LiDAR Safety ($0.28\text{{m}}$) $>$ Space-Time Corridor Reservations $>$ Fault Recovery $>$ Local Planning $>$ CBBA Allocation.
- **Pure Decentralization**: All fault detection, stale state classification, and reconciliation executed within peer agent nodes. No centralized orchestrator or SPOF.
