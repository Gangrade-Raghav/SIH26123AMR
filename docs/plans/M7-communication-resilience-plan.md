# Milestone M7: Communication Degradation & Fleet Resilience Plan

## 1. Executive Summary

Milestone M7 introduces controlled, reproducible, and non-intrusive communication impairment into the decentralized autonomous mobile robot (AMR) coordination system. The core architectural invariant is that algorithmic layers (CBBA task allocation, Rolling-Horizon A* path planning, PIBT space-time reservations, Wait-For Graph deadlock detection, and LiDAR reactive safety braking) remain unchanged in their core logic, while their inter-robot communication boundary (`/fleet/*` topic bus) is subjected to systematic degradation.

The primary objective is to measure, understand, and document how the decentralized fleet behaves under transport latency, delay jitter, random packet loss, correlated burst drops, temporary communication blackouts, reconnection cycles, and sub-fleet network partitioning.

---

## 2. Architecture & Ingress/Egress Isolation

```
=============================================================================
                          DECENTRALIZED AMR NODE ARCHITECTURE
=============================================================================

                   +-----------------------------+
                   |     M3 Task Dispatcher      |
                   +--------------+--------------+
                                  |
                                  v
                   +-----------------------------+
                   |   M4 CBBA Task Consensus    |
                   +--------------+--------------+
                                  |
                                  v
                   +-----------------------------+
                   |  M5 Rolling-Horizon Planner |
                   +--------------+--------------+
                                  |
                                  v
                   +-----------------------------+
                   |   M6 Space-Time / PIBT      |
                   |   Coordination & Deadlock   |
                   +--------------+--------------+
                                  |
                   +--------------v--------------+
                   |    M7 Degradation Layer     |<--- /fleet/comm_profile
                   | (Gilbert-Elliott / Latency) |
                   +--------------+--------------+
                                  |
          +-----------------------+-----------------------+
          |                                               |
          v                                               v
+-------------------+                           +-------------------+
|  /fleet/* Topics  |                           | Local Safety Layer|
| (Coordination Bus)|                           | (LiDAR Scan & E-Stop)
| - cbba_bids       |                           +---------+---------+
| - reservations    |                                     |
| - conflicts       |                                     v
| - deadlocks       |                           +-------------------+
+-------------------+                           |  /cmd_vel Wheels  |
                                                +-------------------+
=============================================================================
```

### Invariant Rules
1. **Coordination Topic Scope**: Impairment is strictly scoped to `/fleet/cbba_bids`, `/fleet/reservations`, `/fleet/coordination_status`, `/fleet/conflicts`, and `/fleet/deadlocks`.
2. **Local Safety Exemption**: Local LiDAR sensor streams (`/{r_id}/scan`), local odometry (`/{r_id}/odom`), joint states, transform trees (`/tf`), and direct motor actuator commands (`/{r_id}/cmd_vel`) are strictly exempt from communication impairment.
3. **No Centralized Controller**: Impairment is modeled at each peer's incoming transport boundary using a deterministic, seedable pseudorandom generator.

---

## 3. Communication Impairment Models

### 3.1 Transport Latency & Jitter
- **Deterministic Latency**: Configurable constant transport delay $\tau_{\text{base}} \in [0, 1000]\text{ ms}$.
- **Stochastic Jitter**: Additive zero-mean uniform delay variation $\delta \in [-\Delta_j, +\Delta_j]$, yielding effective delay:
  $$\tau_{\text{eff}} = \max(0.0, \tau_{\text{base}} + \delta)$$
- **Non-blocking Ingress Queue**: Incoming delayed packets are scheduled into a `DelayedMessageQueue` priority queue and delivered without thread starvation or blocking the ROS 2 executor.

### 3.2 Packet Loss Models
- **Independent Random Loss**: Uniform i.i.d. Bernoulli trial with drop probability $p_{\text{loss}} \in [0.0, 1.0]$.
- **Correlated Burst Loss (Gilbert-Elliott 2-State Markov Model)**:
  - **Good State ($G$)**: Normal transmission, transition to Bad with probability $p_{\text{burst\_start}}$.
  - **Bad State ($B$)**: Continuous packet drop streak, recovery to Good with probability $p_{\text{recover}} = \frac{1}{\mu_{\text{burst\_length}}}$.

### 3.3 Outage & Partition Models
- **Scheduled Outage**: Total blackout window where all messages are dropped for $t \in [t_{\text{start}}, t_{\text{start}} + \tau_{\text{outage}}]$.
- **Network Partition**: Matrix-based isolation dividing the fleet into disjoint subsets (e.g. Sub-fleet A: `[amr_0, amr_1, amr_2]` vs Sub-fleet B: `[amr_3, amr_4]`). Messages crossing partition boundaries are dropped immediately.

---

## 4. Stale State & Resilience Protocol

Each AMR maintains a `StaleStateManager` tracking peer heartbeat age $\Delta t = t_{\text{curr}} - t_{\text{rx}}$:
1. $\Delta t < \tau_{\text{stale}}$ (default 1.5s): **CURRENT** — normal operation.
2. $\tau_{\text{stale}} \le \Delta t < \tau_{\text{expiry}}$ (default 4.0s): **STALE** — warning flagged, conservative PIBT yielding.
3. $\Delta t \ge \tau_{\text{expiry}}$: **EXPIRED** — remote reservations held by the silent peer are automatically pruned from the local reservation table to prevent dead phantom reservations from permanently blocking corridors.
4. **Reconnection Detection**: When a packet arrives from an `EXPIRED` peer, a reconnection event is declared, triggering immediate state re-broadcast (current pose, active reservations, winning CBBA bundle).

---

## 5. Experimental Test Suite & Profiles

| Profile ID | Profile Name | Latency ($\tau$) | Jitter ($\Delta_j$) | Loss ($p$) | Burst ($p_b$) | Outage ($\tau_o$) | Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **A** | `NORMAL` | 0 ms | 0 ms | 0.0% | 0.0% | 0.0s | Baseline unconstrained |
| **B1** | `LOW_LATENCY` | 100 ms | 10 ms | 0.0% | 0.0% | 0.0s | Minor propagation lag |
| **B2** | `HIGH_LATENCY` | 400 ms | 20 ms | 0.0% | 0.0% | 0.0s | Severe transport lag |
| **C** | `JITTER` | 150 ms | 50 ms | 0.0% | 0.0% | 0.0s | Non-deterministic ordering |
| **D1** | `LOSS_LOW` | 0 ms | 0 ms | 15.0% | 0.0% | 0.0s | Light packet loss |
| **D2** | `LOSS_HIGH` | 0 ms | 0 ms | 35.0% | 0.0% | 0.0s | Heavy packet loss |
| **E** | `BURST_LOSS` | 0 ms | 0 ms | 0.0% | 25.0% | 0.0s | Gilbert-Elliott streaks |
| **F** | `OUTAGE` | 0 ms | 0 ms | 0.0% | 0.0% | 5.0s | 5s total blackout |
| **G** | `OUTAGE_RECOVERY` | 50 ms | 15 ms | 5.0% | 0.0% | 3.0s | Reconnection & sync |
| **H** | `PARTITION` | 20 ms | 5 ms | 0.0% | 0.0% | 0.0s | 3x2 sub-fleet partition |

---

## 6. Verification Criteria (Definition of Done)
- [x] Zero physical collisions ($d_{\text{contact}} < 0.35\text{ m}$) across all 10 evaluation profiles.
- [x] Local LiDAR emergency braking invariant completely decoupled from communication state.
- [x] Pruning of expired space-time reservations for silent peers.
- [x] Reconnection detection and automatic state re-broadcast.
- [x] 100% test pass rate on all unit, integration, and linter suites.
- [x] Complete quantitative empirical dataset saved to `gazebo_m7_results.json` and plotted to `docs/images/m7_degradation_curves.png`.
