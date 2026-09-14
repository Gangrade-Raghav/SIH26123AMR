# Milestone M8B Plan: Adaptive Compute & Degradation-Aware Coordination

**Project**: NRDAS Decentralized AMR Fleet Coordination System  
**Milestone**: M8B — Adaptive Compute & Degradation-Aware Coordination  
**Status**: APPROVED & IMPLEMENTED  

---

## 1. Research Objective & Question

### 1.1 Objective
Investigate whether decentralized AMRs can dynamically adapt their coordination and rolling-horizon planning computational effort according to changing network conditions, computational load, and spatial traffic contention, while strictly preserving the existing safety hierarchy.

### 1.2 Research Question
> *"Can degradation-aware compute allocation improve fleet coordination performance under changing communication and computational conditions without compromising safety?"*

### 1.3 Strict Scope Boundaries & Non-Goals
1. **Zero Replacement of Core Algorithms**: CBBA (consensus bidding), RHCR (rolling-horizon $A^*$), Space-Time Reservations, PIBT (priority inheritance), and WFG Deadlock Recovery remain architecturally unchanged.
2. **Orthogonal Policy Modulation**: The adaptive compute policy modulates operational parameters (replan frequency, horizon depth, execution window, consensus rate, planning budget) without altering algorithm semantics.
3. **Strict Safety Invariance**: The reactive LiDAR safety controller operates independently at 10 Hz and holds absolute preemptive authority. Lowering compute mode NEVER compromises local obstacle avoidance or emergency braking.
4. **Fail-Safe Fallback**: Any telemetry staleness ($>5.0\,\text{s}$), exception, or configuration error causes deterministic, immediate fallback to `COMPUTE_MODE_NORMAL`.
5. **No Synthetic Data**: All evaluation is grounded in real Gazebo Harmonic simulations across deterministic workloads and network profiles.

---

## 2. Discrete Compute Modes Specification

| Parameter | COMPUTE_MODE_LOW | COMPUTE_MODE_NORMAL (Baseline) | COMPUTE_MODE_HIGH |
| :--- | :---: | :---: | :---: |
| **Replanning Frequency ($f_{\text{replan}}$)** | $1.0\,\text{Hz}$ ($T=1.0\,\text{s}$) | $2.0\,\text{Hz}$ ($T=0.5\,\text{s}$) | $4.0\,\text{Hz}$ ($T=0.25\,\text{s}$) |
| **Planning Horizon Steps ($h$)** | $6$ steps ($3.0\,\text{m}$) | $10$ steps ($5.0\,\text{m}$) | $14$ steps ($7.0\,\text{m}$) |
| **Execution Window Steps ($w$)** | $6$ steps ($3.0\,\text{m}$) | $4$ steps ($2.0\,\text{m}$) | $2$ steps ($1.0\,\text{m}$) |
| **CBBA Consensus Rate ($f_{\text{cbba}}$)** | $2.0\,\text{Hz}$ ($T=0.5\,\text{s}$) | $5.0\,\text{Hz}$ ($T=0.2\,\text{s}$) | $10.0\,\text{Hz}$ ($T=0.1\,\text{s}$) |
| **Planning Time Budget ($\tau_{\text{budget}}$)** | $20\,\text{ms}$ | $50\,\text{ms}$ | $100\,\text{ms}$ |
| **Intended Operational Regime** | High packet loss, stale peer state, CPU saturation | Nominal conditions, stable communication | High local conflict density, deadlock recovery |

---

## 3. Deterministic Priority-Ordered State Machine

At the end of every planning cycle, each robot evaluates its local operational state against four deterministic, priority-ordered rules:

```
+-------------------------------------------------------------------+
|               PRIORITY 1: Network / Stale-State Degradation       |
|    Peer age >= 1.5s OR Loss >= 15% OR Partition/Outage Detected   |
|                  --> Transition to COMPUTE_MODE_LOW               |
+-------------------------------------------------------------------+
                                  | False
+-------------------------------------------------------------------+
|               PRIORITY 2: Host Compute / CPU Saturation           |
|            Host CPU >= 85% OR Planning Latency > Budget           |
|                  --> Transition to COMPUTE_MODE_LOW               |
+-------------------------------------------------------------------+
                                  | False
+-------------------------------------------------------------------+
|               PRIORITY 3: Spatial Traffic Contention              |
|        Active Conflicts >= 2 OR Robot in Deadlock Recovery        |
|                  --> Transition to COMPUTE_MODE_HIGH              |
+-------------------------------------------------------------------+
                                  | False
+-------------------------------------------------------------------+
|               PRIORITY 4: Nominal Recovery                        |
|  Peer age <= 0.8s AND Loss <= 5% AND Conflicts == 0 AND CPU < 70% |
|                  --> Transition to COMPUTE_MODE_NORMAL            |
+-------------------------------------------------------------------+
```

---

## 4. Anti-Oscillation & Stability Mechanisms

To prevent rapid chattering or limit-cycle oscillations between compute modes, the policy enforces three layers of defense:

1. **Minimum Dwell Time ($T_{\text{dwell}} = 3.0\,\text{s}$)**:
   Once a transition occurs, the robot must dwell in the new mode for at least $3.0\,\text{s}$ before any subsequent mode shift is permitted.
2. **Consecutive Sample Confirmation ($K = 3$ cycles)**:
   A candidate trigger condition must persist across $K=3$ consecutive evaluation cycles before the mode switch is confirmed. Transient single-cycle spikes are rejected.
3. **Asymmetric Hysteresis**:
   Thresholds to enter a degraded or high-effort mode are separated from the recovery thresholds to return to nominal:
   - Enter LOW on stale state: age $\ge 1.5\,\text{s}$; Recover to NORMAL: age $\le 0.8\,\text{s}$.
   - Enter LOW on packet loss: loss $\ge 15\%$; Recover to NORMAL: loss $\le 5\%$.
   - Enter LOW on CPU load: CPU $\ge 85\%$; Recover to NORMAL: CPU $\le 70\%$.

---

## 5. Architectural Safety Hierarchy

Safety authority is strictly hierarchical:
$$\text{LOCAL SAFETY} \succ \text{RESERVATION / COLLISION AVOIDANCE} \succ \text{PLANNING} \succ \text{TASK ALLOCATION}$$

- **Reactive LiDAR Braking (10 Hz)**: Completely decoupled from the planning timer. Operates on raw LaserScan data at $10\,\text{Hz}$ via `_control_loop`. If an obstacle or peer enters the emergency braking envelope ($d \le 0.40\,\text{m}$), the safety controller zeroes velocity commands immediately, regardless of active compute mode.
- **Independence Guarantee**: Downscaling to `COMPUTE_MODE_LOW` ($1.0\,\text{Hz}$) alters path replan frequency, but reactive braking latency remains bounded by the $100\,\text{ms}$ LiDAR cycle ($10\,\text{Hz}$).

---

## 6. Verification & Validation Protocol

- **Unit Tests**: Parameter validation, state machine transitions, priority order, dwell time, sample confirmation, asymmetric hysteresis, fail-safe timeouts (`test_m8b_adaptive_compute.py`).
- **Gazebo Empirical Trials**: 12 targeted experimental trials ($n=1$ per evaluated condition, seed 42) across Workload 15 (60s horizon) and Workload 30 (120s horizon). Note: this is a targeted campaign, not a complete factorial grid (Workload 30 under LOSS_HIGH was not executed; Workload 15 under HIGH_LATENCY and LOSS_HIGH evaluated baseline NORMAL vs ADAPTIVE).
- **Reporting & Accounting Invariants**: Strict enforcement of $N_{\text{completed}} + N_{\text{remaining}} \equiv N_{\text{generated}}$, physical dropoff distance audit ($\le 0.50\,\text{m}$), and minimum spatial separation ($\ge 0.35\,\text{m}$).
