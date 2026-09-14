# Empirical Evaluation & Results: Milestone M8B Adaptive Compute

**Document**: `docs/research/adaptive-compute-results.md`  
**Milestone**: M8B  
**Environment**: Gazebo Harmonic Warehouse Simulation (5 AMRs)  
**Artifact Visualization**: `docs/images/m8b_adaptive_compute_scaling.png`  
**Sample Size & Scope**: $n=1$ per evaluated experimental condition across 12 discrete runs ($N_{\text{total}}=12$); preliminary single-trial empirical trends within a 5-AMR warehouse simulation, not statistically powered significance claims.

---

## 1. Executive Summary & Core Research Finding

Milestone M8B evaluated the central research question:
> *"Can degradation-aware compute allocation improve fleet coordination performance under changing communication and computational conditions without compromising safety?"*

### Primary Empirical Observations (Within Tested Scenarios):
1. **Packet Loss Resilience (Braking Reduction Trend)**: Under 20% packet loss (`LOSS_HIGH`, Workload 15), fixed `NORMAL` coordination exhibited severe plan thrashing (878 safety brake interventions, 0/15 tasks delivered). In contrast, the `ADAPTIVE` policy detected `HIGH_PACKET_LOSS_RATE`, executed coordinated transitions to `COMPUTE_MODE_LOW` (99.1% time occupancy in LOW), reduced safety brake stops by **68.2%** (down to 279), and completed `task_w15_0001` in $39.28\,\text{s}$.
2. **Congestion Mitigation via Compute Reallocation (Workload 30 Survival)**: Under Workload 30 nominal conditions, fixed `NORMAL` suffered spatial corridor conflict ($d = 0.347\,\text{m} < 0.35\,\text{m}$), triggering a `SAFETY_ABORT` at $T=34.65\,\text{s}$ with 0 tasks delivered. Rather than simply reducing compute, `ADAPTIVE` compute *reallocated* effort upward into frequent replanning (700 replan cycles vs 238 in baseline), maintained a minimum separation of $0.77\,\text{m}$ (well above the $0.35\,\text{m}$ abort threshold), survived the full $120.0\,\text{s}$ horizon, and delivered `task_w30_0026` in $54.23\,\text{s}$.
3. **Boundaries of Adaptive Compute (Observed Negative Result under Compound Stress)**: Under compound stress (Workload 30 + `HIGH_LATENCY`), the adaptive policy switched to `HIGH` mode (505 replans, 5.8% HIGH mode occupancy) to resolve dense corridor contention. However, the $50\,\text{ms}$ transport delay prevented timely dissemination of dynamic reservations in the single narrow aisle. Both fixed `NORMAL` (aborted at $48.30\,\text{s}$, $d=0.340\,\text{m}$, 171 brakes) and `ADAPTIVE` (aborted at $39.35\,\text{s}$, $d=0.346\,\text{m}$, 31 brakes) breached the $0.35\,\text{m}$ threshold. This demonstrates under the tested scenarios that **higher local planning frequency cannot overcome non-negligible transport latency in physical topological bottlenecks without physical waiting bays or topological admission control**.

---

## 2. Empirical Trial Matrix & Coverage

The experimental campaign consists of **12 discrete executed simulation runs** ($n=1$ per evaluated condition). This campaign is intentionally targeted and **not a complete factorial matrix**:

- **Workload 15 ($H=60\,\text{s}$)**:
  - `NORMAL`: Fixed `LOW`, Fixed `NORMAL`, Fixed `HIGH`, `ADAPTIVE`
  - `HIGH_LATENCY`: Fixed `NORMAL`, `ADAPTIVE` (Fixed `LOW` and Fixed `HIGH` were *not* executed)
  - `LOSS_HIGH`: Fixed `NORMAL`, `ADAPTIVE` (Fixed `LOW` and Fixed `HIGH` were *not* executed)
- **Workload 30 ($H=120\,\text{s}$)**:
  - `NORMAL`: Fixed `NORMAL`, `ADAPTIVE` (Fixed `LOW` and Fixed `HIGH` were *not* executed)
  - `HIGH_LATENCY`: Fixed `NORMAL`, `ADAPTIVE` (Fixed `LOW` and Fixed `HIGH` were *not* executed)
  - `LOSS_HIGH`: *Not executed* in this campaign

### Discrete Measured Telemetry Across All 12 Executed Trials:

| Trial ID | Workload | Network Profile | Compute Mode | Delivered / Total | Makespan (s) | Min Dist (m) | Safety Brakes | Host CPU % | Replans | Termination Reason |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `exp_m8b_w15_t0_s42_low` | 15 | `NORMAL` | Fixed `LOW` | 0 / 15 | 60.03 | 1.16 | 476 | 21.1% | 60 | `HORIZON_REACHED` |
| `exp_m8b_w15_t0_s42_normal` | 15 | `NORMAL` | Fixed `NORMAL` | 0 / 15 | 60.00 | 1.13 | 191 | 31.1% | 260 | `HORIZON_REACHED` |
| `exp_m8b_w15_t0_s42_high` | 15 | `NORMAL` | Fixed `HIGH` | 1 / 15 | 39.74 | 1.22 | 620 | 36.6% | 105 | `HORIZON_REACHED` |
| `exp_m8b_w15_t0_s42_adaptive` | 15 | `NORMAL` | `ADAPTIVE` | 1 / 15 | 40.52 | 1.85 | 432 | 32.0% | 56 | `HORIZON_REACHED` |
| `exp_m8b_w15_t0_s42_normal` | 15 | `HIGH_LATENCY` | Fixed `NORMAL` | 0 / 15 | 60.02 | 1.07 | 479 | 35.4% | 110 | `HORIZON_REACHED` |
| `exp_m8b_w15_t0_s42_adaptive` | 15 | `HIGH_LATENCY` | `ADAPTIVE` | 0 / 15 | 60.02 | 1.20 | 47 | 31.2% | 239 | `HORIZON_REACHED` |
| `exp_m8b_w15_t0_s42_normal` | 15 | `LOSS_HIGH` | Fixed `NORMAL` | 0 / 15 | 60.02 | 0.99 | 878 | 31.1% | 73 | `HORIZON_REACHED` |
| `exp_m8b_w15_t0_s42_adaptive` | 15 | `LOSS_HIGH` | `ADAPTIVE` | 1 / 15 | 39.28 | 1.03 | 279 | 28.6% | 60 | `HORIZON_REACHED` |
| `exp_m8b_w30_t0_s42_normal` | 30 | `NORMAL` | Fixed `NORMAL` | 0 / 30 | 34.65 | 0.35 | 77 | 33.2% | 238 | `SAFETY_ABORT` |
| `exp_m8b_w30_t0_s42_adaptive` | 30 | `NORMAL` | `ADAPTIVE` | 1 / 30 | 54.23 | 0.77 | 462 | 31.3% | 700 | `HORIZON_REACHED` |
| `exp_m8b_w30_t0_s42_normal` | 30 | `HIGH_LATENCY` | Fixed `NORMAL` | 0 / 30 | 48.30 | 0.34 | 171 | 31.3% | 308 | `SAFETY_ABORT` |
| `exp_m8b_w30_t0_s42_adaptive` | 30 | `HIGH_LATENCY` | `ADAPTIVE` | 0 / 30 | 39.35 | 0.35 | 31 | 30.7% | 505 | `SAFETY_ABORT` |

> [!NOTE]
> **Sample Size & Statistical Scope**: All reported metrics represent single-trial ($n=1$, seed 42) empirical observations. They provide targeted evidence of operational behavior in this specific 5-AMR warehouse topology under the configured fault profiles. They do not constitute multi-seed, statistically powered significance claims.

---

## 3. Detailed Comparative Analysis & Nuanced Interpretation

### 3.1 Understanding Adaptive Compute: Reallocation vs. Mere Compute Reduction
A critical finding from this trial suite is that **adaptive compute is not simply "reducing compute." It is the dynamic *reallocation* of computational effort to match physical and informational constraints**:
- **Under communication degradation (`LOSS_HIGH`)**: The policy throttles replanning downward to `COMPUTE_MODE_LOW` ($1.0\,\text{Hz}, h=6$). This reduces plan thrashing against stale/lost reservations and prevents acting on corrupted network state, which dramatically reduced reactive braking stops (from 878 to 279).
- **Under spatial congestion (Workload 30 `NORMAL`)**: The policy allocates *more* computational effort into replanning (700 replan cycles vs 238 in baseline), dynamically clearing corridor contention and allowing the fleet to survive the full 120s mission when baseline aborted at 34.65s.
- **Under nominal baseline (Workload 15 `NORMAL`)**: Resource usage scales predictably across fixed modes (Host CPU: 21.1% in `LOW`, 31.1% in `NORMAL`, 36.6% in `HIGH`). Fixed `HIGH` converged CBBA bundles faster ($763.8\,\text{ms}$ vs $9,784\,\text{ms}$ in `NORMAL`), completing 1 task in $39.74\,\text{s}$.

### 3.2 Categorization of Experimental Outcomes

1. **Conditions Where Adaptive Compute Substantially Helped**:
   - **Workload 15 + `LOSS_HIGH`**: 68.2% reduction in emergency braking interventions (878 down to 279), 99.1% time occupancy in `LOW` mode, and completed 1 task vs 0 in fixed `NORMAL`.
   - **Workload 30 + `NORMAL`**: Survived the full 120s horizon without safety abort ($d_{\min} = 0.77\,\text{m}$, 700 replans, 1 task completed) whereas fixed `NORMAL` triggered `SAFETY_ABORT` at $34.65\,\text{s}$ ($d=0.347\,\text{m}$).

2. **Conditions Where Adaptive Compute Did NOT Prevent Failure (Negative Result)**:
   - **Workload 30 + `HIGH_LATENCY`**: Both fixed `NORMAL` (abort at $48.30\,\text{s}$, $d=0.340\,\text{m}$) and `ADAPTIVE` (abort at $39.35\,\text{s}$, $d=0.346\,\text{m}$) suffered safety aborts. Despite `ADAPTIVE` initiating 505 replans and 31 brakes (vs 171 in `NORMAL`), the combination of narrow corridor bottleneck and 50 ms transmission delay prevented safe reservation negotiation.

3. **Conditions Where Differences Were Inconclusive / Minimal**:
   - **Workload 15 + `NORMAL` and `HIGH_LATENCY` Throughput**: Across a 60s horizon with 15 tasks, throughput differences between modes were modest (0/15 or 1/15 completed). Both fixed and adaptive modes maintained safe separation ($d > 1.0\,\text{m}$) without safety aborts.

---

## 4. Verification of Scientific & Accounting Invariants

1. **Task Accounting Invariant**:
   $$\forall \text{ trial } k: \quad N_{\text{completed}}^{(k)} + N_{\text{remaining}}^{(k)} \equiv N_{\text{generated}}^{(k)}$$
   Verified across all 12 trials. Zero tasks were created or destroyed artificially.
2. **Physical Arrival Verification**:
   All reported task completions were mathematically verified against physical odometry coordinates:
   - `task_w15_0001` (Fixed HIGH): `amr_1` at $(7.22, 7.61) \to$ dropoff $(7.00, 8.00)$, distance $= 0.45\,\text{m} \le 0.50\,\text{m}$.
   - `task_w15_0001` (ADAPTIVE Normal): `amr_1` at $(7.24, 7.67) \to$ dropoff $(7.00, 8.00)$, distance $= 0.40\,\text{m} \le 0.50\,\text{m}$.
   - `task_w15_0001` (ADAPTIVE Loss): `amr_1` at $(7.25, 7.79) \to$ dropoff $(7.00, 8.00)$, distance $= 0.33\,\text{m} \le 0.50\,\text{m}$.
   - `task_w30_0026` (ADAPTIVE Normal): `amr_0` at $(8.25, 7.62) \to$ dropoff $(8.00, 8.00)$, distance $= 0.46\,\text{m} \le 0.50\,\text{m}$.
3. **Safety Threshold Verification**:
   The safety abort mechanism was verified to function deterministically. Whenever physical separation dropped below $0.350\,\text{m}$, the trial immediately halted without exception.
