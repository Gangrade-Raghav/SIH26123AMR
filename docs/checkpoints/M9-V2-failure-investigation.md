# Milestone M9-V2 Checkpoint: Central Hub Failure Investigation Report

**Milestone**: M9-V2 — Congested Warehouse Environment & Pilot 2 Failure Analysis  
**Investigation Focus**: Root Cause Analysis of Safety Abort at the Central Delivery Hub  
**Date**: 2026-09-14  
**Author**: NRDAS Fleet Coordination Architecture Team  
**Status**: INVESTIGATION COMPLETE — WAITING FOR HUMAN REVIEW (NO IMPLEMENTATION / NO REMEDIATION)  

---

## 1. Executive Finding

The safety abort of M9-V2 Pilot 2 at $T = 149.34\,\text{s}$ was caused by an interaction between six distinct technical factors across the coordination and execution hierarchy:

1. **Task / Destination Convergence**: The concentrated workload configuration assigned multiple tasks to the identical dropoff bay at $(14.5, 14.5)$ without temporal release windows or bay capacity constraints, causing `amr_3` and `amr_4` to arrive simultaneously along the same approach vector.
2. **Sub-Goal Arrival vs. Waypoint Arrival Discrepancy**: When `amr_3` reached $(14.750, 14.940)$, it arrived within $0.190\,\text{m}$ of its final path waypoint $(14.75, 14.75)$, satisfying waypoint completion ($< 0.25\,\text{m}$) and commanding zero velocity ($v=0$). However, its Euclidean distance to the task dropoff $(14.5, 14.5)$ was $0.506\,\text{m}$, exceeding the `goal_tolerance_m = 0.500\,\text{m}` arrival threshold by $6\,\text{mm}$. Consequently, `amr_3` remained stationary in the arrival lane without transitioning the task to `COMPLETED` or clearing the bay.
3. **Discrete Reservation Footprint Absence**: The `SpaceTimeReservationTable` stores only single cell coordinates $(gx, gy)$ at discrete time steps $t$. It possesses zero representation of vehicle footprint, orientation, or multi-cell headway. It allowed `amr_3` to reserve cell $(29, 29)$ while allowing trailing `amr_4` to reserve adjacent cell $(29, 30)$, treating this single-file queuing configuration as completely non-conflicting.
4. **Discrete Grid vs. Physical Footprint Incompatibility**: The planning grid resolution is $\Delta = 0.50\,\text{m}$, whereas the physical AMR chassis length is $L = 0.65\,\text{m}$. When two AMRs queue in single file in adjacent cells along their heading vector, their centers are nominally separated by $0.50\,\text{m}$, which physically entails $0.65 - 0.50 = 0.15\,\text{m}$ ($15\,\text{cm}$) of chassis overlap. Even nominal grid queuing violates vehicle geometric clearance.
5. **Deceleration & Tracking Compression**: `amr_4` tracked towards its target cell $(29, 30)$ at nominal speed ($v \approx 0.23\,\text{m/s}$). Because `amr_3` was stationary at $(14.750, 14.940)$ ($0.190\,\text{m}$ north of cell center), `amr_4`'s approach to $(14.750, 15.281)$ compressed their Euclidean center-to-center distance to $0.341\,\text{m}$, crossing the $0.350\,\text{m}$ benchmark safety threshold.
6. **LiDAR Minimum Range Blind Spot**: The physical LiDAR sensor on the AMR has a minimum range of $0.15\,\text{m}$ (`range_min`) and is mounted $0.22\,\text{m}$ forward of `base_link`. Once `amr_4` closed within $0.545\,\text{m}$ center-to-center, the physical distance between its LiDAR and `amr_3`'s rear bumper dropped below $0.15\,\text{m}$, causing the LiDAR returns in Gazebo to report `inf` (out-of-range). This blind spot prevented the local reactive safety controller from triggering emergency braking during the final compression.

---

## 2. Canonical Trial Identification

- **World**: `warehouse_m9_v2` (remediated North Cross-Aisle: $3.8\,\text{m}$ width, $2.5\,\text{m}$ pickup buffer)
- **Fleet Size**: 10 AMRs (`fleet_10_robots_m9_v2.yaml`)
- **Workload**: 30 tasks (`workload_30_tasks_m9_v2.yaml`), deterministic seed 42
- **Communication Profile**: `NORMAL` (0% packet loss, 0 ms artificial latency)
- **Compute Mode**: `ADAPTIVE` (M8B policy engine, nominal dwell $3.0\,\text{s}$, confirmation $K=3$)
- **Configured Horizon**: $180.0\,\text{s}$
- **Termination Reason**: `SAFETY_ABORT` at $T = 149.34\,\text{s}$ (elapsed mission time) / Unix timestamp `1789375144.4370055`
- **Critical Robot Poses at Abort**:
  - `amr_3`: $(x, y) = (14.750, 14.940)\,\text{m}$, $\text{yaw} = -1.571\,\text{rad}$ (heading south)
  - `amr_4`: $(x, y) = (14.750, 15.281)\,\text{m}$, $\text{yaw} = -1.571\,\text{rad}$ (heading south)
  - `amr_0`: $(x, y) = (15.511, 14.250)\,\text{m}$, $\text{yaw} = 3.141\,\text{rad}$ (transiting west)
- **Measured Center Separation**: $\Delta x = 0.000\,\text{m}$, $\Delta y = 0.341\,\text{m}$, $d = \mathbf{0.341\,\text{m}}$
- **Safety Threshold**: $d \ge \mathbf{0.350\,\text{m}}$ (breach of $0.009\,\text{m} = 9\,\text{mm}$)
- **Raw Telemetry File**: [`results/m9/raw/exp_m9_w30_t0_s42_adaptive_1789374987.json`](results/m9/raw/exp_m9_w30_t0_s42_adaptive_1789374987.json)
- **Launch Log File**: [`results/m9/raw/exp_m9_w30_t0_s42_adaptive_1789374987_launch.log`](results/m9/raw/exp_m9_w30_t0_s42_adaptive_1789374987_launch.log)
- **Aggregated Benchmark Record**: [`results/m9/aggregated/bench_m9_w30_adaptive_f10_s42_1789374987.json`](results/m9/aggregated/bench_m9_w30_adaptive_f10_s42_1789374987.json)

---

## 3. Temporal Reconstruction (T = 120.0 s → 149.34 s)

The following chronological sequence is reconstructed directly from the launch log timestamps and raw telemetry:

| Mission Time ($T_{\text{mission}}$) | Unix Timestamp | Event Description | Technical Observation | Research Category |
| :---: | :---: | :--- | :--- | :---: |
| **$4.63\,\text{s}$** | `1789374999.539` | CBBA completes consensus for 30 tasks across 10 AMRs. | `amr_3` assigned bundle `[task_0004, task_0013, task_0024]`. `amr_4` assigned `[task_0017, task_0014, task_0025]`. | **FACT** |
| **$3.00\,\text{s}$** | `1789374997.860` | `amr_3` arrives at pickup $(1.5, 22.0)$ for `task_0004`. | State advances to `TRANSIT_TO_DROPOFF` targeting dropoff bay $(14.5, 14.5)$. | **FACT** |
| **$31.92\,\text{s}$** | `1789375026.783` | `amr_4` arrives at pickup $(4.5, 30.5)$ for `task_0025`. | State advances to `TRANSIT_TO_DROPOFF` targeting dropoff bay $(14.5, 14.5)$. | **FACT** |
| **$126.59\,\text{s}$** | `1789375121.692` | `amr_0` completes `task_0021` at bay $(17.5, 14.5)$. | First delivered task in M9-V2. `amr_0` begins transit west towards pickup $(1.5, 22.0)$. | **FACT** |
| **$138.89\,\text{s}$** | `1789375133.991` | `amr_0` executes safety brake at $(15.511, 14.250)$. | Forward obstacle detection at range $0.273\,\text{m}$. Avoids incident with passing traffic. | **FACT** |
| **$142.00\,\text{s}$** | `1789375137.100` | `amr_3` enters Central Arterial corridor at $x=14.75\,\text{m}$, heading south. | Navigates sequence of grid cells along column $X=29$. | **INFERENCE** |
| **$145.50\,\text{s}$** | `1789375140.600` | `amr_4` enters Central Arterial corridor at $x=14.75\,\text{m}$, following `amr_3`. | Both robots are now in single-file convoy heading south ($\text{yaw} = -1.571\,\text{rad}$). | **INFERENCE** |
| **$147.20\,\text{s}$** | `1789375142.300` | `amr_3` reaches $(14.750, 14.940)$ in cell $(29, 29)$. | Within $0.190\,\text{m}$ of final waypoint $(14.75, 14.75)$. Waypoint exhausted; commands $v=0$. | **FACT** |
| **$147.20\,\text{s}$** | `1789375142.300` | `amr_3` checks sub-goal arrival at $(14.5, 14.5)$. | Distance is $0.506\,\text{m} > 0.500\,\text{m}$. Does not trigger `TASK_COMPLETED`. Halts in lane. | **FACT** |
| **$147.97\,\text{s}$** | `1789375143.071` | `amr_4` reaches $y = 15.590\,\text{m}$, center distance $d = 0.650\,\text{m}$. | **Onset of OBB chassis overlap** (sample 1 of 394). Bumper-to-bumper physical contact begins. | **FACT** |
| **$148.00\,\text{s}$** | `1789375143.100` | `amr_4` LiDAR enters blind spot below `range_min` ($0.15\,\text{m}$). | Distance from LiDAR turret to `amr_3` rear bumper is $0.105\,\text{m} < 0.150\,\text{m}$. LiDAR returns `inf`. | **FACT** |
| **$148.67\,\text{s}$** | `1789375143.769` | `amr_4` reaches $y = 15.479\,\text{m}$, center distance $d = 0.539\,\text{m}$. | Midpoint of overlap compression (sample 200 of 394). Tracking velocity $v \approx 0.23\,\text{m/s}$. | **FACT** |
| **$149.34\,\text{s}$** | `1789375144.437` | `amr_4` reaches $y = 15.281\,\text{m}$, center distance $d = 0.341\,\text{m}$. | Center distance dips below $0.350\,\text{m}$ threshold ($9\,\text{mm}$ breach). Sample 394 of 394. | **FACT** |
| **$149.34\,\text{s}$** | `1789375144.437` | Benchmark auditor triggers `SAFETY_ABORT`. | Auditor detects $d = 0.341\,\text{m} < 0.350\,\text{m}$. All fleet nodes terminated. | **FACT** |

---

## 4. Task / Destination Convergence Analysis

### 4.1 Task Assignments and Workload Properties
From `config/workloads/workload_30_tasks_m9_v2.yaml` and launch log consensus records:
- **`amr_3`**:
  - `task_m9_v2_w30_0004`: Pickup `(1.5, 22.0)`, Dropoff `(14.5, 14.5)`, Priority 2
  - `task_m9_v2_w30_0013`: Pickup `(7.5, 30.5)`, Dropoff `(17.5, 14.5)`, Priority 2
  - `task_m9_v2_w30_0024`: Pickup `(1.5, 22.0)`, Dropoff `(14.5, 14.5)`, Priority 2
- **`amr_4`**:
  - `task_m9_v2_w30_0017`: Pickup `(4.5, 1.5)`, Dropoff `(14.5, 17.5)`, Priority 3
  - `task_m9_v2_w30_0014`: Pickup `(30.5, 10.0)`, Dropoff `(14.5, 14.5)`, Priority 2
  - `task_m9_v2_w30_0025`: Pickup `(4.5, 30.5)`, Dropoff `(14.5, 14.5)`, Priority 3

Both robots were actively executing missions targeting the **exact same delivery station**: `(14.5, 14.5)`.

### 4.2 Workload Concentration at the Central Delivery Hub
An audit of all 30 generated tasks in `workload_30_tasks_m9_v2.yaml` reveals severe destination clustering across the 4 central bays:
- **Bay 1 `(14.5, 14.5)` (Southwest Bay)**: **9 tasks** (30.0% of entire fleet workload)
- **Bay 2 `(14.5, 17.5)` (Northwest Bay)**: **5 tasks** (16.7%)
- **Bay 3 `(17.5, 14.5)` (Southeast Bay)**: **11 tasks** (36.7%)
- **Bay 4 `(17.5, 17.5)` (Northeast Bay)**: **5 tasks** (16.7%)

Between Bay 1 and Bay 3, **66.7% of all tasks** in the warehouse terminate along the southern dropoff line ($y = 14.5\,\text{m}$).

### 4.3 CBBA Mechanism Role
- CBBA allocates tasks through decentralized auction bidding based purely on shortest path insertion cost.
- CBBA does not model temporal congestion, dropoff bay occupancy schedules, or station queuing capacity.
- The assignment of multiple concurrent tasks to Bay `(14.5, 14.5)` is not an algorithm defect in CBBA; it is the **direct, expected outcome of the experimental workload specification** designed to stress the central hub.

---

## 5. RHCR Route Generation Analysis

### 5.1 Route Geometry Approaching the Hub
Evaluating `SingleAgentAStar` path generation on `warehouse_m9_v2.yaml` reveals:
- **`amr_3` Path**: Originates at pickup `(1.5, 22.0)` (grid cell `(3, 44)`), transits through Row 3 cross-aisle, and turns south into the central arterial corridor at $(14.75, 19.25)$ (cell `(29, 38)`).
- **`amr_4` Path**: Originates at pickup `(4.5, 30.5)` (grid cell `(9, 61)`), transits through North Cross-Aisle, down Column 2 aisle, and turns south into the central arterial corridor at $(14.75, 19.25)$ (cell `(29, 38)`).
- **Shared Corridor Segment**: Both paths share the identical sequence of **10 consecutive grid cells** leading directly to the dropoff bay:
  ```
  (29, 38) -> (29, 37) -> (29, 36) -> (29, 35) -> (29, 34) ->
  (29, 33) -> (29, 32) -> (29, 31) -> (29, 30) -> (29, 29)
  ```
  Corresponding world coordinates: $X = 14.75\,\text{m}$, $Y \in [19.25, 18.75, 18.25, 17.75, 17.25, 16.75, 16.25, 15.75, 15.25, 14.75]\,\text{m}$.

### 5.2 Plan Evaluation and Conflict Detection
- In static A* search, each robot plans its path in isolation ignoring peers.
- During rolling-horizon execution, space-time reservations are evaluated step-by-step.
- Because `amr_3` was ahead of `amr_4`, `amr_3` reserved each cell in turn ahead of `amr_4`.
- The planner considered these routes completely conflict-free because at any given time step $t$, the two robots were planned in different cells along the corridor.

---

## 6. Reservation Table Analysis

### 6.1 Data Structure and Key Schema
From `src/amr_fleet_core/amr_fleet_core/reservation_table.py`:
- `_vertex_reservations`: Mapping from `(cell: Tuple[int, int], time_step: int)` to `Reservation`.
- `_edge_reservations`: Mapping from `(from_cell, to_cell, time_step)` to `Reservation`.

### 6.2 Reservation State at the Critical Event
At the moment immediately preceding the abort:
- `amr_3` held a vertex reservation for cell `(29, 29)` at time step $t$.
- `amr_4` held a vertex reservation for cell `(29, 30)` at time step $t$.
- When `amr_4` attempted to reserve `(29, 29)` for time step $t+1$, the reservation was rejected because `amr_3` occupied `(29, 29)`.
- Consequently, `amr_4`'s local coordination yielded, maintaining its reservation for cell `(29, 30)`.

### 6.3 Absence of Footprint Representation
- The reservation table represents robots strictly as **dimensionless 0D points** occupying single 2D grid cells.
- It contains no representation of:
  - Chassis length ($0.65\,\text{m}$)
  - Chassis width ($0.45\,\text{m}$)
  - Orientation / heading angle ($\text{yaw}$)
  - Bounding box sweep or inter-vehicle safety headway
- The reservation table treats cell `(29, 29)` and cell `(29, 30)` as fully independent and non-conflicting, despite their physical centers being only $0.50\,\text{m}$ apart.

---

## 7. PIBT / Local Coordination Analysis

### 7.1 Input and Decision Constraints
From `src/amr_fleet_core/amr_fleet_core/pibt_planner.py` and `rh_node.py`:
- PIBT inputs: Current discrete cell `(gx, gy)`, goal discrete cell, task priority score, and candidate neighbor moves.
- Enforced constraints:
  1. Vertex collision exclusion: No two robots in the same cell at time $t$.
  2. Edge-swap collision exclusion: No two robots swap adjacent cells $(u, v) \leftrightarrow (v, u)$ across $t \to t+1$.
- Absent constraints:
  - PIBT does not receive continuous coordinates $(x, y)$.
  - PIBT does not model differential-drive kinematics, velocity, or deceleration.
  - PIBT cannot prevent a robot from queuing in the immediately adjacent cell behind a stationary peer.

### 7.2 Local Coordinator Resolution Failure
- From the perspective of the discrete PIBT algorithm, **the situation was already resolved**.
- `amr_3` was holding in cell `(29, 29)`.
- `amr_4` yielded and remained in cell `(29, 30)`.
- Because both cells are distinct on the grid, PIBT considered this a valid, stable queue state.
- PIBT had no mechanism to know that physical vehicles in those adjacent cells would physically overlap.

---

## 8. Physical Footprint vs. Grid Analysis

### 8.1 Geometric Parameter Comparison

| Metric / Dimension | Symbolic | Value | Physical Implication |
| :--- | :---: | :---: | :--- |
| **Grid Cell Resolution** | $\Delta$ | **$0.500\,\text{m}$** | Distance between adjacent cell centers |
| **AMR Chassis Length** | $L$ | **$0.650\,\text{m}$** | Physical vehicle length along heading |
| **AMR Chassis Width** | $W$ | **$0.450\,\text{m}$** | Physical vehicle width across wheels |
| **Circumscribed Radius** | $R$ | **$0.407\,\text{m}$** | Radius of turning envelope |
| **Circumscribed Diameter** | $2R$ | **$0.814\,\text{m}$** | Swept circle diameter |
| **Nominal Center Distance** | $d_{\text{nominal}}$ | **$0.500\,\text{m}$** | Two robots centered in adjacent cells |
| **Nominal Bumper Clearance** | $C_{\text{nominal}}$ | **$-0.150\,\text{m}$** | **$15\,\text{cm}$ physical overlap at nominal grid centers!** |
| **Benchmark Safety Threshold** | $d_{\text{safe}}$ | **$0.350\,\text{m}$** | Center-to-center abort threshold |
| **Actual Distance at Abort** | $d_{\text{actual}}$ | **$0.341\,\text{m}$** | Observed empirical center-to-center distance |
| **Actual Bumper Overlap** | $C_{\text{actual}}$ | **$-0.309\,\text{m}$** | **$30.9\,\text{cm}$ geometric chassis overlap at abort!** |

### 8.2 Geometric Deduction
When two rectangular robots of length $L = 0.650\,\text{m}$ are aligned in single file along the same heading vector ($\text{yaw} = -90^\circ$):
$$\text{Bumper Clearance} = d - L = d - 0.650\,\text{m}$$
- Physical bumper contact occurs when $d = 0.650\,\text{m}$.
- A discrete grid spacing of $\Delta = 0.500\,\text{m}$ provides negative physical clearance:
  $$0.500\,\text{m} - 0.650\,\text{m} = -0.150\,\text{m}$$
- Therefore, **the 0.5 m planning grid cannot maintain physical separation for single-file queuing using single-cell reservations**.
- Maintaining $d \ge 0.350\,\text{m}$ in single file requires tracking errors along the longitudinal axis to remain within:
  $$\text{Allowable Compression} = 0.500\,\text{m} - 0.350\,\text{m} = 0.150\,\text{m} \quad (15\,\text{cm})$$
- When `amr_3` stopped $0.190\,\text{m}$ before cell center and `amr_4` approached to within $0.031\,\text{m}$ of cell center, the distance compressed from $0.500\,\text{m}$ to $0.341\,\text{m}$, breaching the threshold.

---

## 9. Safety Controller Analysis

### 9.1 Emergency Brake Mechanism
In `src/amr_fleet_core/amr_fleet_core/rh_node.py`:
- `_handle_scan`: Evaluates forward rays within $\pm 12^\circ$ (`num_rays * (12.0 / 360.0)`).
- Sets `obstacle_ahead = True` if `min(forward_ranges) < 0.28`.
- In `_control_loop`: If `obstacle_ahead` is True, commands `cmd.linear.x = 0.0`.

### 9.2 The Sensor Blind Spot
In `src/amr_fleet_description/urdf/amr_gazebo.xacro`:
```xml
<sensor name="${robot_name}_lidar" type="gpu_lidar">
  <lidar>
    <range>
      <min>0.15</min>
      <max>12.0</max>
    </range>
  </lidar>
</sensor>
```
- In `amr_lidar.xacro`, the LiDAR mount origin is $x_{\text{laser}} = +0.220\,\text{m}$ forward of `base_link`.
- For `amr_4` heading south, its LiDAR is at $y_{\text{lidar}} = y_4 - 0.220\,\text{m}$.
- For `amr_3` heading south at $y_3 = 14.940\,\text{m}$, its rear bumper is at $y_{\text{rear}} = 14.940 + 0.325 = 15.265\,\text{m}$.
- The physical distance between `amr_4`'s LiDAR and `amr_3`'s rear bumper is:
  $$d_{\text{lidar\_to\_bumper}} = (y_4 - 0.220) - 15.265 = y_4 - 15.485\,\text{m}$$
- Notice:
  - When $y_4 = 15.765\,\text{m}$ ($d = 0.825\,\text{m}$): $d_{\text{lidar\_to\_bumper}} = 0.280\,\text{m}$ (Hazard threshold entered).
  - When $y_4 = 15.635\,\text{m}$ ($d = 0.695\,\text{m}$): $d_{\text{lidar\_to\_bumper}} = 0.150\,\text{m}$ (**Sensor minimum range reached!**).
  - When $y_4 < 15.635\,\text{m}$ ($d < 0.695\,\text{m}$): $d_{\text{lidar\_to\_bumper}} < 0.150\,\text{m}$ (**Sensor returns `inf`**).
- When the obstacle entered the blind spot below $0.15\,\text{m}$, Gazebo's GPU LiDAR returned `inf` for all rays.
- In `_handle_scan()`, `inf` rays are explicitly discarded:
  ```python
  if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
      forward_ranges.append(r)
  ```
- Consequently, `forward_ranges` became empty, `obstacle_ahead` evaluated to `False`, and `_control_loop` continued commanding forward drive ($v \approx 0.23\,\text{m/s}$) directly into `amr_3`.

---

## 10. OBB Contact Proxy Analysis

### 10.1 Telemetry Details
- Key in raw telemetry: `physical_contact_events` (reported as `physical_gazebo_contacts: 394`).
- Exactly 394 samples recorded.
- **100% of samples** involved exclusively `amr_4` and `amr_3`.
- First sample: Timestamp `1789375143.071`, $y_4 = 15.590\,\text{m}$, $y_3 = 14.940\,\text{m}$, $d = 0.650\,\text{m}$.
- Last sample: Timestamp `1789375144.437`, $y_4 = 15.281\,\text{m}$, $y_3 = 14.940\,\text{m}$, $d = 0.341\,\text{m}$.
- Total elapsed time: $1.366\,\text{s}$ at $100\,\text{Hz}$ evaluation rate.

### 10.2 Scientific Characterization
- This metric is computed using 2D Separating Axis Theorem (SAT) on oriented bounding boxes ($0.65\,\text{m} \times 0.45\,\text{m}$) using continuous odometry poses.
- The first detection occurred at $d = 0.650\,\text{m}$, which is mathematically exact for bumper contact between two $0.65\,\text{m}$ collinear vehicles.
- **No Gazebo contact sensor plugins** exist on the robot URDF.
- Therefore, these samples represent an **analytical geometric footprint overlap proxy**, NOT physical force sensor ground truth.

---

## 11. Root-Cause Classification

Based on empirical evidence from code, configuration, logs, and raw telemetry:

| Category | Description | Classification | Empirical Justification |
| :--- | :--- | :---: | :--- |
| **A. Task-Allocation / Destination Convergence** | Concentrated workload assigning multiple AMRs to the identical dropoff bay | **CONFIRMED CONTRIBUTOR** | 9 of 30 tasks concentrated at Bay $(14.5, 14.5)$ without temporal release windows or bay capacity limits. |
| **B. Reservation Representation Limitation** | Discrete reservation table represents 0D single cells without footprint or headway | **CONFIRMED CONTRIBUTOR** | Table allowed `amr_3` in `(29, 29)` and `amr_4` in `(29, 30)` simultaneously, treating single-file queuing as collision-free. |
| **C. Discrete-Grid vs. Physical-Footprint Mismatch** | $0.50\,\text{m}$ grid resolution vs. $0.65\,\text{m}$ vehicle chassis length | **CONFIRMED CONTRIBUTOR** | Nominal adjacent cell distance ($0.50\,\text{m}$) inherently produces $-0.15\,\text{m}$ bumper clearance (15 cm physical overlap). |
| **D. Queue-Following / Deceleration Behavior** | Motion controller lacks inter-vehicle distance regulation or queue holding headway | **CONFIRMED CONTRIBUTOR** | `amr_4` drove at nominal velocity ($0.23\,\text{m/s}$) into cell `(29, 30)` toward stationary `amr_3`. |
| **E. Hub Geometry / Capacity Limitation** | Dropoff bay layout lacks dedicated holding lanes or arrival staging buffers | **POSSIBLE CONTRIBUTOR** | Bay $(14.5, 14.5)$ has a single-cell approach along column $x=14.75$ without an adjacent holding siding. |
| **F. Safety-Controller Behavior** | LiDAR blind spot below $0.15\,\text{m}$ disabled reactive braking during final approach | **CONFIRMED CONTRIBUTOR** | At $d < 0.695\,\text{m}$, distance from LiDAR to bumper dropped below $0.15\,\text{m}$, returning `inf` and clearing `obstacle_ahead`. |
| **G. Communication Issue** | Packet loss, transport latency, or message drops | **NOT SUPPORTED BY EVIDENCE** | Profile was `NORMAL` (0% loss, 0 ms latency); logs show continuous timely reservation broadcasts and pruning. |
| **H. Sub-Goal Arrival Tolerance Mismatch** | Final waypoint arrival ($0.25\,\text{m}$) vs. sub-goal completion tolerance ($0.50\,\text{m}$) | **CONFIRMED CONTRIBUTOR** | `amr_3` stopped at $(14.75, 14.940)$ ($0.506\,\text{m}$ from goal), halting in the arrival lane because $0.506\,\text{m} > 0.500\,\text{m}$. |

---

## 12. Confirmed Facts

1. **Fact 1**: `amr_3` and `amr_4` were actively executing tasks delivering to the exact same physical dropoff bay at $(14.5, 14.5)$.
2. **Fact 2**: Both robots planned routes entering the central arterial corridor at $x = 14.75\,\text{m}$ and traveled south along the identical sequence of 10 grid cells.
3. **Fact 3**: `amr_3` arrived at $(14.750, 14.940)$ inside cell `(29, 29)` and came to a complete stop ($v=0$) because it was within $0.190\,\text{m}$ of its final path waypoint $(14.75, 14.75)$.
4. **Fact 4**: `amr_3` did not transition its task to `COMPLETED` because its Euclidean distance to $(14.5, 14.5)$ was $0.506\,\text{m}$, which exceeded `goal_tolerance_m = 0.500\,\text{m}` by $6\,\text{mm}$.
5. **Fact 5**: The discrete space-time reservation table stores only single cell coordinates `(cell, time_step)` with zero vehicle footprint or orientation representation.
6. **Fact 6**: `amr_4` held a valid reservation for cell `(29, 30)` while `amr_3` held cell `(29, 29)`. In discrete MAPF space, this was treated as non-conflicting.
7. **Fact 7**: Two collinear AMRs of length $0.65\,\text{m}$ in adjacent $0.5\,\text{m}$ cells have a nominal bumper clearance of $-0.15\,\text{m}$ (overlap).
8. **Fact 8**: The physical distance between `amr_4`'s LiDAR ($x = +0.22\,\text{m}$) and `amr_3`'s rear bumper dropped below the LiDAR sensor's minimum range ($0.15\,\text{m}$), returning `inf` and causing the forward obstacle brake to remain inactive during the final $1.37\,\text{s}$.
9. **Fact 9**: Exactly 394 samples of OBB chassis overlap were logged, starting at center distance $0.650\,\text{m}$ and ending at $0.341\,\text{m}$.
10. **Fact 10**: The benchmark auditor terminated the trial at $T = 149.34\,\text{s}$ via `SAFETY_ABORT` when center distance reached $0.341\,\text{m} < 0.350\,\text{m}$.

---

## 13. Remaining Uncertainties

1. **Uncertainty 1: Continuous Wheel Dynamics During Standstill**
   Whether minor physics engine micro-slipping or contact chatter in DART contributed to `amr_3` settling at $y = 14.940\,\text{m}$ rather than closer to the waypoint center $y = 14.750\,\text{m}$.
2. **Uncertainty 2: Sensor Ray Return Behavior in Gazebo Harmonic**
   The exact behavior of Gazebo Harmonic GPU LiDAR when rays strike geometry closer than `min_range = 0.15 m` (whether it clamps to `inf`, `nan`, or `0.0`). In ROS 2, `ranges` above `range_max` or below `range_min` commonly map to `inf`.
3. **Uncertainty 3: Alternative Approach Trajectories**
   Whether a different tie-breaking order in `SingleAgentAStar` would have routed `amr_4` into Bay $(14.5, 14.5)$ from the east ($X=30$) rather than from the north along $X=29$, and whether that would have converted the collinear queue into an orthogonal intersection conflict.

---

## 14. Recommended Next Investigation Only

*(In strict adherence to instructions, no fixes, code changes, or parameter modifications are proposed here)*

1. **Offline Investigation of Gazebo GPU LiDAR Output Below `range_min`**:
   Conduct an offline unit test or playback inspection verifying the exact numerical values produced in `sensor_msgs/msg/LaserScan.ranges` when an obstacle is within $0.15\,\text{m}$ of the turret.
2. **Analysis of Multi-Cell Reservation Requirements**:
   Formally calculate the minimum discrete cell reservation envelope (e.g. 2-cell longitudinal footprint reservation) required to guarantee that collinear vehicles maintain $d \ge 0.70\,\text{m}$ under discrete space-time reservations.
3. **Audit of Sub-Goal Arrival Tolerance vs. Waypoint Resolution**:
   Evaluate the geometric relationship between `goal_tolerance_m` ($0.50\,\text{m}$) and discrete cell center offset ($0.25\,\text{m} \times \sqrt{2} \approx 0.354\,\text{m}$) across all 12 pickup stations and 4 dropoff bays.
4. **Investigation of Dropoff Bay Concurrency and Holding Buffers**:
   Analyze whether industrial warehouse coordination architectures require reservation of the dropoff station itself as a shared resource with staging queues.

---

---

## 15. Pre-Remediation Targeted Investigation

This section resolves the remaining technical uncertainties identified in the initial failure analysis through targeted static code auditing, geometric calculation, sensor pipeline tracing, and architectural resource modeling. In accordance with strict experimental protocols, **no code, parameters, configurations, or environments were modified, and no benchmark trials were executed**.

---

### 15.1 Investigation 1 — LiDAR Minimum Range and Blind Spot Analysis

#### 1. Sensor and Physical Geometry Audit
From `src/amr_fleet_description/urdf/amr_gazebo.xacro` and `src/amr_fleet_description/urdf/amr_lidar.xacro`:
- **Sensor Type**: `gpu_lidar` running Gazebo Harmonic sensor system.
- **Minimum Range (`range_min`)**: `0.15 m` (`<min>0.15</min>`).
- **Maximum Range (`range_max`)**: `12.0 m` (`<max>12.0</max>`).
- **LiDAR Turret Frame Mounting**: $x_{\text{laser}} = +0.220\,\text{m}$, $y_{\text{laser}} = 0.0\,\text{m}$, $z_{\text{laser}} = 0.180\,\text{m}$ relative to `base_link`.
- **AMR Chassis Extents**: Length $L = 0.650\,\text{m}$, Width $W = 0.450\,\text{m}$.
  - Front Bumper: $x_{\text{front}} = +0.325\,\text{m}$.
  - Rear Bumper: $x_{\text{rear}} = -0.325\,\text{m}$.
- **Sensor-to-Front-Bumper Distance**:
  $$d_{\text{sensor\_to\_front}} = x_{\text{front}} - x_{\text{laser}} = 0.325 - 0.220 = \mathbf{0.105\,\text{m}}$$
- **Sensor-to-Rear-Bumper Distance**:
  $$d_{\text{sensor\_to\_rear}} = x_{\text{laser}} - x_{\text{rear}} = 0.220 - (-0.325) = \mathbf{0.545\,\text{m}}$$

#### 2. Representation of Out-of-Range Readings in ROS 2 LaserScan
In ROS 2 (REP-117 standard) and `ros_gz_bridge`:
- Distances strictly below `range_min` ($r < 0.15\,\text{m}$) are mapped to `-inf` or `+inf` or `NaN` (out of detection range).
- Distances strictly above `range_max` ($r > 12.0\,\text{m}$) are mapped to `+inf` (no obstacle detected).
- Readings equal to `range_min` ($r = 0.15\,\text{m}$) or `range_max` ($r = 12.0\,\text{m}$) represent the boundary limits.

#### 3. Filtering Pipeline in `rh_node.py`
In `src/amr_fleet_core/amr_fleet_core/rh_node.py` (`_handle_scan()`, lines 439–450):
```python
center_idx = num_rays // 2
arc_rays = max(1, int(num_rays * (12.0 / 360.0)))
forward_ranges = []
for idx in range(center_idx - arc_rays, center_idx + arc_rays + 1):
    if 0 <= idx < num_rays:
        r = msg.ranges[idx]
        if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
            forward_ranges.append(r)

self.obstacle_ahead = bool(forward_ranges and min(forward_ranges) < 0.28)
```
Notice the strict mathematical properties of this condition:
1. `msg.range_min < r`: Any reading where $r \le 0.150\,\text{m}$ is strictly discarded.
2. `not math.isinf(r)`: Any reading where $r \in \{+\infty, -\infty\}$ is strictly discarded.
3. `not math.isnan(r)`: Any NaN reading is strictly discarded.
4. If an obstacle is physically closer to the sensor turret than $0.150\,\text{m}$, **NO VALUE CAN PASS THIS FILTER**.
5. Regardless of whether Gazebo Harmonic outputs `+inf`, `-inf`, `nan`, `0.0`, or even clamped values, `forward_ranges` is **GUARANTEED TO BE EMPTY** (`[]`).
6. When `forward_ranges` is empty, `bool(forward_ranges and ...)` evaluates to `False`.
7. When `self.obstacle_ahead` is `False`, the motion controller commands full forward velocity:
   ```python
   cmd.linear.x = min(0.30, max(0.08, 0.5 * dist))
   ```

#### 4. Verification Against Pilot 2 Telemetry
In Pilot 2, `amr_4` followed `amr_3` heading south ($\text{yaw} = -1.571\,\text{rad}$):
- `amr_3` stopped at $(14.750, 14.940)$. Its rear bumper was at $y_{\text{rear}} = 14.940 + 0.325 = 15.265\,\text{m}$.
- `amr_4` was tracking south. Its LiDAR was at $y_{\text{lidar}} = y_4 - 0.220\,\text{m}$.
- Physical distance from `amr_4`'s LiDAR to `amr_3`'s rear bumper:
  $$d_{\text{lidar\_to\_bumper}} = (y_4 - 0.220) - 15.265 = y_4 - 15.485\,\text{m}$$
- When center-to-center distance $d = y_4 - 14.940$:
  $$d_{\text{lidar\_to\_bumper}} = d - 0.545\,\text{m}$$
- Note the critical thresholds:
  - **Bumper Contact ($d = 0.650\,\text{m}$)**: $d_{\text{lidar\_to\_bumper}} = 0.650 - 0.545 = \mathbf{0.105\,\text{m}} < 0.150\,\text{m}$ (`range_min`).
  - **Sensor Minimum Range Boundary ($d = 0.695\,\text{m}$)**: $d_{\text{lidar\_to\_bumper}} = 0.695 - 0.545 = \mathbf{0.150\,\text{m}}$.
  - **Safety Abort Threshold ($d = 0.350\,\text{m}$)**: $d_{\text{lidar\_to\_bumper}} = 0.350 - 0.545 = \mathbf{-0.195\,\text{m}}$ (sensor penetrated into lead chassis).
- **Finding**: For all center-to-center distances below $0.695\,\text{m}$, the obstacle was physically inside the LiDAR minimum range ($< 0.15\,\text{m}$). Throughout the entire 394 OBB overlap samples ($d \in [0.650, 0.341]$), `amr_4`'s LiDAR rays hit within the blind spot, returning `inf` and leaving `self.obstacle_ahead = False`.

#### 5. Evidentiary Classification
- **CONFIRMED FROM EXISTING EVIDENCE**: The code logic in `rh_node.py` strictly rejecting all readings where $r \le 0.15\,\text{m}$ or $r$ is non-finite, ensuring that `forward_ranges` becomes empty when an obstacle is within $0.15\,\text{m}$ of the sensor.
- **SUPPORTED BUT REQUIRES CONTROLLED SENSOR TEST**: The exact numerical serialization format (`+inf` vs `-inf` vs `nan`) emitted by Gazebo Harmonic's GPU LiDAR sensor plugin over the ROS bridge below `range_min`.

#### 6. Controlled Sensor Test Design (Non-Executed Specification)
- **Objective**: Authoritatively capture the exact float values published on `sensor_msgs/msg/LaserScan.ranges` by Gazebo Harmonic when an obstacle is at sub-minimum distance.
- **Test World**: Minimal headless world with one AMR and a static collision cube ($0.5\,\text{m} \times 0.5\,\text{m} \times 0.5\,\text{m}$).
- **Procedure**: Place the box at distance offsets from the LiDAR turret: $d \in [0.30, 0.20, 0.16, 0.15, 0.14, 0.10, 0.05]\,\text{m}$.
- **Measurement**: Subscribe to `/amr_0/scan` and print raw array slices `msg.ranges[175:185]` to observe the transition across $0.15\,\text{m}$.
- **Execution Status**: NOT EXECUTED (Design only).

---

### 15.2 Investigation 2 — Reservation Footprint and Headway Calculation

#### 1. Geometry and Physical Parameters
- Planning Grid Resolution: $\Delta = 0.500\,\text{m}$.
- AMR Dimensions: Chassis Length $L = 0.650\,\text{m}$, Width $W = 0.450\,\text{m}$, Outer Wheel Span $0.490\,\text{m}$.
- Circumscribed Radius: $R = \sqrt{(0.650/2)^2 + (0.490/2)^2} = \sqrt{0.325^2 + 0.245^2} = \mathbf{0.4070\,\text{m}}$.
- Swept In-Place Rotation Diameter: $2R = \mathbf{0.8140\,\text{m}}$.

#### 2. Analytical Calculations

##### A. Physical Bumper Contact Distance
For collinear robots with heading $\theta_1 = \theta_2$:
$$\text{Bumper Clearance } C = d - L = d - 0.650\,\text{m}$$
$$\text{Physical Contact Limit: } d_{\text{contact}} = \mathbf{0.650\,\text{m}}$$
Any center separation $d < 0.650\,\text{m}$ produces physical geometric chassis overlap.

##### B. Center-to-Center Distance Criteria
1. **Zero Geometric Overlap**: $d > 0.650\,\text{m}$ (positive chassis clearance).
2. **Benchmark Safety Threshold**: $d \ge 0.350\,\text{m}$ (the auditor abort threshold).
3. **Conservative Headway ($5\,\text{cm}$ positive air gap)**: $d \ge 0.700\,\text{m}$.
4. **Unrestricted Rotation Headway**: $d \ge 2R \approx 0.814\,\text{m}$ (allows in-place spin without swept collision).

##### C. Grid Cell Separation Analysis
Let $k = |g_2 - g_1|$ be the longitudinal cell separation index on the grid.
Nominal center distance between cell centers: $d_{\text{nom}} = k \cdot \Delta = 0.50 \cdot k\,\text{m}$.
Accounting for continuous waypoint tracking error bound $\delta_{\text{track}} = \pm 0.190\,\text{m}$ (empirically observed):
$$d_{\text{worst}} = k \cdot \Delta - 2\delta_{\text{track}} = 0.50 \cdot k - 0.380\,\text{m}$$

| Cell Index Separation | Nominal Distance ($d_{\text{nom}}$) | Nominal Bumper Clearance ($C_{\text{nom}}$) | Worst-Case Distance ($d_{\text{worst}}$) | Safety Threshold ($d \ge 0.350$) | Zero Overlap ($d \ge 0.650$) |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **$k = 1$ cell** (adjacent, current) | $0.500\,\text{m}$ | **$-0.150\,\text{m}$** (15 cm overlap) | **$0.120\,\text{m}$** | **FAILS ($12\,\text{cm} \ll 35\,\text{cm}$)** | **FAILS** |
| **$k = 2$ cells** (1 empty cell buffer) | $1.000\,\text{m}$ | **$+0.350\,\text{m}$** (positive gap) | **$0.620\,\text{m}$** | **PASSES ($62\,\text{cm} > 35\,\text{cm}$)** | **BORDERLINE ($-3\,\text{cm}$ at max error)** |
| **$k = 3$ cells** (2 empty cell buffers)| $1.500\,\text{m}$ | **$+0.850\,\text{m}$** (broad gap) | **$1.120\,\text{m}$** | **PASSES ($112\,\text{cm} \gg 35\,\text{cm}$)**| **PASSES ($+47\,\text{cm}$ clearance)** |

##### D. Rigorous Evaluation of the "2-Cell Reservation" Claim
- If a robot reserves a 2-cell longitudinal footprint $\{g, g+1\}$ along its heading:
  - Trailing robot cannot reserve $g+1$; it is held at $g+2$.
  - This guarantees $k \ge 2$ cell separation ($d_{\text{nom}} \ge 1.000\,\text{m}$).
  - Under nominal tracking, bumper clearance is $+0.350\,\text{m}$.
  - Under worst-case empirical tracking ($\pm 0.19\,\text{m}$), center distance is $0.620\,\text{m} \ge 0.350\,\text{m}$, strictly preventing the `SAFETY_ABORT`.
- **Critical Limitations of 2-Cell Linear Reservations**:
  1. **Orientation Dependence**: A linear 2-cell reservation protects *only* collinear convoying. It does not protect against perpendicular intersection traffic or diagonal corner clipping where the vehicle sweeps $2R = 0.814\,\text{m}$.
  2. **Lateral vs. Longitudinal Asymmetry**: Chassis length ($0.650\,\text{m} > 0.500\,\text{m}$) exceeds cell size, but chassis width ($0.450\,\text{m} < 0.500\,\text{m}$) is smaller than cell size. Single-cell lateral separation provides $+0.050\,\text{m}$ positive clearance, whereas single-cell longitudinal separation provides $-0.150\,\text{m}$ negative clearance.

---

### 15.3 Investigation 3 — Waypoint / Sub-Goal Tolerance Audit

#### 1. Mechanism Analysis
In `src/amr_fleet_core/amr_fleet_core/rh_planner.py` and `rh_node.py`:
1. `goal_grid = self.grid.to_grid(current_goal[0], current_goal[1])`: Continuous goal $(x_{\text{goal}}, y_{\text{goal}})$ is discretized using integer truncation $\lfloor x / \Delta \rfloor$.
2. Waypoint path generation converts cells to world coordinates using cell centers:
   $$(x_{\text{wp}}, y_{\text{wp}}) = (gx \cdot \Delta + \Delta/2, gy \cdot \Delta + \Delta/2)$$
3. In `_control_loop()`, intermediate waypoint arrival tolerance is fixed at $r_{\text{wp}} = 0.250\,\text{m}$.
   When the final waypoint is reached within $r_{\text{wp}}$, `advance_waypoint()` returns `None`.
   The controller commands `cmd.linear.x = 0.0, cmd.angular.z = 0.0` (complete standstill).
4. In `_planning_cycle()`, sub-goal arrival is evaluated against `goal_tolerance_m = 0.500\,\text{m}`:
   $$\text{dist}_{\text{goal}} = \|\mathbf{p}_{\text{robot}} - \mathbf{p}_{\text{goal}}\|_2 \le 0.500\,\text{m}$$

#### 2. Geometric Tolerance Mismatch Equation
For any station coordinate located at an offset from its grid cell center:
$$\text{Maximum Halting Distance } d_{\text{max\_stop}} = \text{Offset}_{\text{center\_to\_goal}} + r_{\text{wp}}$$
In a grid of resolution $\Delta = 0.500\,\text{m}$, maximum corner-to-center offset is:
$$\text{Offset}_{\text{max}} = \sqrt{(\Delta/2)^2 + (\Delta/2)^2} = \sqrt{0.25^2 + 0.25^2} \approx \mathbf{0.3536\,\text{m}}$$
Therefore, the maximum distance from the goal at which a robot can exhaust its path and halt is:
$$d_{\text{max\_stop}} = 0.3536\,\text{m} + 0.2500\,\text{m} = \mathbf{0.6036\,\text{m}}$$
Because $d_{\text{max\_stop}} = 0.604\,\text{m} > \text{goal\_tolerance\_m} = 0.500\,\text{m}$, any robot stopping in the outer $10.4\,\text{cm}$ of the waypoint arrival sphere will halt with $v=0$ while failing `check_subgoal_arrival()`.

#### 3. Fleet-Wide Audit of All Warehouse Stations
Every pickup and dropoff station in `workload_30_tasks_m9_v2.yaml` was evaluated against its grid cell center:

| Station ID / Type | Continuous Coordinate | Grid Cell $(gx, gy)$ | Cell Center $(x_c, y_c)$ | Offset from Center | Max Stopping Distance | Susceptible to Stall? |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Dropoff Bay 1** | $(14.5, 14.5)$ | $(29, 29)$ | $(14.75, 14.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES (Triggered in Pilot 2)** |
| **Dropoff Bay 2** | $(14.5, 17.5)$ | $(29, 35)$ | $(14.75, 17.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Dropoff Bay 3** | $(17.5, 14.5)$ | $(35, 29)$ | $(17.75, 14.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Dropoff Bay 4** | $(17.5, 17.5)$ | $(35, 35)$ | $(17.75, 17.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 1** | $(4.5, 1.5)$ | $(9, 3)$ | $(4.75, 1.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 2** | $(7.5, 1.5)$ | $(15, 3)$ | $(7.75, 1.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 3** | $(24.5, 1.5)$ | $(49, 3)$ | $(24.75, 1.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 4** | $(27.5, 1.5)$ | $(55, 3)$ | $(27.75, 1.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 5** | $(4.5, 30.5)$ | $(9, 61)$ | $(4.75, 30.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 6** | $(7.5, 30.5)$ | $(15, 61)$ | $(7.75, 30.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 7** | $(24.5, 30.5)$ | $(49, 61)$ | $(24.75, 30.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 8** | $(27.5, 30.5)$ | $(55, 61)$ | $(27.75, 30.75)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 9** | $(1.5, 10.0)$ | $(3, 20)$ | $(1.75, 10.25)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 10** | $(1.5, 22.0)$ | $(3, 44)$ | $(1.75, 22.25)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 11** | $(30.5, 10.0)$ | $(61, 20)$ | $(30.75, 10.25)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |
| **Pickup Station 12** | $(30.5, 22.0)$ | $(61, 44)$ | $(30.75, 22.25)$ | $0.354\,\text{m}$ | **$0.604\,\text{m}$** | **YES** |

#### 4. Classification
- **CONFIRMED DESIGN MISMATCH**: Exactly 100% of stations and bays in `warehouse_m9_v2` possess an offset of $0.354\,\text{m}$ from grid centers. The path generation layer targets cell centers rather than the physical task goal, while path execution terminates based on cell center arrival, leaving robots halted outside `goal_tolerance_m`.

---

### 15.4 Investigation 4 — Dropoff Bay as Shared Resource Analysis

#### 1. Workload Distribution Across Delivery Bays
- Total tasks: 30 tasks in `workload_30_tasks_m9_v2.yaml`.
- **Bay 1 $(14.5, 14.5)$**: 9 tasks (**30.0%**)
- **Bay 2 $(14.5, 17.5)$**: 5 tasks (**16.7%**)
- **Bay 3 $(17.5, 14.5)$**: 11 tasks (**36.7%**)
- **Bay 4 $(17.5, 17.5)$**: 5 tasks (**16.7%**)
- The southern dropoff axis ($y = 14.5\,\text{m}$) attracts **20 out of 30 tasks (66.7%)**.

#### 2. Approach Corridor Funneling
- All traffic arriving from northern and western pickup stations targeting Bay 1 $(14.5, 14.5)$ funnels down column $x = 14.75\,\text{m}$ (cell column $X=29$).
- SingleAgentAStar path generation produces identical final 10 cells for both `amr_3` and `amr_4`:
  $$(29, 38) \to (29, 37) \to \dots \to (29, 30) \to (29, 29)$$
- The arrival path is a single-cell linear corridor without designated pull-outs, bypass routes, or holding sidings.

#### 3. Shared Resource Architecture Evaluation
- **Absence of Bay State**: The coordination architecture has no abstraction for "station occupancy". A bay is treated as an ordinary traversable cell.
- **Absence of Staging Buffers**: There are no holding or queue-staging reservations where following robots can safely wait outside the active transit lane.
- **Corridor Blockage**: When a robot halts at the bay entrance cell $(29, 29)$, trailing robots tracking the same A* path yield in cell $(29, 30)$ directly behind the lead vehicle.
- **Architectural Conclusion**: The central hub failure is an unmanaged **shared-resource bottleneck** resulting from the collision of unpaged task demand, single-cell discrete reservations, and the absence of queue staging infrastructure.

---

### 15.5 Cross-Layer Causal Reconstruction

The failure develops across five distinct architectural layers:

```
[WORKLOAD & ALLOCATION LAYER]
9 tasks assigned to Bay (14.5, 14.5) without temporal release staging
Concurrent delivery demand created for amr_3 and amr_4
                         │
                         ▼
[PLANNING & GOAL LAYER - ENABLING CONDITION]
amr_3 path targets cell center (14.75, 14.75), exhausts waypoints at (14.75, 14.94)
Halts with v=0; distance to goal is 0.506m > 0.500m goal tolerance
amr_3 stalls stationary in arrival cell (29, 29) without completing task
                         │
                         ▼
[COORDINATION & RESERVATION LAYER - SYSTEMIC LIMITATION]
SpaceTimeReservationTable represents 0D points; models cell (29, 29) and (29, 30) as non-conflicting
amr_4 permitted to reserve adjacent cell (29, 30) directly behind amr_3
Grid resolution (0.50m) < Chassis length (0.65m) -> Inherent -0.15m bumper clearance
                         │
                         ▼
[SAFETY CONTROLLER LAYER - BACKSTOP LIMITATION]
amr_4 tracks south at 0.23 m/s towards cell (29, 30)
At d < 0.695m, LiDAR distance drops below range_min (0.15m), returning 'inf'
_handle_scan() discards rays; obstacle_ahead evaluates False; reactive brake inactive
                         │
                         ▼
[BENCHMARK AUDITOR LAYER - TRIGGER]
amr_4 compresses to y = 15.281m; center distance d = 0.341m
Breaches 0.350m threshold -> Benchmark auditor triggers SAFETY_ABORT (T = 149.34s)
```

---

### 15.6 Final Root-Cause Matrix

| Factor | Empirical Telemetry / Code Evidence | Scientific Classification | Causal Role in Failure |
| :--- | :--- | :---: | :--- |
| **Destination Convergence** | 9 of 30 tasks assigned to Bay $(14.5, 14.5)$; concurrent arrival of `amr_3` & `amr_4` | **CONFIRMED CONTRIBUTOR** | **Contributing Factor** |
| **Waypoint / Goal Mismatch** | `amr_3` stopped at $0.506\,\text{m}$ from goal ($> 0.500\,\text{m}$) due to path exhaustion at $(14.75, 14.94)$ | **CONFIRMED DESIGN MISMATCH** | **Enabling Condition** |
| **Reservation Footprint Absence** | `SpaceTimeReservationTable` stores 0D points; allowed adjacent cells `(29, 29)` & `(29, 30)` | **CONFIRMED CONTRIBUTOR** | **Systemic Limitation** |
| **Grid vs. Footprint Mismatch** | $\Delta = 0.50\,\text{m}$ vs $L = 0.65\,\text{m}$; nominal adjacent cells yield $-0.15\,\text{m}$ bumper clearance | **CONFIRMED CONTRIBUTOR** | **Systemic Limitation** |
| **Queue Tracking Dynamics** | `amr_4` tracked at $v \approx 0.23\,\text{m/s}$ toward cell 30 without continuous headway regulation | **CONFIRMED CONTRIBUTOR** | **Contributing Factor** |
| **LiDAR Minimum Range Blind Spot** | Sensor-to-bumper distance $< 0.15\,\text{m}$; returns `inf`; `obstacle_ahead` evaluated to `False` | **CONFIRMED CONTRIBUTOR** | **Safety-Backstop Limitation** |
| **Hub Approach Geometry** | Single-cell arrival chute along $X=29$; no parallel holding siding or bypass lane | **POSSIBLE CONTRIBUTOR** | **Contributing Factor** |
| **Communication Layer** | `NORMAL` profile (0% loss, 0 ms latency); continuous high-frequency status exchange | **NOT SUPPORTED BY EVIDENCE** | **Non-Factor (Excluded)** |
| **Center Separation Breach** | Measured center distance $d = 0.341\,\text{m} < 0.350\,\text{m}$ at $T = 149.34\,\text{s}$ | **CONFIRMED FACT** | **Immediate Abort Trigger** |

---

### 15.7 Remaining Uncertainties

1. **Exact Float Serialization of Sub-Minimum LiDAR Rays**:
   While static code analysis proves that `_handle_scan()` discards any reading $\le 0.15\,\text{m}$ or non-finite, whether Gazebo Harmonic outputs `+inf`, `-inf`, `nan`, or `0.0` over `ros_gz_bridge` remains unverified by live probe. (Controlled test designed in Section 15.1).
2. **DART Physics Engine Micro-Chatter**:
   Whether tire friction micro-slip contributed to `amr_3` coming to rest at $y = 14.940\,\text{m}$ ($0.190\,\text{m}$ from waypoint center $14.750\,\text{m}$).

---

### 15.8 Conceptual Remediation Directions Only (No Implementation)

*(Presented exclusively for subsequent human review; no code, configuration, or parameter changes are proposed or executed here)*

1. **Multi-Cell Footprint Envelope Reservation**:
   Upgrade the space-time reservation table to enforce a 2-cell longitudinal headway envelope ($k \ge 2$) for collinear convoying and queuing, guaranteeing nominal center distance $d \ge 1.000\,\text{m}$ and physical clearance $\ge 0.350\,\text{m}$.
2. **Harmonization of Waypoint Path Exhaustion and Task Completion**:
   Ensure that a robot's motion controller does not command $v=0$ upon waypoint exhaustion unless `check_subgoal_arrival()` is satisfied, or append the continuous station coordinates $(x_{\text{goal}}, y_{\text{goal}})$ as the explicit final waypoint.
3. **Continuous Safety-Brake Blind Spot Protection**:
   Widen the LiDAR emergency brake forward envelope or incorporate continuous odometric peer-distance tracking into local safety controller logic so vehicles cannot approach within $0.650\,\text{m}$ even if LiDAR rays enter the sub-$0.15\,\text{m}$ blind zone.
4. **Shared Station Capacity Management & Staging Buffers**:
   Introduce bay occupancy locks or holding queue waypoints so arriving AMRs stage in an open buffer zone rather than blocking active arrival corridors.

---
