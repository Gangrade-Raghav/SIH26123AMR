# NRDAS-FR // Fault Injection & Resilience Testing Dashboard

**Component**: Dedicated GUI Testbed & Observability Console  
**Port**: `8081` (Default)  
**Host**: `0.0.0.0` / `localhost`  
**Location**: `scripts/resilience_dashboard.py`  
**Test Suite**: `src/amr_fleet_core/test/test_resilience_dashboard.py`

---

## 1. Architectural Principles & System Decoupling

The Fault Injection & Resilience Testing Dashboard on **Port 8081** is a dedicated human-in-the-loop testbed created specifically for adversarial experimentation, fault injection, and recovery observability.

```
+-------------------------------------------------------------------------+
|                  HUMAN OPERATOR / BROWSER CLIENT                       |
|                       http://localhost:8081                             |
+------------------------------------+------------------------------------+
                                     | REST JSON / HTTP GET / POST
                                     v
+-------------------------------------------------------------------------+
|              RESILIENCE MONITOR & INJECTOR (Port 8081)                  |
|             (scripts/resilience_dashboard.py — Zero SPOF)               |
+------------------+----------------------------------+-------------------+
                   |                                  |
    ROS 2 Service  | /{robot_id}/inject_fault         | Telemetry Topics
    Calls (Clients)|                                  | (Subscribers)
                   v                                  v
+-------------------------------------------------------------------------+
|                 DECENTRALIZED AUTONOMOUS FLEET NODES                    |
|   amr_0                     amr_1                     amr_2             |
|   - FaultDetector           - FaultDetector           - FaultDetector   |
|   - CBBAAgent               - CBBAAgent               - CBBAAgent       |
|   - RHCR Planner            - RHCR Planner            - RHCR Planner    |
+-------------------------------------------------------------------------+
```

### Absolute Architectural Rule
> **Zero Recovery Logic in the Dashboard**:  
> The dashboard is strictly an **observability and injection console**. All failure detection, belief purging, task reclamation, obstacle insertion, and CBBA re-auctions occur purely inside the decentralized nodes (`fault_detector.py`, `rh_node.py`, `cbba_node.py`, `task_manager_node.py`). If the dashboard process is killed or closed, the fleet's autonomous recovery continues uninterrupted without degradation.

---

## 2. Key Dashboard Capabilities & Panels

### 2.1 Fleet Health & State Panel
- Displays live cards for all active AMRs (`amr_0`, `amr_1`, `amr_2`, ...).
- Explicit state indicators with distinctive color codes:
  - `HEALTHY`: Green badge (`#10b981`)
  - `COMM_LOSS`: Amber badge (`#f59e0b`)
  - `FAILED` / `KILL`: Red badge (`#ef4444`)
  - `ACTUATOR_FAIL`: Orange-Red badge
  - `NAVIGATION_STUCK`: Warning Yellow badge
  - `BATTERY_CRITICAL`: Deep Amber badge
  - `EMERGENCY_STOP`: Purple badge (`#8b5cf6`)
- Real-time telemetry: localized $(x, y)$ coordinates, yaw heading, linear speed, heartbeat counter, and age since last heartbeat ($t_{\text{now}} - t_{\text{last}}$).
- Active task ID and currently allocated CBBA bundle items.
- Quick action buttons per card: **Kill**, **Comm Cut**, **Restore**.

### 2.2 Active Fault Injection Console
- **Target AMR Selector**: Target any specific robot or broadcast to `ALL_ROBOTS`.
- **Fault Type Selector**:
  - `KILL`: Simulates node crash / fatal exception / SIGKILL.
  - `HEARTBEAT_TIMEOUT`: Freezes heartbeat publishing while keeping nodes alive.
  - `COMM_LOSS`: Simulates temporary network isolation / WiFi dropout.
  - `ACTUATOR_FAIL`: Simulates motor driver or gearbox failure.
  - `NAVIGATION_STUCK`: Simulates physical floor entanglement / wheel slip stall.
  - `BATTERY_CRITICAL`: Simulates sudden battery voltage drop.
  - `RESTORE`: Revives robot, resets local state, and notifies fleet of return.
  - `EMERGENCY_STOP`: Imposes immediate zero-velocity halt.
- **Duration Configuration**: Set transient duration in seconds (0.0 for permanent).
- **Fleet Emergency Stop**: Giant red **FLEET EMERGENCY STOP** button halts all robots instantly; **RESUME ALL** restores normal operations.

### 2.3 Interactive 2D Warehouse World & Chassis Obstacle Tracker
- Scalable vector rendering of warehouse layout, walls, storage racks, and pickup/dropoff stations.
- Real-time AMR markers showing heading direction, speed, and health halos.
- **Stranded Chassis Visualization**: Disabled robots are rendered with a prominent **red cross-hatched warning box** and circular keep-out radius ($0.8\text{m}$).
- Dynamic path overlays showing active AMRs rerouting around stranded chassis.

### 2.4 Decentralized 7-Stage Recovery Pipeline Tracker
Tracks the 7 canonical stages of NRDAS-FR autonomous recovery in real time:
1. **Stage 1 (Fault Injected)**: Fault command received and dispatched ($t_0$).
2. **Stage 2 (Peer Detected)**: Surviving peer confirms heartbeat silence ($t_{\text{detect}}$).
3. **Stage 3 (Beliefs Purged)**: Failed peer's bids wiped; spacetime reservations cleared ($t_{\text{purge}}$).
4. **Stage 4 (Task Reclaimed)**: CAS precondition check executes; task $\to$ `PENDING` ($t_{\text{reclaim}}$).
5. **Stage 5 (Obstacle Inserted)**: Stranded chassis registered in GridWorld obstacle layer ($t_{\text{obs}}$).
6. **Stage 6 (Task Reassigned)**: CBBA decentralized re-auction won by surviving peer ($t_{\text{assign}}$).
7. **Stage 7 (Execution Resumed)**: Winning peer resumes physical transit to task ($t_{\text{resume}}$).

Each stage displays measured delta $\Delta t$ from fault onset.

### 2.5 Safety & Research Invariants Panel
- **Zero Task Duplication**: Continuous invariant check verifying no duplicate task ownership across evaluated cases.
- **Failed Chassis Avoidance**: Live calculation of minimum clearance distance from moving AMRs to any stranded chassis ($d_{\text{min}} \ge 0.45\text{m}$).
- **Gazebo Safety Proxy**: Confirms zero collisions via 2D Oriented Bounding Box (OBB) geometric testing.
- **Provenance Disclosure**: Explicitly labels contact detection as an odometry-based geometric proxy rather than raw bumper physics.

### 2.6 M1 Validation Scenarios Quick-Triggers
One-click execution of pre-configured research scenarios:
- `M1-A`: Single Robot Hard Kill during Task Execution
- `M1-B`: Heartbeat Timeout during Narrow Corridor Transit
- `M1-C`: Simultaneous Dual Detection & Re-auction Race
- `M1-D`: Actuator Failure with Stranded Chassis Avoidance
- `M1-E`: Operator Restoration & Re-integration into Fleet
- `M1-F`: Comm Loss vs Node Kill Distinction

### 2.7 Rolling Event Log & Data Export
- Live stream of all fault events, state transitions, and CAS operations.
- One-click export buttons:
  - **Export JSON**: Complete structured JSON telemetry log for external scripts.
  - **Export Markdown**: Formatted research report ready for inclusion in publications.

### 2.8 Network Impairment Console (Milestone 2 Extension)
- Comprehensive communication degradation modeling for adversarial network evaluation.
- Configurable preset profiles: `NORMAL`, `LOW_LATENCY`, `HIGH_LATENCY`, `JITTER`, `LOSS_LOW`, `LOSS_HIGH`, `BURST_LOSS`, `OUTAGE`, `OUTAGE_RECOVERY`, `PARTITION`.
- Parametric sliders:
  - **Latency**: 0 to 1000 ms
  - **Jitter**: 0 to 200 ms
  - **Packet Loss**: 0 to 100%
  - **Burst Loss Probability**: 0 to 100%
  - **Network Partition**: Select isolated robots (e.g. `amr_2` or `amr_3, amr_4`)
- Target AMR selector: target individual robot or fleet broadcast.

### 2.9 Dual-Mode Recovery Pipeline Stepper (M1 vs M2)
The recovery stepper dynamically adapts based on the active test mode:
- **M1 Mode (7 Stages)**: Robot Failure & Mid-Task Recovery (`FAULT_INJECTED` $\to$ `PEER_DETECTED` $\to$ `BELIEFS_PURGED` $\to$ `TASK_RECLAIMED` $\to$ `OBSTACLE_INSERTED` $\to$ `TASK_REASSIGNED` $\to$ `EXECUTION_RESUMED`).
- **M2 Mode (9 Stages)**: Network Failure, Local Autonomy & Reconnection:
  1. `NOMINAL`: Full fleet connectivity and normal coordination.
  2. `COMM_LOSS_DETECTED`: Heartbeat silence exceeds $\tau_{\text{loss}} = 1.0\text{s}$.
  3. `LOCAL_AUTONOMY_ACTIVE`: AMR continues on reserved path ($v \le 0.4\text{ m/s}$) with LiDAR active.
  4. `RESERVATION_EXPIRY_HOLD`: Space-time reservation expires; AMR enters `LOCAL_SAFETY_HOLD` ($v = 0.0\text{ m/s}$).
  5. `PEER_FAILURE_DEBOUNCE`: Outage exceeds $\tau_{\text{fail}} = 3.5\text{s}$; peer fleet confirms `FAILED`.
  6. `AUTONOMOUS_RECLAIM`: Peer fleet purges beliefs and executes CAS task re-auction.
  7. `NETWORK_RESTORED`: Communication link reconnects.
  8. `RECONCILIATION_YIELD`: Reconnected AMR reconciles timestamps and yields task without duplication.
  9. `FLEET_RECONVERGED`: Fleet state converged; zero duplicate ownership ($I_{\text{uniq}}$).

### 2.10 Live Network Telemetry & Degradation Invariants
- Live network telemetry table tracking:
  - Per-robot network status: `ONLINE`, `DEGRADED`, `COMM_LOSS`, `OFFLINE`.
  - Configured loss vs. Empirical measured loss rate ($\text{dropped}/\text{sent}$).
  - Messages sent, delivered, and dropped.
  - P50 latency, P95 latency, and jitter.
- M2 Research Invariants live verification:
  - `false_failure_rejection`: Transient outages ($\le 3.5\text{s}$) strictly reject failure declarations.
  - `reconnection_zero_duplication`: Invariant $I_{\text{uniq}}$ preserved upon reconnection.
  - `reservation_hold_on_expiry`: AMR stops immediately when space-time reservations expire.

### 2.11 M1 & M2 Validation Quick-Triggers
- **M1 Scenarios**: `M1-A` through `M1-F` (Robot Fail-Stop, Corridor Transit, Re-auction Race, Chassis Avoidance, Operator Restoration, Comm Loss vs Kill).
- **M2 Scenarios**: `M2-A` through `M2-G` and `M2-B2` (Short Outage, Outage in Assigned, Active Nav COMM_LOSS, Outage in In-Progress, Long Outage Reclaim, Reconnection Reconciliation, 35% Packet Loss, Network Partition; evaluated at `INTEGRATION / SIMULATION` execution level).

### 2.12 Environmental & Adversarial Resilience Console (Milestone 3 Extension)
- **Dynamic Aisle Blockage Controller**:
  - Live injection and clearance of single or multi-cell corridor obstructions.
  - Configurable duration (transient or persistent).
  - Visualized on 2D warehouse map as high-visibility hatched hazard zones.
- **Adversarial Trajectory & Reservation Conflict Injector**:
  - Direct execution of non-corrupting synthetic conflict conditions: `SAME_CELL`, `OPPOSING_CORRIDOR`, `CROSSING_TRAJECTORIES`, `RESERVATION_CONFLICT`, `HIGH_CONTENTION_INTERSECTION`.
  - Real-time logging of conflict detection and PIBT fallback invocations.
- **Tri-Mode Recovery Pipeline Stepper (M1 vs M2 vs M3)**:
  - Supports stepping through the 9 discrete stages of environmental resilience:
    1. `STAGE_1_OBSTACLE_INJECTED`: Blockage or conflict introduced into simulation.
    2. `STAGE_2_SENSOR_OBSERVED`: Onboard LiDAR detects obstacle ($d \le 1.5\text{m}$).
    3. `STAGE_3_LOCAL_SAFETY_EVALUATED`: Safety hold evaluated; reactive braking ($0.28\text{m}$) armed.
    4. `STAGE_4_CANDIDATE_CLEARANCE_CHECKED`: Footprint ($0.65\text{m} \times 0.45\text{m}$) and reservations verified.
    5. `STAGE_5_GRAPH_WITHDRAWN`: Cells removed from local and shared traversability graph.
    6. `STAGE_6_RESERVATIONS_INVALIDATED`: Active reservations for obstructed cells revoked.
    7. `STAGE_7_REPLAN_COMPLETED`: Single-agent A* computes valid collision-free detour.
    8. `STAGE_8_PIBT_FALLBACK_RESOLVED`: Multi-agent vertex/edge conflicts resolved via PIBT push-and-yield.
    9. `STAGE_9_EXECUTION_RESUMED`: Motion execution resumes along detour; 0 geometric overlaps.
- **M3 Validation Quick-Triggers**:
  - Interactive execution of all 9 M3 scenarios: `M3-A`, `M3-B`, `M3-B2`, `M3-C1`, `M3-C2`, `M3-C3`, `M3-C4`, `M3-C5`, `M3-H`.

---

## 3. REST API Reference

| Method | Endpoint | Description | Payload Example |
| :--- | :--- | :--- | :--- |
| `GET` | `/` | Web console single-page interface | None |
| `GET` | `/api/state` | Full fleet health, telemetry, invariants, and logs | None |
| `POST` | `/api/fault/inject` | Inject simulated robot fault | `{"robot_id": "amr_1", "fault_type": "KILL", "duration_sec": 0.0}` |
| `POST` | `/api/fault/restore` | Restore robot to healthy state | `{"robot_id": "amr_1"}` |
| `POST` | `/api/network/impairment` | Apply network impairment profile | `{"profile_name": "LOSS_HIGH", "loss_probability": 0.35, "latency_ms": 50}` |
| `POST` | `/api/network/reconnect` | Reconnect robot or restore normal network | `{"robot_id": "amr_1"}` |
| `POST` | `/api/environment/blockage` | Inject or clear dynamic aisle blockage | `{"action": "INJECT", "blockage_id": "BLK_01", "cells": [[7, 4], [7, 5]], "duration_sec": 0.0}` |
| `POST` | `/api/environment/conflict` | Inject adversarial contention condition | `{"conflict_type": "CROSSING_TRAJECTORIES", "robot_ids": ["amr_0", "amr_1"], "cell": [4, 5], "time_step": 3}` |
| `POST` | `/api/environment/step` | Advance M3 recovery pipeline stepper | `{"stage_name": "01_OBSTACLE_DETECTED"}` |
| `POST` | `/api/fleet/estop` | Emergency stop all AMRs | None |
| `POST` | `/api/fleet/resume` | Resume all AMRs to normal operation | None |
| `POST` | `/api/scenario/trigger` | Trigger automated validation scenario | `{"scenario_id": "M3-A"}` |
| `GET` | `/api/export` | Download JSON experiment report | None |
| `GET` | `/api/export/markdown` | Download Markdown experiment report | None |

---

## 4. How to Launch and Use

### Launching the Dashboard
```bash
# Sourcing environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch live dashboard on default port 8081
python3 scripts/resilience_dashboard.py --port 8081

# Or launch in autonomous simulation testbed mode (runs without live Gazebo)
python3 scripts/resilience_dashboard.py --port 8081 --sim-mode
```

### Accessing the Web UI
Open any modern web browser to:
```
http://localhost:8081
```
*(No external npm, node_modules, or build steps required; zero dependencies).*
