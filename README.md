# NRDAS: Decentralized Autonomous Mobile Robot (AMR) Fleet Coordination

> **A Research-Grade Decentralized Coordination, Planning, and Resilience Architecture for Multi-AMR Warehouse Intralogistics.**  
> Built with **ROS 2 Jazzy**, **Gazebo Sim 8.11.0 (Harmonic)**, **DART Physics**, and **Python 3.12**.

---

## 1. What Is This?

This repository contains the complete implementation, benchmark infrastructure, and empirical research findings for a **fully decentralized multi-robot coordination system** designed for Autonomous Mobile Robots (AMRs) operating in congested industrial warehouses.

Unlike conventional warehouse automation that relies on centralized dispatchers, this architecture distributes task bidding, multi-agent pathfinding, spatio-temporal corridor reservations, and emergency braking across onboard AMR compute nodes. The system is resilient against wireless packet loss, dynamic corridor obstructions, and unexpected demand spikes.

---

## 2. What Research Question Does It Address?

> **Central Research Question**:  
> *"Can a fully decentralized multi-AMR fleet maintain safe, deadlock-free, and coherent operation in a congested warehouse layout when subjected to simultaneous dynamic task arrivals, physical corridor blockages, and wireless communication degradation?"*

Across 8 experimental milestones culminating in Milestone **M9-V3-E**, this system demonstrates that hierarchical decoupling:
$$\text{LOCAL SAFETY} \succ \text{RESERVATION / PIBT COORDINATION} \succ \text{RHCR PLANNING} \succ \text{CBBA TASK ALLOCATION}$$
empirically prevents collisions and deadlocks under severe concurrent operational disruptions.

---

## 3. What Has Been Implemented?

1. **Decentralized Task Allocation (CBBA / ACBBA)**:
   - Asynchronous Consensus-Based Bundle Algorithm allocating pick-and-place tasks with Diminishing Marginal Utility (DMU).
   - Mid-mission dynamic bundle expansion ($t=45\,\text{s}$) without preemption or disruption of in-progress tasks.
2. **Rolling-Horizon Collision Resolution (RHCR)**:
   - Windowed spatio-temporal A* search over a 10-step horizon ($h=10$), committing only 4 steps ($w=4$) at $2.0\,\text{Hz}$.
   - Station-resource queuing and headway corridor reservations eliminating gridlock at delivery hubs.
3. **Space-Time Reservations & PIBT Local Coordination**:
   - Spatio-temporal occupancy claims ($[x, y, t_{\text{start}}, t_{\text{end}}]$) exchanged peer-to-peer.
   - Priority Inheritance Backtracking (PIBT) 1-step local avoidance resolving head-on and crossing corridor conflicts.
4. **Wait-For Graph (WFG) Deadlock Detection**:
   - Directed cycle detection across claimed space-time resources with deterministic evasion yielding.
5. **Dynamic Environmental Adaptation**:
   - Gazebo Harmonic physical obstacle spawning/removal ($1.8\times0.6\times1.4\,\text{m}$ blocker at Aisle 1 South).
   - Real-time local `GridWorld` rasterization updating 12 obstacle cells within $150\,\text{ms}$.
6. **Communication Degradation Resilience**:
   - Empirical impairment model simulating latency, jitter, burst outages, and uniform packet drop ($p_{\text{loss}} = 0.35$).
   - Autonomous time-to-live (TTL) reservation pruning preventing ghost corridor reservations.
7. **Adaptive Compute Policy**:
   - Cross-cutting telemetry-driven policy modulating planning rates ($1\,\text{Hz}$ to $4\,\text{Hz}$) based on host load and network health.
8. **Decoupled LiDAR Safety Backstop**:
   - Independent $10\,\text{Hz}$ reactive braking controller directly monitoring 2D LaserScan with absolute stopping authority ($<0.35\,\text{m}$ buffer).
9. **Observability & Visual Dashboard**:
   - Zero-dependency web dashboard (`http://localhost:8080`) with SVG mini-map and rich terminal TUI.
10. **Operator Task Allocation & Live Task Control**:
   - Live ROS 2 service layer (`/tasks/create`, `/tasks/control`, `/tasks/cancel`, `/tasks/requeue`).
   - Dynamic task creation with bounds checking ($[0.0, 30.0]\,\text{m}$), duplicate detection, and sequential `T###` generation.
   - Dual dispatch modes: **AUTO** (decentralized CBBA auction) and **DIRECT** (specific AMR target constraint).
   - In-flight task cancellation and failure requeuing with clean bundle purging.
   - Web console modal and tactical controls with real-time CBBA Bid Inspector.
   - Standalone operator CLI tool: `scripts/create_task.py`.

---

## 4. How Do I Build It?

### Prerequisites
- Ubuntu 24.04 LTS (x86_64)
- ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`)
- Gazebo Sim 8.11.0 (Harmonic)

### Build Sequence
```bash
# Clone and enter workspace root
cd .

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build the 5 packages
colcon build --symlink-install

# Source workspace overlays
source install/setup.bash
```

### Run Regression Test Suite
```bash
colcon test && colcon test-result --verbose
```
*Result*: **216 tests passed, 0 failures, 0 errors, 0 skipped**.

---

## 5. How Do I Run the Live Demos?

### Normal Visual Demonstration (5 AMRs, `warehouse_small`)
```bash
# Terminal 1 — Gazebo Simulation & AMR Fleet
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=5 world:=warehouse_small headless:=false \
  compute_mode:=ADAPTIVE comm_profile:=NORMAL workload_file:=config/workloads/workload_15_tasks.yaml

# Terminal 2 — 3D RViz Visualization
rviz2 -d $(ros2 pkg prefix amr_fleet_bringup)/share/amr_fleet_bringup/rviz/fleet_default.rviz

# Terminal 3 — Live Observability Dashboard
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 5 --world warehouse_small --tui
# Open browser at: http://localhost:8080
```

---

## 6. How Do I Run the 10-AMR Congested Fleet Demo?

```bash
# Terminal 1 — Launch 10 AMRs in the 32m x 32m Congested Warehouse
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=10 world:=warehouse_m9_v2 headless:=false \
  compute_mode:=ADAPTIVE comm_profile:=NORMAL workload_file:=config/workloads/workload_30_tasks_m9_v2.yaml

# Terminal 2 — 10-AMR Observability Dashboard
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 10 --world warehouse_m9_v2 --tui
# Open browser at: http://localhost:8080
```

---

## 7. How Do I Use Operator Task Allocation & Live Control?

Operators can inject tasks dynamically or control in-flight missions using either the Web Dashboard or the CLI tool.

### Option A: Via Web Dashboard (`http://localhost:8080`)
1. Click the **`+ CREATE TASK`** button in the header navigation bar.
2. Enter pickup and dropoff coordinates ($[0.0, 30.0]\,\text{m}$) and select Priority (`NORMAL`, `HIGH`, `LOW`).
3. Choose Allocation Mode:
   - **AUTO**: Enters the decentralized CBBA auction pool; the fleet dynamically outbids and converges.
   - **DIRECT**: Selects a specific target AMR (e.g. `amr_3`); only the designated AMR bids on it.
4. Use the **CANCEL** or **REQUEUE** action buttons directly on rows in the live task table.
5. Click any task row to inspect multi-robot bids, winning bid values, and consensus status in the **CBBA Bid Inspector**.

### Option B: Via Operator CLI Tool (`scripts/create_task.py`)
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Dispatch an AUTO task for decentralized CBBA auction
python3 scripts/create_task.py \
  --pickup-x 2.5 --pickup-y 4.0 \
  --dropoff-x 14.0 --dropoff-y 12.0 \
  --priority HIGH

# Dispatch a DIRECT task constrained to amr_2
python3 scripts/create_task.py \
  --pickup-x 8.0 --pickup-y 3.0 \
  --dropoff-x 16.0 --dropoff-y 6.0 \
  --priority NORMAL \
  --robot amr_2
```
For complete details, see [`docs/TASK_ALLOCATION_GUIDE.md`](docs/TASK_ALLOCATION_GUIDE.md).

---

## 8. Key Documentation Links

| Document | File Path | Purpose |
| :--- | :--- | :--- |
| **Task Allocation Operator Guide** | [`docs/TASK_ALLOCATION_GUIDE.md`](docs/TASK_ALLOCATION_GUIDE.md) | Operator guide for live task creation, CLI, and dashboard controls |
| **Final Operator Runbook** | [`docs/FINAL_RUNBOOK.md`](docs/FINAL_RUNBOOK.md) | Terminal-by-terminal commands for build, demo, inspection, and shutdown |
| **Command Cheat Sheet** | [`docs/COMMAND_CHEATSHEET.md`](docs/COMMAND_CHEATSHEET.md) | Quick-reference copy-pasteable commands |
| **Final Research Report** | [`docs/FINAL_RESEARCH_REPORT.md`](docs/FINAL_RESEARCH_REPORT.md) | 27-section comprehensive academic research paper |
| **System Architecture** | [`docs/FINAL_ARCHITECTURE.md`](docs/FINAL_ARCHITECTURE.md) | Architectural hierarchy, authority models, and preemption rules |
| **Experiment Matrix** | [`results/final/FINAL_EXPERIMENT_MATRIX.md`](results/final/FINAL_EXPERIMENT_MATRIX.md) | Empirical cross-milestone benchmark table (M7 through M9-V3-E) |
| **Presentation & Demo Script** | [`docs/DEMO_SCRIPT.md`](docs/DEMO_SCRIPT.md) | 30s, 2m, and 5m demonstration scripts with judge Q&A |
| **Definition of Done** | [`docs/FINAL_DEFINITION_OF_DONE.md`](docs/FINAL_DEFINITION_OF_DONE.md) | Complete engineering and research sign-off checklist |

---

## 9. Where Are the Results & Experimental Artifacts?

All empirical artifacts are archived and structured under [`results/final/`](results/final/):

- **Canonical Raw Trial Telemetry**: `results/final/canonical/` (e.g. `m9_v3_e_canonical_raw.json`)
- **Aggregated Summaries**: `results/final/aggregated/` (e.g. `m9_v3_e_aggregated.json`)
- **Publication-Grade Figures**: `results/final/figures/` (PNG diagrams generated from real data)
- **Extracted CSV Tables**: `results/final/tables/` (`m9_scenarios_comparison.csv`)

### Summary of Final Canonical Experiment (M9-V3-E Combined Stress, 180s Horizon)
- **Fleet & World**: 10 AMRs, `warehouse_m9_v2` ($32\,\text{m} \times 32\,\text{m}$, 4-bay central hub)
- **Physical Blockage**: Aisle 1 South blocked from $t=45\,\text{s}$ to $90\,\text{s}$ (12 cells updated in local grids)
- **Communication Impairment**: $35\%$ packet drop from $t=75\,\text{s}$ to $120\,\text{s}$ (5,642 packets dropped)
- **Dynamic Task Arrival**: 15 tasks injected at $t=45\,\text{s}$ (CBBA converged in $167.6\,\text{ms}$)
- **Concurrent Stress Window ($t\in[75, 90]\,\text{s}$)**: Simultaneous physical corridor obstruction and 35% packet loss
- **Gazebo Physical Contacts**: **0** (Physics contact sensor ground truth)
- **OBB Chassis Overlaps**: **0** (Geometric SAT proxy)
- **Safety Aborts & Deadlocks**: **0 / 0**
- **Minimum Distance**: **$5.45\,\text{m}$** (Safety threshold $\ge 0.35\,\text{m}$)
- **Task Accounting Invariant**: $\sum S_i = 30 = 0\text{ Staged} + 0\text{ Pending} + 28\text{ Assigned} + 2\text{ In-Progress} + 0\text{ Completed} + 0\text{ Failed} + 0\text{ Cancelled}$

---

## 10. Limitations & Scientific Scope

1. **Deterministic Single-Run Benchmarks ($n=1$)**:
   The canonical benchmarks were evaluated with fixed pseudo-random seed 42 to establish bitwise reproducible baseline timelines. They do not constitute broad statistical distributions or claims across unseed variance.
2. **Safety Ground Truth vs. Proxies**:
   `OBB chassis-overlap samples` are a 2D geometric SAT bounding-box proxy. True collision ground truth is strictly derived from Gazebo DART physics engine contact sensors.
3. **Horizon vs. Task Throughput**:
   In the $1024\,\text{m}^2$ warehouse, travel times between perimeter racks and the central delivery hub exceed $120\,\text{s}$ under calibrated differential-drive limits ($0.5\,\text{m/s}$). Tasks in progress at $180\,\text{s}$ are explicitly accounted as `IN_PROGRESS` rather than `COMPLETED`.
