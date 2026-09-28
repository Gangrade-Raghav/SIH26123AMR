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
11. **NRDAS-FR Fault Injection & Resilience Testing System**:
   - Dedicated human-in-the-loop web console (`http://localhost:8081`) operating as an Adversarial Experiment Controller (`scripts/resilience_dashboard.py`).
   - Zero-SPOF decoupled architecture: autonomous recovery logic lives entirely on decentralized AMR nodes; the console strictly injects perturbations and monitors invariants.
12. **Milestone 4 Compound Fault & Multi-Domain Stress Testing**:
   - Simultaneous composition across 3 orthogonal disturbance domains: Robot Faults (`KILL`, `COMM_LOSS`, `ACTUATOR_FAIL`), Network Impairments (`LOSS_HIGH` 35%, `OUTAGE`, `PARTITION`), and Environmental Blockages (corridor obstacles, choke points).
   - Live **Three-Tier Fleet Response Telemetry**: Tier 1 (CBBA Reallocation via atomic CAS), Tier 2 (Dynamic Spacetime Replanning via SingleAgentAStar / RHCR), Tier 3 (Local Safety & $0.28\,\text{m}$ LiDAR envelope).
   - Continuous formal invariant verification: $I_1$ (Task Uniqueness), $I_2$ (Spacetime Reservation Exclusivity), $I_3$ (Local Clearance $\ge 0.28\,\text{m}$), Non-Blocking Tiered Fault Discrimination ($T_{\text{transient}} \le 1.5\,\text{s} \ll T_{\text{fail}} = 3.5\,\text{s}$), and Monotonic CAS Reconnection.
   - 7 automated adversarial benchmark scenarios (`M4-A` through `M4-G`) with 7-stage recovery pipeline stepper and structured JSON/Markdown export.

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

### Run Full Test Suites
```bash
# Core regression test suite (216 tests)
colcon test && colcon test-result --verbose

# Dedicated NRDAS-FR Resilience test suite (79 unit & integration tests)
export ROS_DOMAIN_ID=42
python3 -m pytest \
  src/amr_fleet_core/test/test_fault_resilience.py \
  src/amr_fleet_core/test/test_m2_network_resilience.py \
  src/amr_fleet_core/test/test_m3_adversarial_resilience.py \
  src/amr_fleet_core/test/test_m4_compound_resilience.py \
  src/amr_fleet_core/test/test_resilience_dashboard.py -v

# Milestone 4 Compound Scenarios Validation Harness (7/7 Benchmark Scenarios)
python3 scripts/validate_m4_compound_scenarios.py
```
*Result*: **295 total tests passed, 0 failures, 0 errors**.

---

## 5. Master Launch (All-in-One Single Command)

To launch the complete system—**Gazebo Harmonic 3D GUI**, **RViz 2**, **Fleet Autonomy Stack**, **Fleet Observability Dashboard (Port 8080)**, and the **M4 Adversarial Resilience Dashboard (Port 8081)**—in a single terminal with clean `Ctrl+C` process teardown and ROS domain isolation:

```bash
cd /home/raghav/Downloads/SIH26123AMR
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Master launch (10 AMRs, Big Congested Warehouse, 30-task schedule):
./scripts/launch_all_in_one.sh

# Or headless mode (low GPU/CPU overhead):
./scripts/launch_all_in_one.sh --headless
```

- **Fleet Observability & Task Console**: [http://localhost:8080](http://localhost:8080)
- **M4 Adversarial Resilience Console**: [http://localhost:8081](http://localhost:8081)

---

## 6. How Do I Run the Live Demos Individually?

### Normal Visual Demonstration (5 AMRs, `warehouse_small`)
```bash
# Terminal 1 — Gazebo Simulation & AMR Fleet
export ROS_DOMAIN_ID=42
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=5 world:=warehouse_small headless:=false \
  compute_mode:=ADAPTIVE comm_profile:=NORMAL workload_file:=config/workloads/workload_15_tasks.yaml

# Terminal 2 — 3D RViz Visualization
export ROS_DOMAIN_ID=42
rviz2 -d $(ros2 pkg prefix amr_fleet_bringup)/share/amr_fleet_bringup/rviz/fleet_default.rviz

# Terminal 3 — Live Observability Dashboard
export ROS_DOMAIN_ID=42
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 5 --world warehouse_small --tui
# Open browser at: http://localhost:8080
```

---

## 7. How Do I Run the 10-AMR Congested Fleet Demo?

```bash
# Terminal 1 — Launch 10 AMRs in the 32m x 32m Congested Warehouse
export ROS_DOMAIN_ID=42
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=10 world:=warehouse_m9_v2 headless:=false \
  compute_mode:=ADAPTIVE comm_profile:=NORMAL workload_file:=config/workloads/workload_30_tasks_m9_v2.yaml

# Terminal 2 — 10-AMR Observability Dashboard
export ROS_DOMAIN_ID=42
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 10 --world warehouse_m9_v2 --tui
# Open browser at: http://localhost:8080
```

---

## 8. How Do I Run the M4 Compound Resilience Testing System?

The **Milestone 4 (M4)** testbed allows operators to inject compound, multi-domain disturbances (simultaneous robot failure + network degradation + corridor obstruction) and observe the fleet's decentralized response.

### Option A: Standalone Simulation Mode (No Gazebo Needed)
```bash
cd /home/raghav/Downloads/SIH26123AMR
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/resilience_dashboard.py --sim-mode --port 8081
# Open browser at: http://localhost:8081
```

### Option B: Benchmark Scenario Validation Harness
```bash
python3 scripts/validate_m4_compound_scenarios.py
```
Validates the 7 core M4 benchmarks:
1. **M4-A**: Robot Failure + Dynamic Blockage (Dual Obstacle Corridor Detour)
2. **M4-B**: Robot Failure + Comm Loss Discrimination (1.5s vs 3.5s; 0 false failures)
3. **M4-C**: Multiple Overlapping Robot Failures (Staggered crashes; atomic CAS task reclamation)
4. **M4-D**: Robot Failure + Sensor-Visible Obstacle (Decoupled 0.9m perception vs 0.22m reactive brake)
5. **M4-E**: Network Partition + Robot Failure (Monotonic Lamport timestamp reconciliation)
6. **M4-F**: Network Loss + Dynamic Blockage (Safe local hold upon dynamic obstruction)
7. **M4-G**: Master Compound Quad Failure (2 crashes + 50% loss + corridor blockage)

---

## 9. How Do I Use Operator Task Allocation & Live Control?

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

## 10. Key Documentation Links

| Document | File Path | Purpose |
| :--- | :--- | :--- |
| **M4 Compound Resilience Guide** | [`docs/NRDAS_FR_M4_COMPOUND_RESILIENCE.md`](docs/NRDAS_FR_M4_COMPOUND_RESILIENCE.md) | M4 compound disturbance formalization, mathematical invariants, and validation |
| **Resilience Dashboard Spec** | [`docs/NRDAS_FR_FAULT_DASHBOARD.md`](docs/NRDAS_FR_FAULT_DASHBOARD.md) | Dedicated testbed documentation, REST API, and recovery pipeline stepper |
| **M4 Validation Evidence** | [`docs/evidence/m4_compound_validation.md`](docs/evidence/m4_compound_validation.md) | Canonical benchmark execution results and invariant verification logs |
| **Task Allocation Operator Guide** | [`docs/TASK_ALLOCATION_GUIDE.md`](docs/TASK_ALLOCATION_GUIDE.md) | Operator guide for live task creation, CLI, and dashboard controls |
| **Final Operator Runbook** | [`docs/FINAL_RUNBOOK.md`](docs/FINAL_RUNBOOK.md) | Terminal-by-terminal commands for build, demo, inspection, and shutdown |
| **Command Cheat Sheet** | [`docs/COMMAND_CHEATSHEET.md`](docs/COMMAND_CHEATSHEET.md) | Quick-reference copy-pasteable commands |
| **Final Research Report** | [`docs/FINAL_RESEARCH_REPORT.md`](docs/FINAL_RESEARCH_REPORT.md) | 27-section comprehensive academic research paper |
| **System Architecture** | [`docs/FINAL_ARCHITECTURE.md`](docs/FINAL_ARCHITECTURE.md) | Architectural hierarchy, authority models, and preemption rules |
| **Mathematical Specification** | [`docs/MATHEMATICAL_AND_ALGORITHMIC_SPECIFICATION.md`](docs/MATHEMATICAL_AND_ALGORITHMIC_SPECIFICATION.md) | Core derivations, kinematics, CBBA, RHCR, PIBT, and invariants |
| **Experiment Matrix** | [`results/final/FINAL_EXPERIMENT_MATRIX.md`](results/final/FINAL_EXPERIMENT_MATRIX.md) | Empirical cross-milestone benchmark table (M7 through M9-V3-E) |
| **Presentation & Demo Script** | [`docs/DEMO_SCRIPT.md`](docs/DEMO_SCRIPT.md) | Demonstration sequences and technically honest judge Q&A |
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
