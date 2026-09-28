# NRDAS AMR Fleet Coordination: Final Operator Runbook

> **Audience**: Systems engineers, research evaluators, and operators reproducing or demonstrating the multi-AMR coordination system.  
> **Environment**: Ubuntu 24.04 / Linux, ROS 2 Jazzy, Gazebo Sim 8.11.0 (Harmonic), Python 3.12.

---

## SECTION A — Clean Environment Start & Process Cleanup

Before starting any simulation or test, ensure no orphaned ROS 2 daemons or Gazebo processes occupy ports or system resources.

### Safe Process Cleanup Command
Execute in any terminal (targeted to AMR fleet processes, preserving any other user ROS projects):
```bash
pkill -9 -f "amr_fleet" 2>/dev/null || true
pkill -9 -f "warehouse_m9" 2>/dev/null || true
pkill -9 -f "fleet_dashboard" 2>/dev/null || true
pkill -9 -f "resilience_dashboard" 2>/dev/null || true
killall -9 parameter_bridge task_manager cbba_node rh_node warehouse_visualizer 2>/dev/null || true
rm -f /dev/shm/sem.fastrtps* /dev/shm/fastrtps* 2>/dev/null || true
sleep 1
ps aux | grep -E 'amr_fleet|warehouse_m9|fleet_dashboard|resilience_dashboard' | grep -v grep || echo "AMR processes all clean!"
```

### Environment Setup Verification & Domain Isolation
In every terminal used for this project, source the ROS 2 and workspace underlays and export the isolated domain ID:
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
```

Verify the environment:
```bash
echo "ROS_DISTRO: $ROS_DISTRO"
gz sim --version
python3 --version
```
*Expected Output*:
- `ROS_DISTRO: jazzy`
- `Gazebo Sim, version 8.11.0`
- `Python 3.12.x`

---

## SECTION B — Clean Workspace Build Sequence

To compile the 5 core packages from source with symlink installation:

### Terminal 1 — Build Sequence
```bash
cd .
source /opt/ros/jazzy/setup.bash

# Clean build
colcon build --symlink-install

# Source the newly generated workspace overlays
source install/setup.bash
```
*Expected Output*:
```
Summary: 5 packages finished [~2.5s]
```

### Verification of Build Packages
```bash
ros2 pkg list | grep amr_fleet
```
*Expected Output*:
- `amr_fleet_bringup`
- `amr_fleet_core`
- `amr_fleet_description`
- `amr_fleet_msgs`
- `amr_fleet_sim`

---

## SECTION C — Normal Visual Demonstration (5-AMR Baseline)

This demonstration showcases 5 AMRs operating in the baseline warehouse (`warehouse_small`, $16\,\text{m} \times 16\,\text{m}$) with real-time 3D Gazebo rendering, RViz visualization, and the interactive web dashboard.

### TERMINAL 1 — Gazebo Simulation & AMR Fleet Bringup
```bash
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch 5 AMRs in warehouse_small with GUI enabled
ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=5 \
  world:=warehouse_small \
  headless:=false \
  compute_mode:=ADAPTIVE \
  comm_profile:=NORMAL \
  workload_file:=config/workloads/workload_15_tasks.yaml
```
- **Expected Visible Result**: Gazebo Harmonic 3D window opens showing the $16\,\text{m} \times 16\,\text{m}$ warehouse with storage racks and 5 spawned differential-drive AMRs (`amr_0` through `amr_4`).
- **How to Terminate**: Press `Ctrl+C` in Terminal 1.

---

### TERMINAL 2 — 3D Fleet RViz Visualization
*(Wait until Terminal 1 prints `[amr_task_manager]: Task Manager Node initialized`)*
```bash
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch RViz with preconfigured multi-robot display
rviz2 -d $(ros2 pkg prefix amr_fleet_bringup)/share/amr_fleet_bringup/rviz/fleet_default.rviz
```
- **Expected Visible Result**: RViz window opens displaying TF frames (`map`, `amr_i/odom`, `amr_i/base_footprint`), LiDAR laser scans (`/amr_i/scan`), and warehouse footprint markers.
- **How to Terminate**: Press `Ctrl+C` in Terminal 2.

---

### TERMINAL 3 — Fleet Observability Web Dashboard & TUI
```bash
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch live dashboard on port 8080 with Rich TUI
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 5 --world warehouse_small --tui
```
- **Expected Visible Result**: Terminal 3 renders a Rich TUI dashboard showing per-robot battery, odometry, task bundle, and system CPU/RAM. Open browser at `http://localhost:8080` to view the 2D SVG warehouse mini-map with real-time AMR pose updates.
- **How to Terminate**: Press `Ctrl+C` in Terminal 3.

---

### TERMINAL 4 — Fleet Presentation Trajectory Demo (Optional)
*(If running the deterministic multi-stage waypoint presentation coordinator)*
```bash
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Execute coordinated waypoint navigation across the 5 AMRs
python3 scripts/run_fleet_demo.py --robot-count 5 --loop
```
- **Expected Visible Result**: AMRs begin executing smooth waypoint loops through the warehouse aisles in both Gazebo and RViz without inter-robot collisions.
- **How to Terminate**: Press `Ctrl+C` in Terminal 4.

---

## SECTION D — 10-AMR Congested Fleet Demonstration

This demonstration launches the full 10-AMR fleet in the expanded congested warehouse (`warehouse_m9_v2`, $32\,\text{m} \times 32\,\text{m}$) evaluating decentralized CBBA allocation, rolling-horizon planning, and station-resource reservation.

### TERMINAL 1 — 10-AMR Congested Warehouse Simulation
```bash
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=10 \
  world:=warehouse_m9_v2 \
  headless:=false \
  compute_mode:=ADAPTIVE \
  comm_profile:=NORMAL \
  workload_file:=config/workloads/workload_30_tasks_m9_v2.yaml
```
- **Robot Namespaces**: `amr_0`, `amr_1`, `amr_2`, `amr_3`, `amr_4`, `amr_5`, `amr_6`, `amr_7`, `amr_8`, `amr_9`.
- **Expected ROS Topics**: 10 sets of `/{robot_id}/odom`, `cmd_vel`, `scan`, `rh_plan`, `cbba_bundle`.
- **Expected TF Tree**: `map` -> `amr_i/odom` -> `amr_i/base_footprint` -> `amr_i/base_link` -> `amr_i/lidar_link`.

---

### TERMINAL 2 — Fleet Dashboard (10 AMRs)
```bash
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 10 --world warehouse_m9_v2 --tui
```
- **Expected Result**: Dashboard tracks all 10 AMRs concurrently at `http://localhost:8080`.

---

## SECTION E — Canonical Benchmark Execution (Historical Reference)

> [!WARNING]
> **ALREADY EXECUTED — DO NOT RERUN UNLESS EXPLICITLY AUTHORIZED**  
> The canonical benchmarks below have completed full validation, passed peer audit, and their raw outputs are archived in `results/final/canonical/`.

### 1. Milestone M8A: Workload Scaling (30 Tasks)
```bash
python3 scripts/run_benchmark.py --workload 30 --fleet-size 5 --compute-mode NORMAL --seed 42 --trials 1 --horizon 60.0
```

### 2. Milestone M8B: Adaptive Compute Scaling (30 Tasks)
```bash
python3 scripts/run_benchmark.py --workload 30 --fleet-size 5 --compute-mode ADAPTIVE --communication NORMAL --seed 42 --trials 1 --horizon 60.0
```

### 3. Milestone M9-V2: Congested Warehouse Baseline (10 AMRs, 30 Tasks)
```bash
python3 scripts/run_benchmark.py --workload 30 --world warehouse_m9_v2 --fleet-size 10 --compute-mode ADAPTIVE --communication NORMAL --seed 42 --horizon 180.0
```

### 4. Milestone M9-V3-D: Dynamic Task Arrival (15 Initial + 15 at 45s)
```bash
python3 scripts/run_benchmark.py --workload 30 --world warehouse_m9_v2 --fleet-size 10 --compute-mode ADAPTIVE --communication NORMAL --seed 42 --horizon 180.0 --v3-d
```

### 5. Milestone M9-V3-A: Temporary Aisle Blockage (Aisle 1 South, 45s to 90s)
```bash
python3 scripts/run_benchmark.py --workload 30 --world warehouse_m9_v2 --fleet-size 10 --compute-mode ADAPTIVE --communication NORMAL --seed 42 --horizon 180.0 --v3-a --v3-a-block-time 45.0 --v3-a-unblock-time 90.0
```

### 6. Milestone M9-V3-E: Final Combined Stress Benchmark
```bash
python3 scripts/run_benchmark.py \
  --workload 30 \
  --world warehouse_m9_v2 \
  --fleet-size 10 \
  --compute-mode ADAPTIVE \
  --communication NORMAL \
  --seed 42 \
  --horizon 180.0 \
  --v3-e \
  --v3-e-comm-degrade-time 75.0 \
  --v3-e-comm-recover-time 120.0 \
  --v3-e-comm-profile LOSS_HIGH \
  --v3-a-block-time 45.0 \
  --v3-a-unblock-time 90.0
```

---

## SECTION E.2 — Milestone 4 Compound Fault & Adversarial Resilience Testing Console

The **NRDAS-FR Milestone 4** testbed evaluates the fleet's response to compound, simultaneous multi-domain disturbances on **Port 8081**.

### 1. Launching the M4 Adversarial Resilience Dashboard
```bash
cd /home/raghav/Downloads/SIH26123AMR
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Standalone simulation mode (no live Gazebo needed; instant testbed):
python3 scripts/resilience_dashboard.py --port 8081 --sim-mode

# Or live ROS 2 mode (with running Gazebo fleet):
python3 scripts/resilience_dashboard.py --port 8081 --world warehouse_m9_v2
```
Access GUI at: **`http://localhost:8081`**

### 2. Executing Automated M4 Validation Scenarios
```bash
# Run the 7-benchmark compound validation harness:
python3 scripts/validate_m4_compound_scenarios.py
```
*Expected Output*:
- `[PASS] M4-A - Robot Failure + Dynamic Blockage` (0 overlaps, 13-cell detour)
- `[PASS] M4-B - Comm Loss vs Hard Failure Discrimination` (0 false positives)
- `[PASS] M4-C - Multiple Overlapping Robot Failures` (atomic CAS, 0 stale reservations)
- `[PASS] M4-D - Sensor-Visible Decoupled Obstacle` (reactive brake to 0.0 m/s)
- `[PASS] M4-E - Network Partition + Robot Failure` (monotonic Lamport reconciliation)
- `[PASS] M4-F - Network Loss + Dynamic Blockage` (local safety hold, 0 advances)
- `[PASS] M4-G - Master Compound Quad Failure` (full composite recovery converged)

### 3. REST API Interaction & Experiment Control
```bash
# Query active scenario and 5 formal mathematical invariants:
curl -s http://localhost:8081/api/m4/status | jq .

# Trigger M4-G Master Compound Quad Failure benchmark:
curl -X POST http://localhost:8081/api/scenario/trigger \
  -H "Content-Type: application/json" \
  -d '{"scenario_id": "M4-G"}'

# Compose a custom multi-disturbance experiment:
curl -X POST http://localhost:8081/api/m4/compose_and_run \
  -H "Content-Type: application/json" \
  -d '{
    "scenario_id": "CUSTOM_COMPOUND",
    "robot_ids": ["amr_1"],
    "fault_type": "KILL",
    "network_profile": "LOSS_HIGH",
    "packet_loss_rate": 0.35,
    "duration_sec": 5.0,
    "blockage_cells": [[7, 4], [7, 5]]
  }'

# Reset all faults and clear obstacles:
curl -X POST http://localhost:8081/api/fault/restore -H "Content-Type: application/json" -d '{"robot_id": "ALL_ROBOTS"}'
curl -X POST http://localhost:8081/api/network/reconnect -H "Content-Type: application/json" -d '{"robot_id": "ALL_ROBOTS"}'
curl -X POST http://localhost:8081/api/environment/blockage -H "Content-Type: application/json" -d '{"action": "CLEAR", "blockage_id": "ALL"}'
```

---

## SECTION F — Live Demonstration Flow (Step-by-Step Script)

1. **Step 1: Introduction (30s)**:
   - State research question: *"Can decentralized multi-agent coordination maintain safety and efficiency under dynamic arrivals, physical corridor blockages, and wireless degradation?"*
   - Launch Section D (10-AMR simulation) in Terminal 1 or use `./scripts/launch_all_in_one.sh`.
2. **Step 2: Decentralized Task Allocation (1m)**:
   - Observe Terminal 1 logs: CBBA consensus achieves initial bundle allocation across 10 AMRs in under $15\,\text{ms}$.
   - Open Web Dashboard (`http://localhost:8080`) to show assigned task bundles per AMR.
3. **Step 3: Rolling-Horizon Planning & Spatio-Temporal Reservations (1m)**:
   - AMRs depart home stations toward perimeter rack pickup bays.
   - Show how Space-Time Reservations prevent conflicting corridor occupancy.
4. **Step 4: Dynamic Disturbance Presentation (1.5m)**:
   - Explain the 3 disturbance mechanisms validated in M9:
     1. Dynamic arrival of 15 tasks at $t=45\,\text{s}$ (CBBA reallocates without disrupting active tasks).
     2. Physical corridor obstruction at Aisle 1 South for $45\,\text{s}$ (local GridWorlds detect and route around).
     3. Wireless packet degradation to $35\%$ packet drop (stale reservations pruned, zero safety aborts).
5. **Step 5: M4 Compound Fault Resilience Testing (1m)**:
   - Open M4 Resilience Dashboard (`http://localhost:8081`).
   - Trigger benchmark scenario `M4-A` (Crash + Corridor Blockage) or `M4-G` (Master Quad Failure).
   - Demonstrate the live Three-Tier Fleet Response Telemetry (CBBA Reallocation, Dynamic Replanning, Local Safety) and 5 formal invariant badges.
6. **Step 6: Results & Verification (30s)**:
   - Present `results/final/figures/final_m9_cross_scenario_comparison.png` and `docs/evidence/m4_compound_validation.md`.
   - Conclude with zero physical contacts and 100% invariant satisfaction.

---

## SECTION G — Live Inspection & Diagnostics Commands

While the simulation is running, execute these inspection commands in a fresh terminal:

### Inspect Active ROS 2 Nodes
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
ros2 node list
```
*Expected*: 20+ nodes (`/amr_task_manager`, `/warehouse_visualizer`, `/amr_i/cbba_node`, `/amr_i/rh_node`, `/amr_i/robot_state_publisher`).

### Inspect Active ROS 2 Topics
```bash
ros2 topic list
```

### Inspect Live Odometry Telemetry
```bash
ros2 topic echo /amr_0/odom --once
```

### Inspect Live CBBA Task Bundles
```bash
ros2 topic echo /amr_0/cbba_bundle --once
```

### Inspect Live Rolling-Horizon Plans
```bash
ros2 topic echo /amr_0/rh_plan --once
```

### Inspect Live TF Coordinate Transforms
```bash
ros2 run tf2_ros tf2_echo map amr_0/base_footprint
```

### Inspect Active Gazebo Entities & Models
```bash
gz model --list
```
*Expected*: `warehouse_m9_v2`, `amr_0`, `amr_1`, `amr_2`, `amr_3`, `amr_4`, `amr_5`, `amr_6`, `amr_7`, `amr_8`, `amr_9`.

---

## SECTION H — Safe Shutdown Procedure

1. **Foreground Processes**:
   In each active terminal (Terminals 1, 2, 3, 4), press:
   ```
   Ctrl + C
   ```
   Wait 3–5 seconds for ROS 2 nodes and Gazebo Harmonic to perform graceful resource cleanup.

2. **Clean Teardown Verification**:
   Verify that all simulation processes have terminated:
   ```bash
   ps aux | grep -E 'amr_fleet|warehouse_m9|fleet_dashboard|resilience_dashboard' | grep -v grep || echo "AMR processes all stopped."
   ```

3. **Safe Process Cleanup (preserves other host ROS projects)**:
   ```bash
   pkill -9 -f "amr_fleet" 2>/dev/null || true
   pkill -9 -f "warehouse_m9" 2>/dev/null || true
   pkill -9 -f "fleet_dashboard" 2>/dev/null || true
   pkill -9 -f "resilience_dashboard" 2>/dev/null || true
   killall -9 parameter_bridge task_manager cbba_node rh_node warehouse_visualizer 2>/dev/null || true
   rm -f /dev/shm/sem.fastrtps* /dev/shm/fastrtps* 2>/dev/null || true
   ```
