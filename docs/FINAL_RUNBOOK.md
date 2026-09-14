# NRDAS AMR Fleet Coordination: Final Operator Runbook

> **Audience**: Systems engineers, research evaluators, and operators reproducing or demonstrating the multi-AMR coordination system.  
> **Environment**: Ubuntu 24.04 / Linux, ROS 2 Jazzy, Gazebo Sim 8.11.0 (Harmonic), Python 3.12.

---

## SECTION A — Clean Environment Start & Process Cleanup

Before starting any simulation or test, ensure no orphaned ROS 2 daemons or Gazebo processes occupy ports or system resources.

### Emergency Process Cleanup Command
Execute in any terminal:
```bash
killall -9 gz sim-server sim-gui ruby ros2 rviz2 python3 2>/dev/null || true
sleep 1
ps aux | grep -E 'gz|ros2|amr_fleet' | grep -v grep || true
```
*Expected Output*: No processes returned by `grep`.

### Environment Setup Verification
In every terminal used for this project, source the ROS 2 and workspace underlays:
```bash
source /opt/ros/jazzy/setup.bash
source /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project/install/setup.bash
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
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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

## SECTION F — Live Demonstration Flow (Step-by-Step Script)

1. **Step 1: Introduction (30s)**:
   - State research question: *"Can decentralized multi-agent coordination maintain safety and efficiency under dynamic arrivals, physical corridor blockages, and wireless degradation?"*
   - Launch Section D (10-AMR simulation) in Terminal 1.
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
5. **Step 5: Results & Verification (1m)**:
   - Present `results/final/figures/final_m9_cross_scenario_comparison.png` and `final_m9_v3_e_stress_timeline.png`.
   - Conclude with zero physical contacts across all canonical experiments.

---

## SECTION G — Live Inspection & Diagnostics Commands

While the simulation is running, execute these inspection commands in a fresh terminal:

### Inspect Active ROS 2 Nodes
```bash
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
   ps aux | grep -E 'gz|ros2|amr_fleet' | grep -v grep || true
   ```

3. **Emergency Cleanup (if processes hang)**:
   ```bash
   killall -9 gz sim-server sim-gui ruby ros2 rviz2 python3 2>/dev/null || true
   ```
