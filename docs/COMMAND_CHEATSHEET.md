# NRDAS AMR Fleet Coordination: Command Cheat Sheet

> **Quick Reference**: Copy-pasteable terminal commands for building, running, monitoring, and shutting down the multi-AMR coordination system.

---

### ⭐ ALL-IN-ONE SINGLE COMMAND LAUNCH (EVERYTHING IN ONE GO)
Lauches **Gazebo Harmonic (3D GUI)** + **RViz 2** + **AMR Fleet Autonomy Stack** + **Fleet Observability Dashboard (Port 8080)** + **Resilience & Fault Dashboard (Port 8081)** all in a single command with clean `Ctrl+C` teardown!

#### Option 1: Big Congested Warehouse (10 AMRs, `warehouse_m9_v2`) [Recommended]
```bash
cd . && ./scripts/launch_all_in_one.sh --world warehouse_m9_v2 --robots 10
```

#### Option 2: Standard Warehouse (5 AMRs, `warehouse_small`)
```bash
cd . && ./scripts/launch_all_in_one.sh --world warehouse_small --robots 5
```

> **Web Interfaces**:
> - Fleet Observability: http://localhost:8080
> - Resilience & Fault Console: http://localhost:8081
>
> **Teardown**: Press `Ctrl + C` in the terminal to automatically and cleanly terminate all background simulators, visualizers, nodes, and web servers.

---

### 0. CLEAN START & ENVIRONMENT RESET
```bash
# Safe targeted cleanup of AMR processes (preserves other host ROS projects like solar_cleaning_ws)
pkill -9 -f "amr_fleet" 2>/dev/null || true
pkill -9 -f "warehouse_m9" 2>/dev/null || true
pkill -9 -f "fleet_dashboard" 2>/dev/null || true
pkill -9 -f "resilience_dashboard" 2>/dev/null || true
killall -9 parameter_bridge task_manager cbba_node rh_node warehouse_visualizer 2>/dev/null || true
rm -f /dev/shm/sem.fastrtps* /dev/shm/fastrtps* 2>/dev/null || true
sleep 1
ps aux | grep -E 'amr_fleet|warehouse_m9|fleet_dashboard|resilience_dashboard' | grep -v grep || echo "AMR processes all clean!"
```

---

### 1. BUILD WORKSPACE
```bash
# Terminal 1 — Build from repository root
cd /home/raghav/Downloads/SIH26123AMR
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 2. EXECUTE UNIT & REGRESSION TESTS
```bash
# Terminal 1 — Run fast unit tests & full regression suite (216 tests)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test && colcon test-result --verbose

# Run NRDAS-FR Resilience test suite (79 unit & integration tests)
export ROS_DOMAIN_ID=42
python3 -m pytest \
  src/amr_fleet_core/test/test_fault_resilience.py \
  src/amr_fleet_core/test/test_m2_network_resilience.py \
  src/amr_fleet_core/test/test_m3_adversarial_resilience.py \
  src/amr_fleet_core/test/test_m4_compound_resilience.py \
  src/amr_fleet_core/test/test_resilience_dashboard.py -v

# Run Milestone 4 Compound Scenarios Validation Harness (7/7 benchmarks)
python3 scripts/validate_m4_compound_scenarios.py

# Static analysis (flake8 & pep257)
ament_flake8 src/amr_fleet_core
ament_pep257 src/amr_fleet_core
```

---

### 3. LAUNCH GAZEBO SIMULATION & FLEET (TERMINAL 1)

#### Option A: Normal Visual Demo (5 AMRs, warehouse_small)
```bash
# Terminal 1
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py \
  robot_count:=5 \
  world:=warehouse_small \
  headless:=false \
  compute_mode:=ADAPTIVE \
  comm_profile:=NORMAL \
  workload_file:=config/workloads/workload_15_tasks.yaml
```

#### Option B: Congested Warehouse Demo (10 AMRs, warehouse_m9_v2)
```bash
# Terminal 1
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

---

### 4. LAUNCH RVIZ VISUALIZATION (TERMINAL 2)
```bash
# Terminal 2 — Start RViz once Terminal 1 simulation is running
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

rviz2 -d $(ros2 pkg prefix amr_fleet_bringup)/share/amr_fleet_bringup/rviz/fleet_default.rviz
```

---

### 5. LAUNCH OBSERVABILITY DASHBOARD (TERMINAL 3)
```bash
# Terminal 3 — Launch Dashboard Server & Terminal UI
cd .
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# For 5-AMR warehouse_small:
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 5 --world warehouse_small --tui

# For 10-AMR warehouse_m9_v2:
python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 10 --world warehouse_m9_v2 --tui

# Open Web Interface in Browser:
# http://localhost:8080
```

---

### 5B. LAUNCH RESILIENCE & FAULT INJECTION DASHBOARD (TERMINAL 3B)
```bash
# Terminal 3B — Launch Resilience Console (Port 8081)
cd /home/raghav/Downloads/SIH26123AMR
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Option 1: Live mode with Gazebo fleet
python3 scripts/resilience_dashboard.py --port 8081 --world warehouse_m9_v2

# Option 2: Standalone autonomous simulation mode (no Gazebo needed)
python3 scripts/resilience_dashboard.py --port 8081 --sim-mode

# Open Web Interface in Browser:
# http://localhost:8081
```

---

### 5C. M4 COMPOUND FAULT REST API & ADVERSARIAL SCENARIOS
```bash
# Trigger M4-G Master Compound Quad Failure scenario
curl -X POST http://localhost:8081/api/scenario/trigger \
  -H "Content-Type: application/json" \
  -d '{"scenario_id": "M4-G"}'

# Query live M4 compound status & 5 formal mathematical invariants
curl -s http://localhost:8081/api/m4/status | jq .

# Compose custom compound experiment (amr_1 crash + 35% packet loss + aisle blockage)
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

# Reset all fleet faults and clear all spatial obstacles
curl -X POST http://localhost:8081/api/fault/restore -H "Content-Type: application/json" -d '{"robot_id": "ALL_ROBOTS"}'
curl -X POST http://localhost:8081/api/network/reconnect -H "Content-Type: application/json" -d '{"robot_id": "ALL_ROBOTS"}'
curl -X POST http://localhost:8081/api/environment/blockage -H "Content-Type: application/json" -d '{"action": "CLEAR", "blockage_id": "ALL"}'

# Download experiment evidence reports
curl -s http://localhost:8081/api/export > m4_evidence.json
curl -s http://localhost:8081/api/export/markdown > m4_evidence.md
```

---

### 6. RUN DETERMINISTIC WAYPOINT DEMO TRAJECTORY (TERMINAL 4)
```bash
# Terminal 4 — Optional waypoint presentation controller
cd /home/raghav/Downloads/SIH26123AMR
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/run_fleet_demo.py --robot-count 5 --loop
```

---

### 7. LIVE SYSTEM INSPECTION (TERMINAL 5)
```bash
# Terminal 5 — Query active ROS 2 and Gazebo graph
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash

# List active nodes
ros2 node list

# List active topics
ros2 topic list

# Echo live odometry for amr_0
ros2 topic echo /amr_0/odom --once

# Echo live CBBA bundle for amr_0
ros2 topic echo /amr_0/cbba_bundle --once

# Echo live rolling-horizon plan for amr_0
ros2 topic echo /amr_0/rh_plan --once

# Echo coordinate transform from map to amr_0
ros2 run tf2_ros tf2_echo map amr_0/base_footprint

# List Gazebo models
gz model --list
```

---

### 8. SYSTEM SHUTDOWN
```bash
# In each terminal: Press Ctrl + C

# Safe targeted AMR process cleanup (preserves other host ROS projects like solar_cleaning_ws):
pkill -9 -f "amr_fleet" 2>/dev/null || true
pkill -9 -f "warehouse_m9" 2>/dev/null || true
pkill -9 -f "fleet_dashboard" 2>/dev/null || true
pkill -9 -f "resilience_dashboard" 2>/dev/null || true
killall -9 parameter_bridge task_manager cbba_node rh_node warehouse_visualizer 2>/dev/null || true
rm -f /dev/shm/sem.fastrtps* /dev/shm/fastrtps* 2>/dev/null || true
```
