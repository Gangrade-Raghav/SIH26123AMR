# NRDAS AMR Fleet Coordination: Command Cheat Sheet

> **Quick Reference**: Copy-pasteable terminal commands for building, running, monitoring, and shutting down the multi-AMR coordination system.

---

### 0. CLEAN START & ENVIRONMENT RESET
```bash
# Terminal 1 — Kill any dangling ROS 2 or Gazebo processes
killall -9 gz sim-server sim-gui ruby ros2 rviz2 python3 2>/dev/null || true
sleep 1
ps aux | grep -E 'gz|ros2|amr_fleet' | grep -v grep || true
```

---

### 1. BUILD WORKSPACE
```bash
# Terminal 1 — Build from repository root
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 2. EXECUTE UNIT & REGRESSION TESTS
```bash
# Terminal 1 — Run fast unit tests & full regression suite
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test && colcon test-result --verbose

# Static analysis (flake8 & pep257)
ament_flake8 src/amr_fleet_core
ament_pep257 src/amr_fleet_core
```

---

### 3. LAUNCH GAZEBO SIMULATION & FLEET (TERMINAL 1)

#### Option A: Normal Visual Demo (5 AMRs, warehouse_small)
```bash
# Terminal 1
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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

---

### 4. LAUNCH RVIZ VISUALIZATION (TERMINAL 2)
```bash
# Terminal 2 — Start RViz once Terminal 1 simulation is running
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

rviz2 -d $(ros2 pkg prefix amr_fleet_bringup)/share/amr_fleet_bringup/rviz/fleet_default.rviz
```

---

### 5. LAUNCH OBSERVABILITY DASHBOARD (TERMINAL 3)
```bash
# Terminal 3 — Launch Dashboard Server & Terminal UI
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
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

### 6. RUN DETERMINISTIC WAYPOINT DEMO TRAJECTORY (TERMINAL 4)
```bash
# Terminal 4 — Optional waypoint presentation controller
cd /home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/run_fleet_demo.py --robot-count 5 --loop
```

---

### 7. LIVE SYSTEM INSPECTION (TERMINAL 5)
```bash
# Terminal 5 — Query active ROS 2 and Gazebo graph
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

# Emergency cleanup if any process hangs:
killall -9 gz sim-server sim-gui ruby ros2 rviz2 python3 2>/dev/null || true
```
