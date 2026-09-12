# Implementation Plan: Milestone M2 — Parameterized Multi-Robot Fleet Simulation

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Status**: Executed & Verified (Pending Human Review)  
**Target Milestone**: M2 — Parameterized Multi-Robot Fleet Simulation  
**Target System**: ROS 2 Jazzy, Gazebo Harmonic 8.11.0, Ubuntu 24.04 LTS  

---

## 1. Goal Description

Transform the validated Milestone M1 single-AMR simulation into a parameterized multi-AMR simulation environment supporting 1 to 10 robots in Gazebo Harmonic with ROS 2 Jazzy.

The architecture must support multi-robot fleet scaling purely through configuration and parameterized launch files, with zero modifications to the underlying robot description package (`amr_fleet_description`) and zero premature MAPF/coordination algorithm implementation.

Target validation progression:
$$\text{Single AMR (M1)} \longrightarrow \text{2 Robots (M2.1)} \longrightarrow \text{5 Robots (M2.2)} \longrightarrow \text{10 Robots (M2.3)}$$

---

## 2. Requirements Breakdown

1. **Parameterized Fleet Launch**: Single launch entrypoint `ros2 launch amr_fleet_bringup fleet.launch.py robot_count:=N` with configurable `fleet_config`, `world`, `headless`, `rviz`, `use_sim_time`.
2. **Automated Spawning**: Programmatic instantiation of $N$ robots via iteration; zero robot-specific launch file duplication.
3. **Namespace Isolation**: Full isolation of topics across robots (`/amr_i/cmd_vel`, `/amr_i/odom`, `/amr_i/scan`, `/amr_i/joint_states`, `/amr_i/robot_description`).
4. **TF Isolation**: Clean, non-colliding coordinate frame trees using `${robot_name}/` prefix for all links and joints.
5. **Independent Control & Cross-talk Verification**: Command `amr_0` to move while `amr_1` stays still, then vice-versa; measure displacement of both.
6. **Sensor Isolation**: Independent 2D planar LiDAR streams (`/amr_i/scan`), verifying frame IDs and message reception.
7. **Odometry Isolation**: Independent odometry feeds (`/amr_i/odom`), verifying distinct coordinate progression.
8. **Deterministic Initialization**: Configurations stored under `config/robots/` (`fleet_2_robots.yaml`, `fleet_5_robots.yaml`, `fleet_10_robots.yaml`, `fleet_default.yaml`) avoiding immediate spawn collisions in `warehouse_small.sdf`.
9. **Fleet State Discovery**: Lightweight, infrastructure-only abstraction in `amr_fleet_core` enumerating robot metadata and active topics without coordination logic.
10. **Clean Startup & Teardown**: Graceful process group termination on SIGINT with process sweeps.
11. **Resource Observability**: Logging startup latency, CPU usage, RAM footprint, real-time factor, and topic discovery times.
12. **Backward Compatibility**: Full regression validation of M0 and M1 capabilities.

---

## 3. Architectural Design

### 3.1. Fleet Configuration Model (`config/robots/`)
Configurations are stored as YAML files defining the fleet list:
```yaml
fleet:
  world: "warehouse_small"
  robots:
    - id: "amr_0"
      x: 2.0
      y: 2.0
      z: 0.15
      yaw: 0.0
    - id: "amr_1"
      x: 2.0
      y: 5.0
      z: 0.15
      yaw: 0.0
```

### 3.2. Spawning Architecture
`fleet.launch.py` uses `OpaqueFunction` to dynamically load the robot configuration. For each robot $i \in [0, N-1]$, it includes `spawn_robot.launch.py` with parameters:
- `robot_name = id`
- `x = x`, `y = y`, `z = z`, `yaw = yaw`
- `use_sim_time = true`

In `spawn_robot.launch.py`:
- `robot_state_publisher` runs in namespace `/{robot_name}`.
- `ros_gz_sim create` spawns entity `robot_name` into Gazebo.
- `ros_gz_bridge parameter_bridge` bridges `/{robot_name}/cmd_vel`, `/{robot_name}/odom`, `/{robot_name}/scan`, and `/{robot_name}/tf` to `/tf`.
- Bridge node named `[robot_name, '_bridge']` to avoid ROS 2 node naming collisions.

### 3.3. TF Frame Isolation Strategy
All robot links and joints use the parameterized prefix `prefix:=${robot_name}/`:
```text
/tf, /tf_static:
  ├── amr_0/odom -> amr_0/base_footprint -> amr_0/base_link -> amr_0/laser_frame
  ├── amr_1/odom -> amr_1/base_footprint -> amr_1/base_link -> amr_1/laser_frame
  └── amr_k/odom -> amr_k/base_footprint -> amr_k/base_link -> amr_k/laser_frame
```
No global frame collisions occur because every robot's kinematic chain is rooted in its unique namespaced frames.

### 3.4. Fleet State Discovery Abstraction (`amr_fleet_core`)
A read-only discovery abstraction in `src/amr_fleet_core/amr_fleet_core/fleet_state.py`:
- `RobotInfo`: Dataclass storing `robot_id`, `namespace`, `pose_topic`, `cmd_vel_topic`, `scan_topic`, `status`, and `initial_pose`.
- `FleetState`: Container with `from_yaml()` and `from_node()` methods to inspect active fleet members.

---

## 4. Verification Plan

1. **Unit & Linter Suite**:
   - `colcon test` across all 5 packages.
   - New unit tests for `FleetState` in `test_fleet_state.py`.
2. **M1 Single-Robot Regression**:
   - Run `python3 scripts/verify_m1_simulation.py` to ensure single-AMR baseline remains 100% operational.
3. **M2 Multi-Robot Validation (`scripts/verify_m2_fleet.py`)**:
   - **M2.1 (2 robots)**: Spawning, topic discovery, cross-talk motion isolation test, clean shutdown.
   - **M2.2 (5 robots)**: Spawning, topic discovery, simultaneous odometry and sensor acquisition, clean shutdown.
   - **M2.3 (10 robots)**: 10-robot fleet launch in Gazebo Harmonic, full topic and TF validation, resource benchmarking, clean shutdown.
4. **Checkpoint Documentation**:
   - Compile empirical metrics into `docs/checkpoints/M2.md`.
   - Update `docs/architecture.md`.
