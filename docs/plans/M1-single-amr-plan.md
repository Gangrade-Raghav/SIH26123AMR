# M1 Implementation Plan — Single AMR Simulation Foundation

**Milestone**: M1 — Single AMR Simulation Foundation  
**Author**: Lead Autonomous Engineering Agent (Antigravity)  
**Target Repository**: `/home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project`  
**Status**: APPROVED & IN PROGRESS  

---

## 1. Objective

Build, integrate, and experimentally validate a fully functional, parameterized single Autonomous Mobile Robot (AMR) simulation in Gazebo Harmonic using ROS 2 Jazzy. Establish clean separation between robot description, simulation physics, sensor models, ROS 2 bridges, launch orchestration, and automated verification—ensuring the design seamlessly scales to multi-robot fleets (M2) without modifying the core robot model.

---

## 2. Scope & Boundaries

### In Scope
1. **Parameterized Robot Description (`amr_fleet_description`)**:
   - Reusable URDF/Xacro model supporting namespaces, prefixes, configurable dimensions, kinematics, and inertia.
   - Differential drive locomotion with left/right drive wheels and dual balancing casters.
   - Sensor mounts: 2D LiDAR (`laser_frame`) and base frames (`base_footprint`, `base_link`).
   - Embedded Gazebo Harmonic system plugins: `DiffDrive`, `JointStatePublisher`, and `Sensors` (`gpu_lidar`).
2. **Simulation World (`amr_fleet_bringup/worlds/`)**:
   - `warehouse_small.sdf`: A bounded warehouse simulation world with ground plane, physical boundaries, storage racks, and standard system plugins (`Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors`).
3. **Launch & Spawning Architecture**:
   - `spawn_robot.launch.py`: Reusable single-robot spawner accepting `robot_name`, initial coordinates (`x`, `y`, `z`, `yaw`), and simulation time flags.
   - `single_amr_simulation.launch.py`: Top-level orchestrator launching Gazebo Harmonic (defaulting to headless), clock bridge, robot spawning, and optional RViz2.
4. **ROS 2 Communication Bridges (`ros_gz_bridge`)**:
   - Namespaced bidirectional and unidirectional topic mappings:
     - `/clock` -> ROS Clock
     - `/<robot_name>/cmd_vel` -> Gazebo Twist
     - `/<robot_name>/odom` -> ROS Odometry
     - `/<robot_name>/scan` -> ROS LaserScan
     - `/<robot_name>/tf` -> ROS TF
5. **Comprehensive Verification & Testing**:
   - Automated end-to-end integration test (`scripts/verify_m1_simulation.py`) verifying spawn, topic presence, TF tree, motion response to `/cmd_vel`, safe stopping, sensor publication, and clean shutdown.
   - Existing M0 unit/linter tests preserved with zero regressions.

### Explicitly Excluded (Deferred to Future Milestones)
- Multi-robot fleet scaling, inter-robot namespaces conflicts (M2)
- Task generator and task allocation baselines (M3)
- CBBA/ACBBA distributed allocation (M4)
- PIBT / RHCR / GD-RHCR lifelong MAPF planners (M6, M7, M11)
- WFG deadlock detection and recovery (M8)
- Network fault injection (M9)
- Compute-aware adaptation (M10)

---

## 3. Package & File Architecture

```text
src/
├── amr_fleet_description/                [NEW PACKAGE: Robot modeling & URDF]
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── urdf/
│   │   ├── amr.urdf.xacro                [Top-level parameterized robot xacro]
│   │   ├── amr_core.xacro                [Chassis, wheels, casters, materials]
│   │   ├── amr_gazebo.xacro              [Gazebo Harmonic plugins: DiffDrive, Lidar]
│   │   └── amr_lidar.xacro               [2D LiDAR sensor mount & properties]
│   ├── rviz/
│   │   └── view_robot.rviz               [Single robot description inspection]
│   └── launch/
│       └── view_robot.launch.py          [RSP + joint_state_publisher + RViz]
├── amr_fleet_bringup/                    [MODIFIED: Add worlds, bridges, and launch]
│   ├── package.xml
│   ├── setup.py
│   ├── worlds/
│   │   └── warehouse_small.sdf           [Deterministic warehouse SDF world]
│   ├── launch/
│   │   ├── spawn_robot.launch.py         [Reusable parameterized robot spawner]
│   │   └── single_amr_simulation.launch.py [M1 single AMR launch entrypoint]
│   └── rviz/
│       └── single_amr.rviz               [RViz display for single AMR + LaserScan]
└── amr_fleet_msgs/, amr_fleet_core/, amr_fleet_sim/ [Preserved from M0]

scripts/
├── verify_m1_simulation.py               [NEW: Automated motion/sensor/spawn test]
├── run_single_amr.sh                     [NEW: Quick interactive runner]
├── build_workspace.sh                    [M0 build script]
└── run_tests.sh                          [M0 test runner]
```

---

## 4. Parameterized Design for M2 Fleet Extensibility

To ensure M2 can spawn $N$ AMRs without altering the robot description:
1. **Frame Namespacing**: Every frame ID in the URDF will be constructed as `$(arg prefix)base_link`, `$(arg prefix)laser_frame`, etc. If `prefix` is `amr_0/`, the TF frames will be `amr_0/base_link`, `amr_0/odom`.
2. **Topic Isolation**:
   - Control: `/<robot_name>/cmd_vel`
   - Telemetry: `/<robot_name>/odom`
   - Perception: `/<robot_name>/scan`
   - Description: `/<robot_name>/robot_description`
3. **Gazebo Entity Isolation**:
   - Model name passed to Gazebo will be `$(arg robot_name)`
   - DiffDrive plugin binds to `/model/$(arg robot_name)/cmd_vel` and publishes `/model/$(arg robot_name)/odometry`.

---

## 5. Verification Commands

1. **Workspace Compilation**:
   ```bash
   ./scripts/build_workspace.sh
   ```
2. **Package Unit & Linter Tests**:
   ```bash
   ./scripts/run_tests.sh
   ```
3. **Automated End-to-End M1 Smoke Test**:
   ```bash
   python3 scripts/verify_m1_simulation.py --headless --timeout 30
   ```
4. **Manual Topic and TF Introspection**:
   ```bash
   ros2 topic list
   ros2 topic echo /amr_0/odom --once
   ros2 topic echo /amr_0/scan --once
   ros2 run tf2_ros tf2_echo amr_0/odom amr_0/base_footprint
   ```

---

## 6. Definition of Done Checklist for M1

- [ ] Reusable AMR model exists with complete visual, collision, and inertial properties.
- [ ] Gazebo Harmonic simulates differential drive kinematics accurately.
- [ ] 2D LiDAR publishes valid laser scan data on `/<robot_name>/scan`.
- [ ] Odometry publishes on `/<robot_name>/odom` and produces consistent pose shifts when commanded.
- [ ] Clean TF tree connects `odom` -> `base_footprint` -> `base_link` -> `laser_frame`.
- [ ] Spawning is fully parameterized (`robot_name`, `x`, `y`, `z`, `yaw`).
- [ ] Clean shutdown without hanging background processes.
- [ ] Automated integration test passes cleanly.
- [ ] Zero regressions against M0 test suite.
- [ ] `docs/checkpoints/M1.md` created with complete human review evidence.
