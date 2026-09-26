# M0 Bootstrap Implementation Plan — NRDAS AMR Fleet Coordination

**Milestone**: M0 — Environment & Repository Bootstrap  
**Author**: Raghav Gangrade  
**Status**: AWAITING HUMAN APPROVAL (Checkpoint)  
**Target Root**: `.`  

---

## 1. Objective

Bootstrap the development environment and repository control plane for the decentralized AMR fleet coordination research project. Establish a clean Git repository, modular ROS 2 Jazzy package skeleton, unified message interfaces, deterministic build/test pipelines, and automated environment verification scripts—without prematurely implementing high-level coordination or path planning algorithms.

---

## 2. Current State

1. **Host Environment**: Verified Ubuntu 24.04.4 LTS on Intel Core i7-13620H (16 vCPUs), 30 GiB RAM, NVMe storage (209 GiB free).
2. **ROS 2 & Middleware**: ROS 2 Jazzy desktop is fully installed and active. Gazebo Sim 8.11.0 (Harmonic) and `ros_gz_sim`/`ros_gz_bridge` (v1.0.22) verified functional in headless mode. `rmw_zenoh_cpp` (v0.2.10) and `rmw_zenohd` are installed and verified loadable.
3. **Repository State**:
   - The project directory `.` contains complete control plane documentation (`AGENTS.md`, `PRD.md`, `README.md`, `docs/`, `.agents/`).
   - The directory is **not yet a Git repository** (`fatal: not a git repository`).
   - No `src/` directory or ROS 2 packages exist yet.
   - An audit report has been compiled at `docs/audit-reports/M0-environment-audit.md`.

---

## 3. Proposed Implementation

The M0 implementation is divided into four sequential stages:

### Stage 1: Git Repository Initialization & Control Plane Baseline
- Initialize git repository (`git init`).
- Verify `.gitignore` covers ROS 2 build artifacts (`build/`, `install/`, `log/`), ROS bags (`*.bag`, `*.db3`), Python cache, and raw experiment logs.
- Commit the initial project control plane (`AGENTS.md`, `PRD.md`, `docs/`, `.agents/`, `config/`, `results/`).

### Stage 2: ROS 2 Package Skeleton Creation (`src/`)
Create four core packages under `src/` adhering strictly to ROS 2 Jazzy and modern Ament standards:

1. **`amr_fleet_msgs`** (C++ / `ament_cmake`):
   - Custom ROS 2 interface package for decentralized fleet communication.
   - Initial messages to declare:
     - `RobotStatus.msg`: Robot ID, pose, battery/compute metrics, current state machine state, plan epoch.
     - `TaskBid.msg`: Auction bidding message for CBBA/ACBBA (bidder ID, task ID, bid value, path cost, timestamp).
     - `TaskAssignment.msg`: Task assignment commitment and epoch.
     - `BlockedResource.msg`: Resource reservation and edge conflict notification for Wait-For-Graph (WFG).
     - `FleetMetrics.msg`: Real-time observability metrics (throughput, planning latency P50/P95/P99, network bytes).
2. **`amr_fleet_core`** (Python / `ament_python` or C++ hybrid):
   - Abstract base classes defining the core research interfaces specified in `docs/architecture.md`:
     - `TaskAllocator` (interface for Centralized baseline and CBBA)
     - `GlobalPlanner` (interface for RHCR and GD-RHCR)
     - `LocalPlanner` (interface for PIBT fallback and local controllers)
     - `DeadlockManager` (interface for WFG cycle detection and recovery)
     - `MetricsCollector` (interface for experiment telemetry)
   - Baseline unit tests validating interface contracts and error handling.
3. **`amr_fleet_bringup`** (Python / `ament_python`):
   - Parameterized launch scaffolding:
     - `fleet_simulation.launch.py`: Multi-robot Gazebo Harmonic launcher supporting namespaces, headless flag, and robot count.
     - `zenoh_router.launch.py`: Launch harness for `rmw_zenohd`.
   - RViz configuration templates with isolated display topics.
   - Gazebo Harmonic bridge configuration templates.
4. **`amr_fleet_sim`** (Python / `ament_python`):
   - Lightweight discrete-event / grid MAPF simulation scaffolding for Tier B algorithmic benchmarking (500–10,000 agents) without Gazebo physics overhead.

### Stage 3: Configuration Layout (`config/`)
Structure externalized configuration directories as dictated by `config/README.md`:
- `config/robots/amr_default.yaml`: Differential drive physical limits, collision radii, sensor specs.
- `config/maps/warehouse_grid_small.yaml`: Initial benchmark grid layout.
- `config/planners/baseline_planner.yaml`: Planning horizons and timeout parameters.
- `config/experiments/m0_verification.yaml`: Verification test parameters.

### Stage 4: Verification & Automation Tooling
- `scripts/verify_environment.py`: Automated, non-destructive script verifying Jazzy, Gazebo, Zenoh, Python modules, and package builds.
- `scripts/build_workspace.sh`: Standardized build script (`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`).
- `scripts/run_tests.sh`: Standardized test runner executing `colcon test` and parsing test results.

---

## 4. Files and Packages to Create

```text
SIH26123AMR/
├── .git/                                         [Stage 1: Git initialization]
├── scripts/
│   ├── verify_environment.py                     [Stage 4: Automated environment audit]
│   ├── build_workspace.sh                        [Stage 4: Colcon build helper]
│   └── run_tests.sh                              [Stage 4: Colcon test helper]
├── src/
│   ├── amr_fleet_msgs/                           [Stage 2: Interface package]
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── msg/
│   │       ├── RobotStatus.msg
│   │       ├── TaskBid.msg
│   │       ├── TaskAssignment.msg
│   │       ├── BlockedResource.msg
│   │       └── FleetMetrics.msg
│   ├── amr_fleet_core/                           [Stage 2: Core interfaces package]
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── amr_fleet_core/
│   │   │   ├── __init__.py
│   │   │   ├── interfaces.py                     [Abstract base classes]
│   │   │   └── state_machine.py                  [Robot lifecycle states]
│   │   └── test/
│   │       ├── test_interfaces.py
│   │       ├── test_copyright.py
│   │       ├── test_flake8.py
│   │       └── test_pep257.py
│   ├── amr_fleet_bringup/                        [Stage 2: Bringup & launch package]
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── amr_fleet_bringup/
│   │   │   └── __init__.py
│   │   ├── launch/
│   │   │   ├── fleet_bringup.launch.py
│   │   │   └── zenoh_router.launch.py
│   │   └── rviz/
│   │       └── fleet_default.rviz
│   └── amr_fleet_sim/                            [Stage 2: Simulation backend package]
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── amr_fleet_sim/
│       │   ├── __init__.py
│       │   └── grid_world.py                     [Minimal deterministic grid]
│       └── test/
│           └── test_grid_world.py
└── config/
    ├── robots/
    │   └── amr_default.yaml
    ├── maps/
    │   └── warehouse_grid_small.yaml
    ├── planners/
    │   └── baseline_planner.yaml
    └── experiments/
        └── m0_verification.yaml
```

---

## 5. Dependencies

All dependencies are present or fulfillable with official Ubuntu 24.04 / ROS 2 Jazzy packages:
- **Build System**: `ament_cmake`, `ament_cmake_python`, `colcon-core`, `rosidl_default_generators`, `rosidl_default_runtime`.
- **ROS 2 Core**: `rclcpp`, `rclpy`, `std_msgs`, `geometry_msgs`, `builtin_interfaces`.
- **Testing & Quality**: `ament_lint_auto`, `ament_lint_common`, `ament_flake8`, `ament_pep257`, `pytest`.
- **Simulation / Middleware**: `ros_gz_sim`, `ros_gz_bridge`, `rmw_zenoh_cpp`.
- **Optional Python**: `python3-networkx`, `python3-pandas` (can be added via apt when required in subsequent milestones).

---

## 6. Risks and Mitigations

| Risk | Impact | Mitigation Strategy |
|---|---|---|
| **RMW Switching Flakiness** | Zenoh nodes fail to discover peers if router daemon is missing. | Provide explicit launch harness for `rmw_zenohd`; keep default RMW as `rmw_fastrtps_cpp` during base builds and enable `rmw_zenoh_cpp` via explicit environment variable during Zenoh testing. |
| **Gazebo Headless Dependency** | Intel UHD graphics performance drop in 3D GUI. | Make `headless:=true` the default parameter in simulation launch files. |
| **Interface Premature Locking** | Over-specifying message formats before algorithm implementation creates refactoring debt. | Keep message definitions minimal and focused on core fields with explicit version headers. |
| **Namespace Collisions in Fleet** | Topics and TF frames leaking across AMRs. | Enforce strict namespacing rules in launch files and unit test frame prefixes (`robot_1/base_link`). |

---

## 7. Verification Commands

Upon execution approval, M0 will be verified with the following command sequence:

1. **Git Status & Commit Integrity**:
   ```bash
   git status
   git log -n 1 --oneline
   ```
2. **Environment Verification Script**:
   ```bash
   python3 scripts/verify_environment.py
   ```
3. **Clean Colcon Build**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --cmake-clean-cache
   ```
4. **Colcon Automated Test Suite**:
   ```bash
   colcon test --event-handlers console_direct+
   colcon test-result --all --verbose
   ```
5. **RMW Zenoh Loading Test**:
   ```bash
   RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 doctor --report | grep "middleware name"
   ```
6. **Headless Gazebo Launch Smoke Test**:
   ```bash
   gz sim -s -r -v 2 --iterations 50
   ```

---

## 8. Acceptance Criteria

M0 will be accepted when:
- [ ] Git repository is initialized with a clean working tree and initial commit.
- [ ] Workspace builds cleanly (`colcon build`) with 0 errors across `amr_fleet_msgs`, `amr_fleet_core`, `amr_fleet_bringup`, and `amr_fleet_sim`.
- [ ] All package unit tests pass (`colcon test`) with 0 failures and 0 errors.
- [ ] Automated verification script `scripts/verify_environment.py` exits with code 0.
- [ ] No algorithmic implementations (CBBA, RHCR, PIBT, WFG) are introduced prematurely.
- [ ] A detailed completion report `docs/audit-reports/M0-completion-report.md` is generated.

---

## 9. Definition of Done

In accordance with `docs/definition-of-done.md`, M0 is DONE only when an independent engineer can:
1. Understand the environment requirements and audit findings from `docs/audit-reports/M0-environment-audit.md`.
2. Inspect the repository git history and package layout.
3. Build the workspace cleanly with `colcon build` without missing dependencies.
4. Execute `colcon test` and `scripts/verify_environment.py` and see 100% passing results.
5. Review the externalized configuration layout in `config/`.
6. Proceed to Milestone M1 (Single AMR Simulation) with verified confidence in the underlying platform.
