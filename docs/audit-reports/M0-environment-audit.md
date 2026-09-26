# M0 Environment Audit Report — NRDAS AMR Fleet Coordination

**Date**: September 13, 2026  
**Auditor**: Raghav Gangrade  
**Target Repository**: `.`  
**Milestone**: M0 — Environment & Repository Bootstrap  

---

## 1. Environment Summary

A comprehensive, non-destructive audit of the host machine and development environment was conducted. The host system is an x86_64 workstation running Ubuntu 24.04.4 LTS (Noble Numbat) on a 13th Gen Intel Core i7-13620H processor with 30 GiB RAM and 209 GiB of available storage in `/home`.

The ROS 2 ecosystem is in an exceptionally strong starting state: **ROS 2 Jazzy Jalisco is fully installed** under `/opt/ros/jazzy` and sourced by default. **Gazebo Harmonic (v8.11.0)** and the official ROS-Gazebo bridge/sim packages (`ros_gz_sim` v1.0.22, `ros_gz_bridge` v1.0.22) are installed and verified working in headless server mode. **Nav2 (v1.3.12)** and **RViz2 (v14.1.22)** are present. **Zenoh middleware (`rmw_zenoh_cpp` v0.2.10)** and the Zenoh router daemon (`rmw_zenohd`) are installed and functional.

Key findings requiring attention:
1. The project directory is not yet initialized as a Git repository.
2. The system uses Intel UHD Graphics (no discrete NVIDIA GPU / CUDA). Gazebo GUI rendering will rely on Mesa OpenGL/Vulkan, making headless simulation (`gz sim -s`) preferred for multi-robot runs.
3. No active swap space is configured, though 30 GiB physical RAM provides substantial headroom.
4. Python graph/data libraries (`networkx`, `pandas`, `pydantic`) are not installed system-wide (available via Ubuntu apt repos when needed).
5. Docker is not installed on the host.

---

## 2. Component Audit Matrix

| # | Component | Expected / Target | Installed Version | Status | Evidence / Command Used | Compatibility Concerns | Recommended Action |
|---|---|---|---|---|---|---|---|
| 1 | **Ubuntu OS** | Ubuntu 24.04 LTS (Noble) | Ubuntu 24.04.4 LTS (Kernel 6.8.0-139-generic) | **PASS** | `lsb_release -a` & `uname -a` | None. Tier 1 target for ROS 2 Jazzy. | Keep standard system updates. |
| 2 | **CPU** | Multi-core x86_64 | Intel Core i7-13620H (10 cores / 16 threads, up to 4.9 GHz) | **PASS** | `lscpu` | None. Excellent multi-threaded capacity for MAPF algorithms and multi-robot nodes. | Allocate thread pools appropriately. |
| 3 | **RAM** | $\ge 16$ GiB | 30 GiB total (24 GiB available, 0B swap) | **PASS** | `free -h` | No swap configured. OOM risk only under extreme simulation loads. | Monitor memory during large multi-robot launches; configure 4-8 GiB swap if needed. |
| 4 | **GPU & Driver** | Modern GPU / Mesa or NVIDIA | Intel Raptor Lake-P [UHD Graphics] (rev 04), Driver `i915`, Mesa 25.2.8 | **WARNING** | `lspci -nnk`, `lshw -C display` | No discrete GPU / CUDA. Heavy 3D Gazebo GUI rendering with complex meshes may drop framerates. Headless server (`gz sim -s`) runs cleanly. | Use headless Gazebo for automated runs and CI; keep visual fidelity moderate in GUI. |
| 5 | **Storage** | $\ge 20$ GiB free NVMe | NVMe SSD: `/` has 26 GB free (43%), `/home` has 209 GB free (4%) | **PASS** | `df -h` | Ample disk space in `/home`. Root `/` has 26 GB. | Place workspace builds, logs, and experiment datasets in `/home`. |
| 6 | **ROS 2 Distribution** | ROS 2 Jazzy Jalisco | ROS 2 Jazzy (active, desktop full, 393 pkgs) | **PASS** | `ros2 doctor --report` & `printenv ROS_DISTRO` | None. Base environment is clean and sourced via `/etc/bash.bashrc` / `~/.bashrc`. | Maintain `/opt/ros/jazzy` as underlay. |
| 7 | **Gazebo Sim** | Gazebo Harmonic (Gz 8) | Gazebo Sim 8.11.0 (Harmonic) | **PASS** | `gz sim --versions` & `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` | None. Tested headless execution (`gz sim -s -r --iterations 50`) passed without errors. | Target Harmonic SDF specifications and plugins. |
| 8 | **ros_gz_sim Integration** | `ros_gz_sim`, `ros_gz_bridge` for Jazzy | `ros_gz_sim` 1.0.22, `ros_gz_bridge` 1.0.22 | **PASS** | `ros2 pkg xml ros_gz_sim` & `ros2 pkg xml ros_gz_bridge` | None. Standard upstream Gazebo Harmonic ROS bridge. | Use `ros_gz_bridge parameter_bridge` for clock, odometry, cmd_vel, laser scans. |
| 9 | **Nav2 Stack** | Nav2 for Jazzy | Nav2 1.3.12 (bringup, core, costmap_2d, controller, planner, etc.) | **PASS** | `ros2 pkg xml nav2_bringup` & `ros2 pkg list \| grep nav2` | None. Full stack present. | Use standard Nav2 lifecycle nodes and planners for single/multi-robot integration. |
| 10 | **RViz2** | RViz2 for Jazzy | RViz2 14.1.22 (`/opt/ros/jazzy/bin/rviz2`) | **PASS** | `ros2 pkg xml rviz2` & `which rviz2` | None. Works with Mesa OpenGL. | Maintain parameterized rviz configurations per robot namespace. |
| 11 | **Zenoh & rmw_zenoh_cpp** | `rmw_zenoh_cpp` for Jazzy | `rmw_zenoh_cpp` 0.2.10, `zenoh_cpp_vendor` 0.2.10, `rmw_zenohd` binary | **PASS** | `RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 doctor` | Tested runtime loading. Zenoh router executable is `ros2 run rmw_zenoh_cpp rmw_zenohd`. | Provide automated scripts to start/manage `rmw_zenohd` when running Zenoh experiments. |
| 12 | **Python Toolchain** | Python $\ge 3.10$ | Python 3.12.3, numpy 1.26.4, scipy 1.11.4, matplotlib 3.6.3, pyyaml 6.0.1, pytest 7.4.4, flake8 7.0.0 | **PASS** | `python3 --version` & module import checks | `networkx`, `pandas`, `pydantic` not installed system-wide. | Pure Python MAPF implementations can run without extra dependencies; install `python3-networkx` / `python3-pandas` if needed. |
| 13 | **C++ Toolchain** | GCC $\ge 13$, CMake $\ge 3.22$ | GCC/G++ 13.3.0, CMake 3.28.3, Make 4.3, Ninja 1.11.1 | **PASS** | `g++ --version`, `cmake --version`, `ninja --version` | None. Full C++20 and C++23 standards supported. | Use modern C++20 standard in CMake targets. |
| 14 | **colcon Build System** | colcon with ament extensions | colcon-core 0.21.1 with complete ament/cmake/python plugins | **PASS** | `colcon version-check` | None. Standard ROS 2 build toolchain functional. | Use `colcon build --symlink-install`. |
| 15 | **Git Version Control** | Git $\ge 2.30$ | Git 2.43.0 | **WARNING** | `git --version` & `git status` | Directory `.` is **not a git repository**. | Initialize Git repository (`git init`) in M0 bootstrap and commit baseline control plane. |
| 16 | **Docker** | Containerization tool | Not installed | **WARNING** | `which docker` | No Docker daemon available on host. Container-based orchestration cannot run natively without Docker. | Native Ubuntu 24.04 execution is preferred; Docker is optional for M0-M3. Document as optional. |
| 17 | **Existing Workspaces** | Inspect user workspaces | Found `~/nomeer_ws`, `~/sih_ws`, `~/install`, and `amr_fleet_ws.zip` | **PASS** | `find /home/raghav -maxdepth 2 -name "*ws*"` | `nomeer_ws` has `bcr_bot` (industrial AMR model); `sih_ws` has `sih_fleet`; `amr_fleet_ws.zip` has starter packages. `~/.bashrc` only sources `/opt/ros/jazzy/setup.bash`. | Do not overwrite or contaminate existing workspaces. Consider referencing `bcr_bot` URDF/SDF assets if needed. |
| 18 | **Installed ROS Robot Pkgs** | Robot descriptions / drivers | `nav2_minimal_tb3_sim`, `nav2_minimal_tb4_description`, `nav2_minimal_tb4_sim`, `diff_drive_controller`, `robot_state_publisher`, `xacro` | **PASS** | `ros2 pkg list \| grep -iE 'turtle\|tb\|diff\|robot'` | Standard differential drive controllers and robot description tools exist out of the box. | Utilize differential drive controller models and parameterized URDF/Xacro for fleet simulation. |

---

## 3. Evidence & Verification Commands

1. **Operating System & Kernel**:
   ```bash
   lsb_release -a
   # Distributor ID: Ubuntu, Description: Ubuntu 24.04.4 LTS, Release: 24.04, Codename: noble
   uname -a
   # Linux Raze 6.8.0-139-generic #139-Ubuntu SMP PREEMPT_DYNAMIC Sat Aug 1 03:52:05 UTC 2026 x86_64
   ```

2. **Compute & Memory**:
   ```bash
   lscpu | grep "Model name\|CPU(s):"
   # CPU(s): 16, Model name: 13th Gen Intel(R) Core(TM) i7-13620H (10 cores, 16 threads)
   free -h
   # Mem: 30Gi total, 6.6Gi used, 16Gi free, 24Gi available. Swap: 0B
   ```

3. **Storage**:
   ```bash
   df -h / /home
   # /dev/nvme0n1p5: 46G total, 19G used, 26G avail (43%) on /
   # /dev/nvme0n1p6: 229G total, 7.6G used, 209G avail (4%) on /home
   ```

4. **Graphics & Acceleration**:
   ```bash
   lspci -nnk | grep -A 3 -iE 'vga|3d|display'
   # 0000:00:02.0 VGA compatible controller [0300]: Intel Corporation Raptor Lake-P [UHD Graphics] [8086:a7a8] (rev 04)
   # Kernel driver in use: i915
   ```

5. **ROS 2 Jazzy & RMW**:
   ```bash
   ros2 doctor --report | grep -E 'distribution name|release platforms|middleware name'
   # distribution name: jazzy
   # release platforms: {'debian': ['bookworm'], 'rhel': ['9'], 'ubuntu': ['noble']}
   # middleware name: rmw_fastrtps_cpp
   ```

6. **Gazebo Harmonic**:
   ```bash
   gz sim --versions
   # 8.11.0
   gz sim -s -r -v 2 --iterations 50
   # Simulation runner executed 50 iterations cleanly, exit code 0.
   ```

7. **ros_gz Integration**:
   ```bash
   ros2 pkg xml ros_gz_sim | grep version
   # <version>1.0.22</version>
   ros2 pkg xml ros_gz_bridge | grep version
   # <version>1.0.22</version>
   ```

8. **Zenoh Integration**:
   ```bash
   ros2 pkg xml rmw_zenoh_cpp | grep version
   # <version>0.2.10</version>
   ros2 pkg executables rmw_zenoh_cpp
   # rmw_zenoh_cpp rmw_zenohd
   RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 doctor --report | grep -A 2 "RMW MIDDLEWARE"
   # middleware name: rmw_zenoh_cpp
   ```

9. **Nav2 & RViz2**:
   ```bash
   ros2 pkg xml nav2_bringup | grep version
   # <version>1.3.12</version>
   ros2 pkg xml rviz2 | grep version
   # <version>14.1.22</version>
   ```

10. **Build & Version Control**:
    ```bash
    python3 --version # Python 3.12.3
    g++ --version | head -n 1 # g++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0
    cmake --version | head -n 1 # cmake version 3.28.3
    colcon-core # 0.21.1
    git --version # git version 2.43.0
    git status # fatal: not a git repository
    ```

---

## 4. Compatibility Concerns & Risk Analysis

1. **Absence of Discrete GPU**:
   - *Impact*: Intel UHD integrated graphics will experience performance bottlenecks if attempting to render multi-robot 3D visual environments with high mesh counts in the Gazebo GUI.
   - *Mitigation*: Run simulations headless (`gz sim -s -r`) during regression tests, automated benchmarking, and fleet scaling sweeps. Use RViz2 with simplified URDF visual geometries when inspection is required.
   - *Feasibility*: Algorithmic scaling (Tier B: 500–10,000 agents) uses a lightweight discrete-event MAPF backend which runs entirely on CPU and is completely unconstrained by GPU availability.

2. **Git Repository Uninitialized**:
   - *Impact*: Inability to record git commit hashes in experiment metadata (violating PRD FR-10 and research reproducibility rules) until initialized.
   - *Mitigation*: Initialize git repository in M0, configure `.gitignore` (already present), and make an initial commit of the project control plane.

3. **rmw_zenoh_cpp Router Requirement**:
   - *Impact*: When selecting `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, nodes in router mode require an active Zenoh router (`rmw_zenohd`) for peer discovery and pub/sub bridging.
   - *Mitigation*: Include automated router lifecycle management in launch files or test harnesses (`ros2 run rmw_zenoh_cpp rmw_zenohd`).

4. **Missing Python Graph Packages**:
   - *Impact*: If algorithms use `networkx` or data export uses `pandas`, imports will fail in standard python scripts.
   - *Mitigation*: Keep core MAPF algorithms self-contained with zero third-party dependencies (pure C++ or standard Python library `collections`, `heapq`, `math`). If higher-level analysis requires `networkx`/`pandas`, record them in requirements and install via system package manager when approved.

---

## 5. Recommended Actions for M0 Bootstrap

1. **Initialize Git Repository**: Run `git init` in `.` and create the initial commit covering the control plane.
2. **Establish ROS 2 Workspace Structure**: Create `src/` directory with a standard modular package skeleton:
   - `amr_fleet_msgs`: Custom message, service, and action definitions.
   - `amr_fleet_core`: Core algorithmic abstractions (TaskAllocator, GlobalPlanner, LocalPlanner, DeadlockManager).
   - `amr_fleet_bringup`: Launch scripts, robot descriptions, Gazebo world definitions.
   - `amr_fleet_sim`: Lightweight MAPF simulation backend and headless Gazebo orchestration.
3. **Setup Colcon Build & Test Scripts**: Implement deterministic build (`colcon build --symlink-install`) and test scripts (`colcon test`) with automated linting and unit test reporting.
4. **Implement Environment Verification Script**: Write a standalone automated script (`scripts/verify_environment.sh` or `scripts/verify_m0.py`) that checks the environment reproducibly.
