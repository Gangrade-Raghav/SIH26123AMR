# Milestone M4 Implementation Plan: Decentralized Task Allocation with CBBA

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Status**: APPROVED & COMPLETED  
**Milestone**: M4 — Decentralized Task Allocation with Consensus-Based Bundle Algorithm (CBBA)  
**Target Environment**: ROS 2 Jazzy, Gazebo Harmonic 8.11.0 / 8.15.0, Ubuntu 24.04 LTS  

---

## 1. Executive Objective

The objective of Milestone M4 is to introduce decentralized multi-robot task allocation using the **Consensus-Based Bundle Algorithm (CBBA)** (Choi, Brunet, How, IEEE Trans. Robotics, 2009).

Given:
- Multiple AMRs with spatial positions derived from ROS 2 odometry (`/${robot_id}/odom`).
- A set of available tasks published by the M3 Task Manager (`/tasks/available`).
- Task pickup and dropoff locations, priorities (LOW, NORMAL, HIGH, CRITICAL), and optional deadlines.

AMR robot agents must independently evaluate available tasks, compute marginal utilities, construct task bundles, exchange bid vectors over decentralized communication topics (`/fleet/cbba_bids`), and reach fleet-wide consensus without any centralized master coordinator or hidden solver.

---

## 2. Architectural Boundaries & Non-Goals

Strict milestone invariants are observed:
1. **Genuinely Decentralized**: Each robot executes its own autonomous node instance (`CBBANode`) within its namespace. No centralized solver, master node, or hidden shared memory allocator is permitted.
2. **M3 Lifecycle Integration**: When an agent's bundle converges and stabilizes, tasks transition from `PENDING` to `ASSIGNED` via `/tasks/update_status`. Robots do **NOT** execute motion, plan paths across obstacles, or mark tasks `IN_PROGRESS`/`COMPLETED` (those belong to M5/M6/M7).
3. **Zero Physics Alterations**: Robot URDFs, controllers, friction/inertial models, and Gazebo world parameters are 100% preserved.
4. **Subsystem Isolation**:
   - Rolling-Horizon Collision Resolution (RHCR / GD-RHCR) is strictly deferred to M7.
   - Priority-Inheritance Bump (PIBT) velocity coordination is strictly deferred to M6.
   - Wait-For-Graph (WFG) deadlock handling is strictly deferred to M8.
   - Network fault injection (packet loss, comms dropout) is strictly deferred to M9.

---

## 3. Implementation Breakdown

### 3.1. ROS 2 Interface Messages (`amr_fleet_msgs`)
- **`CBBABid.msg`**:
  - `std_msgs/Header header`
  - `string robot_id`
  - `uint32 iteration`
  - `string[] task_ids`
  - `float64[] winning_bids`
  - `string[] winning_robots`
  - `float64[] timestamps`
- **`RobotBundle.msg`**:
  - `std_msgs/Header header`
  - `string robot_id`
  - `string[] task_ids`
  - `float64[] bid_values`
  - `bool is_converged`

### 3.2. CBBA Core Algorithmic Engine (`amr_fleet_core`)
- **`cbba_agent.py`**:
  - `CBBAConfig`: Max bundle size $L_t$, priority weights, distance weight, deadline late penalty, discount factor $\lambda$, nominal speed, and tie-breaking epsilon.
  - `CBBALocalState`: Bundle sequence $b_i$, path waypoints $p_i$, winning bids $y_i$, winning robots $z_i$, decision timestamps $s_i$.
  - `compute_marginal_utility`: Cumulative route distance scoring guaranteeing Diminishing Marginal Utility (DMU).
  - `build_bundle`: Greedy bundle expansion with marginal scoring up to capacity limit.
  - `resolve_conflicts`: 18-rule CBBA Table 1 consensus matrix with cascade bundle drop and deterministic lexicographical tie-breaking.
- **`cbba_allocator.py`**:
  - Implements `TaskAllocator` interface for batch/algorithmic simulation and benchmarks. Simulates decentralized communication diameter convergence.
- **`cbba_node.py`**:
  - ROS 2 node running in robot namespace (`amr_0`, `amr_1`, etc.).
  - Subscribes to `/${robot_id}/odom`, `/tasks/available`, and `/fleet/cbba_bids`.
  - Publishes to `/fleet/cbba_bids`, `/${robot_id}/bundle`, and `/tasks/update_status`.

### 3.3. Launch & Demonstration Layer (`amr_fleet_bringup`)
- **`cbba_fleet.launch.py`**:
  - Launches Gazebo warehouse simulation, spawns $N$ AMRs, launches M3 Task Manager with workload, and launches $N$ decentralized `cbba_node` instances.

### 3.4. Telemetry & Observability (`scripts/fleet_dashboard.py`)
- Live CBBA consensus state display, bid broadcast counter, allocated robot bundles, makespan estimation, and 2D canvas bundle route rendering.

---

## 4. Verification Strategy

1. **Automated Unit Tests**:
   - `test_cbba_bidding.py`: Distance awareness, priority weighting, deadline penalty, bundle capacity.
   - `test_cbba_consensus.py`: Outbidding, cascade drop, deterministic tie-breaking.
   - `test_cbba_allocation.py`: Multi-robot disjoint assignment, 100% bitwise determinism.
2. **Empirical ROS 2 Integration**:
   - `scripts/verify_m4_cbba.py`: 5 live AMRs converging in ROS 2, assigning 15 workload tasks with 0 duplicates, validating M3 Task Manager state transitions.
3. **M0–M3 Regression**:
   - Zero regression in Gazebo simulation, robot isolation, cross-talk displacement ($0.000\,\text{m}$), and task state machines.
