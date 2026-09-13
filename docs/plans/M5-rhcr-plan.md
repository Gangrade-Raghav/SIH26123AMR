# Milestone M5 Implementation Plan: Rolling-Horizon Task Planning

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Status**: COMPLETED & VERIFIED  
**Milestone**: M5 — Rolling-Horizon Task Planning (RHCR Core Engine)  
**Target Environment**: ROS 2 Jazzy, Gazebo Harmonic 8.11.0 / 8.15.0, Ubuntu 24.04 LTS  

---

## 1. Executive Objective

The objective of Milestone M5 is to convert each autonomous mobile robot's (AMR) CBBA-assigned task bundle into a deterministic rolling-horizon execution plan.

While **M4** answers:
> *"Which robot gets which tasks?"*

**M5** answers:
> *"Given my assigned tasks, what should I execute next, and what route/schedule should I request over the planning horizon ($h$) and execution window ($w$)?"*

```
CBBA Task Allocation (M4)
          ↓
Assigned Bundle [t_1, t_2, ..., t_k]
          ↓
Task Sequencer Heuristics (Priority / Shortest Path / Deadline)
          ↓
Rolling-Horizon Task Planner (h=10 steps, w=4 steps)
          ↓
Deterministic Single-Agent A* on 2D Grid
          ↓
Planning Request & Response Messages
          ↓
Motion Execution along Window w
          ↓
Deterministic Replanning Triggers (Arrival, Window Expiry, Bundle Change)
```

---

## 2. Invariants, Scope Boundaries & Non-Goals

1. **Strict M5 Boundary (Single-Agent Rolling Horizon)**:
   - M5 focuses exclusively on converting bundles into sequenced sub-goals, single-agent A* trajectories, horizon truncation ($h$), and execution window execution ($w$).
   - **DO NOT** implement multi-agent reservation tables, token passing, or Priority-Inheritance Bump (PIBT) — strictly deferred to **M6**.
   - **DO NOT** implement Wait-For-Graph (WFG) deadlock resolution — strictly deferred to **M8**.
   - **DO NOT** implement Non-Holonomic ORCA (NH-ORCA) velocity obstacle filtering — strictly deferred to **M6**.
   - **DO NOT** implement communication fault injection or adaptive compute degradation — strictly deferred to **M9/M10**.
2. **100% Bundle Preservation Invariant**:
   - The task sequencer strictly processes the set of tasks assigned to the robot by CBBA. Zero tasks are dropped, duplicated, or reassigned during sequencing.
3. **Sub-Goal State Machine**:
   - Each task is decomposed into two sequential spatial sub-goals:
     $$\text{IDLE} \longrightarrow \text{TRANSIT\_TO\_PICKUP} \longrightarrow \text{TRANSIT\_TO\_DROPOFF} \longrightarrow \text{NEXT\_TASK} \dots$$
   - Sub-goal arrival triggers automatic task lifecycle transition notifications to the M3 Task Manager (`/tasks/update_status`).
4. **Deterministic Single-Agent Pathfinding**:
   - 4-connected grid A* pathfinder with deterministic lexicographical tie-breaking $(f, h, \text{counter}, \text{pos})$.
5. **Deterministic Replanning Triggers**:
   - Replanning occurs if and only if:
     1. Arrival at current sub-goal waypoint (within $0.5\,\text{m}$ tolerance).
     2. Completion of $w$ steps within the current execution window.
     3. Mutation in the robot's assigned CBBA bundle or task metadata.
     4. Current plan path is empty while tasks remain queued.

---

## 3. Implementation Breakdown

### 3.1. ROS 2 Interface Messages (`amr_fleet_msgs`)
- **`PlanningRequest.msg`**:
  - `std_msgs/Header header`
  - `string robot_id`
  - `geometry_msgs/Point start_pose`
  - `geometry_msgs/Point goal_pose`
  - `uint32 horizon_steps`
  - `uint32 execution_window`
  - `string task_id`
  - `string sub_goal_type` (`PICKUP`, `DROPOFF`)
- **`PlanningResponse.msg`**:
  - `std_msgs/Header header`
  - `string robot_id`
  - `string task_id`
  - `string sub_goal_type`
  - `geometry_msgs/Point[] full_path`
  - `geometry_msgs/Point[] horizon_path`
  - `geometry_msgs/Point[] execution_path`
  - `float64 total_cost`
  - `float64 planning_latency_ms`
  - `bool success`
  - `string status_message`
  - `uint32 horizon_steps`
  - `uint32 execution_window`
  - `uint32 replan_count`
- **`RollingHorizonPlan.msg`**:
  - `std_msgs/Header header`
  - `string robot_id`
  - `string[] assigned_bundle`
  - `string current_task_id`
  - `string current_phase` (`IDLE`, `TRANSIT_TO_PICKUP`, `TRANSIT_TO_DROPOFF`)
  - `geometry_msgs/Point current_goal`
  - `geometry_msgs/Point[] horizon_path`
  - `geometry_msgs/Point[] execution_path`
  - `uint32 horizon_steps`
  - `uint32 execution_window`
  - `uint32 replan_count`
  - `float64 planning_latency_ms`
  - `bool is_valid`

### 3.2. Warehouse Grid Representation (`amr_fleet_sim`)
- Extended `GridWorld` with world-to-grid (`to_grid`) and grid-to-world (`to_world`) coordinate mappings, obstacle addition/removal, and factory method `create_warehouse_grid(resolution=0.5, warehouse_size=16.0)` representing the physical obstacles (shelving racks, perimeter walls) of the Gazebo warehouse.

### 3.3. Algorithmic Planning Engine (`amr_fleet_core`)
- **`SingleAgentAStar`**: Deterministic Manhattan A* pathfinding.
- **`TaskSequencer`**: Modular heuristic ordering:
  - `PRIORITY_FIRST`: Sorts tasks by priority descending, then distance ascending.
  - `SHORTEST_PATH_FIRST`: Greedy nearest-neighbor TSP heuristic starting from robot pose.
  - `DEADLINE_FIRST`: Earliest deadline first (EDF).
  - `BUNDLE_ORDER`: Retains exact sequence produced by CBBA.
- **`RollingHorizonPlanner`**:
  - Maintains state machine (`IDLE`, `TRANSIT_TO_PICKUP`, `TRANSIT_TO_DROPOFF`).
  - Computes full path, truncates to horizon $h$, extracts window $w$.
  - Tracks steps executed in window and verifies replan triggers.
- **`RollingHorizonPlannerNode` (`rh_node`)**:
  - Decentralized node running per AMR.
  - Publishes `/{robot_id}/rolling_plan`, `/{robot_id}/planning_request`, `/{robot_id}/planning_response`, `/{robot_id}/cmd_vel`, and `/tasks/update_status`.
  - Maps odometry coordinates with robot spawn offsets (`spawn_x`, `spawn_y`) into the global warehouse reference frame.

### 3.4. System Launch & Telemetry (`amr_fleet_bringup`, `scripts`)
- **`m5_rh_fleet.launch.py`**: Full fleet launch file starting Gazebo Harmonic, AMR spawners, Task Manager, CBBA nodes, and RH planner nodes.
- **Brutalist Fleet Dashboard (`scripts/fleet_dashboard.py`)**: Real-time rendering of rolling-horizon paths (dashed orange lines), execution windows (solid white lines), sub-goal phases, replan counters, and planning latencies.

---

## 4. Verification & Validation Summary

| Test Suite / Script | Coverage Area | Result |
| :--- | :--- | :--- |
| `test_rh_planning.py` | A* straight-line, obstacles, unreachable, identical start/goal, determinism, empty/single bundle | 100% Pass (7/7) |
| `test_rh_horizon.py` | Horizon truncation ($h$), execution window ($w$), arrival replanning, window exhaustion replanning | 100% Pass (5/5) |
| `test_rh_sequencing.py` | `PRIORITY_FIRST`, `SHORTEST_PATH_FIRST`, `DEADLINE_FIRST`, bundle preservation, sub-goal lifecycle | 100% Pass (5/5) |
| `verify_m5_rhcr.py` | End-to-end algorithmic and live ROS 2 node interface verification | 100% Pass (5/5) |
| `demonstrate_real_gazebo_m5.py` | Real Gazebo Harmonic simulation with 5 AMRs, CBBA allocation, and live motion execution | 100% Pass (5/5 AMRs moved, 37 plans generated) |
| `scripts/run_tests.sh` | Full repository pytest suite across all 5 packages | 92 tests, 0 failures |
| `verify_m1` - `verify_m4` | Complete regression test suite across simulation, fleet isolation, task manager, and CBBA | 100% Pass |
