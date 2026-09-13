# Implementation Plan: Milestone M3 — Task Generation & Task Lifecycle Infrastructure

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Status**: Ready for Implementation  
**Target Milestone**: M3 — Task Generation & Task Lifecycle Infrastructure  
**Target System**: ROS 2 Jazzy, Gazebo Harmonic 8.11.0, Ubuntu 24.04 LTS  

---

## 1. Goal Description

Establish a rigorous, deterministic, decoupled task representation, generation system, and lifecycle state machine for the NRDAS AMR fleet coordination project.

This milestone provides the formal task foundation that future decentralized coordination algorithms (Milestone M4 CBBA / ACBBA) and lifelong MAPF planners (Milestone M6 PIBT / M7 RHCR) will consume.

```
+-----------------------------------------------------------------------------------+
|                           M3 TASK ARCHITECTURE                                   |
|                                                                                   |
|  +---------------------------+       +-----------------------------------------+  |
|  | Workload Configuration    | ----> | Deterministic Task Generator            |  |
|  | - config/workloads/*.yaml |       | - Seeded PRNG                           |  |
|  | - Station & free regions  |       | - Map boundary & obstacle validation    |  |
|  | - Priority & deadlines    |       | - Independent of robot assignment       |  |
|  +---------------------------+       +-------------------+---------------------+  |
|                                                          |                        |
|                                                          v                        |
|                                      +-----------------------------------------+  |
|                                      | Task Lifecycle State Machine            |  |
|                                      | - PENDING, ASSIGNED, IN_PROGRESS,       |  |
|                                      |   COMPLETED, FAILED, CANCELLED          |  |
|                                      | - Enforced valid/invalid transitions    |  |
|                                      | - Complete lifecycle event log          |  |
|                                      | - Truthful duration & waiting metrics   |  |
|                                      +-------------------+---------------------+  |
|                                                          |                        |
|                                                          v                        |
|                                      +-----------------------------------------+  |
|                                      | ROS 2 Task Interface (amr_fleet_msgs)   |  |
|                                      | - /tasks/available (TaskDefinition[])   |  |
|                                      | - /tasks/events (TaskEvent)             |  |
|                                      | - /tasks/update_status (Service/Topic)  |  |
|                                      +-------------------+---------------------+  |
|                                                          |                        |
|                        +---------------------------------+                        |
|                        |                                                          |
|                        v [FUTURE M4]                                              |
|      +-----------------------------------+                                        |
|      | Decentralized CBBA Task Allocator | (NOT implemented in M3)                |
|      +-----------------------------------+                                        |
+-----------------------------------------------------------------------------------+
```

---

## 2. Strict Boundary Invariants

- **No Premature Allocation**: The task generator creates work. It does **not** decide which robot performs the work. CBBA/ACBBA or auction algorithms must **not** be implemented in M3.
- **No Path Planning**: The task system creates pickup and dropoff spatial coordinates. It does **not** compute collision-free trajectories or assign time-space reservations.
- **Zero Simulation Physics Alterations**: Robot descriptions, physics parameters, sensor bridges, and Gazebo world models must remain $100\%$ untouched.
- **Truthful Telemetry**: Task metrics (waiting time, execution duration, lateness) are computed strictly from real event timestamps. If an event has not occurred, the metric is `None` / unavailable.

---

## 3. Detailed Component Plan

### 3.1. Task Data Model & Lifecycle State Machine (`src/amr_fleet_core/amr_fleet_core/`)
1. **`task_model.py`**:
   - `TaskPriority`: Enum (`LOW = 1`, `NORMAL = 2`, `HIGH = 3`, `CRITICAL = 4`).
   - `TaskLifecycleState`: Enum (`PENDING`, `ASSIGNED`, `IN_PROGRESS`, `COMPLETED`, `FAILED`, `CANCELLED`).
   - `TaskEvent`: Dataclass capturing timestamp, event type, from_state, to_state, robot_id, and details.
   - `Task`: Dataclass with `task_id`, `pickup` $(x, y)$, `dropoff` $(x, y)$, `priority`, `created_at`, `deadline`, `current_state`, `assigned_robot_id`, and `events`.
   - `TaskStateMachine`: Explicit transition validator:
     - `PENDING -> ASSIGNED`
     - `PENDING -> CANCELLED`
     - `ASSIGNED -> IN_PROGRESS`
     - `ASSIGNED -> PENDING` (e.g. preemption or task release)
     - `ASSIGNED -> CANCELLED`
     - `ASSIGNED -> FAILED`
     - `IN_PROGRESS -> COMPLETED`
     - `IN_PROGRESS -> FAILED`
     - `IN_PROGRESS -> CANCELLED`
     - Rejects any other transition with `InvalidTaskTransitionError`.
   - Lifecycle duration calculations:
     - `waiting_time`: $\Delta t$ between `created_at` and `started_at`.
     - `execution_time`: $\Delta t$ between `started_at` and completion/failure/cancellation.
     - `total_duration`: $\Delta t$ between `created_at` and completion/failure/cancellation.
     - `deadline_slack`: $\text{deadline} - \text{completed_at}$ (positive = on-time, negative = tardy).

### 3.2. Deterministic Task Generator (`task_generator.py`)
1. **Configuration (`TaskGeneratorConfig`)**:
   - `task_count`: Total tasks to generate.
   - `seed`: Integer PRNG seed for exact repeatability.
   - `pickup_mode` / `dropoff_mode`: `STATION_PAIR`, `BOUNDED_RANDOM`, `CORRIDOR_TRANSIT`.
   - `priority_weights`: Mapping of `TaskPriority` to relative distribution weights.
   - `deadline_window`: Optional $(\Delta t_{\min}, \Delta t_{\max})$ from task creation.
   - `min_pickup_dropoff_distance`: Minimum Euclidean distance between pickup and dropoff (e.g., $1.5\,\text{m}$).
2. **Spatial Validation**:
   - Validates coordinates are strictly within warehouse map bounds ($0.5 \le x \le 15.5$, $0.5 \le y \le 15.5$).
   - Validates coordinates do not lie inside static obstacle bounding boxes (racks at $[4.5, 5.5]$, $[4.5, 10.5]$, $[11.5, 5.5]$, $[11.5, 10.5]$).
3. **Determinism Guarantee**:
   - Identical `(config, seed)` produces byte-identical task sequence.
   - Different seeds produce statistically distinct task sequences.

### 3.3. Workload Configuration Loader & Benchmark Scenarios (`config/workloads/`)
1. **`workload_small_deterministic.yaml`**: 5 tasks, deterministic station pairs, seed 42.
2. **`workload_medium_priority.yaml`**: 15 tasks, mixed priority distribution (LOW/NORMAL/HIGH/CRITICAL), seed 101.
3. **`workload_benchmark_deadlines.yaml`**: 25 tasks, tight deadline constraints, seed 202.

### 3.4. ROS 2 Message Definitions (`src/amr_fleet_msgs/`)
1. **`TaskDefinition.msg`**:
   ```
   std_msgs/Header header
   string task_id
   geometry_msgs/Point pickup_pose
   geometry_msgs/Point dropoff_pose
   int32 priority
   builtin_interfaces/Time created_at
   builtin_interfaces/Time deadline
   string status
   string assigned_robot_id
   ```
2. **`TaskEvent.msg`**:
   ```
   std_msgs/Header header
   string task_id
   string event_type
   string previous_state
   string new_state
   string robot_id
   builtin_interfaces/Time timestamp
   string details
   ```
3. **`TaskList.msg`**:
   ```
   std_msgs/Header header
   amr_fleet_msgs/TaskDefinition[] tasks
   ```

### 3.5. ROS 2 Task Manager Node (`src/amr_fleet_core/amr_fleet_core/task_manager_node.py`)
- Subscribes to / serves:
  - Publishes `/tasks/all` (`TaskList`) and `/tasks/available` (`TaskList`) periodically.
  - Publishes `/tasks/events` (`TaskEvent`) on state changes.
  - Exposes topic `/tasks/transition` or `/tasks/update_status` for future allocators to claim or update tasks.

### 3.6. Fleet Dashboard Integration (`scripts/fleet_dashboard.py`)
- Update `scripts/fleet_dashboard.py` to subscribe to `/tasks/all` and `/tasks/events`!
- Instead of displaying static placeholder text, the dashboard will now render live task counts (Pending, Assigned, In Progress, Completed, Failed, Cancelled) and task queue cards!

---

## 4. Verification & Testing Strategy

1. **Unit Test Suite (`src/amr_fleet_core/test/`)**:
   - `test_task_model.py`: Test task creation, valid transitions, invalid transitions rejection, metrics calculation, and serialization.
   - `test_task_generator.py`: Test deterministic reproducibility across identical seeds, divergence across different seeds, obstacle collision avoidance, boundary checking, and priority weighting.
   - `test_workload_config.py`: Test YAML parsing, schema validation, and benchmark workload loading.
2. **Regression & Full Suite**:
   - Run `colcon test` across all 5 packages.
   - Run `verify_m1_simulation.py` and `verify_m2_fleet.py`.
   - Run `run_real_integration_validation.py` for 2, 5, and 10 robots.
   - Verify M2.5 visualization launches without degradation.

---

## 5. Checkpoint Deliverables
- `docs/plans/M3-task-system-plan.md`
- `docs/checkpoints/M3.md`
- Final line: `M3 CHECKPOINT — WAITING FOR HUMAN APPROVAL`
