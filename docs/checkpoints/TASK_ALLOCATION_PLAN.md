# NRDAS — OPERATOR TASK ALLOCATION & LIVE TASK CONTROL PLAN

**Milestone / Feature**: Operator Task Allocation & Live Task Control Layer  
**Target Repository**: `antigravity_amr_project`  
**Date**: September 15, 2026  
**Status**: DESIGN COMPLETE — AWAITING IMPLEMENTATION

---

## 1. Existing Task Architecture

The current NRDAS task execution pipeline is structured across four primary layers:

1. **Task Data Model (`amr_fleet_core.task_model`)**:
   - Discrete state machine: `STAGED`, `PENDING`, `ASSIGNED`, `IN_PROGRESS`, `COMPLETED`, `FAILED`, `CANCELLED`.
   - Strict transition guards via `VALID_TRANSITIONS` and `InvalidTaskTransitionError`.
   - Immutable audit trail recording every state transition (`TaskEvent`) with timestamps and optional robot attribution.
   - Domain class `Task` holds `task_id`, `pickup (x, y)`, `dropoff (x, y)`, `priority`, `created_at`, `deadline`, and `release_time_sec`.

2. **Task Manager Node (`amr_fleet_core.task_manager_node`)**:
   - Central authority for initial workload ingestion from YAML files.
   - Periodically publishes `/tasks/all` (`amr_fleet_msgs/TaskList`) and `/tasks/available` (`amr_fleet_msgs/TaskList` containing only `PENDING` tasks).
   - Manages dynamic release of `STAGED` tasks when simulation elapsed time crosses `task.release_time_sec`.
   - Listens to `/tasks/update_status` (`TaskEventMsg`) to advance tasks to `ASSIGNED`, `IN_PROGRESS`, `COMPLETED`, or `FAILED`.

3. **Decentralized Task Allocation Layer (`amr_fleet_core.cbba_node` / `cbba_agent`)**:
   - Each AMR runs an independent, decentralized Consensus-Based Bundle Algorithm (CBBA) instance.
   - Subscribes to `/tasks/available` and evaluates marginal utility based on distance, priority weighting, and deadline tightness.
   - Communicates peer-to-peer over `/fleet/cbba_bids` through simulated network degradation filters (latency, jitter, packet loss).
   - Once bundle consensus converges, the winning AMR publishes an `ASSIGNED` event on `/tasks/update_status` and outputs its bundle on `/{robot_id}/bundle`.

4. **Rolling-Horizon Execution & Coordination Layer (`rh_planner` / `rh_node`)**:
   - Ingests converged bundles from `/{robot_id}/bundle`.
   - Sequences pickup and dropoff sub-goals using heuristic ordering (e.g. shortest path first).
   - Coordinates multi-agent trajectories via Space-Time Reservations (`/fleet/reservations`), PIBT local resolution, and Wait-For-Graph (WFG) deadlock recovery.
   - Triggers task status progression (`IN_PROGRESS` at pickup, `COMPLETED` at dropoff).

---

## 2. Existing ROS 2 Interfaces

### Topics
- `/tasks/all` (`amr_fleet_msgs/msg/TaskList`): Complete catalog of all known tasks with current lifecycle status.
- `/tasks/available` (`amr_fleet_msgs/msg/TaskList`): Filtered subset of tasks in `PENDING` state available for CBBA bidding.
- `/tasks/events` (`amr_fleet_msgs/msg/TaskEvent`): Event stream publishing all task state transitions and lifecycle audits.
- `/tasks/update_status` (`amr_fleet_msgs/msg/TaskEvent`): Ingestion topic where agents report task transitions.
- `/tasks/start_mission` (`amr_fleet_msgs/msg/TaskEvent`): Mission clock anchor synchronization.
- `/fleet/cbba_bids` (`amr_fleet_msgs/msg/CBBABid`): Broadcast topic for peer-to-peer CBBA bid vectors.
- `/{robot_id}/bundle` (`amr_fleet_msgs/msg/RobotBundle`): Robot-specific converged task bundle and bid values.
- `/fleet/reservations` (`amr_fleet_msgs/msg/SpaceTimeReservation`): Fleet-wide space-time coordination reservations.
- `/fleet/conflicts` (`amr_fleet_msgs/msg/ConflictReport`): Collision and conflict event reporting.
- `/fleet/deadlocks` (`amr_fleet_msgs/msg/DeadlockEvent`): Wait-for-graph cycle and deadlock resolution events.
- `/fleet/coordination_status` (`amr_fleet_msgs/msg/CoordinationStatus`): Aggregate multi-robot coordination metrics.

### Services
- Currently, **zero** custom ROS 2 services exist in `amr_fleet_msgs/srv`. All existing interactions are asynchronous topic broadcasts.

---

## 3. Existing Dashboard Functionality

The NRDAS Fleet Observability Dashboard (`scripts/fleet_dashboard.py`) currently provides:
- A standalone web server running at `http://localhost:8080` (zero external npm/pip dependencies).
- Read-only telemetry:
  - Discovered AMRs table (pose, yaw, linear speed, distance traveled, LiDAR frequency, obstacle distance).
  - 2D warehouse mini-map rendering robot positions, orientations, and waypoints.
  - High-level task summary counts (`total`, `pending`, `assigned`, `in_progress`, `completed`, `failed`, `cancelled`).
  - System performance (host CPU %, RAM usage, simulation RTF, simulation elapsed time).
  - Multi-agent coordination metrics (reservations count, conflicts count, deadlocks count).
  - Communication degradation metrics (packet delivery, drop rate, latency, jitter).
- Terminal TUI mode using `rich` for headless CLI monitoring.
- REST endpoint `GET /api/state` returning JSON representation of the entire fleet telemetry.

**Current Limitations**:
- Read-only: no capability to submit or create new tasks.
- No task control buttons (cannot cancel or requeue tasks).
- No individual task inspection table showing pickup/dropoff coordinates, timestamps, and assigned robots.
- No CBBA bid vector breakdown showing candidate bids from individual AMRs for each task.

---

## 4. Missing Capabilities

To fulfill the operator task allocation requirements, the following capabilities must be implemented:
1. **Live Task Creation Service**:
   - A ROS 2 service `/tasks/create` that accepts task parameters (pickup, dropoff, priority, deadline, optional requested robot) and instantiates a valid `Task` inside `TaskManagerNode`.
2. **Task Control Service**:
   - A ROS 2 service `/tasks/control` allowing operators to safely cancel or requeue tasks.
3. **Allocation Constraint Modeling (`AUTO` vs `DIRECT`)**:
   - Support for `requested_robot` field in task definitions.
   - When set to `AUTO` (or omitted), decentralized CBBA evaluates and allocates the task automatically.
   - When set to a specific robot ID (e.g. `amr_4`), only that robot bids on and accepts the task. If that robot cannot accept or is invalid, the system reports an explicit rejection or failure rather than silently falsifying CBBA consensus.
4. **Bundle Invalidation & Purging**:
   - CBBA agents must purge `CANCELLED` and `FAILED` tasks from their bundles and winning bid maps so capacity is released.
   - RHCR planners must detect if an active task was cancelled/requeued and safely decelerate to zero, clear space-time reservations, and transition to IDLE or the next task without kinematically abrupt or unsafe maneuvers.
5. **Command-Line Interface**:
   - A CLI utility `scripts/create_task.py` allowing human operators to create tasks directly from the terminal.
6. **Interactive Dashboard UI**:
   - A modal or form `+ CREATE TASK` on the web dashboard.
   - REST endpoints `POST /api/task/create` and `POST /api/task/control` in the dashboard HTTP server that bridge web requests to ROS 2 services.
   - A live Task Management Table displaying all tasks, coordinates, states, assigned robots, and action buttons.
   - A CBBA Allocation Viewer showing winner and verified bid values for tasks.

---

## 5. Proposed Architecture

```
                 +-----------------------+      +--------------------------+
                 |  Web Dashboard UI     |      |  CLI Tool                |
                 |  (Form + Action Btns) |      |  (scripts/create_task.py)|
                 +-----------+-----------+      +------------+-------------+
                             |                               |
                   POST /api/task/create                     |
                             |                               |
                             v                               v
                     [Dashboard Bridge]                      |
                             |                               |
                             +---------------+---------------+
                                             |
                                    ROS 2 Service Call
                                     `/tasks/create`
                                     `/tasks/control`
                                             |
                                             v
                             +-------------------------------+
                             |      TaskManagerNode          |
                             |  - Validates coordinates      |
                             |  - Generates unique Task ID   |
                             |  - Checks requested_robot     |
                             |  - Ingests into Task Pool     |
                             |  - Emits TaskEvent            |
                             +---------------+---------------+
                                             |
                                   Publishes /tasks/available
                                   (State: PENDING)
                                             |
                                             v
                      +---------------------------------------------+
                      |       Decentralized CBBA Fleet Layer        |
                      |  - If AUTO: All AMRs bid & reach consensus  |
                      |  - If DIRECT: Only requested AMR bids       |
                      +----------------------+----------------------+
                                             |
                                    Winner commits to
                                   `/tasks/update_status`
                                             |
                                             v
                             +-------------------------------+
                             |      TaskManagerNode          |
                             |  (State: PENDING -> ASSIGNED) |
                             +---------------+---------------+
                                             |
                                   Emits /{robot_id}/bundle
                                             |
                                             v
                             +-------------------------------+
                             |    RHCR Execution & Safety    |
                             |  - Sequences sub-goals        |
                             |  - Space-Time Reservations    |
                             |  - PIBT Local Coordination    |
                             |  - Emergency Braking Backstop |
                             +---------------+---------------+
```

### Safety Authority Invariant
$$\text{Local Safety / Emergency Braking} \succ \text{Reservations / PIBT Collision Avoidance} \succ \text{RHCR Path Planning} \succ \text{Task Allocation}$$

Task allocation remains strictly at the bottom of the authority hierarchy. Operator commands (creation, cancellation, requeue) can never disable emergency braking, bypass reservations, or command velocities directly.

---

## 6. Files to Modify

1. `src/amr_fleet_msgs/CMakeLists.txt`:
   - Register new service files (`srv/CreateTask.srv`, `srv/ControlTask.srv`).
2. `src/amr_fleet_msgs/msg/TaskDefinition.msg`:
   - Add `string requested_robot` to expose allocation constraints over ROS 2 topics.
3. `src/amr_fleet_core/amr_fleet_core/task_model.py`:
   - Support `requested_robot` attribute in `Task` class, dictionary serialization, and deserialization.
   - Update `VALID_TRANSITIONS` to support `FAILED -> PENDING` (requeue) and audit event generation for `REQUEUED`.
4. `src/amr_fleet_core/amr_fleet_core/task_manager_node.py`:
   - Implement service servers for `/tasks/create` and `/tasks/control`.
   - Add input validation: bounding box checks against warehouse dimensions, duplicate task ID checks, auto-generation of task IDs (`T{idx}`), priority validation, and robot existence validation.
   - Handle cancellation and requeue requests safely with state transitions and event emission.
5. `src/amr_fleet_core/amr_fleet_core/cbba_agent.py`:
   - In `build_bundle`: filter candidate tasks based on `requested_robot`. If a task specifies a robot and `task['requested_robot'] != self.robot_id`, skip it.
6. `src/amr_fleet_core/amr_fleet_core/cbba_node.py`:
   - In `_consensus_cycle`: purge any tasks from `self.agent.state.bundle` that have transitioned to `CANCELLED` or `FAILED`.
   - Propagate `requested_robot` from `TaskDefinition` into `task_pool`.
7. `src/amr_fleet_core/amr_fleet_core/rh_planner.py`:
   - In `update_assigned_bundle`: if an active in-progress task is removed or cancelled from the assigned bundle, cleanly transition `active_phase` to `IDLE` and clear `current_goal` so the robot safely comes to a halt without kinematic instability.
8. `src/amr_fleet_core/amr_fleet_core/rh_node.py`:
   - Ensure clean deceleration and space-time reservation release when active tasks are aborted or cancelled.
9. `scripts/fleet_dashboard.py`:
   - Add HTTP `POST /api/task/create` and `POST /api/task/control` handlers that call ROS 2 services.
   - Add web UI modal and form for `+ CREATE TASK` (Task ID, Pickup X/Y, Dropoff X/Y, Priority, Deadline, Assignment mode AUTO/DIRECT, Robot dropdown).
   - Add interactive Live Task Table with status indicators and action buttons (Cancel, Requeue).
   - Add CBBA Allocation Details panel showing winning bids and robot allocations.

---

## 7. New Files, Messages, and Services

1. **`src/amr_fleet_msgs/srv/CreateTask.srv`**:
   ```
   string task_id
   float64 pickup_x
   float64 pickup_y
   float64 dropoff_x
   float64 dropoff_y
   int32 priority
   float64 deadline
   string requested_robot
   ---
   bool accepted
   string task_id
   string message
   ```
2. **`src/amr_fleet_msgs/srv/ControlTask.srv`**:
   ```
   string task_id
   string action
   ---
   bool success
   string message
   ```
3. **`scripts/create_task.py`**:
   - Standalone CLI tool providing argument parsing (`--pickup-x`, `--pickup-y`, `--dropoff-x`, `--dropoff-y`, `--priority`, `--robot`, `--task-id`, `--deadline`), calling `/tasks/create`, and reporting outcome.
4. **`src/amr_fleet_core/test/test_operator_task_control.py`**:
   - Comprehensive unit test suite for task creation, validation, lifecycle transitions, AUTO vs DIRECT constraints, and cancellation/requeue semantics.
5. **`scripts/test_live_task_integration.py`**:
   - Automated integration script verifying end-to-end task creation, CBBA bidding, assignment, and execution telemetry.
6. **`docs/TASK_ALLOCATION_GUIDE.md`**:
   - Complete operator user manual.
7. **`docs/checkpoints/TASK_ALLOCATION.md`**:
   - Final human checkpoint and Definition of Done verification report.

---

## 8. Test Plan

### Unit Testing (`test_operator_task_control.py`)
- **Validation**: Reject coordinates outside warehouse bounds ($x \notin [0, 30], y \notin [0, 30]$). Reject non-existent robot targets for DIRECT assignment.
- **Auto ID Generation**: Verify automatic ID assignment (`T...`) when task ID is empty.
- **Duplicate Prevention**: Reject task creation if task ID already exists in the pool.
- **AUTO Allocation**: Verify all CBBA agents consider an unconstrained task and the closest/highest-utility agent wins.
- **DIRECT Allocation**: Verify that when `requested_robot='amr_3'`, only `amr_3` bids on and accepts the task; other AMRs ignore it.
- **Cancellation**:
  - Cancel a `PENDING` task -> immediate transition to `CANCELLED`.
  - Cancel an `ASSIGNED` task -> transitions to `CANCELLED`, removed from winning bundle, capacity freed.
- **Requeue**:
  - Requeue an `ASSIGNED` or `FAILED` task -> transitions back to `PENDING`, enters CBBA auction again.
- **Audit Integrity**: Every state transition generates a validated `TaskEvent` record with timestamp and details.

### Integration Testing (`test_live_task_integration.py`)
- Spin up ROS 2 test nodes (Task Manager, 3 CBBA nodes).
- Send `/tasks/create` request for an AUTO task.
- Confirm task reaches `PENDING` state on `/tasks/available`.
- Confirm CBBA nodes exchange bids and reach consensus.
- Confirm task state transitions to `ASSIGNED` with winning robot recorded.
- Send `/tasks/create` with DIRECT constraint for a specific robot.
- Confirm only the specified robot is assigned.
- Send `/tasks/control` CANCEL request and verify bundle purging.

### Real Simulation Demonstration
- Launch 5 AMRs in Gazebo Harmonic (`warehouse_m9_v2`).
- Launch Fleet Dashboard.
- Create Task 1 via web dashboard form (AUTO).
- Observe Task 1 transition: `PENDING` -> CBBA consensus -> `ASSIGNED` -> `IN_PROGRESS` -> AMR begins navigation.
- Create Task 2 via CLI (`scripts/create_task.py`) with DIRECT constraint to a specific AMR.
- Confirm winning AMR matches DIRECT constraint.

---

## 9. Safety & Integrity Constraints

1. **Hierarchy Preservation**:
   - Operator actions only inject or modify entries in the task pool.
   - Motion control, path finding, and collision avoidance remain completely isolated in `rh_node` and `safety_monitor`.
   - Emergency braking (LiDAR < $0.350\,\text{m}$) cannot be overridden under any circumstances.
2. **Safe Deceleration**:
   - If an active executing task is cancelled, the robot does not perform an unsafe discontinuous stop or jerk; rather, the rolling-horizon planner completes its current control cycle with a smooth deceleration and releases space-time reservations.
3. **No Centralized Bypass**:
   - For all `AUTO` tasks, the Task Manager does NOT select a robot. Decentralized CBBA bidding and consensus across peer robots strictly determines assignment.

---

## 10. Research Integrity Statement

All canonical benchmark results, configurations, and logs for M7, M8A, M8B, M9-V1, M9-V2, M9-V3-D, M9-V3-A, and M9-V3-E remain **frozen and immutable**. This operator task control capability is strictly an operational extension layer. No canonical experiments will be modified, rerun, or claimed to have altered performance metrics.
