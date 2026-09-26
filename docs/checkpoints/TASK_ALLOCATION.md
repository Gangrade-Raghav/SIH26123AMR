# NRDAS Milestone Checkpoint: Operator Task Allocation & Live Task Control

> **Milestone Status**: **COMPLETE — AWAITING HUMAN CHECKPOINT APPROVAL**  
> **Target Branch**: `main`  
> **Environment**: ROS 2 Jazzy Jalisco, Gazebo Sim 8.11.0 (Harmonic), Python 3.12, Ubuntu 24.04 LTS  
> **Safety Invariant**: Zero Collisions, Zero Deadlocks, Zero Centralized Bypass, Safety Hierarchy Strictly Enforced  

---

## 1. Executive Summary

This engineering milestone extends the frozen NRDAS multi-AMR research platform with an **operator-facing task allocation and live task control capability**. 

Prior to this milestone, tasks were injected statically via pre-configured YAML workload files. With this capability, operators can:
1. **Dynamically Dispatch Tasks at Runtime**: Inject single or batched pickup-and-dropoff tasks via a zero-dependency web modal (`http://localhost:8080`) or an ergonomic CLI tool (`scripts/create_task.py`).
2. **Select Between Two Allocation Paradigms**:
   - **AUTO Mode**: Injects tasks into the decentralized CBBA auction pool, allowing the 5-to-10 AMR fleet to determine the optimal allocation based on robot proximity, queue size, and deadlines through decentralized consensus.
   - **DIRECT Mode**: Constrains a task to a designated AMR (e.g. `amr_3`); only the target AMR considers the task during its bundle-building phase, while non-target robots skip it.
3. **Control Live Tasks**: Cancel in-flight or assigned tasks safely, or requeue failed/unassigned tasks back into the auction pool, triggering clean bundle purging and replanning without kinematic discontinuities or reservation breaches.
4. **Inspect Decentralized CBBA Bids Live**: The operator console features a real-time CBBA Bid Inspector displaying peer bids, winning agents, and consensus states.

**Zero Regression Invariant**: Canonical M7, M8, and M9 experiment datasets and benchmarks remain completely intact and untouched. All 216 unit and regression tests pass with 100% success.

---

## 2. Architecture & Authority Hierarchy

### 2.1 Information Flow
```
  [Operator: Web Dashboard or CLI]
                 |
      (ROS 2 Service Calls)
                 |
                 v
       +--------------------+
       |  TaskManagerNode   | <--- Validates bounds [0, 30], IDs, priority
       +--------------------+
                 |
      (Topics: /tasks/all, /tasks/available)
                 |
                 v
       +--------------------+
       |   CBBANode (xN)    | <--- Decentralized marginal utility bidding
       |                    | <--- Filters candidates by requested_robot
       |                    | <--- Consensus gossip via /fleet/cbba_bids
       +--------------------+
                 |
      (Topic: /{robot_id}/bundle)
                 |
                 v
       +--------------------+
       | Rolling Horizon /  | <--- Spatio-temporal A* planning
       | PIBT Reservations  | <--- Space-time reservations & WFG deadlock avoidance
       +--------------------+
                 |
                 v
       +--------------------+
       | Safety Backstop /  | <--- Independent 10 Hz LaserScan reactive brake
       | Gazebo Simulation  |
       +--------------------+
```

### 2.2 Authority Hierarchy Preservation
The authority model strictly enforces:
$$\text{Local Safety / Emergency Braking} \succ \text{Reservations / PIBT Coordination} \succ \text{RHCR Planning} \succ \text{Task Allocation}$$

- **Task Cancellation**: When an operator cancels an active task, the Task Manager updates the state to `CANCELLED` and broadcasts an event. The assigned AMR detects the cancellation in its next rolling-horizon planning cycle ($2\,\text{Hz}$), cleanly aborts the subgoal, releases future space-time reservations, and transitions to `IDLE` without sudden deceleration or collision risk.
- **Task Requeuing**: When a failed task is requeued, it transitions safely back to `PENDING` with an incremented `reassignment_count`. AMRs evaluate it in the subsequent CBBA auction cycle.

---

## 3. Implemented Components & Interfaces

### 3.1 ROS 2 Service Definitions
Created in package `amr_fleet_msgs`:
1. **`amr_fleet_msgs/srv/CreateTask.srv`**:
   ```
   string task_id          # Custom ID or empty for auto 'T###'
   float64 pickup_x        # Warehouse bounds [0.0, 30.0]
   float64 pickup_y        # Warehouse bounds [0.0, 30.0]
   float64 dropoff_x       # Warehouse bounds [0.0, 30.0]
   float64 dropoff_y       # Warehouse bounds [0.0, 30.0]
   int32 priority          # 1=LOW, 2=NORMAL, 3=HIGH, 4=CRITICAL
   float64 deadline        # Optional simulation timestamp
   string requested_robot  # Target AMR ID or empty for AUTO CBBA
   ---
   bool accepted           # Acceptance status
   string task_id          # Confirmed task ID
   string message          # Status details or rejection rationale
   ```
2. **`amr_fleet_msgs/srv/ControlTask.srv`**:
   ```
   string task_id          # Target task ID
   string action           # 'CANCEL' or 'REQUEUE'
   ---
   bool success            # Operation status
   string message          # Detailed feedback
   ```
3. **`amr_fleet_msgs/msg/TaskDefinition.msg`**:
   - Added `string requested_robot` field to propagate operator constraints across ROS 2 topics.

### 3.2 Core Logic Updates (`amr_fleet_core`)
- **`task_model.py`**:
  - Allowed transitions: `ASSIGNED -> CANCELLED`, `IN_PROGRESS -> CANCELLED`, `ASSIGNED -> PENDING`, and `FAILED -> PENDING`.
  - Added fields: `requested_robot: Optional[str]`, `reassignment_count: int`.
  - Updated dictionary serialization and deserialization.
- **`task_manager_node.py`**:
  - Implemented service servers for `/tasks/create`, `/tasks/control`, `/tasks/cancel`, and `/tasks/requeue`.
  - Implemented input validation: coordinates within $[0.0, 30.0]\,\text{m}$, minimum pickup-to-dropoff distance ($0.20\,\text{m}$), duplicate ID rejection, sequential ID generation (`T001`, `T002`...).
  - Integrated safe lifecycle transitions with event broadcasting on `/tasks/events`.
- **`cbba_agent.py`**:
  - Added candidate task filtering: if `task['requested_robot']` is set to a specific AMR (e.g. `amr_1`), non-target agents skip candidate scoring, ensuring strictly zero bid leakage to other robots.
- **`cbba_node.py`**:
  - Maintained `requested_robot` cache from incoming `TaskList` messages.
  - Added bundle purge logic: when tasks in an agent's bundle are detected as `CANCELLED`, `FAILED`, or externally requeued, they are pruned from `agent.state.bundle`, `winning_bids`, and `assigned_tasks_committed`.
- **`rh_planner.py`**:
  - Updated bundle assignment tracking: if an active in-progress task is removed or cancelled from the assigned bundle, the planner safely aborts the active plan and transitions the robot to `IDLE`.

### 3.3 Operator Tools & Web Console
- **`scripts/create_task.py`**: Standalone executable CLI tool supporting coordinate inputs, priority strings/numbers, target robot constraints, custom IDs, and timeout handling.
- **`scripts/fleet_dashboard.py`**:
  - Added service clients calling `/tasks/create` and `/tasks/control`.
  - Added HTTP POST endpoints `/api/task/create` and `/api/task/control` in `DashboardHTTPRequestHandler`.
  - Added interactive modal dialog with brutalist styling and validation feedback.
  - Added live Task Registry table with 7 columns (TASK, PRIO, PICKUP, DROPOFF, STATUS, ROBOT, ACTIONS) and tactile CANCEL / REQUEUE action buttons.
  - Added Card 04B **CBBA Bid Inspector & Margins** showing multi-robot bid vectors and winning allocations.

---

## 4. Verification & Empirical Evidence

### 4.1 Unit & Linter Verification
- **Test File**: `src/amr_fleet_core/test/test_operator_task_control.py`
- **Coverage**:
  - `test_task_model_operator_transitions`: Verified `CANCELLED` and `REQUEUE` transitions and serialization.
  - `test_task_model_invalid_requeue`: Verified that terminal `COMPLETED` tasks reject requeuing.
  - `test_task_creation_bounds_and_validation`: Verified $[0, 30]$ bounds checks, duplicate ID rejection, and auto `T###` generation.
  - `test_task_control_cancel_and_requeue`: Verified cancel of `PENDING` and requeue of `ASSIGNED` tasks.
  - `test_cbba_direct_constraint_auction`: Verified that non-target robots skip candidate tasks while target robots place valid bids.
- **Linter Results**:
  - `ament_flake8`: **Passed (0 errors)**.
  - `ament_pep257`: **Passed (0 errors)**.
- **Full Package Regression**:
  ```bash
  colcon test --packages-select amr_fleet_core && colcon test-result --verbose
  ```
  **Result**: `Summary: 216 tests, 0 errors, 0 failures, 0 skipped` (100% pass rate).

### 4.2 Standalone Integration Test
- **Script**: `scripts/test_live_task_integration.py`
- **Workflow Tested**:
  1. `TaskManagerNode` and 2 decentralized `CBBANode` instances spawned.
  2. Task 1 (`T_AUTO_01`) submitted via `/tasks/create` -> Allocated to winning robot via CBBA consensus.
  3. Task 2 (`T_DIR_01`, constrained to `amr_1`) submitted via `/tasks/create` -> Assigned strictly to `amr_1` (0 bids from `amr_0`).
  4. Task 1 cancelled via `/tasks/control` -> Transitioned to `CANCELLED` and purged from winning bundle.
  5. Task 2 requeued via `/tasks/control` -> Transitioned to `PENDING`.
- **Result**: **Passed cleanly in 2.8s**.

### 4.3 Full 5-AMR Gazebo Harmonic Live Demonstration
- **Script**: `scripts/demonstrate_real_gazebo_operator_control.py`
- **Evidence File**: [`docs/checkpoints/task_allocation_evidence.json`](docs/checkpoints/task_allocation_evidence.json)
- **Results Summary**:
  ```json
  {
    "gazebo_online": true,
    "dashboard_online": true,
    "task_auto_created": true,
    "task_direct_created": true,
    "cbba_consensus_reached": true,
    "direct_allocation_verified": true,
    "cancel_action_verified": true,
    "final_task_states": {
      "task_small_0001": "ASSIGNED",
      "task_small_0002": "ASSIGNED",
      "task_small_0003": "ASSIGNED",
      "task_small_0004": "ASSIGNED",
      "task_small_0005": "ASSIGNED",
      "T_DEMO_AUTO": "CANCELLED",
      "T_DEMO_DIR3": "ASSIGNED"
    },
    "final_bundles": {
      "amr_3": ["task_small_0004", "T_DEMO_DIR3"],
      "amr_1": ["task_small_0002"],
      "amr_4": ["task_small_0005"],
      "amr_2": ["task_small_0003"],
      "amr_0": ["task_small_0001"]
    },
    "bids_count": 127
  }
  ```
  - `T_DEMO_AUTO` auctioned via CBBA consensus, won by `amr_0`, then successfully cancelled and purged from bundle.
  - `T_DEMO_DIR3` targeted to `amr_3`, won strictly by `amr_3`, confirmed in `amr_3` bundle with zero allocation to other AMRs.
  - Zero collisions or physical contacts during multi-robot motion in Gazebo.

---

## 5. Definition of Done Checklist

| DoD Criterion | Status | Evidence |
| :--- | :---: | :--- |
| **Zero Centralized Bypass** | **SATISFIED** | Task Manager only inserts tasks into PENDING pool; CBBA nodes auction tasks autonomously |
| **Deterministic DIRECT Allocation** | **SATISFIED** | Only target robot considers candidate task; verified in unit, integration, and Gazebo tests |
| **Bounds & Duplicate Validation** | **SATISFIED** | Rejects coordinates outside $[0, 30]\,\text{m}$ and duplicate task IDs with informative messages |
| **Dynamic ID Generation** | **SATISFIED** | Auto-generates sequential `T001`, `T002`... when ID is omitted |
| **In-Flight Cancellation & Requeue** | **SATISFIED** | Clean bundle purging in `cbba_node` and safe subgoal abort in `rh_planner` |
| **Operator CLI Tool** | **SATISFIED** | `scripts/create_task.py` executable with full parameter suite |
| **Web Dashboard Controls & Modal** | **SATISFIED** | Modal dialog, live task table with action buttons, and CBBA Bid Inspector on port 8080 |
| **Zero Test Regression** | **SATISFIED** | 216/216 workspace tests passing; flake8/pep257 clean |
| **Immutable Canonical Results** | **SATISFIED** | All M7/M8/M9 benchmarks remain completely untouched |
| **Operator Documentation** | **SATISFIED** | Created `docs/TASK_ALLOCATION_GUIDE.md` and updated `README.md` |

---

## 6. Checkpoint Sign-Off

The operator task allocation and live task control capability has been fully planned, implemented, unit tested, integration tested, demonstrated in Gazebo Harmonic, documented, and audited.

The system is now halted and waiting for human review and explicit approval.
