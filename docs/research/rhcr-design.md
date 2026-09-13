# Rolling-Horizon Task Planning (RHCR) Architecture & Design

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Context**: Milestone M5 Architectural Document  
**Target Platform**: ROS 2 Jazzy, Gazebo Harmonic, NRDAS Autonomous Mobile Robot Fleet  

---

## 1. Introduction & Theoretical Motivation

In multi-robot automated material handling systems (AMR fleets), task allocation determines *which* robot services each task (Milestone M4, CBBA). However, assigning a bundle of tasks $B_i = [T_1, T_2, \dots, T_k]$ to robot $i$ is insufficient for operational execution. The robot requires:
1. An ordered sequence of execution among its assigned tasks.
2. A trajectory through the physical environment adhering to geometric obstacles.
3. A principled balance between global optimality and computational responsiveness.

Computing complete, static, end-to-end paths for entire task bundles in dynamic warehouse environments is computationally intractable and brittle:
- Obstacles, dynamic agents, and task additions invalidate long-range static trajectories.
- Complete Multi-Agent Path Finding (MAPF) over infinite horizons suffers from exponential state spaces.

To solve this, **Rolling-Horizon Task Planning** (derived from Rolling-Horizon Collision Resolution, Li et al., 2021) decomposes trajectory computation into finite lookahead horizons ($h$) and execution windows ($w$).

---

## 2. Mathematical Formulation

### 2.1. Horizon ($h$) and Execution Window ($w$)

Let the global discrete time step be $t \in \mathbb{N}_0$. At time $t$, agent $i$ at localized configuration $x_i(t) \in \mathbb{R}^2$ plans a collision-free path towards its current sub-goal $g_i \in \mathbb{R}^2$:

$$\Pi_i(t) = \big( \pi_i(t), \pi_i(t+1), \dots, \pi_i(t + L) \big)$$

where $\pi_i(t) = x_i(t)$ and $\pi_i(t + L) = g_i$.

Under rolling-horizon planning:
1. **Planning Horizon ($h$)**: The planner computes or truncates the trajectory up to $h$ lookahead steps:
   $$\Pi_i^{\text{horizon}}(t) = \big( \pi_i(t), \pi_i(t+1), \dots, \pi_i(t + \min(L, h)) \big)$$
2. **Execution Window ($w$)**: The agent commits to executing only the initial $w$ steps ($w \le h$):
   $$\Pi_i^{\text{exec}}(t) = \big( \pi_i(t), \pi_i(t+1), \dots, \pi_i(t + \min(L, w)) \big)$$
3. **Advance & Replan**: After executing $w$ steps, or upon arriving at sub-goal $g_i$, the agent triggers a replan cycle at time $t' = t + w$ with updated initial state $x_i(t')$.

#### Parameter Selection Trade-Offs:
- **$h = 10$ steps ($5.0\,\text{m}$ at $0.5\,\text{m}/\text{grid}$)**: Sufficient lookahead to route around major warehouse shelving clusters without excessive search graph expansion.
- **$w = 4$ steps ($2.0\,\text{m}$)**: Provides smooth, continuous execution while bounding unobserved environment deviations to 2.0 meters before recalculation.
- **Average Planning Latency**: Measured empirically at $< 0.1\,\text{ms}$ per cycle on single-agent 4-connected grid A*.

---

## 3. Sub-Goal Progression & Task Lifecycle

Each warehouse transport task $T_j$ consists of a pickup location $p_j$ and a dropoff location $d_j$. The rolling-horizon planner treats each task as a two-phase sequential sub-goal:

```
[IDLE]
  │
  ▼ Ingest assigned bundle from CBBA
[TRANSIT_TO_PICKUP]  (Goal: p_j)
  │
  ▼ Arrived at pickup (dist <= 0.5m) -> Notify TaskManager: IN_PROGRESS
[TRANSIT_TO_DROPOFF] (Goal: d_j)
  │
  ▼ Arrived at dropoff (dist <= 0.5m) -> Notify TaskManager: COMPLETED
[NEXT TASK / IDLE]
```

### Deterministic Sub-Goal State Machine:
1. **`IDLE`**: No tasks in assigned bundle. Robot holds position.
2. **`TRANSIT_TO_PICKUP`**: Robot tracks execution window towards task $T_j$'s pickup coordinates.
3. **`TRANSIT_TO_DROPOFF`**: Robot tracks execution window towards task $T_j$'s dropoff coordinates.
4. **`ALL_COMPLETED`**: All tasks in bundle executed; robot returns to `IDLE` state.

---

## 4. Task Sequencing Heuristics

The `TaskSequencer` module orders the tasks within an agent's assigned CBBA bundle $B_i$.

### Invariant: 100% Bundle Preservation
$$\text{set}(\text{ordered\_tasks}) \equiv \text{set}(B_i)$$
No tasks are dropped, duplicated, or reassigned during sequencing.

### Available Heuristics:
1. **`PRIORITY_FIRST`** (Default):
   $$\text{Key}(T_j) = \big( -\text{priority}(T_j), \, \|\text{pickup}(T_j) - x_i\|_2, \, \text{task\_id} \big)$$
   Prioritizes high-value tasks while breaking ties using proximity to the robot's current position.
2. **`SHORTEST_PATH_FIRST`**:
   Greedy nearest-neighbor Traveling Salesperson heuristic. Starting from $x_i$, greedily selects the task with minimal pickup distance from the preceding dropoff.
3. **`DEADLINE_FIRST`**:
   Earliest Deadline First (EDF) dispatching:
   $$\text{Key}(T_j) = \big( \text{deadline}(T_j), \, -\text{priority}(T_j), \, \text{task\_id} \big)$$
4. **`BUNDLE_ORDER`**:
   Preserves the original insertion order established during M4 CBBA bundle construction.

---

## 5. Single-Agent Grid Pathfinding (A*)

### Grid Representation:
- Discretized 2D grid world with resolution $\Delta x = 0.5\,\text{m}$.
- Obstacle map includes warehouse perimeter walls and interior shelving storage racks.
- Coordinate transformations:
  $$\text{to\_grid}(x, y) = \Big( \big\lfloor \frac{x}{\Delta x} \big\rfloor, \, \big\lfloor \frac{y}{\Delta x} \big\rfloor \Big)$$
  $$\text{to\_world}(g_x, g_y) = \Big( (g_x + 0.5) \Delta x, \, (g_y + 0.5) \Delta x \Big)$$

### Deterministic A* Formulation:
- Admissible Manhattan heuristic: $h(u, v) = |u_x - v_x| + |u_y - v_y|$.
- 4-connected cardinal movements (North, South, East, West).
- Deterministic lexicographical tie-breaking in priority queue:
  $$\text{Priority} = \big( f(u), \, h(u), \, \text{counter}, \, u \big)$$
  guarantees 100% bitwise repeatable paths across identical initial conditions.

---

## 6. Deterministic Replanning Triggers

To prevent unnecessary CPU overhead and race conditions, replanning is governed by deterministic condition evaluation:

$$\text{Trigger} = \mathcal{C}_{\text{arrival}} \lor \mathcal{C}_{\text{window}} \lor \mathcal{C}_{\text{bundle}} \lor \mathcal{C}_{\text{empty}}$$

1. **$\mathcal{C}_{\text{arrival}}$ (Sub-Goal Arrival)**:
   $$\|x_i(t) - g_i\|_2 \le \delta_{\text{tol}} \quad (\delta_{\text{tol}} = 0.5\,\text{m})$$
2. **$\mathcal{C}_{\text{window}}$ (Execution Window Exhaustion)**:
   $$\text{steps\_executed} \ge w \quad (w = 4\,\text{steps})$$
3. **$\mathcal{C}_{\text{bundle}}$ (Bundle Mutation)**:
   $$\text{incoming\_bundle} \neq \text{active\_bundle}$$
4. **$\mathcal{C}_{\text{empty}}$ (Empty Path)**:
   $$\text{len}(\Pi_i) = 0 \land g_i \neq \text{None}$$

---

## 7. Decoupled Architecture & Pathway to M6+

```
┌─────────────────────────────────────────────────────────────┐
│ M4: Task Allocation Layer (CBBA)                           │
│ Output: Assigned bundles per AMR                            │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│ M5: Rolling-Horizon Task Planning (RHCR Core Engine)        │
│ Sub-goal sequence + Single-Agent A* Path (h=10, w=4)         │
└──────────────────────────────┬──────────────────────────────┘
                               │
            ┌──────────────────┴──────────────────┐
            ▼ (Future Milestone M6)                ▼ (Future Milestone M6)
┌──────────────────────────────┐        ┌──────────────────────────────┐
│ Priority-Inheritance Bump    │        │ Non-Holonomic ORCA           │
│ (PIBT) Space-Time Conflict   │        │ (NH-ORCA) Continuous Velocity│
│ Resolution                   │        │ Obstacle Filtering           │
└──────────────────────────────┘        └──────────────────────────────┘
```

The M5 interface messages (`PlanningRequest`, `PlanningResponse`, and `RollingHorizonPlan`) establish a clean abstraction layer. When multi-agent collision avoidance (PIBT / reservation tables) is introduced in Milestone M6, it will ingest the `horizon_path` and `execution_path` from M5 and resolve spatio-temporal conflicts without altering M5's task sequencing or sub-goal decomposition logic.
