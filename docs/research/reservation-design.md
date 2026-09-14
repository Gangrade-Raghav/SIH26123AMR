# Space-Time Reservation System Architecture & Design

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Context**: Milestone M6 Research & Design Document  
**Target Platform**: ROS 2 Jazzy, Gazebo Harmonic, NRDAS Autonomous Mobile Robot Fleet  

---

## 1. Introduction & Problem Formulation

In decentralized multi-robot systems, spatial path planning alone is insufficient to prevent collisions. Two autonomous mobile robots (AMRs) may follow paths that intersect in Euclidean space without ever colliding if they arrive at the intersection at different times. Conversely, two robots following valid spatial paths will collide if their spatial occupancy overlaps in time.

To coordinate motion safely in a discretized warehouse environment without resorting to a high-dimensional joint-configuration-space solver (which scales exponentially $\mathcal{O}(|V|^N)$ with fleet size $N$), Milestone M6 employs a **Discrete Space-Time Reservation Table**.

The reservation table serves as a shared coordination substrate that records future claims over spatial cells across discrete time steps.

---

## 2. Discrete Space-Time Representation

### 2.1. Coordinate Discretization
The continuous workspace $\mathcal{W} \subset \mathbb{R}^2$ is discretized into a 2D regular grid with spatial resolution $\Delta s = 0.5\,\text{m}$.
- Metric coordinates $(x, y)$ map to discrete grid vertices $v = (gx, gy) \in \mathbb{Z}^2$:
  $$gx = \left\lfloor \frac{x}{\Delta s} \right\rfloor, \quad gy = \left\lfloor \frac{y}{\Delta s} \right\rfloor$$
- Time is discretized into uniform intervals with step $\Delta t = 1.0\,\text{s}$, matching the nominal execution interval of discrete motion primitives ($0.5\,\text{m/s}$ speed over $0.5\,\text{m}$ grid step). Time index $t \in \mathbb{N}_0$.

### 2.2. Reservation Primitives

A space-time reservation is an exclusive ownership lease granted to an agent $r \in \mathcal{R}$:

1. **Vertex Reservation ($R_v$)**:
   $$R_v = \langle v, t, r, \tau_{\text{created}} \rangle \in \mathcal{V} \times \mathcal{T} \times \mathcal{R} \times \mathbb{R}^+$$
   Asserts that robot $r$ exclusively occupies vertex $v = (gx, gy)$ during discrete time step $t$.

2. **Edge Reservation ($R_e$)**:
   $$R_e = \langle u, v, t, r, \tau_{\text{created}} \rangle \in \mathcal{V} \times \mathcal{V} \times \mathcal{T} \times \mathcal{R} \times \mathbb{R}^+$$
   Asserts that robot $r$ traverses the directed edge from vertex $u$ at time $t$ to vertex $v$ at time $t+1$.

---

## 3. Mathematical Taxonomy of Multi-Agent Conflicts

The reservation system explicitly identifies and categorizes three fundamental classes of space-time conflicts:

```
    VERTEX CONFLICT                    EDGE-SWAP CONFLICT                   WAITING CONFLICT
   (Same Cell & Time)                 (Simultaneous Reversal)             (Entering Occupied Cell)

     amr_0       amr_1                   amr_0       amr_1                   amr_0       amr_1
       ↓           ↓                       →           ←                       |         (holds)
   [ (5, 5) at t=3 ]                   (4,5)       (5,5)                       ↓         (5,5)
                                      amr_0: (4,5) → (5,5) at t            amr_0: (4,5) → (5,5)
                                      amr_1: (5,5) → (4,5) at t            amr_1: (5,5) → (5,5)
```

### 3.1. Vertex Conflict
A vertex conflict occurs when two or more distinct agents claim the same spatial vertex at the same time step:
$$\text{Conflict}_{\text{vertex}}(r_a, r_b, v, t) \iff r_a \neq r_b \land \big( \langle v, t, r_a \rangle \in \mathcal{R}_V \land \langle v, t, r_b \rangle \in \mathcal{R}_V \big)$$
*Physical consequence in warehouse:* If uncoordinated, both AMRs attempt to enter the same physical footprint, causing a physical collision.

### 3.2. Edge-Swap Conflict
An edge-swap conflict occurs when two agents attempt to traverse the same physical corridor segment in opposite directions during the same time interval:
$$\text{Conflict}_{\text{edge}}(r_a, r_b, u, v, t) \iff r_a \neq r_b \land \big( \langle u, v, t, r_a \rangle \in \mathcal{R}_E \land \langle v, u, t, r_b \rangle \in \mathcal{R}_E \big)$$
*Physical consequence in warehouse:* In single-aisle shelving corridors, robots attempting to pass each other head-on will meet midway across the corridor, resulting in a head-on collision or dead-end stall. Standard vertex reservations alone do *not* catch this if the agents start on adjacent cells at time $t$ and swap at $t+1$.

### 3.3. Waiting Conflict
A waiting conflict occurs when an agent $r_b$ remains stationary at cell $u$, while agent $r_a$ attempts to enter cell $u$:
$$\text{Conflict}_{\text{wait}}(r_a, r_b, u, t) \iff r_a \neq r_b \land \big( \langle u, t+1, r_a \rangle \in \mathcal{R}_V \land \langle u, u, t, r_b \rangle \in \mathcal{R}_E \big)$$

---

## 4. Reservation Table Implementation & Data Structures

Implemented in `src/amr_fleet_core/amr_fleet_core/reservation_table.py`, the `SpaceTimeReservationTable` utilizes nested hash maps providing $\mathcal{O}(1)$ query and insertion complexity:

```python
# Internal Storage
self._vertex_reservations: Dict[Tuple[int, int], Dict[int, Reservation]]
# Key: (gx, gy) -> Value: {time_step: Reservation}

self._edge_reservations: Dict[Tuple[Tuple[int, int], Tuple[int, int]], Dict[int, Reservation]]
# Key: ((u_x, u_y), (v_x, v_y)) -> Value: {time_step: Reservation}
```

### 4.1. Core Application Programming Interfaces (APIs)

1. `reserve(cell: Tuple[int, int], time: int, robot_id: str, duration: int = 1) -> bool`:
   Attempts to reserve `cell` at `time` for `robot_id`. Returns `True` if successfully reserved, or `False` if already reserved by another robot.
2. `reserve_edge(from_cell: Tuple[int, int], to_cell: Tuple[int, int], time: int, robot_id: str) -> bool`:
   Attempts to reserve directed edge $(u \to v)$ at `time`. Checks for reverse edge $(v \to u)$ ownership. Rejects if an opposing edge reservation exists.
3. `is_reserved(cell: Tuple[int, int], time: int, ignore_robot: Optional[str] = None) -> bool`:
   Returns `True` if `cell` at `time` is held by any robot other than `ignore_robot`.
4. `is_edge_conflict(from_cell: Tuple[int, int], to_cell: Tuple[int, int], time: int, robot_id: str) -> bool`:
   Checks if edge $(u \to v)$ at `time` is blocked by $(v \to u)$ from another robot.
5. `release_robot(robot_id: str) -> int`:
   Evicts all active vertex and edge reservations owned by `robot_id` (used during replanning or deadlock recovery).
6. `release_time_before(time: int) -> int`:
   **Rolling-Horizon Expiration**: Prunes all reservations with timestamp $t < \text{time}$. Prevents unbounded memory growth during continuous operation.
7. `get_reservation(cell: Tuple[int, int], time: int) -> Optional[Reservation]`:
   Returns the active reservation record, or `None`.

---

## 5. Decentralization Semantics: Shared State vs. Centralized Planner

A critical design requirement of Milestone M6 is that **reservations represent shared coordination state, not a centralized optimizer**.

### 5.1. Architectural Distinction
| Dimension | Centralized MAPF (e.g., CBS / Centralized A*) | M6 Decentralized Reservation Table |
| :--- | :--- | :--- |
| **Computation Location** | Single central node calculates all paths jointly. | Each AMR computes its own path in its local `rh_node`. |
| **Authority** | Central solver assigns velocities/actions to robots. | AMRs own their motion; reservation table acts as shared ledger. |
| **Communication** | Central node commands robots over ROS topics. | Peer-to-peer broadcast of reservations (`/fleet/reservations`). |
| **Failure Mode** | Central node crash halts entire fleet. | AMR node crash leaves other AMRs free to coordinate around it. |

### 5.2. Synchronization Protocol
In ROS 2, each AMR instance runs an independent `rh_node` hosting its own local `SpaceTimeReservationTable` instance:
1. When AMR $i$ plans an execution window $w$, it publishes its proposed reservations to `/fleet/reservations` using `amr_fleet_msgs/SpaceTimeReservation`.
2. Peer AMRs subscribe to `/fleet/reservations` and update their local tables.
3. If two AMRs publish conflicting claims nearly simultaneously, deterministic priority rules (evaluated locally by both nodes using identical priority formulas) dictate which robot keeps the reservation and which yields.

---

## 6. Rolling-Horizon Pruning & Memory Bounds

Because warehouse fleets operate continuously over hours or days, reservation tables without garbage collection would suffer unbounded memory growth $\mathcal{O}(T \cdot |\mathcal{V}|)$.

In M6:
- The rolling horizon execution window is $w = 4$ steps, lookahead $h = 10$ steps.
- Each AMR maintains reservations up to $t_{\text{current}} + h$.
- At each planning cycle, `release_time_before(current_time - 1)` is called, evicting expired time steps.
- Table memory is strictly bounded by $\mathcal{O}(N \cdot h)$, independent of total mission runtime.
