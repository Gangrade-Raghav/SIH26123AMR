# Milestone M9 Plan: Large-Scale Warehouse & Scenario Expansion
## Phase 1 (Design Approved) & Phase 2 (M9-V1 Implementation & Pilot Validation)

**Project**: NRDAS Autonomous Mobile Robot Fleet Coordination System  
**Milestone**: M9 — Large-Scale Warehouse & Scenario Expansion  
**Phase**: Phase 2 — M9-V1 Implementation & Pilot Complete  
**Baseline**: M8B (Approved & Frozen Baseline)  
**Date**: 2026-09-14  
**Status**: M9-V1 IMPLEMENTED & VALIDATED — WAITING FOR HUMAN APPROVAL  

---

## 1. Executive Summary & Research Motivation

Milestones M0 through M8B established, benchmarked, and stress-tested a complete decentralized fleet coordination architecture for autonomous mobile robots (AMRs):
- **Decentralized Task Allocation**: Consensus-Based Bundle Algorithm (CBBA)
- **Lifelong Path Planning**: Rolling-Horizon Conflict Resolution (RHCR / Rolling-Horizon $A^*$)
- **Coordination & Deconfliction**: Space-Time Reservation Tables with Priority Inheritance Backtracking (PIBT)
- **Deadlock Resolution**: Wait-For-Graph (WFG) cycle detection with deterministic priority yielding
- **Resilience Layer**: Communication impairment handling (packet loss, latency, jitter, partitions)
- **Adaptive Compute Layer**: Dynamic reallocation of replan rates, horizons, and consensus frequencies
- **Safety Guarantee**: Decoupled $10\,\text{Hz}$ reactive LiDAR emergency braking with absolute preemption authority

However, all empirical evaluations in M6–M8B were conducted within a single, relatively compact $16\,\text{m} \times 16\,\text{m}$ warehouse arena containing 4 storage racks and a 5-AMR fleet. While this environment exposed critical coordination phenomena (such as corridor congestion and packet loss thrashing), it represents a constrained topological testbed.

**The Objective of Milestone M9 is NOT merely "make the map bigger."**

The primary scientific objective is to **evaluate whether the existing decentralized coordination architecture transfers, scales, or exposes fundamental limitations when subjected to progressively harder, topologically diverse, and congestion-stressed factory/warehouse environments**.

M9 introduces:
1. **Four Progressive Environment Tiers**: From an expanded baseline (M9-V1) up to a multi-zone research stress environment (M9-V4).
2. **Progressive Fleet Scaling**: Systematic scaling from 5 AMRs to 10, 15, and 20+ AMRs.
3. **Challenging Spatial Topologies**: Single-lane corridors with passing bays, multi-way intersections, chokepoints, shared dropoff hubs, and charging pad contention.
4. **Structured Traffic Regimes**: Bidirectional flow, spatially concentrated demand hotspots, and asymmetric cross-zone logistics.
5. **Rigorous Experimental Control**: Grounded in deterministic seeds, explicit physical feasibility assumptions, and preservation of all frozen M0–M8B algorithmic invariants.

---

## 2. Current Environment Audit

An exhaustive architectural audit of the active repository was conducted to inspect the Gazebo world, grid representations, robot spawner, task generator, visualizers, and benchmark harnesses.

### 2.1 Gazebo World (`src/amr_fleet_bringup/worlds/warehouse_small.sdf`)
- **Arena Dimensions**: $16.0\,\text{m} \times 16.0\,\text{m}$ ($256\,\text{m}^2$).
- **Perimeter Walls**: 4 boundary walls ($0.4\,\text{m}$ thick, $2.0\,\text{m}$ height).
- **Shelving Racks**: 4 industrial racks (`rack_1` to `rack_4`), each $1.2\,\text{m} \times 3.0\,\text{m} \times 2.0\,\text{m}$, arranged symmetrically:
  - `rack_1`: Center $(4.5, 5.5)$ | `rack_2`: Center $(4.5, 10.5)$
  - `rack_3`: Center $(11.5, 5.5)$ | `rack_4`: Center $(11.5, 10.5)$
- **Corridors**:
  - West corridor: $x \in [1.0, 3.5]$ ($2.5\,\text{m}$ wide)
  - Center arterial: $x \in [6.5, 9.5]$ ($3.0\,\text{m}$ wide, with dropoff hub at center $(8.0, 8.0)$)
  - East corridor: $x \in [12.5, 15.0]$ ($2.5\,\text{m}$ wide)
  - North cross-aisle: $y \in [12.5, 14.5]$ ($2.0\,\text{m}$ wide)
  - South cross-aisle: $y \in [1.5, 3.5]$ ($2.0\,\text{m}$ wide)
  - Mid crosswalks: $y \approx 8.0$ between racks $(4.9, 8.0)$ and $(11.1, 8.0)$
- **Stations**:
  - 4 corner pickups: $(2, 2), (2, 13), (13, 2), (13, 13)$
  - 1 central dropoff hub: $(8, 8)$ with 4 bays $(7.4, 7.4), (7.4, 8.6), (8.6, 7.4), (8.6, 8.6)$
  - 4 charging pads: South wall $(4.5, 0.9), (6.0, 0.9), (10.0, 0.9), (11.5, 0.9)$
- **Audit Assessment**: The environment has broad aisles ($\ge 2.0\,\text{m}$), allowing easy two-way passing everywhere except when AMRs directly converge at the $(8, 8)$ dropoff hub.

### 2.2 Discrete Map Representation (`src/amr_fleet_sim/amr_fleet_sim/grid_world.py`)
- The class `GridWorld` implements 2D 4-connected discrete space with resolution $\delta = 0.5\,\text{m}$ ($32 \times 32$ cells).
- **Hardcoding Finding**: The factory constructor `create_warehouse_grid(resolution=0.5, warehouse_size=16.0)` hardcodes the 4 rack coordinates and $16.0\,\text{m}$ boundaries.
- `rh_node.py`, `rh_planner.py`, and `multi_agent_coordinator.py` instantiate `self.grid = GridWorld.create_warehouse_grid(resolution=res)`.
- `config/maps/warehouse_grid_small.yaml` provides a $1.0\,\text{m}$ resolution map specification used by the RViz visualizer.

### 2.3 Robot Spawning & Launch Infrastructure
- `fleet_5_robots.yaml`: Spawns 5 AMRs in a single column in the western aisle at $(2.0, 2.0)$, $(2.0, 5.0)$, $(2.0, 8.0)$, $(2.0, 11.0)$, $(2.0, 14.0)$.
- `fleet_10_robots.yaml`: Spawns 5 in the western aisle and 5 in the central aisle at $x=8.0$.
- `fleet.launch.py`: Uses modular configuration from `config/robots/fleet_{N}_robots.yaml`. Fallback formula: `x = 2.0 if (i // 5) == 0 else 8.0; y = 2.0 + (i % 5) * 3.0`.
- `m8b_adaptive_fleet.launch.py`: Hardcodes the same formula in lines 249–253 for RH node parameter instantiation.

### 2.4 Task Generation (`src/amr_fleet_core/amr_fleet_core/task_generator.py` & `workload.py`)
- `TaskGenerator` defaults to `DEFAULT_PICKUP_STATIONS` (4 corners), `DEFAULT_DROPOFF_STATIONS` (center hub), and `DEFAULT_OBSTACLE_BOUNDS` (4 racks).
- `WorkloadManager` supports overriding stations via YAML configuration.
- Workload files in `config/workloads/` (`workload_15_tasks.yaml`, etc.) specify pickup/dropoff stations explicitly.

### 2.5 Observability & Visualization Infrastructure
- `src/amr_fleet_bringup/amr_fleet_bringup/warehouse_visualizer.py`: Publishes 3D RViz markers on `/warehouse_markers` using `config/maps/warehouse_grid_small.yaml`.
- `scripts/fleet_dashboard.py`: Renders a 2D HTML5 canvas on port 8080.
  - **Audit Finding**: Canvas transformation strictly hardcodes `16.0m`: `toX(x) { return pad + (x / 16.0) * mapW; }` and hardcodes small warehouse lane lines.
- `scripts/run_benchmark.py`: Hardcodes `map_id: 'warehouse_grid_small'`, `robot_count: 5`, and launch parameters for `m8b_adaptive_fleet.launch.py`.

### 2.6 Key Architectural Takeaways for M9
1. **Decouple Map Geometry from Planners**: `GridWorld` must support dynamic initialization from declarative map YAML files (`warehouse_m9_v1.yaml`, etc.) or generic parameterized constructors, rather than hardcoding a single $16\times 16$ layout.
2. **Decouple Spawn Points**: Robot spawn coordinates must be read cleanly from `config/robots/` YAML files without hardcoding column offsets in launch scripts.
3. **Decouple Task Stations**: Task generator configs must bind to map definitions.
4. **Parameterize Dashboard & Visualizer**: Web canvas and RViz markers must dynamically adapt to world dimensions $(W, H)$ read from map configuration.

---

## 3. Mandatory Progressive Environment Design: Tiers M9-V1 to M9-V4

The M9 environment progression consists of four distinct tiers designed to systematically elevate coordination difficulty:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                             ENVIRONMENT PROGRESSION                         │
├───────────────────┬───────────────────┬───────────────────┬─────────────────┤
│      M9-V1        │      M9-V2        │      M9-V3        │      M9-V4      │
│ Expanded Baseline │ Congested Storage │ Hard Coordination │ Research Stress │
├───────────────────┼───────────────────┼───────────────────┼─────────────────┤
│ 32m x 32m         │ 32m x 32m         │ 48m x 32m         │ 60m x 40m (GZ)  │
│ 16 Shelves        │ 24 Shelves        │ 3-Zone Layout     │ Multi-Hall Dual │
│ 4x Area vs M8     │ Narrow Aisles     │ Single-Lane Pass  │ Abstract MAPF   │
│ Standard Clearance│ Dense Crossroads  │ Passing Bays      │ Dynamic Blocks  │
│ Validation Scaling│ Traffic Hotspots  │ Chokepoint Queue  │ Extreme Fleets  │
└───────────────────┴───────────────────┴───────────────────┴─────────────────┘
```

---

### 3.1 Tier M9-V1 — Expanded Warehouse (Baseline Transfer)

**Objective**: Verify that the frozen M8B architecture transfers seamlessly to a substantially larger operational footprint without structural deadlock or unbounded compute degradation.

```
       0m                    16m                   32m
   32m ┌──────────────────────┬──────────────────────┐
       │   [P1]       [P2]    │   [P3]       [P4]    │
       │  ┌───┐      ┌───┐    │  ┌───┐      ┌───┐    │
       │  │R1 │      │R2 │    │  │R5 │      │R6 │    │
       │  └───┘      └───┘    │  └───┘      └───┘    │
       │  ┌───┐      ┌───┐    │  ┌───┐      ┌───┐    │
       │  │R3 │      │R4 │    │  │R7 │      │R8 │    │
       │  └───┘      └───┘    │  └───┘      └───┘    │
       │        [D1]          │        [D2]          │
   16m ├──────────────────────┼──────────────────────┤  <-- Arterial Highway
       │        [D3]          │        [D4]          │
       │  ┌───┐      ┌───┐    │  ┌───┐      ┌───┐    │
       │  │R9 │      │R10│    │  │R13│      │R14│    │
       │  └───┘      └───┘    │  └───┘      └───┘    │
       │  ┌───┐      ┌───┐    │  ┌───┐      ┌───┐    │
       │  │R11│      │R12│    │  │R15│      │R16│    │
       │  └───┘      └───┘    │  └───┘      └───┘    │
       │   [P5]       [P6]    │   [P7]       [P8]    │
    0m └──────────────────────┴──────────────────────┘
       0m                    16m                   32m
```

- **Dimensions**: $32.0\,\text{m} \times 32.0\,\text{m}$ ($1,024\,\text{m}^2$, $4\times$ the area of M8).
- **Shelving Storage**: 16 industrial racks ($1.2\,\text{m} \times 3.0\,\text{m} \times 2.0\,\text{m}$), arranged in four $2 \times 2$ quadrants.
- **Corridor Topology**:
  - 5 north-south aisles: West ($x=3.0$), Mid-West ($x=9.0$), Central Arterial ($x=16.0$), Mid-East ($x=23.0$), East ($x=29.0$).
  - 5 east-west cross-aisles: South ($y=2.5$), Mid-South ($y=9.5$), Central Highway ($y=16.0$), Mid-North ($y=22.5$), North ($y=29.5$).
  - Aisle Width: $2.4\,\text{m} - 3.0\,\text{m}$ (wide, two-way passing clearance $\ge 1.4\,\text{m}$).
  - Intersections: 25 distinct 4-way and T-intersections.
- **Stations**:
  - 8 pickup stations: Distributed along perimeter aisles $(3, 2), (9, 2), (23, 2), (29, 2), (3, 30), (9, 30), (23, 30), (29, 30)$.
  - 4 quadrant dropoff hubs: $(9.5, 9.5), (22.5, 9.5), (9.5, 22.5), (22.5, 22.5)$.
  - 8 charging bays: Along south wall ($y=1.0$).
- **Spawning Configurations**: Deterministic spawn positions in `config/robots/fleet_{5,10,15}_robots_m9_v1.yaml`.
- **Purpose**: Establishes baseline metrics on longer transit paths ($15\,\text{m} - 40\,\text{m}$ vs $5\,\text{m} - 12\,\text{m}$ in M8) with distributed dropoffs.

---

### 3.2 Tier M9-V2 — Congested Warehouse (High Traffic Density)

**Objective**: Stress space-time reservations, PIBT priority yielding, and rolling-horizon replanning under severe spatial contention and converging traffic.

- **Dimensions**: $32.0\,\text{m} \times 32.0\,\text{m}$ ($1,024\,\text{m}^2$).
- **Shelving Storage**: 24 industrial racks (increased density, reducing open floor space by 40%).
- **Corridor Topology**:
  - 7 narrow north-south aisles ($1.6\,\text{m} - 1.8\,\text{m}$ width).
  - 3 east-west cross-aisles ($1.8\,\text{m}$ width).
  - Aisle Clearance: Lateral passing gap is reduced to $0.6\,\text{m} - 0.8\,\text{m}$ when two AMRs ($0.5\,\text{m}$ footprint) meet. While passing is physically feasible, it requires tight tracking and triggers the $0.40\,\text{m}$ braking envelope if either robot deviates.
- **Shared Hub Contention**:
  - All 24 storage aisles feed into a **single consolidated central dropoff hub** at $(16.0, 16.0)$ with only 4 delivery bays.
  - Pickup stations (8) are located at the far ends of the narrow aisles.
- **Expected Coordination Stress**:
  - Route convergence on the central hub creates high space-time reservation density.
  - Competing robots approaching 4-way intersections trigger PIBT priority inheritance and frequent A* replans.
  - Potential WFG cycles when multiple AMRs attempt to enter the central hub simultaneously.

---

### 3.3 Tier M9-V3 — Hard Coordination Warehouse (Chokepoints & Single-Lane Corridors)

**Objective**: Rigorously investigate the physical and algorithmic boundaries of decentralized coordination by introducing topological chokepoints where simultaneous bidirectional passage is physically impossible.

```
       0m                       20m               28m                 48m
   32m ┌─────────────────────────┬─────────────────┬───────────────────┐
       │                         │   PASSING BAY   │                   │
       │       ZONE A            │  ┌───────────┐  │      ZONE C       │
       │  High-Density Storage   │  │ (24, 26)  │  │  Logistics Depot  │
       │  (12 Racks, Narrow)     │  └─────┬─────┘  │  & Staging Hub    │
       │                         │        │        │                   │
   16m │                         │════════╪════════│                   │  <-- Single-Lane Chokepoint
       │                         │ 1.1m Single Lane│                   │      (Passing Impossible!)
       │                         │        │        │                   │
       │       ZONE B            │  ┌─────┴─────┐  │   CHARGING ROW    │
       │  Fast Transit & Buffers │  │ (24, 6)   │  │   [C1] [C2] [C3]  │
       │                         │  │PASSING BAY│  │   Contention Site │
    0m └─────────────────────────┴─────────────────┴───────────────────┘
       0m                       20m               28m                 48m
```

- **Dimensions**: $48.0\,\text{m} \times 32.0\,\text{m}$ ($1,536\,\text{m}^2$).
- **Three Heterogeneous Zones**:
  - **Zone A (West, $x \in [0, 20]$)**: High-density storage area with 12 racks and narrow aisles.
  - **Zone B (Central Chokepoint, $x \in [20, 28]$)**: Separator wall with **two single-lane corridors**:
    - North Single-Lane Tunnel: $y \in [22.0, 23.1]$ ($w=1.1\,\text{m}$, length $8.0\,\text{m}$)
    - South Single-Lane Tunnel: $y \in [8.0, 9.1]$ ($w=1.1\,\text{m}$, length $8.0\,\text{m}$)
  - **Zone C (East, $x \in [28, 48]$)**: Consolidated receiving/shipping depot, centralized dropoff sorting, and fleet charging station bank.
- **Passing Bays (Crucial Physical Requirement)**:
  - Inside each single-lane corridor, simultaneous bidirectional passing is physically impossible ($1.1\,\text{m} < 1.7\,\text{m}$ required for two $0.5\,\text{m}$ AMRs + safety margins).
  - Dedicated passing alcoves ($2.5\,\text{m} \times 2.0\,\text{m}$) are engineered at $(24.0, 26.0)$ and $(24.0, 6.0)$ to allow an AMR to pull aside, yield to oncoming traffic, and re-enter.
- **Asymmetric Traffic Demand**:
  - 80% of generated tasks require picking up in Zone A and dropping off in Zone C, forcing heavy bidirectional transit through the two single-lane corridors.
- **Charging Contention**:
  - 4 charging pads for a 10–15 AMR fleet in Zone C, creating parking and queue contention.
- **Expected Coordination Stress & Failure Modes**:
  - Exposes whether PIBT priority inheritance and space-time reservations can prevent head-on deadlocks inside single-lane tunnels.
  - Tests whether WFG deadlock detection correctly identifies tunnel blockages and commands the lower-priority AMR to reverse into a passing bay.
  - Evaluates whether network latency ($50\,\text{ms}$) under high speed causes safety aborts inside the tunnel.

---

### 3.4 Tier M9-V4 — Research Stress Environment (Dual-Scale Multi-Hall)

**Objective**: Provide an industrial-scale research testbed capable of stressing the entire stack across communication, compute, traffic, and dynamic disruptions.

- **Dual-Scale Architecture**:
  1. **Realistic Gazebo Scale ($60.0\,\text{m} \times 40.0\,\text{m}$, $2,400\,\text{m}^2$)**:
     - Dual-hall factory layout (Hall 1: Manufacturing & Assembly; Hall 2: Finished Goods & Dispatch).
     - Connected by 3 inter-hall transit highways (two wide, one restricted).
     - Supports up to 15–20 full-physics Gazebo AMRs with realistic sensors and DART physics.
     - Includes designated dynamic obstacle zones (simulating temporary pallet blockages).
  2. **Abstract Simulation Scale ($120.0\,\text{m} \times 80.0\,\text{m}$, $9,600\,\text{m}^2$)**:
     - Evaluated via discrete grid simulation (`GridWorld` + kinematic step, matching M6 Tier B protocol).
     - Supports 50, 100, and 200 abstract AMRs to evaluate CBBA bundle convergence time, reservation table memory consumption, and A* planning latency scaling without physics simulator overhead.

---

## 4. Progressive Fleet Scaling Plan

| Fleet Size | Target Tiers | Operational Purpose & Scientific Utility | Evaluation Regime |
| :---: | :---: | :--- | :---: |
| **5 AMRs** | M9-V1, V2, V3, V4 | **Controlled Baseline Comparison**: Directly isolates the pure effect of map scale and topology from robot density. Matches M0–M8B baseline exactly. | Full Gazebo Simulation |
| **10 AMRs** | M9-V1, V2, V3, V4 | **Moderate Congestion ($2\times$)**: Tests CBBA consensus scaling ($10$ bidders), multi-agent intersection deconfliction, and WFG cycle detection with $|V|=10$. | Full Gazebo Simulation |
| **15 AMRs** | M9-V1, V2, V3, V4 | **High Density ($3\times$)**: Stresses host CPU/RAM, tests queuing at chokepoints and shared dropoffs, evaluates communication bandwidth across 120 ROS 2 topics. | Full Gazebo Simulation (Host Permitting) |
| **20 AMRs** | M9-V1, M9-V4 | **Simulation Boundary Stress**: Assesses Gazebo Harmonic Real-Time Factor (RTF) limit and host core saturation on a single workstation. | Full Gazebo Simulation |
| **50–100 AMRs**| M9-V4 Abstract | **Algorithmic Scaling Limit**: Evaluates CBBA auction convergence latency, space-time reservation memory, and A* search overhead in discrete kinematic simulation. | Abstract MAPF Simulator |

---

## 5. Comprehensive Scenario Matrix (Scenarios A through J)

| ID | Scenario Name | Tier | Fleet | Workload | Task Distribution | Expected Coordination Stress | Primary Metrics | Secondary Metrics | Relevant Failure Modes |
| :---: | :--- | :---: | :---: | :---: | :--- | :--- | :--- | :--- | :--- |
| **A** | **Low Congestion** | M9-V1 | 5 AMRs | 15 Tasks | Uniform random across 8 pickups, 4 dropoffs | Baseline pathing, long travel horizons | Throughput, makespan | Replan count, CPU % | Path divergence, goal timeout |
| **B** | **Moderate Congestion**| M9-V1 | 10 AMRs | 30 Tasks | Uniform random | Intersection crossings, multi-agent pacing | Completed tasks, min separation | CBBA convergence, conflicts | Intersection stall, near-miss |
| **C** | **Severe Congestion** | M9-V2 | 15 AMRs | 50 Tasks | High density across narrow aisles | Dense reservation clashes, frequent PIBT yields | Safety brakes, min separation | Replan latency, WFG cycles | Corridor deadlock, safety abort |
| **D** | **Bidirectional Conflict**| M9-V3 | 10 AMRs | 30 Tasks | Opposing cross-aisle traffic | Head-on confrontation in narrow corridors | Deadlocks detected/recovered | Recovery latency, brakes | Head-on lock, safety abort |
| **E** | **Chokepoint Contention** | M9-V3 | 10 AMRs | 30 Tasks | 100% flow forced through single-lane tunnels | Queuing at entrance, passing bay utilization | Chokepoint transit time | Queue length, PIBT push | Entrance deadlock, FIFO inversion |
| **F** | **Intersection Clashes** | M9-V2 | 10 AMRs | 30 Tasks | Converging 4-way crossroad traffic | Multi-robot cyclic dependency at crossroads | Conflicts resolved, WFG cycles | Replan count, wait time | 4-way circular deadlock |
| **G** | **Charging Contention** | M9-V3 | 10 AMRs | 20 Tasks | Low battery return dispatch (4 chargers) | Parking pad contention, blocking chargers | Charge queue wait time | Station occupancy, brakes | Charger entrance blockage |
| **H** | **Task Hotspot** | M9-V2 | 10 AMRs | 30 Tasks | 80% tasks converge to single dropoff hub | Spatial saturation of central hub approaches | Delivery makespan, safety stops| Dropoff bay utilization | Hub gridlock, starvation |
| **I** | **Asymmetric Demand** | M9-V3 | 10 AMRs | 40 Tasks | 80% flow Zone A $\to$ Zone C, 20% reverse | Unbalanced tunnel utilization, return priority | Directional flow rate | Pass-bay wait time, throughput| Reverse starvation |
| **J** | **Comm Stress in Transit**| M9-V3 | 10 AMRs | 30 Tasks | `HIGH_LATENCY` (50ms) + `LOSS_HIGH` (20%) | Stale reservations during chokepoint negotiation | Safety abort count, min separation| Adaptive mode occupancy | Delayed reservation abort |

---

## 6. Metrics & Invariant Preservation Matrix

M9 strictly preserves all standardized metrics established in M6, M7, M8A, and M8B:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           NRDAS METRICS TAXONOMY                            │
├───────────────────┬───────────────────┬───────────────────┬─────────────────┤
│      SAFETY       │    PERFORMANCE    │   COORDINATION    │  COMMUNICATION  │
├───────────────────┼───────────────────┼───────────────────┼─────────────────┤
│ • Contact Count   │ • Throughput      │ • Conflicts Det.  │ • Packet Loss % │
│ • Min Distance (m)│ • Makespan (s)    │ • Conflicts Res.  │ • Latency (ms)  │
│ • Safety Brakes   │ • Task Completion │ • Replan Count    │ • Jitter (ms)   │
│ • Safety Aborts   │ • Task Accounting │ • Deadlocks Det.  │ • Stale Age (s) │
│                   │ • Flowtime (s)    │ • Recovery Latency│ • Outage Recov. │
├───────────────────┴───────────────────┴───────────────────┴─────────────────┤
│                             COMPUTE & SCALING                               │
├───────────────────────────────────────┬─────────────────────────────────────┤
│ • Host CPU % (Mean / Peak)            │ • Real-Time Factor (RTF)            │
│ • Host RAM MB (Mean / Peak)           │ • Startup & Discovery Time (s)      │
│ • Planning Latency P50/P95/P99 (ms)   │ • Planning Rate Stability (Hz)      │
│ • CBBA Convergence Time (ms)          │ • Scaling Overhead vs Fleet Size    │
│ • Mode Occupancy (LOW/NORMAL/HIGH %)  │                                     │
└───────────────────────────────────────┴─────────────────────────────────────┘
```

### Core Invariants Maintained:
1. **Task Accounting Invariant**:
   $$\forall \text{ trial } k: \quad N_{\text{completed}}^{(k)} + N_{\text{remaining}}^{(k)} \equiv N_{\text{generated}}^{(k)}$$
2. **Physical Arrival Verification**:
   $$\|\mathbf{p}_{\text{AMR}}(t_{\text{complete}}) - \mathbf{p}_{\text{dropoff}}\|_2 \le 0.50\,\text{m}$$
3. **Safety Abort Threshold**:
   $$\min_{j \neq i} \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|_2 < 0.350\,\text{m} \implies \text{Immediate SAFETY\_ABORT}$$
4. **Safety Preemption Independence**:
   Reactive LiDAR safety controller ($10\,\text{Hz}$) holds unconditional preemption authority over motor velocity commands regardless of planning horizon or compute mode.

---

## 7. Initial Pilot Experiment (Phase 2 Sanity Check)

To ensure rapid validation before launching large experimental sweeps, Phase 2 will begin with a tightly scoped pilot experiment:

- **Environment**: Tier M9-V1 ($32\,\text{m} \times 32\,\text{m}$ Expanded Warehouse).
- **Fleet Size**: 5 AMRs (`amr_0` through `amr_4`).
- **Workload**: 15 tasks (`workload_15_tasks_m9_v1.yaml`), seed 42.
- **Mission Horizon**: $120.0\,\text{s}$ (calibrated for larger $32\times 32$ distances).
- **Communication Profile**: `NORMAL` (nominal network, 0% loss, $<1\,\text{ms}$ latency).
- **Compute Mode**: `ADAPTIVE` (M8B policy active).
- **Execution Command**:
  ```bash
  python3 scripts/run_benchmark.py --workload 15 --world warehouse_m9_v1 \
      --fleet-size 5 --compute-mode ADAPTIVE --horizon 120.0 --seed 42 --m9
  ```
- **Success Criteria**:
  1. Gazebo Harmonic spawns world and 5 AMRs cleanly within $45\,\text{s}$.
  2. All 15 tasks discovered on `/tasks/all` and allocated via CBBA.
  3. AMRs traverse routes exceeding $15\,\text{m}$ without pathing exceptions.
  4. Minimum inter-robot separation remains $\ge 0.350\,\text{m}$ (0 safety aborts).
  5. Raw telemetry JSON saved to `results/m9/raw/` adhering to schema `m9.v1`.

---

## 8. Physical Assumptions & Constraint Integrity

The environment design strictly adheres to physical robotics reality. No physically impossible scenarios are created merely to force coordination failures:

1. **AMR Physical Dimensions**:
   - Chassis: Differential-drive mobile platform, diameter $D = 0.50\,\text{m}$, wheel baseline $b = 0.30\,\text{m}$, height $h = 0.25\,\text{m}$.
   - Safety envelope: $R_{\text{robot}} = 0.25\,\text{m} + 0.10\,\text{m}$ buffer $= 0.35\,\text{m}$ radius ($0.70\,\text{m}$ minimum bounding circle).
2. **Corridor Width Analysis**:
   - **Wide Aisles ($\ge 2.4\,\text{m}$)**:
     - Combined robot width: $2 \times 0.50 = 1.0\,\text{m}$.
     - Safety margin: $2.4 - 1.0 = 1.4\,\text{m}$ free lateral clearance. Two-way traffic flows with zero physical risk.
   - **Narrow Aisles ($1.6\,\text{m} - 1.8\,\text{m}$)**:
     - Free lateral clearance: $1.6 - 1.0 = 0.6\,\text{m}$.
     - Safe passing is physically possible if robots hug the lane edges. Triggers caution and PIBT prioritization.
   - **Single-Lane Chokepoints ($1.1\,\text{m}$)**:
     - Free lateral clearance: $1.1 - 1.0 = 0.1\,\text{m} < 0.35\,\text{m}$ safety threshold!
     - **Passing is physically impossible**.
     - Two opposing robots entering simultaneously will collide or abort.
     - **Physical Solution**: Dedicated passing bays ($2.5\,\text{m} \times 2.0\,\text{m}$) are placed at entrances and midpoints. Expected behavior: one robot yields in the bay while the higher-priority robot traverses.
3. **Sensor Specifications**:
   - 2D Planar LiDAR: $360^\circ$ FOV, range $10.0\,\text{m}$, minimum detection $0.10\,\text{m}$, scan rate $10.0\,\text{Hz}$.
   - Wheel Encoders: Integrated odometry at $50.0\,\text{Hz}$.

---

## 9. Computational & Resource Risks

1. **Gazebo Physics Overhead**:
   - DART physics engine calculates multi-body contact dynamics and wheel-ground friction.
   - At 5 AMRs, RTF is typically $\approx 1.0$.
   - At 15–20 AMRs, physics collision checks scale quadratically ($O(N^2)$), risking RTF degradation down to $0.4 - 0.6$.
   - **Mitigation**: Headless simulation execution (`headless:=true`), optimized collision meshes (primitives only), and explicit RTF logging.
2. **ROS 2 Topic & DDS Graph Overhead**:
   - With 15 AMRs, the ROS 2 graph contains over 120 topics (odometry, scans, command velocities, reservations, CBBA bids).
   - Zenoh router / rmw_zenoh_cpp prevents discovery storms and decouples DDS broadcast domains.
3. **Planning Time Budget Scaling**:
   - A* search space on a $32 \times 32$ grid ($1,024$ cells) vs $16 \times 16$ ($256$ cells) expands by $4\times$.
   - On a $64 \times 64$ grid, state space expands by $16\times$.
   - In M8B, planning budget was $50\,\text{ms}$ in NORMAL mode. A* searches on $32\times 32$ typically execute in $1 - 4\,\text{ms}$, well within budget.

---

## 10. File Modification & Protection Plan

### 10.1 Files Proposed for Creation / Modification in Phase 2
- **New SDF World Files**:
  - `src/amr_fleet_bringup/worlds/warehouse_m9_v1.sdf`
  - `src/amr_fleet_bringup/worlds/warehouse_m9_v2.sdf`
  - `src/amr_fleet_bringup/worlds/warehouse_m9_v3.sdf`
- **New Map Configurations**:
  - `config/maps/warehouse_m9_v1.yaml`
  - `config/maps/warehouse_m9_v2.yaml`
  - `config/maps/warehouse_m9_v3.yaml`
- **New Fleet Spawning Configurations**:
  - `config/robots/fleet_5_robots_m9_v1.yaml`
  - `config/robots/fleet_10_robots_m9_v1.yaml`
  - `config/robots/fleet_15_robots_m9_v1.yaml`
  - `config/robots/fleet_10_robots_m9_v3.yaml`
- **New Workload Configurations**:
  - `config/workloads/workload_15_tasks_m9_v1.yaml`
  - `config/workloads/workload_30_tasks_m9_v1.yaml`
  - `config/workloads/workload_30_tasks_m9_v3.yaml`
- **Infrastructure Code Updates (Parametric Map Support)**:
  - `src/amr_fleet_sim/amr_fleet_sim/grid_world.py`: Add `load_from_yaml(yaml_path)` to load any map geometry dynamically.
  - `src/amr_fleet_core/amr_fleet_core/rh_node.py`: Accept `map_config_file` parameter to initialize `GridWorld` dynamically.
  - `src/amr_fleet_bringup/amr_fleet_bringup/warehouse_visualizer.py`: Read map dimensions dynamically from loaded YAML.
  - `scripts/fleet_dashboard.py`: Parameterize canvas scaling and lane drawing based on active map parameters.
  - `scripts/run_benchmark.py`: Add `--world`, `--map-config`, `--fleet-size` arguments and support M9 schema.

### 10.2 Files Explicitly FROZEN & PROTECTED from Modification
The following core coordination and safety files must NOT be modified under any circumstances:
- `src/amr_fleet_core/amr_fleet_core/cbba_allocator.py` (CBBA auction allocation logic)
- `src/amr_fleet_core/amr_fleet_core/cbba_agent.py` (CBBA agent state machine)
- `src/amr_fleet_core/amr_fleet_core/rh_planner.py` (Rolling-horizon A* planning algorithm)
- `src/amr_fleet_core/amr_fleet_core/pibt_planner.py` (PIBT priority inheritance logic)
- `src/amr_fleet_core/amr_fleet_core/reservation_table.py` (Space-time reservation table data structures)
- `src/amr_fleet_core/amr_fleet_core/wfg_deadlock.py` (Wait-For-Graph cycle detection)
- `src/amr_fleet_core/amr_fleet_core/deadlock_recovery.py` (Deadlock yielding and recovery actions)
- `src/amr_fleet_core/amr_fleet_core/compute_modes.py` (Compute mode enum and configuration dataclass)
- `src/amr_fleet_core/amr_fleet_core/adaptive_compute_policy.py` (Adaptive compute state machine & thresholds)
- `src/amr_fleet_core/amr_fleet_core/communication_model.py` (Network fault injection model)
- Reactive safety braking controller in `rh_node.py` ($10\,\text{Hz}$ emergency braking preemption)
- Historical checkpoints and results: `docs/checkpoints/M8A.md`, `docs/checkpoints/M8B.md`, `results/m8a/`, `results/m8b/`

---

## 11. Implementation Roadmap for Phase 2

1. **Step 1: Generic Map Loading in `GridWorld`**:
   Extend `GridWorld` with `from_yaml(yaml_path)` to parse dimensions, obstacles, and stations declaratively, maintaining 100% backward compatibility with `create_warehouse_grid()`.
2. **Step 2: World Modeling & Generation**:
   Construct Gazebo Harmonic SDF files (`warehouse_m9_v1.sdf`, `warehouse_m9_v2.sdf`, `warehouse_m9_v3.sdf`) using standardized industrial materials and collision geometry.
3. **Step 3: Configuration Ecosystem**:
   Create corresponding YAML definitions for maps, fleet spawns (5, 10, 15 AMRs), and workloads (15, 30, 50 tasks).
4. **Step 4: Launch & Node Parameterization**:
   Update `m8b_adaptive_fleet.launch.py` and `fleet.launch.py` to accept `world`, `map_config`, and `fleet_config` cleanly.
5. **Step 5: Observability Upgrades**:
   Update `warehouse_visualizer.py` and `fleet_dashboard.py` to auto-scale visualization to active map dimensions.
6. **Step 6: Pilot Execution & Validation**:
   Execute the M9-V1 pilot experiment, verify zero regressions in test suite (157/157 passing), and review findings.

---

## 12. Research Questions M9 Will Answer

1. **RQ1 (Topological Transferability)**:
   Does the decentralized rolling-horizon + reservation architecture maintain collision-free safety and stable throughput when transferred from a $16\,\text{m}\times 16\,\text{m}$ arena to a $32\,\text{m}\times 32\,\text{m}$ ($4\times$) and $48\,\text{m}\times 32\,\text{m}$ ($6\times$) topology without algorithmic tuning?
2. **RQ2 (Chokepoint Yielding & Deadlock Resolution)**:
   Can local PIBT priority inheritance and WFG cycle recovery autonomously deconflict single-lane bidirectional corridors using passing bays, or does decentralized state staleness lead to deadlocks or safety aborts?
3. **RQ3 (Fleet Scaling Dynamics)**:
   How do coordination metrics (conflict frequency, WFG recovery events, CBBA convergence latency) scale as fleet size grows from 5 to 10 and 15 AMRs in constrained warehouse topologies?
4. **RQ4 (Adaptive Compute under Spatial Bottlenecks)**:
   How does the M8B adaptive compute policy respond to spatial bottlenecks (chokepoints) compared to communication degradation? Does the reallocation to frequent replanning successfully resolve physical queues?

---

## 13. Limitations & Non-Goals

1. **Non-Goal: Algorithmic Modification**:
   M9 will not introduce new path planners, new auction algorithms, or centralized coordinators. The goal is strictly to evaluate the frozen M8B system.
2. **Non-Goal: Physics-Level Crowds (100+ Gazebo AMRs)**:
   Simulating 100+ full-physics differential-drive robots with laser scanners in Gazebo Harmonic is computationally intractable on single-workstation hardware. Fleet sizes $\ge 20$ will be evaluated in abstract MAPF simulation.
3. **Limitation: Physical Passing Impossibility**:
   In corridors narrower than $1.7\,\text{m}$, two-way passing without passing bays is physically impossible. The system cannot resolve physically impossible topologies.
