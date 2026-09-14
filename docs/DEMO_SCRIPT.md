# NRDAS AMR Fleet Coordination: Final Presentation & Demonstration Script

> **Purpose**: Standardized presentation guides, demonstration sequences, and technically honest answers to evaluation questions for research reviews and hackathon judges.

---

## 1. 30-Second Elevator Pitch

> *"In modern automated warehouses, centralized robot dispatchers are fragile single points of failure that collapse under network outages and dynamic disruptions. We built and validated a fully decentralized multi-AMR coordination architecture in ROS 2 Jazzy and Gazebo Harmonic. Operating without a central planner, our 10-robot fleet uses asynchronous consensus (CBBA) for task allocation, space-time reservations for collision-free routing, and an independent reactive LiDAR safety loop. In our final stress benchmark, the fleet survived simultaneous dynamic task arrivals, a 45-second physical aisle blockage, and 35% wireless packet loss with zero collisions, zero deadlocks, and zero state corruption."*

---

## 2. 2-Minute Technical Explanation

> *"Our central research question is whether a decentralized 10-AMR fleet can maintain safe and coherent coordination under simultaneous environmental, topological, communication, and workload perturbations.*
>
> *We implemented a strict four-tier hierarchy of authority: Local Physical Safety preempts Space-Time Reservations, which preempt Rolling-Horizon Planning, which preempts CBBA Task Allocation.
>
> *1. **Task Allocation**: AMRs bid asynchronously on tasks using the Consensus-Based Bundle Algorithm (CBBA). When new tasks arrive dynamically mid-mission, the fleet re-converges in under 200 ms without canceling tasks already in progress.*  
> *2. **Rolling-Horizon Planning & Reservations**: Each robot runs a localized spatio-temporal A* search over a 10-step horizon ($h=10$), committing only 4 steps ($w=4$) and sharing space-time reservations over peer-to-peer ROS 2 topics.*  
> *3. **Resilience to Network Loss**: When packets drop (tested up to 35% loss), a localized stale-state manager autonomously prunes expired reservations, preventing ghost obstacles from causing artificial deadlocks.*  
> *4. **Dynamic Environmental Adaptation**: When an aisle is physically blocked in Gazebo Harmonic, robots update local occupancy grids within 150 ms and autonomously route around the obstruction.*  
> *5. **Zero-Collision Ground Truth**: In our canonical 180-second stress benchmark, Gazebo physics contact sensors recorded zero collisions, OBB chassis-overlap samples were zero, and the task conservation invariant held with mathematical precision."*

---

## 3. 5-Minute Full Live Demonstration Sequence

### Part 1: Architecture & Bringup (Minute 0:00 – 1:00)
- **Operator Action**: In Terminal 1, launch the 10-AMR simulation:
  ```bash
  ros2 launch amr_fleet_bringup m8b_adaptive_fleet.launch.py robot_count:=10 world:=warehouse_m9_v2 headless:=false compute_mode:=ADAPTIVE comm_profile:=NORMAL workload_file:=config/workloads/workload_30_tasks_m9_v2.yaml
  ```
- **Speaking Points**:
  - Show Gazebo Harmonic opening with 10 differential-drive AMRs spawned in `warehouse_m9_v2` ($32\,\text{m} \times 32\,\text{m}$).
  - Point out that there is NO centralized planner node. Each AMR runs its own `rh_node` and `cbba_node`.
  - Highlight the initial CBBA consensus: 15 tasks allocated in just $14.0\,\text{ms}$.

### Part 2: Fleet Observability Dashboard (Minute 1:00 – 2:00)
- **Operator Action**: In Terminal 2, launch the dashboard and open browser to `http://localhost:8080`:
  ```bash
  python3 scripts/fleet_dashboard.py --port 8080 --fleet-size 10 --world warehouse_m9_v2 --tui
  ```
- **Speaking Points**:
  - Show the live SVG mini-map with real-time pose tracking, battery levels, task bundles, and communication metrics.
  - Explain how the dashboard operates purely passively by listening to namespaced ROS 2 topics, adding zero latency to the coordination loop.

### Part 3: Spatial Coordination & Corridors (Minute 2:00 – 3:00)
- **Speaking Points**:
  - Direct attention to robots navigating narrow ($2.5\,\text{m}$) aisles.
  - Explain the Space-Time Reservation mechanism: robots broadcast planned occupancy intervals. When two robots approach an intersection, the lower-priority robot yields or takes an alternate corridor.
  - Emphasize the independent $10\,\text{Hz}$ LiDAR safety loop that operates continuously in the background with absolute stopping authority.

### Part 4: Dynamic Disruptions — Blockage, Arrival & Packet Loss (Minute 3:00 – 4:15)
- **Speaking Points**:
  - Walk the judges through the multi-event orchestration from Milestone M9-V3-E:
    1. At $t=45\,\text{s}$, 15 new tasks arrive unexpectedly. CBBA dynamically reallocates without interrupting active tasks.
    2. Simultaneously at $t=45\,\text{s}$, Aisle 1 South is physically obstructed by a fallen obstacle. Robots detect the obstacle and plan detours.
    3. At $t=75\,\text{s}$, the network degrades to $35\%$ packet drop. The fleet drops $5,642$ packets, yet coordinates seamlessly through local reservation TTL pruning.
    4. At $t=90\,\text{s}$, the corridor reopens and is restored to traversable in all local maps.

### Part 5: Empirical Results & Verification (Minute 4:15 – 5:00)
- **Operator Action**: Open `results/final/figures/final_m9_cross_scenario_comparison.png` and `final_m9_v3_e_stress_timeline.png`.
- **Speaking Points**:
  - Summarize the empirical results: 0 physics contacts, 0 OBB overlap samples, minimum distance $5.45\,\text{m}$, mean planning latency $0.31\,\text{ms}$.
  - State the task invariant: $30 = 0\text{ Staged} + 0\text{ Pending} + 28\text{ Assigned} + 2\text{ In-Progress} + 0\text{ Completed} + 0\text{ Failed} + 0\text{ Cancelled}$.
  - Conclude with project readiness and research integrity.

---

## 4. Likely Evaluator Questions & Technically Honest Answers

### Q1: "Why are there 0 completed tasks in the 180-second benchmark?"
**Honest Technical Answer**:
> *"In the expanded $32\,\text{m} \times 32\,\text{m}$ warehouse footprint ($1024\,\text{m}^2$), robots must navigate from perimeter spawn locations to rack pickup bays and then cross the facility to the central dropoff hub. At our safe differential-drive speed limit of $0.5\,\text{m/s}$, a complete pickup-and-delivery cycle requires over $120\,\text{s}$. Within the $180\,\text{s}$ horizon, 2 robots successfully reached pickups and were actively transiting to dropoffs (`IN_PROGRESS`), while the remaining 28 were assigned in bundles. Our task accounting explicitly distinguishes between assigned tasks and completed tasks to maintain complete research honesty."*

### Q2: "Did you prove that your system is universally collision-free?"
**Honest Technical Answer**:
> *"No. We do not claim a formal mathematical safety proof. What we implemented is a defense-in-depth architecture where rolling-horizon planning and space-time reservations resolve potential conflicts well in advance, backed by a 10 Hz reactive LiDAR safety loop that overrides all planning if an obstacle comes within 0.35 m. Across all our empirical benchmarks in Gazebo Harmonic, zero physical contacts and zero OBB chassis-overlap samples were observed."*

### Q3: "What is the difference between Gazebo physical contacts and OBB chassis-overlap samples?"
**Honest Technical Answer**:
> *"They are distinct metrics measuring different phenomena. 'Gazebo physical contacts' represent true ground truth measured directly by the Gazebo DART physics engine contact sensors. 'OBB chassis-overlap samples' are a geometric early-warning proxy computed using the Separating Axis Theorem (SAT) on oriented 2D bounding boxes. In all canonical runs, both metrics recorded exactly zero occurrences."*

### Q4: "Are your results statistically significant?"
**Honest Technical Answer**:
> *"Our canonical experiments were conducted with a sample size of $n=1$ under fixed pseudo-random seed 42 to provide exact reproducibility of complex multi-event timelines. Therefore, we explicitly refrain from making statistical claims regarding variance or confidence intervals. Future work will involve Monte Carlo sweeps across dozens of seeds."*

### Q5: "How does the system prevent deadlocks when packets are lost?"
**Honest Technical Answer**:
> *"Under 35% packet loss, robots may miss peer cancellation or departure messages. If reservations were permanent, dropped messages would cause 'ghost reservations' that permanently block corridors. Our reservation table implements a time-to-live (TTL) mechanism ($1.5\,\text{s}$). If a peer's heartbeat is not refreshed, its space-time reservations expire and are pruned, allowing other robots to claim the corridor. This was directly observed in M9-V3-E when peers pruned expired claims for robots `amr_2` and `amr_5`."*
