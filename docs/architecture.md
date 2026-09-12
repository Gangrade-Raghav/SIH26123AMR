# System Architecture

## Authority hierarchy

```text
Safety
  ↓
Local collision avoidance
  ↓
Traffic/resource coordination
  ↓
MAPF
  ↓
Task allocation
  ↓
Global optimization
```

## Proposed software architecture

```text
                     TASK GENERATOR
                           │
                           ▼
                    TASK ALLOCATOR
                    ┌──────┴──────┐
                    │             │
              Centralized       CBBA
                                  │
                                  ▼
                          LIFELONG PLANNER
                          ┌──────┴──────┐
                         RHCR        GD-RHCR
                          │             │
                          └──────┬──────┘
                                 ▼
                           PIBT FALLBACK
                                 │
                                 ▼
                      TRAFFIC / RESERVATION
                                 │
                                 ▼
                              WFG
                                 │
                                 ▼
                        LOCAL SAFETY LAYER
                                 │
                          Controller
                                 │
                                 ▼
                           AMR / Gazebo

          ┌────────────────────────────────────┐
          │ Zenoh / rmw_zenoh_cpp              │
          │ distributed communication substrate│
          └────────────────────────────────────┘
```

## Interfaces

The implementation should expose stable interfaces rather than coupling algorithms directly.

### TaskAllocator

Input:
- robot states,
- task set,
- communication state.

Output:
- task ownership decisions,
- allocation epoch/version.

### GlobalPlanner

Input:
- robot states,
- assigned tasks,
- map,
- reservations,
- planning configuration.

Output:
- candidate multi-agent plan,
- plan version,
- validity interval.

### LocalPlanner

Input:
- current state,
- global plan,
- neighbors,
- safety constraints.

Output:
- safe local motion command / next action.

### DeadlockManager

Input:
- dependency edges,
- resource ownership,
- version/epoch.

Output:
- deadlock state,
- selected recovery action.

## Plan handoff

Never replace an active plan blindly.

Use:

```text
LIVE PLAN
   ↓
candidate improved/replanned plan
   ↓
safety validation
   ↓
version/reservation check
   ↓
commit
   ↓
execute
```

## Compute adaptation

The planner policy may switch according to measured compute pressure:

```text
Normal
  ↓
RHCR / GD-RHCR
  ↓
Compute pressure
  ↓
PIBT fallback
  ↓
Recovery
  ↓
Normal planning
```

The policy itself must be benchmarked. It is not assumed to be optimal.

## Multi-Robot Simulation Architecture (M2 Foundation)

Milestone M2 establishes the decentralized simulation infrastructure supporting multi-robot fleet scaling without algorithm leakage or robot-specific code duplication.

### 1. Namespace Isolation Strategy
Every AMR in the fleet operates under its own isolated ROS 2 namespace `/{robot_name}` (e.g., `/amr_0`, `/amr_1`, ..., `/amr_N-1`):
- Command velocity: `/{robot_name}/cmd_vel` (`geometry_msgs/msg/Twist`)
- Odometry telemetry: `/{robot_name}/odom` (`nav_msgs/msg/Odometry`)
- Planar LiDAR stream: `/{robot_name}/scan` (`sensor_msgs/msg/LaserScan`)
- Joint states: `/{robot_name}/joint_states` (`sensor_msgs/msg/JointState`)
- Model description: `/{robot_name}/robot_description` (`std_msgs/msg/String`)

Common infrastructure topics remain shared at the root:
- `/clock` (`rosgraph_msgs/msg/Clock`)
- `/tf` (`tf2_msgs/msg/TFMessage`)
- `/tf_static` (`tf2_msgs/msg/TFMessage`, `transient_local` QoS)

### 2. TF Frame Isolation Strategy
To prevent frame collisions across the shared `/tf` topic, every robot's kinematic and sensor links/joints are dynamically prefixed with `${robot_name}/`:
```text
/tf, /tf_static:
  ├── amr_0/odom -> amr_0/base_footprint -> amr_0/base_link -> amr_0/laser_frame
  ├── amr_1/odom -> amr_1/base_footprint -> amr_1/base_link -> amr_1/laser_frame
  └── amr_k/odom -> amr_k/base_footprint -> amr_k/base_link -> amr_k/laser_frame
```
- `amr_k/odom -> amr_k/base_footprint`: Broadcast dynamically by `gz-sim-diff-drive-system` at 30Hz.
- `amr_k/base_footprint -> amr_k/base_link -> amr_k/laser_frame`: Broadcast statically by `robot_state_publisher` with `transient_local` durability.
- Zero frame collision: no ambiguous un-namespaced frames exist.

### 3. Robot Configuration Model
Initial poses, dimensions, and world assignments are strictly externalized in YAML configurations stored under `config/robots/`:
- `config/robots/fleet_2_robots.yaml`
- `config/robots/fleet_5_robots.yaml`
- `config/robots/fleet_10_robots.yaml`
- `config/robots/fleet_default.yaml`

Coordinates are positioned in open warehouse corridors ($\ge 3.0$m inter-robot separation) to prevent spawn-time physics collisions.

### 4. Spawning Architecture
A single, parameterized launcher (`amr_fleet_bringup/launch/fleet.launch.py`) manages fleet bringup:
1. Loads robot configuration from YAML (or synthesizes deterministic corridor poses if $N > \text{config\_size}$).
2. Launches Gazebo Harmonic server (`gz_sim`) in headless mode (`-s -r`).
3. Bridges simulation clock (`/clock`).
4. Iteratively includes `spawn_robot.launch.py` for each robot instance.
5. In `spawn_robot.launch.py`, launches:
   - `robot_state_publisher` in `/{robot_name}` namespace.
   - `ros_gz_sim create` to inject the model into Gazebo.
   - `ros_gz_bridge parameter_bridge` uniquely named `/{robot_name}_bridge`.

### 5. Fleet State Discovery Abstraction
Infrastructure abstraction in `amr_fleet_core.fleet_state`:
- `RobotInfo`: Strongly typed metadata container (`robot_id`, `namespace`, `pose_topic`, `cmd_vel_topic`, `scan_topic`, `status`, `initial_pose`).
- `FleetState`: Central registry providing discovery via configuration (`from_yaml()`) or live ROS 2 topic introspection (`from_topics()`).

### 6. Known Limitations & Boundaries
- Multi-robot physical collisions are modeled by DART physics, but decentralized navigation/avoidance is deferred to M3+.
- Dynamic obstacle agents (e.g. human workers, forklifts) are not modeled in M2.
- Headless execution is established as default due to Intel integrated GPU constraints.
- No decentralized coordination (CBBA, RHCR, PIBT, WFG) is active in M2.

