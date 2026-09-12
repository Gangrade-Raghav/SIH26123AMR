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
