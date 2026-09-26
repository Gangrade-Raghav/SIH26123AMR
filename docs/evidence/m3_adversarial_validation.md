# Milestone 3 Adversarial Environment & Collision Resilience Validation

**Generated:** 2026-09-23T16:57:59.950437+00:00  
**Execution Level:** INTEGRATION / SIMULATION  
**Result:** ALL PASSED (9/9)

| ID | Scenario | Pathway | Resolving Mechanism | Geometric Overlap Proxy Count | Status |
| :--- | :--- | :--- | :--- | :---: | :---: |
| **M3-A** | Dynamic Aisle Blockage | Environment Oracle | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-B** | Sensor-Visible Dynamic Obstacle | Sensor + Oracle | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-B2** | Sensor-Only Dynamic Obstacle | Sensor Only | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-C1** | Same-Cell Contention | Adversarial Injector | SpaceTimeReservationTable + PIBT | 0 | ✅ PASS |
| **M3-C2** | Narrow Corridor Opposing Entry | Adversarial Injector | SpaceTimeReservationTable + PIBT | 0 | ✅ PASS |
| **M3-C3** | Crossing Trajectories | Adversarial Injector | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-C4** | Injected Reservation Conflict | Adversarial Injector | SingleAgentAStar + ResTable | 0 | ✅ PASS |
| **M3-C5** | High-Contention Intersection | Adversarial Injector | PIBT Local Planner | 0 | ✅ PASS |
| **M3-H** | Failed Robot in Contested Choke Point | Choke Point Failure | SingleAgentAStar + ResTable | 0 | ✅ PASS |

## Scenario Verification Notes

### M3-A: Dynamic Aisle Blockage (Planned / Oracle Event)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** Graph withdrawn for 3 cells; 1 reservations revoked; A* detoured (+4 cells, 0.15ms); 0 geometric overlap proxy violations; restored upon unblock.
- **Detailed Metrics:**
```json
{
  "baseline_path_len": 11,
  "detour_path_len": 15,
  "revoked_reservations": 1,
  "blockage_intersecting_cells": 0,
  "replan_duration_ms": 0.15,
  "geometric_overlap_proxy_count": 0,
  "restored_path_matches_baseline": true
}
```

### M3-B: Sensor-Visible Dynamic Obstacle (Discrete 8-Timestamp Breakdown)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** SYNTHETIC TIMING FIXTURE: 8-timestamp pipeline verified strictly monotonic. Timestamps are synthetic fixtures validating sequence ordering and telemetry instrumentation, NOT empirical sensor or planner latency measurements. 0 geometric overlap proxy violations.
- **Detailed Metrics:**
```json
{
  "timing_mode": "SYNTHETIC TIMING FIXTURE (Instrumentation / Sequence Verification Only)",
  "timestamp_injection_s": 100.0,
  "timestamp_sensor_obs_s": 100.042,
  "timestamp_local_safety_s": 100.05,
  "timestamp_graph_update_s": 100.05499999999999,
  "timestamp_res_update_s": 100.059,
  "timestamp_replan_start_s": 100.06099999999999,
  "timestamp_replan_done_s": 100.076,
  "timestamp_resume_s": 100.08399999999999,
  "fixture_sensor_obs_delta_ms": 42.0,
  "fixture_local_safety_delta_ms": 8.0,
  "fixture_replan_delta_ms": 15.0,
  "fixture_total_recovery_delta_ms": 42.0,
  "geometric_overlap_proxy_count": 0
}
```

### M3-B2: Sensor-Only Dynamic Obstacle (Zero Oracle Event)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** SYNTHESIZED LASERSCAN INPUT -> SENSOR PIPELINE INTEGRATION -> LOCAL RECOVERY -> REPLANNING: Exactly zero oracle notifications emitted. Synthesized LaserScan ranges detected obstacle at (3, 4); evaluated LOCAL_SIDESTEP; graph updated locally; 1 reservations revoked; A* synthesized 9-cell detour avoiding obstruction; 0 geometric overlap proxy violations. (Note: Validated as sensor-pipeline integration test, not physical Gazebo LiDAR).
- **Detailed Metrics:**
```json
{
  "pipeline_stage_sequence": "SYNTHESIZED LASERSCAN INPUT -> SENSOR PIPELINE INTEGRATION -> LOCAL RECOVERY -> REPLANNING",
  "sensor_test_classification": "SENSOR_PIPELINE_INTEGRATION (Synthesized LaserScan range array fed to LocalObstacleDetector; not Gazebo physical LiDAR simulation)",
  "oracle_event_count": 0,
  "sensor_detected_cells_count": 1,
  "local_recovery_action": "LOCAL_SIDESTEP",
  "revoked_reservations_count": 1,
  "detour_path_len": 9,
  "geometric_overlap_proxy_count": 0
}
```

### M3-C1: Same-Cell Contention (Adversarial Injection)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** Vertex contention materialized at (5, 5); duplicate write rejected; PIBT ordered passage in 0.124ms: amr_0 assigned (5, 5), amr_1 yielded to (5, 4); 0 geometric overlap proxy violations. (Classified as PLANNER_LEVEL_PIBT_INTEGRATION).
- **Detailed Metrics:**
```json
{
  "conflict_materialized": true,
  "table_rejected_duplicate": true,
  "resolving_component": "SpaceTimeReservationTable + PIBT",
  "pibt_execution_classification": "PLANNER_LEVEL_PIBT_INTEGRATION (Direct call to PIBTLocalPlanner.plan_step; not full runtime node invocation)",
  "distinct_cells_assigned": true,
  "pibt_telemetry": {
    "pibt_invoked": true,
    "trigger": "SAME_CELL_VERTEX_CONTENTION",
    "robots_involved": [
      "amr_0",
      "amr_1"
    ],
    "action": "PRIORITIZED_PUSH_AND_YIELD",
    "result": "SUCCESS",
    "duration_ms": 0.124,
    "execution_level": "PLANNER_LEVEL_PIBT_INTEGRATION",
    "assigned_moves": {
      "amr_0": [
        5,
        5
      ],
      "amr_1": [
        5,
        4
      ]
    }
  },
  "geometric_overlap_proxy_count": 0
}
```

### M3-C2: Narrow Corridor Opposing Entry (Adversarial Injection)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** Opposing entry materialized edge swap at t=0; reservation table rejected swap; PIBT serialized movement in 0.082ms without deadlock; 0 geometric overlap proxy violations. (Classified as PLANNER_LEVEL_PIBT_INTEGRATION).
- **Detailed Metrics:**
```json
{
  "conflict_materialized": true,
  "edge_swap_detected": true,
  "table_rejected_swap": true,
  "resolving_component": "SpaceTimeReservationTable + PIBT",
  "pibt_execution_classification": "PLANNER_LEVEL_PIBT_INTEGRATION (Direct call to PIBTLocalPlanner.plan_step; not full runtime node invocation)",
  "no_head_on_collision": true,
  "pibt_telemetry": {
    "pibt_invoked": true,
    "trigger": "OPPOSING_CORRIDOR_EDGE_SWAP",
    "robots_involved": [
      "amr_0",
      "amr_1"
    ],
    "action": "PRIORITIZED_PUSH_AND_YIELD",
    "result": "SUCCESS",
    "duration_ms": 0.082,
    "execution_level": "PLANNER_LEVEL_PIBT_INTEGRATION",
    "assigned_moves": {
      "amr_1": [
        3,
        4
      ],
      "amr_0": [
        3,
        5
      ]
    }
  },
  "geometric_overlap_proxy_count": 0
}
```

### M3-C3: Crossing Trajectories (Adversarial Injection)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** Trajectory conflict materialized at (6, 6) for t=3; table serialized arrivals: amr_0 cleared at t=3, amr_1 crossed at t=4; 0 geometric overlap proxy violations.
- **Detailed Metrics:**
```json
{
  "conflict_materialized": true,
  "contested_junction": [
    6,
    6
  ],
  "arrival_step": 3,
  "first_grantee": "amr_0",
  "yielded_grantee": "amr_1",
  "serialized_passage": true,
  "geometric_overlap_proxy_count": 0
}
```

### M3-C4: Injected Reservation Conflict (Non-Corrupting State Injection)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** Adversarial duplicate write on (5, 5) at t=2 rejected; table uncorrupted; authoritative owners == {"amr_0"} (total: 1); amr_1 is NOT an authoritative owner; 0 geometric overlap proxy violations.
- **Detailed Metrics:**
```json
{
  "synthetic_write_rejected": true,
  "authoritative_owners": [
    "amr_0"
  ],
  "total_authoritative_owners": 1,
  "amr0_is_owner": true,
  "amr1_is_owner": false,
  "table_corruption_detected": false,
  "geometric_overlap_proxy_count": 0
}
```

### M3-C5: High-Contention Intersection (3 AMRs Converging, PIBT Resolution)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** 3 AMRs converged on (7, 7); PIBT resolved in 0.095ms with 0 overlaps; amr_0 entered first, peers queued/yielded safely; full PIBT telemetry recorded; 0 geometric overlap proxy violations. (Classified as PLANNER_LEVEL_PIBT_INTEGRATION).
- **Detailed Metrics:**
```json
{
  "converging_agents_count": 3,
  "intersection_center": [
    7,
    7
  ],
  "assigned_moves": {
    "amr_0": [
      7,
      7
    ],
    "amr_1": [
      7,
      6
    ],
    "amr_2": [
      8,
      7
    ]
  },
  "zero_geometric_overlap": true,
  "pibt_execution_classification": "PLANNER_LEVEL_PIBT_INTEGRATION (Direct call to PIBTLocalPlanner.plan_step; not full runtime node invocation)",
  "pibt_telemetry": {
    "pibt_invoked": true,
    "trigger": "HIGH_CONTENTION_INTERSECTION_ARBITRATION",
    "robots_involved": [
      "amr_0",
      "amr_1",
      "amr_2"
    ],
    "action": "PRIORITIZED_PUSH_AND_YIELD",
    "result": "SUCCESS",
    "duration_ms": 0.095,
    "execution_level": "PLANNER_LEVEL_PIBT_INTEGRATION",
    "assigned_moves": {
      "amr_0": [
        7,
        7
      ],
      "amr_1": [
        7,
        6
      ],
      "amr_2": [
        8,
        7
      ]
    }
  },
  "geometric_overlap_proxy_count": 0
}
```

### M3-H: Failed Robot in Contested Choke Point (0.8m Envelope & CBBA Reallocation)
- **Execution Level:** `INTEGRATION / SIMULATION`
- **Verification Evidence:** amr_1 failed at choke point (7, 7); reservations cleared; 0.8m experimental envelope (25 cells) blocked; amr_0 detoured around envelope (0.13ms); task reclaimed and reallocated to amr_0 via CBBA; 0 duplicate ownership; 0 geometric overlap proxy violations.
- **Detailed Metrics:**
```json
{
  "victim_robot_id": "amr_1",
  "choke_cell": [
    7,
    7
  ],
  "keep_out_envelope_m": 0.8,
  "envelope_cells_withdrawn": 25,
  "detour_path_len": 13,
  "detour_intersects_envelope": 0,
  "replan_duration_ms": 0.13,
  "task_reclaimed": true,
  "new_owner_robot_id": "amr_0",
  "duplicate_ownership_observed": false,
  "geometric_overlap_proxy_count": 0
}
```

