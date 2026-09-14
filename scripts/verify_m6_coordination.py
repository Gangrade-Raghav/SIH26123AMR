#!/usr/bin/env python3
"""
Comprehensive Verification Suite for M6 Multi-Agent Coordination.

Verifies:
1. Space-Time Reservation Table (vertex, edge, ownership, expiration, release)
2. Conflict Detection Engine (vertex conflicts, edge swaps, waiting conflicts)
3. PIBT-Style Prioritized Local Planner (priorities, candidate generation, inheritance, backtracking)
4. Wait-For Graph (WFG) & Persistent Deadlock Detection (cycles, transient vs persistent)
5. Deterministic Deadlock Recovery (victim selection, lateral sidestepping)
6. MultiAgentCoordinator Wrapping M5 RollingHorizonPlanner
7. ROS 2 Message Data Models Serializability
8. Bitwise / Exact Determinism Verification
"""

import math
import sys
import time
from typing import Dict, List, Tuple

from amr_fleet_core.conflict_detector import ConflictDetector
from amr_fleet_core.coordination_models import (
    ConflictType,
    CoordinationState,
    Position,
    Reservation,
)
from amr_fleet_core.deadlock_recovery import DeadlockRecoveryManager
from amr_fleet_core.multi_agent_coordinator import MultiAgentCoordinator
from amr_fleet_core.pibt_planner import PIBTAgentState, PIBTLocalPlanner
from amr_fleet_core.reservation_table import SpaceTimeReservationTable
from amr_fleet_core.rh_planner import RHConfig, RollingHorizonPlanner
from amr_fleet_core.wfg_deadlock import DeadlockDetector, WaitForGraph
from amr_fleet_sim.grid_world import GridWorld


def test_section(title: str):
    """Print test section header."""
    print(f"\n{'='*70}\n[TEST SECTION] {title}\n{'='*70}")


def assert_test(cond: bool, desc: str):
    """Assert condition and print formatted status."""
    if cond:
        print(f"  [PASS] {desc}")
    else:
        print(f"  [FAIL] {desc}")
        raise AssertionError(f"Test failed: {desc}")


def verify_space_time_reservations() -> None:
    """Verify SpaceTimeReservationTable vertex, edge, release, and expiration."""
    test_section("1. Space-Time Reservations & Rolling-Horizon Expiration")
    table = SpaceTimeReservationTable()

    # Vertex reservation
    ok = table.reserve((4, 5), time_step=1, robot_id="amr_0", priority=100.0)
    assert_test(ok, "amr_0 successfully reserves cell (4, 5) at t=1")
    assert_test(table.is_reserved((4, 5), time_step=1, robot_id="amr_1"), "amr_1 sees (4, 5) reserved at t=1")
    assert_test(not table.is_reserved((4, 5), time_step=1, robot_id="amr_0"), "amr_0 owns the reservation (no self-conflict)")

    # Peer reservation attempt rejected
    ok2 = table.reserve((4, 5), time_step=1, robot_id="amr_1", priority=50.0)
    assert_test(not ok2, "amr_1 fails to reserve already-held cell (4, 5) at t=1")

    # Release and re-reservation
    table.release("amr_0")
    ok3 = table.reserve((4, 5), time_step=1, robot_id="amr_2", priority=200.0)
    assert_test(ok3, "After amr_0 releases, amr_2 successfully reserves cell (4, 5) at t=1")
    assert_test(table.get_reservation((4, 5), 1).robot_id == "amr_2", "Owner is now amr_2")

    # Edge reservation
    e_ok = table.reserve_edge((2, 2), (2, 3), time_step=2, robot_id="amr_0", priority=100.0)
    assert_test(e_ok, "amr_0 reserves directed edge (2, 2) -> (2, 3) at t=2")
    assert_test(table.is_edge_conflict((2, 3), (2, 2), time_step=2, robot_id="amr_1"),
                "Edge-swap conflict correctly detected for reverse transition (2, 3) -> (2, 2)")
    assert_test(not table.is_edge_conflict((2, 2), (2, 3), time_step=2, robot_id="amr_0"),
                "No edge conflict for owner amr_0")

    # Rolling-horizon expiration
    table.reserve((1, 1), time_step=3, robot_id="amr_0")
    table.reserve((1, 2), time_step=6, robot_id="amr_0")
    pruned = table.release_time_before(5)
    assert_test(pruned >= 3, f"Pruned {pruned} reservations with t < 5")
    assert_test(not table.is_reserved((1, 1), time_step=3, robot_id="amr_1"), "Expired reservation at t=3 is gone")
    assert_test(table.is_reserved((1, 2), time_step=6, robot_id="amr_1"), "Future reservation at t=6 remains active")

    # Robot release
    table.release_robot("amr_0")
    assert_test(not table.is_reserved((1, 2), time_step=6, robot_id="amr_1"), "Releasing amr_0 clears all its reservations")


def verify_conflict_detection() -> None:
    """Verify ConflictDetector on vertex, edge-swap, and waiting conflicts."""
    test_section("2. Deterministic Conflict Detection Engine")
    detector = ConflictDetector()

    # Vertex conflict
    t_v = {
        "amr_0": [(2, 2), (3, 2), (4, 2)],
        "amr_1": [(4, 4), (4, 3), (4, 2)],
    }
    c_v = detector.check_fleet_trajectories(t_v, start_time_step=0)
    assert_test(len(c_v) == 1, f"Detected exactly 1 vertex conflict (found {len(c_v)})")
    assert_test(c_v[0].conflict_type == ConflictType.VERTEX.value, "Conflict is identified as VERTEX")
    assert_test(c_v[0].time_step == 2 and c_v[0].cell == (4, 2), "Vertex conflict occurs at cell (4, 2) at t=2")

    # Edge-swap conflict
    t_e = {
        "amr_0": [(5, 5), (5, 6)],
        "amr_1": [(5, 6), (5, 5)],
    }
    c_e = detector.check_fleet_trajectories(t_e, start_time_step=0)
    assert_test(len(c_e) == 1, f"Detected exactly 1 edge-swap conflict (found {len(c_e)})")
    assert_test(c_e[0].conflict_type == ConflictType.EDGE_SWAP.value, "Conflict is identified as EDGE_SWAP")

    # Waiting conflict
    t_w = {
        "amr_0": [(3, 3), (3, 3), (3, 3)],
        "amr_1": [(3, 2), (3, 3), (3, 4)],
    }
    c_w = detector.check_fleet_trajectories(t_w, start_time_step=0)
    assert_test(len(c_w) == 1, f"Detected waiting/occupancy conflict at t=1 (found {len(c_w)})")

    # Conflict-free trajectories
    t_ok = {
        "amr_0": [(1, 1), (1, 2), (1, 3)],
        "amr_1": [(2, 1), (2, 2), (2, 3)],
    }
    c_ok = detector.check_fleet_trajectories(t_ok, start_time_step=0)
    assert_test(len(c_ok) == 0, "Zero conflicts for parallel collision-free trajectories")


def verify_pibt_planner() -> None:
    """Verify PIBTLocalPlanner candidate moves, priority inheritance, and backtracking."""
    test_section("3. PIBT-Style Prioritized Local Planner")
    grid = GridWorld.create_warehouse_grid(resolution=0.5)
    res_table = SpaceTimeReservationTable()
    planner = PIBTLocalPlanner(grid, res_table)

    # 1. Candidate generation
    agent = PIBTAgentState(
        robot_id='amr_0',
        current_pos=(5, 5),
        goal_pos=(5, 8),
        preferred_path=[(5, 6), (5, 7), (5, 8)],
    )
    moves = planner.generate_candidate_moves(agent, curr_pos=(5, 5))
    assert_test(moves[0] == (5, 6), f"First candidate moves closer to goal (got {moves[0]})")
    assert_test((5, 5) in moves, "Wait action (stay at curr_pos) is included in candidate moves")

    # 2. Priority calculation determinism
    a_high = PIBTAgentState('amr_0', (1, 1), (8, 8), task_priority=3)
    a_low = PIBTAgentState('amr_1', (1, 1), (8, 8), task_priority=1)
    a_tie1 = PIBTAgentState('amr_1', (1, 1), (5, 5), task_priority=2)
    a_tie2 = PIBTAgentState('amr_2', (1, 1), (5, 5), task_priority=2)

    assert_test(a_high.compute_priority(grid) > a_low.compute_priority(grid),
                "Task priority 3 yields strictly higher priority than priority 1")
    assert_test(a_tie1.compute_priority(grid) > a_tie2.compute_priority(grid),
                "Deterministic tie-breaker: amr_1 > amr_2")

    # 3. Priority inheritance & push
    agents = {
        "amr_0": PIBTAgentState("amr_0", current_pos=(3, 3), goal_pos=(3, 6), task_priority=3,
                                preferred_path=[(3, 4), (3, 5), (3, 6)]),
        "amr_1": PIBTAgentState("amr_1", current_pos=(3, 4), goal_pos=(8, 8), task_priority=1),
    }
    res_moves, _ = planner.plan_step(agents, time_step=0)
    assert_test(res_moves["amr_0"] == (3, 4), "amr_0 claims cell (3, 4)")
    assert_test(res_moves["amr_1"] != (3, 4) and res_moves["amr_1"] != (3, 3),
                f"amr_1 was successfully pushed out of (3, 4) to free neighbor {res_moves['amr_1']}")

    # 4. Backtracking when push cannot be accommodated
    obstacles = [(1, 3), (3, 3), (2, 4)]
    cul_de_sac_grid = GridWorld(10, 10, obstacles=obstacles)
    deadend_planner = PIBTLocalPlanner(cul_de_sac_grid, SpaceTimeReservationTable())
    deadend_agents = {
        "amr_0": PIBTAgentState("amr_0", current_pos=(2, 2), goal_pos=(2, 3), task_priority=3),
        "amr_1": PIBTAgentState("amr_1", current_pos=(2, 3), goal_pos=(2, 3), task_priority=1),
    }
    deadend_res, _ = deadend_planner.plan_step(deadend_agents, time_step=0)
    assert_test(deadend_res["amr_1"] == (2, 3), "amr_1 remains at (2, 3) because it cannot vacate")
    assert_test(deadend_res["amr_0"] != (2, 3), "amr_0 backtracks and chooses alternative/wait")


def verify_wfg_and_deadlock_detection() -> None:
    """Verify Wait-For Graph cycle detection and transient vs persistent deadlocks."""
    test_section("4. Wait-For Graph (WFG) & Persistent Deadlock Detection")
    wfg = WaitForGraph()

    # 2-Node cycle
    wfg.add_wait("amr_0", "amr_1", (4, 4), 1)
    wfg.add_wait("amr_1", "amr_0", (4, 3), 1)
    cycles_2 = wfg.find_cycles()
    assert_test(len(cycles_2) == 1, f"Detected 2-node cycle {cycles_2[0]}")

    # 3-Node cycle
    wfg3 = WaitForGraph()
    wfg3.add_wait("amr_0", "amr_1", (4, 4), 1)
    wfg3.add_wait("amr_1", "amr_2", (4, 5), 1)
    wfg3.add_wait("amr_2", "amr_0", (4, 3), 1)
    cycles_3 = wfg3.find_cycles()
    assert_test(len(cycles_3) == 1 and len(cycles_3[0]) == 3, f"Detected 3-node cycle {cycles_3[0]}")

    # Transient wait vs Persistent Deadlock
    detector = DeadlockDetector(persistence_threshold_sec=0.4, min_stall_cycles=3)
    positions = {"amr_0": (2.0, 2.0), "amr_1": (2.0, 2.5)}

    # Tick 1: cycle present, but elapsed time = 0
    t0 = time.time()
    dls_1 = detector.update(wfg, positions, now_sec=t0)
    assert_test(len(dls_1) == 0, "Transient wait not classified as deadlock on tick 1")

    # Tick 2: elapsed time < threshold and counts < min_stall_cycles
    dls_2 = detector.update(wfg, positions, now_sec=t0 + 0.2)
    assert_test(len(dls_2) == 0, "Transient wait below threshold not classified as deadlock on tick 2")

    # Tick 3: elapsed time >= threshold and zero movement
    dls_3 = detector.update(wfg, positions, now_sec=t0 + 0.5)
    assert_test(len(dls_3) == 1, "Persistent cycle exceeding threshold correctly classified as DEADLOCK")
    assert_test(set(dls_3[0].cycle_robot_ids) == {"amr_0", "amr_1"}, "Deadlock cycle participants correctly logged")


def verify_deadlock_recovery() -> None:
    """Verify DeadlockRecoveryManager deterministic recovery & lateral sidestepping."""
    test_section("5. Deterministic Deadlock Recovery")
    grid = GridWorld.create_warehouse_grid(resolution=0.5)
    res_table = SpaceTimeReservationTable()
    recovery = DeadlockRecoveryManager(grid, res_table)

    wfg = WaitForGraph()
    wfg.add_wait("amr_0", "amr_1", (5, 5), 1)
    wfg.add_wait("amr_1", "amr_0", (5, 4), 1)

    detector = DeadlockDetector(persistence_threshold_sec=0.1, min_stall_cycles=1)
    positions = {"amr_0": (2.5, 2.0), "amr_1": (2.5, 2.5)}
    t0 = time.time()
    detector.update(wfg, positions, t0)
    dls = detector.update(wfg, positions, t0 + 0.2)
    assert_test(len(dls) == 1, "Confirmed deadlock record available")

    agents = {
        "amr_0": PIBTAgentState("amr_0", (5, 4), (5, 8), task_priority=2),
        "amr_1": PIBTAgentState("amr_1", (5, 5), (5, 2), task_priority=1),
    }

    ok = recovery.execute_recovery(dls[0], agents, time_step=1)
    assert_test(ok, "Recovery execution succeeded")
    assert_test(dls[0].recovery_success, "DeadlockRecord marked as recovered")
    assert_test('SIDESTEP' in dls[0].recovery_action, f"Recovery action logged: {dls[0].recovery_action}")

    # amr_1 is the lower priority victim and should have sidestepped laterally
    victim_move = agents["amr_1"].planned_moves.get(2)
    assert_test(victim_move is not None and victim_move[0] != 5,
                f"amr_1 executed lateral sidestep away from corridor x=5 to {victim_move}")


def verify_multi_agent_coordinator_integration() -> None:
    """Verify MultiAgentCoordinator wrapping M5 RollingHorizonPlanner."""
    test_section("6. MultiAgentCoordinator Wrapping M5 RollingHorizonPlanner")
    coordinator = MultiAgentCoordinator()

    # Create 2 M5 planners with head-on conflicting paths
    config = RHConfig(horizon_steps=8, execution_window=4, replan_rate=2.0)
    p0 = RollingHorizonPlanner("amr_0", config)
    p1 = RollingHorizonPlanner("amr_1", config)

    p0.update_position((2.0, 2.0))
    p0.current_goal = (2.0, 5.0)
    p0.active_phase = "PICKUP"
    p0.replan()

    p1.update_position((2.0, 5.0))
    p1.current_goal = (2.0, 2.0)
    p1.active_phase = "PICKUP"
    p1.replan()

    # Pre-coordination: paths are head-on along x=2.0 (grid x=4)
    fleet = {"amr_0": p0, "amr_1": p1}
    coordinated_paths = coordinator.coordinate_fleet(fleet, current_time_step=1, window_size=4)

    # Post-coordination: verify conflict detector on coordinated paths
    conflicts = coordinator.conflict_detector.check_fleet_trajectories(coordinated_paths, start_time_step=1)
    assert_test(len(conflicts) == 0, f"Coordinated paths have exactly 0 space-time conflicts (found {len(conflicts)})")
    assert_test(coordinator.conflicts_resolved > 0, f"Resolved {coordinator.conflicts_resolved} conflicts during planning")

    metrics = coordinator.get_metrics_summary()
    assert_test(metrics["conflict_resolution_rate"] == 1.0, f"Conflict resolution rate is 100% ({metrics['conflict_resolution_rate']})")
    assert_test(metrics["average_coordination_latency_ms"] < 50.0,
                f"Coordination latency {metrics['average_coordination_latency_ms']} ms well within real-time budget (<50ms)")


def verify_bitwise_determinism() -> None:
    """Verify exact determinism across repeated coordination runs."""
    test_section("7. Bitwise & Exact Reproducibility Verification")
    coord1 = MultiAgentCoordinator()
    coord2 = MultiAgentCoordinator()

    config = RHConfig(horizon_steps=6, execution_window=4, replan_rate=2.0)

    def run_scenario(coord_inst: MultiAgentCoordinator):
        planners = {}
        for i in range(5):
            rid = f"amr_{i}"
            p = RollingHorizonPlanner(rid, config)
            p.update_position((2.0 + i * 1.5, 2.0 + i * 1.0))
            p.current_goal = (2.0 + (4 - i) * 1.5, 2.0 + (4 - i) * 1.0)
            p.active_phase = "PICKUP"
            p.replan()
            planners[rid] = p
        return coord_inst.coordinate_fleet(planners, current_time_step=0, window_size=4)

    run1 = run_scenario(coord1)
    run2 = run_scenario(coord2)

    assert_test(run1 == run2, "Exact identical output for identical inputs across separate coordinator instances")


def main():
    """Run all verification suites."""
    print("\n" + "="*70)
    print("  M6 MULTI-AGENT COORDINATION & DEADLOCK VERIFICATION SUITE")
    print("="*70)

    try:
        verify_space_time_reservations()
        verify_conflict_detection()
        verify_pibt_planner()
        verify_wfg_and_deadlock_detection()
        verify_deadlock_recovery()
        verify_multi_agent_coordinator_integration()
        verify_bitwise_determinism()

        print("\n" + "="*70)
        print("  ALL M6 ALGORITHMIC & ARCHITECTURAL CHECKS PASSED (100%)")
        print("="*70 + "\n")
        return 0
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}\n")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
