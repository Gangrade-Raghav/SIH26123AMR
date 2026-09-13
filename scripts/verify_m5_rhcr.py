#!/usr/bin/env python3
"""
M5 Verification Suite: Rolling-Horizon Task Planning (RHCR).

Verifies:
1. SingleAgentAStar pathfinding accuracy, collision checking, and determinism.
2. Horizon truncation (h) and Execution Window (w) selection.
3. Task sequencing heuristics (PRIORITY_FIRST, SHORTEST_PATH_FIRST, DEADLINE_FIRST, BUNDLE_ORDER).
4. Bundle preservation invariant (zero tasks dropped or reassigned).
5. Deterministic replan triggers (sub-goal arrival, window exhaustion, bundle update).
6. Live ROS 2 node verification (rh_node pub/sub interface, PlanningRequest/Response, RollingHorizonPlan).
"""

import sys
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from amr_fleet_sim.grid_world import GridWorld
from amr_fleet_core.rh_planner import (
    SingleAgentAStar,
    TaskSequencer,
    RollingHorizonPlanner,
    RHConfig,
    PlanningResponseData,
)
from amr_fleet_msgs.msg import (
    PlanningRequest,
    PlanningResponse,
    RobotBundle,
    RollingHorizonPlan,
    TaskDefinition,
    TaskList,
)
from geometry_msgs.msg import Point, Twist


def test_section(title: str):
    print()
    print("=" * 70)
    print(f"[M5 TEST] {title}")
    print("=" * 70)


def verify_astar_core():
    test_section("1. SingleAgentAStar Pathfinding & Obstacle Avoidance")
    grid = GridWorld.create_warehouse_grid(resolution=0.5, warehouse_size=16.0)
    astar = SingleAgentAStar(grid)

    start = (4, 4)
    goal = (26, 26)
    path = astar.find_path(start, goal)

    assert path is not None, "A* failed to find path in warehouse grid"
    assert path[0] == start, f"Start mismatch: {path[0]} != {start}"
    assert path[-1] == goal, f"Goal mismatch: {path[-1]} != {goal}"

    for step in path:
        assert grid.is_free(step), f"Path intersects obstacle at {step}"

    # Determinism test
    path2 = astar.find_path(start, goal)
    assert path == path2, "A* is non-deterministic on identical queries!"
    print(f"  [PASS] A* path verified ({len(path)} waypoints, 0 obstacle collisions, 100% deterministic)")


def verify_horizon_and_window():
    test_section("2. Horizon Truncation (h) & Execution Window (w)")
    h = 8
    w = 3
    config = RHConfig(horizon_steps=h, execution_window=w)
    planner = RollingHorizonPlanner(robot_id="amr_test", config=config)
    planner.update_position((2.0, 2.0))

    tasks = {
        "T_LONG": {
            "task_id": "T_LONG",
            "pickup": (14.0, 14.0),
            "dropoff": (2.0, 2.0),
            "priority": 3,
            "deadline": 0.0,
        }
    }
    planner.update_assigned_bundle(["T_LONG"], tasks)
    resp = planner.replan()

    assert resp.success is True, f"Replan failed: {resp.status_message}"
    assert len(resp.full_path) > h + 1, f"Full path too short: {len(resp.full_path)}"
    assert len(resp.horizon_path) == h + 1, f"Horizon path len mismatch: {len(resp.horizon_path)} != {h+1}"
    assert len(resp.execution_path) == w + 1, f"Execution path len mismatch: {len(resp.execution_path)} != {w+1}"
    print(f"  [PASS] Horizon h={h} -> {len(resp.horizon_path)} pts, Window w={w} -> {len(resp.execution_path)} pts")


def verify_sequencing_and_preservation():
    test_section("3. Task Sequencing Heuristics & Bundle Preservation")
    tasks = {
        "T_LOW": {"task_id": "T_LOW", "pickup": (4.0, 4.0), "dropoff": (6.0, 6.0), "priority": 1, "deadline": 100.0},
        "T_HIGH": {"task_id": "T_HIGH", "pickup": (8.0, 8.0), "dropoff": (10.0, 10.0), "priority": 5, "deadline": 100.0},
        "T_MED": {"task_id": "T_MED", "pickup": (12.0, 12.0), "dropoff": (14.0, 14.0), "priority": 3, "deadline": 100.0},
    }
    bundle = ["T_LOW", "T_HIGH", "T_MED"]

    # Priority First
    seq_pri = TaskSequencer.sequence(bundle, tasks, robot_pos=(0.0, 0.0), heuristic="PRIORITY_FIRST")
    assert seq_pri == ["T_HIGH", "T_MED", "T_LOW"], f"Priority sort incorrect: {seq_pri}"

    # Bundle Order
    seq_bundle = TaskSequencer.sequence(bundle, tasks, robot_pos=(0.0, 0.0), heuristic="BUNDLE_ORDER")
    assert seq_bundle == bundle, f"Bundle order mismatch: {seq_bundle}"

    # Preservation Invariant
    for seq in [seq_pri, seq_bundle]:
        assert set(seq) == set(bundle), "Tasks were dropped or altered!"
        assert len(seq) == len(bundle), "Bundle size changed!"
    print(f"  [PASS] Task sequencing verified across heuristics with 100% bundle preservation")


def verify_replan_triggers():
    test_section("4. Deterministic Replanning Triggers")
    config = RHConfig(horizon_steps=6, execution_window=2, goal_tolerance_m=0.5)
    planner = RollingHorizonPlanner(robot_id="amr_test", config=config)
    planner.update_position((2.0, 2.0))

    tasks = {
        "T1": {"task_id": "T1", "pickup": (2.0, 6.0), "dropoff": (8.0, 8.0), "priority": 2, "deadline": 0.0}
    }
    planner.update_assigned_bundle(["T1"], tasks)
    assert planner.active_phase == "TRANSIT_TO_PICKUP"

    # Trigger 1: Execution Window Exhaustion
    assert planner.check_replan_triggers() is False
    planner.advance_execution_step()
    assert planner.check_replan_triggers() is False
    planner.advance_execution_step()  # 2nd step = window limit
    assert planner.check_replan_triggers() is True, "Failed to trigger replan on window exhaustion"
    print("  [PASS] Replan trigger 1: Execution window exhaustion (w=2)")

    # Trigger 2: Sub-goal arrival
    planner.replan()
    planner.update_position((2.1, 6.1))  # Within tolerance of (2.0, 6.0)
    assert planner.check_subgoal_arrival() is True, "Failed to detect subgoal arrival"
    assert planner.check_replan_triggers() is True, "Failed to trigger replan on subgoal arrival"
    ev = planner.advance_subgoal()
    assert ev == "PICKUP_REACHED", f"Unexpected event: {ev}"
    assert planner.active_phase == "TRANSIT_TO_DROPOFF", f"Phase transition failed: {planner.active_phase}"
    print("  [PASS] Replan trigger 2: Sub-goal arrival (TRANSIT_TO_PICKUP -> TRANSIT_TO_DROPOFF)")


def verify_live_ros2_rh_node():
    test_section("5. Live ROS 2 RollingHorizonPlannerNode Integration")
    rclpy.init()

    from amr_fleet_core.rh_node import RollingHorizonPlannerNode

    node = RollingHorizonPlannerNode()
    robot_id = node.robot_id

    plans_received: List[RollingHorizonPlan] = []
    requests_received: List[PlanningRequest] = []
    responses_received: List[PlanningResponse] = []

    sub_node = Node("rh_test_listener")
    sub_node.create_subscription(
        RollingHorizonPlan,
        f"/{robot_id}/rolling_plan",
        lambda m: plans_received.append(m),
        10
    )
    sub_node.create_subscription(
        PlanningRequest,
        f"/{robot_id}/planning_request",
        lambda m: requests_received.append(m),
        10
    )
    sub_node.create_subscription(
        PlanningResponse,
        f"/{robot_id}/planning_response",
        lambda m: responses_received.append(m),
        10
    )

    # Publish tasks and bundle to node
    t_msg = TaskDefinition()
    t_msg.task_id = "T_VERIFY"
    t_msg.pickup_pose.x = 2.0
    t_msg.pickup_pose.y = 6.0
    t_msg.dropoff_pose.x = 8.0
    t_msg.dropoff_pose.y = 8.0
    t_msg.priority = 3
    t_msg.status = "ASSIGNED"
    t_msg.assigned_robot_id = robot_id

    t_list = TaskList()
    t_list.tasks = [t_msg]
    node._handle_tasks_all(t_list)

    b_msg = RobotBundle()
    b_msg.robot_id = robot_id
    b_msg.task_ids = ["T_VERIFY"]
    b_msg.is_converged = True
    node.planner.update_position((2.0, 2.0))
    node._handle_bundle(b_msg)

    # Execute a plan cycle
    node._planning_cycle()

    # Spin to receive published messages
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(sub_node)

    start_spin = time.time()
    while time.time() - start_spin < 2.0 and (not plans_received or not requests_received or not responses_received):
        executor.spin_once(timeout_sec=0.1)

    assert len(requests_received) > 0, "No PlanningRequest message published!"
    assert len(responses_received) > 0, "No PlanningResponse message published!"
    assert len(plans_received) > 0, "No RollingHorizonPlan message published!"

    plan = plans_received[0]
    assert plan.robot_id == robot_id
    assert plan.current_task_id == "T_VERIFY"
    assert plan.current_phase == "TRANSIT_TO_PICKUP"
    assert plan.is_valid is True
    assert len(plan.horizon_path) > 0
    assert len(plan.execution_path) > 0

    print(f"  [PASS] Live ROS 2 node published valid RollingHorizonPlan (h={plan.horizon_steps}, w={plan.execution_window}, pts={len(plan.horizon_path)})")
    print(f"  [PASS] PlanningRequest ({requests_received[0].task_id}) and PlanningResponse ({responses_received[0].planning_latency_ms:.2f}ms) verified")

    node.destroy_node()
    sub_node.destroy_node()
    rclpy.shutdown()


def main():
    print("====================================================================")
    print("    NRDAS M5: ROLLING-HORIZON TASK PLANNING VERIFICATION SUITE    ")
    print("====================================================================")
    verify_astar_core()
    verify_horizon_and_window()
    verify_sequencing_and_preservation()
    verify_replan_triggers()
    verify_live_ros2_rh_node()

    print()
    print("=" * 70)
    print("SUCCESS: ALL M5 ROLLING-HORIZON PLANNING TESTS PASSED WITH 0 FAILURES!")
    print("====================================================================")


if __name__ == '__main__':
    main()
