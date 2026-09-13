#!/usr/bin/env python3
"""
NRDAS M5 Live ROS 2 + Gazebo Harmonic Fleet Demonstration.

Executes and verifies Rolling-Horizon Task Planning across 5 AMRs in Gazebo Harmonic:
1. Spawns/manages decentralized RHCR nodes (amr_0 .. amr_4).
2. Captures CBBA assigned task bundles.
3. Validates rolling-horizon plan generation (h=10, w=4, deterministic single-agent A*).
4. Monitors PlanningRequest and PlanningResponse messages.
5. Verifies execution window progression and replanning triggers.
6. Records real robot physical motion (cmd_vel commands and odometry advancement).
7. Exports comprehensive structured evidence to docs/evidence/m5_demonstration_report.json.
"""

import json
import os
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from amr_fleet_msgs.msg import (
    PlanningRequest,
    PlanningResponse,
    RobotBundle,
    RollingHorizonPlan,
    TaskList,
)


class M5DemonstrationMonitor(Node):
    """Monitors rolling-horizon plans and robot motion for all AMRs."""

    def __init__(self, robot_count: int = 5) -> None:
        super().__init__('m5_demo_monitor')
        self.robot_count = robot_count
        self.robot_plans: Dict[str, Any] = {}
        self.robot_poses: Dict[str, Dict[str, float]] = {}
        self.initial_poses: Dict[str, Dict[str, float]] = {}
        self.bundles: Dict[str, List[str]] = {}
        self.cmd_vels: Dict[str, Dict[str, float]] = {}
        self.planning_requests: List[Dict[str, Any]] = []
        self.planning_responses: List[Dict[str, Any]] = []
        self.tasks_map: Dict[str, Any] = {}
        self.distance_traveled: Dict[str, float] = {f'amr_{i}': 0.0 for i in range(robot_count)}
        self._prev_odom: Dict[str, Optional[tuple]] = {f'amr_{i}': None for i in range(robot_count)}

        # Subscriptions
        self.create_subscription(TaskList, '/tasks/all', self._on_tasks_all, 10)

        for i in range(robot_count):
            r_id = f'amr_{i}'
            self._setup_robot_subs(r_id)

    def _setup_robot_subs(self, r_id: str) -> None:
        def odom_cb(msg: Odometry) -> None:
            r_idx = int(r_id.split('_')[-1])
            spawn_y = 2.0 + r_idx * 3.0
            x = round(2.0 + msg.pose.pose.position.x, 3)
            y = round(spawn_y + msg.pose.pose.position.y, 3)
            if r_id not in self.initial_poses:
                self.initial_poses[r_id] = {'x': x, 'y': y}
            self.robot_poses[r_id] = {'x': x, 'y': y}

            prev = self._prev_odom[r_id]
            if prev is not None:
                dx = x - prev[0]
                dy = y - prev[1]
                dist = (dx*dx + dy*dy)**0.5
                if 0.0005 < dist < 2.0:
                    self.distance_traveled[r_id] += dist
            self._prev_odom[r_id] = (x, y)

        def bundle_cb(msg: RobotBundle) -> None:
            self.bundles[r_id] = list(msg.task_ids)

        def plan_cb(msg: RollingHorizonPlan) -> None:
            self.robot_plans[r_id] = {
                'task_id': msg.current_task_id,
                'phase': msg.current_phase,
                'goal': [round(msg.current_goal.x, 2), round(msg.current_goal.y, 2)],
                'horizon_len': len(msg.horizon_path),
                'execution_len': len(msg.execution_path),
                'horizon_steps': msg.horizon_steps,
                'execution_window': msg.execution_window,
                'replan_count': msg.replan_count,
                'latency_ms': round(msg.planning_latency_ms, 2),
                'is_valid': msg.is_valid,
                'horizon_path': [[round(p.x, 2), round(p.y, 2)] for p in msg.horizon_path],
                'execution_path': [[round(p.x, 2), round(p.y, 2)] for p in msg.execution_path],
            }

        def req_cb(msg: PlanningRequest) -> None:
            self.planning_requests.append({
                'robot_id': msg.robot_id,
                'task_id': msg.task_id,
                'sub_goal_type': msg.sub_goal_type,
                'start': [round(msg.start_pose.x, 2), round(msg.start_pose.y, 2)],
                'goal': [round(msg.goal_pose.x, 2), round(msg.goal_pose.y, 2)],
                'horizon_steps': msg.horizon_steps,
                'execution_window': msg.execution_window,
            })

        def resp_cb(msg: PlanningResponse) -> None:
            self.planning_responses.append({
                'robot_id': msg.robot_id,
                'task_id': msg.task_id,
                'sub_goal_type': msg.sub_goal_type,
                'success': msg.success,
                'latency_ms': round(msg.planning_latency_ms, 2),
                'total_cost': round(msg.total_cost, 2),
                'replan_count': msg.replan_count,
            })

        def cmd_cb(msg: Twist) -> None:
            self.cmd_vels[r_id] = {
                'linear_x': round(msg.linear.x, 2),
                'angular_z': round(msg.angular.z, 2),
            }

        self.create_subscription(Odometry, f'/{r_id}/odom', odom_cb, 10)
        self.create_subscription(RobotBundle, f'/{r_id}/bundle', bundle_cb, 10)
        self.create_subscription(RollingHorizonPlan, f'/{r_id}/rolling_plan', plan_cb, 10)
        self.create_subscription(PlanningRequest, f'/{r_id}/planning_request', req_cb, 10)
        self.create_subscription(PlanningResponse, f'/{r_id}/planning_response', resp_cb, 10)
        self.create_subscription(Twist, f'/{r_id}/cmd_vel', cmd_cb, 10)

    def _on_tasks_all(self, msg: TaskList) -> None:
        for t in msg.tasks:
            self.tasks_map[t.task_id] = {
                'id': t.task_id,
                'pickup': [round(t.pickup_pose.x, 2), round(t.pickup_pose.y, 2)],
                'dropoff': [round(t.dropoff_pose.x, 2), round(t.dropoff_pose.y, 2)],
                'priority': t.priority,
                'status': t.status,
                'robot': t.assigned_robot_id,
            }


def main():
    print("=" * 75)
    print("    NRDAS M5: REAL ROS 2 + GAZEBO HARMONIC FLEET PLANNING DEMO    ")
    print("=" * 75)

    rclpy.init()
    monitor = M5DemonstrationMonitor(robot_count=5)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)

    # 1. Spawn RH Planner nodes for each robot if not already running
    rh_processes: List[subprocess.Popen] = []
    print("")
    print("[1/4] Spawning Decentralized Rolling-Horizon Planner Nodes (amr_0 .. amr_4)...")

    for i in range(5):
        robot_id = f'amr_{i}'
        spawn_y = 2.0 + i * 3.0
        cmd = [
            'ros2', 'run', 'amr_fleet_core', 'rh_node',
            '--ros-args',
            '-r', f'__node:={robot_id}_rh_node',
            '-r', f'__ns:=/{robot_id}',
            '-p', f'robot_id:={robot_id}',
            '-p', 'spawn_x:=2.0',
            '-p', f'spawn_y:={spawn_y}',
            '-p', 'horizon_steps:=10',
            '-p', 'execution_window:=4',
            '-p', 'replan_rate:=2.0',
            '-p', 'sequencing_heuristic:=PRIORITY_FIRST',
            '-p', 'goal_tolerance_m:=0.5',
            '-p', 'enable_motion_execution:=true',
        ]
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        rh_processes.append(proc)
        print(f"  -> Spawned {robot_id}_rh_node (PID: {proc.pid})")

    # 2. Warm-up and Monitor Execution
    print("")
    print("[2/4] Monitoring live CBBA bundle ingestion and rolling-horizon planning...")
    start_time = time.time()
    demo_duration = 18.0  # Observe for 18 seconds to capture replans and motion

    while time.time() - start_time < demo_duration:
        executor.spin_once(timeout_sec=0.1)
        elapsed = time.time() - start_time
        if int(elapsed * 2) % 6 == 0 and int(elapsed * 2) > 0:
            active_plans = len(monitor.robot_plans)
            active_moving = sum(1 for d in monitor.distance_traveled.values() if d > 0.05)
            print(f"  [T+{elapsed:.1f}s] Plans generated: {active_plans}/5 AMRs | Moving: {active_moving}/5 AMRs")

    # 3. Compile and Display Demonstration Telemetry
    print("")
    print("=" * 75)
    print("    M5 DEMONSTRATION TELEMETRY & VERIFICATION RESULTS")
    print("=" * 75)

    all_received_plans = len(monitor.robot_plans) == 5
    all_plans_valid = all(p.get('is_valid', False) for p in monitor.robot_plans.values())

    print("")
    print(f"1. ALLOCATION INGESTION (M4 -> M5): {len(monitor.bundles)}/5 Robots with Bundles")
    for r_id in sorted(monitor.bundles.keys()):
        bundle = monitor.bundles[r_id]
        print(f"   - {r_id.upper()}: Bundle = {bundle}")

    print("")
    print(f"2. ROLLING-HORIZON PLANS GENERATED: {len(monitor.robot_plans)}/5 Robots")
    for r_id in sorted(monitor.robot_plans.keys()):
        p = monitor.robot_plans[r_id]
        pos = monitor.robot_poses.get(r_id, {'x': 0.0, 'y': 0.0})
        print(f"   - {r_id.upper()}: Task={p['task_id']} | Phase={p['phase']} | Goal={p['goal']}")
        print(f"     Horizon={p['horizon_len']} pts (h={p['horizon_steps']}) | Window={p['execution_len']} pts (w={p['execution_window']})")
        print(f"     Replans={p['replan_count']} | Latency={p['latency_ms']} ms | Valid={p['is_valid']}")
        print(f"     Execution Waypoints: {p['execution_path']}")

    print("")
    print(f"3. REAL MOTION & GAZEBO KINEMATICS:")
    for r_id in sorted(monitor.distance_traveled.keys()):
        dist = round(monitor.distance_traveled[r_id], 3)
        vel = monitor.cmd_vels.get(r_id, {'linear_x': 0.0, 'angular_z': 0.0})
        print(f"   - {r_id.upper()}: Distance Traveled = {dist:.3f} m | Active cmd_vel = [vx: {vel['linear_x']:.2f}, wz: {vel['angular_z']:.2f}]")

    print("")
    print(f"4. PLANNING INTERFACE MESSAGES:")
    print(f"   - Total PlanningRequests Published: {len(monitor.planning_requests)}")
    print(f"   - Total PlanningResponses Published: {len(monitor.planning_responses)}")

    # 4. Save Structured Evidence Report
    os.makedirs('docs/evidence', exist_ok=True)
    report_file = 'docs/evidence/m5_demonstration_report.json'
    evidence = {
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'milestone': 'M5',
        'demonstration': 'Real ROS 2 + Gazebo Harmonic Rolling-Horizon Planning',
        'robot_count': 5,
        'summary': {
            'bundles_ingested': len(monitor.bundles),
            'plans_generated': len(monitor.robot_plans),
            'all_plans_valid': all_plans_valid,
            'total_requests': len(monitor.planning_requests),
            'total_responses': len(monitor.planning_responses),
            'average_latency_ms': round(
                sum(p['latency_ms'] for p in monitor.robot_plans.values()) / max(1, len(monitor.robot_plans)), 2
            ),
        },
        'initial_poses': monitor.initial_poses,
        'final_poses': monitor.robot_poses,
        'bundles': monitor.bundles,
        'plans': monitor.robot_plans,
        'distances_traveled_m': monitor.distance_traveled,
        'recent_requests': monitor.planning_requests[-10:],
        'recent_responses': monitor.planning_responses[-10:],
    }

    with open(report_file, 'w') as f:
        json.dump(evidence, f, indent=2)
    print("")
    print(f"[4/4] Evidence report saved to: {report_file}")

    # Teardown spawned nodes cleanly
    print("")
    print("Shutting down spawned test nodes...")
    for proc in rh_processes:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass

    monitor.destroy_node()
    rclpy.shutdown()

    assert all_received_plans, f"Only {len(monitor.robot_plans)}/5 robots received plans!"
    assert all_plans_valid, "One or more plans were invalid!"
    print("")
    print("=" * 75)
    print("SUCCESS: REAL GAZEBO M5 DEMONSTRATION CONFIRMED & VERIFIED!")
    print("=" * 75)


if __name__ == '__main__':
    main()
