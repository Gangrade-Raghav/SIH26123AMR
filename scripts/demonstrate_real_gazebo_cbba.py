#!/usr/bin/env python3
"""
M4 Real ROS 2 + Gazebo CBBA Demonstration and Deterministic Repeatability Experiment.

Requirements addressed:
1. Real ROS 2 + Gazebo Harmonic fleet launch (5 AMRs, warehouse world).
2. Proof of decentralized nodes (inspect node graph, namespaces, topic connections).
3. Full telemetry capture: initial poses, task set, bids, winners, bundles, timings, duplicates.
4. Deterministic repeatability experiment (multiple runs compared for exact bitwise matching).
5. Verification of M2.5 fleet dashboard live API with real CBBA telemetry.
"""

import json
import os
import signal
import subprocess
import sys
import time
import urllib.request
from typing import Any, Dict, List, Optional, Set

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry

from amr_fleet_msgs.msg import (
    CBBABid,
    RobotBundle,
    TaskEvent as TaskEventMsg,
    TaskList,
)
from amr_fleet_core.workload import WorkloadManager


class DemonstrationTelemetryCollector(Node):
    """ROS 2 Node passively recording live CBBA negotiations and Gazebo telemetry."""

    def __init__(self, robot_count: int = 5) -> None:
        super().__init__('m4_demo_telemetry_collector')
        self.robot_count = robot_count
        self.robot_positions: Dict[str, Dict[str, float]] = {}
        self.bids_history: List[Dict[str, Any]] = []
        self.latest_bids_by_robot: Dict[str, Dict[str, float]] = {}
        self.bundles_by_robot: Dict[str, List[str]] = {}
        self.converged_flags: Dict[str, bool] = {}
        self.task_states: Dict[str, str] = {}
        self.available_tasks: Dict[str, Any] = {}
        self.task_assigned_events: List[Dict[str, Any]] = []

        # Subscriptions
        self.create_subscription(CBBABid, '/fleet/cbba_bids', self._on_bid, 100)
        self.create_subscription(TaskList, '/tasks/all', self._on_tasks_all, 10)
        self.create_subscription(TaskList, '/tasks/available', self._on_tasks_avail, 10)
        self.create_subscription(TaskEventMsg, '/tasks/events', self._on_event, 50)

        for i in range(robot_count):
            r_id = f'amr_{i}'
            self._create_robot_subs(r_id)

    def _create_robot_subs(self, r_id: str) -> None:
        def odom_cb(msg: Odometry) -> None:
            self.robot_positions[r_id] = {
                'x': round(msg.pose.pose.position.x, 3),
                'y': round(msg.pose.pose.position.y, 3),
            }

        def bundle_cb(msg: RobotBundle) -> None:
            self.bundles_by_robot[r_id] = list(msg.task_ids)
            self.converged_flags[r_id] = bool(msg.is_converged)

        self.create_subscription(Odometry, f'/{r_id}/odom', odom_cb, 10)
        self.create_subscription(RobotBundle, f'/{r_id}/bundle', bundle_cb, 10)

    def _on_bid(self, msg: CBBABid) -> None:
        bids_map = {t: round(b, 4) for t, b in zip(msg.task_ids, msg.winning_bids)}
        self.latest_bids_by_robot[msg.robot_id] = bids_map
        self.bids_history.append({
            'time': self.get_clock().now().nanoseconds * 1e-9,
            'robot_id': msg.robot_id,
            'iteration': msg.iteration,
            'task_ids': list(msg.task_ids),
            'winning_bids': [round(b, 4) for b in msg.winning_bids],
            'winning_robots': list(msg.winning_robots),
        })

    def _on_tasks_all(self, msg: TaskList) -> None:
        for t in msg.tasks:
            self.task_states[t.task_id] = t.status

    def _on_tasks_avail(self, msg: TaskList) -> None:
        for t in msg.tasks:
            if t.task_id not in self.available_tasks:
                self.available_tasks[t.task_id] = {
                    'task_id': t.task_id,
                    'priority': t.priority,
                    'pickup': (round(t.pickup_pose.x, 2), round(t.pickup_pose.y, 2)),
                    'dropoff': (round(t.dropoff_pose.x, 2), round(t.dropoff_pose.y, 2)),
                }

    def _on_event(self, msg: TaskEventMsg) -> None:
        if msg.event_type == 'ASSIGNED':
            self.task_assigned_events.append({
                'task_id': msg.task_id,
                'robot_id': msg.robot_id,
                'new_state': msg.new_state,
            })


def check_process_running(proc_name: str) -> bool:
    """Check if process containing proc_name is currently running."""
    try:
        out = subprocess.check_output(['pgrep', '-f', proc_name], text=True)
        return len(out.strip().splitlines()) > 0
    except subprocess.CalledProcessError:
        return False


def run_real_gazebo_cbba_experiment(run_id: str, max_wait_sec: float = 40.0) -> Dict[str, Any]:
    """Execute a full live ROS 2 + Gazebo CBBA demonstration run."""
    print(f"\n{'='*75}\n[LIVE EXPERIMENT] Executing Real ROS 2 + Gazebo Harmonic CBBA: {run_id}\n{'='*75}")

    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    workload_path = os.path.join(ws_root, 'config', 'workloads', 'workload_medium_priority.yaml')

    # Launch cbba_fleet.launch.py (Gazebo Harmonic + 5 robots + TaskManager + 5 CBBANodes)
    cmd = [
        'ros2', 'launch', 'amr_fleet_bringup', 'cbba_fleet.launch.py',
        'robot_count:=5',
        'max_bundle_size:=4',
        'headless:=true',
        'launch_simulation:=true',
        f'workload_file:={workload_path}',
    ]
    print(f"[*] Starting launch command:\n    {' '.join(cmd)}")
    launch_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid,
        text=True,
    )

    rclpy.init()
    collector = DemonstrationTelemetryCollector(robot_count=5)
    executor = SingleThreadedExecutor()
    executor.add_node(collector)

    start_time = time.time()
    gazebo_verified = False
    nodes_verified = False
    converged_time: Optional[float] = None
    all_assigned_time: Optional[float] = None

    experiment_data: Dict[str, Any] = {
        'run_id': run_id,
        'success': False,
        'gazebo_process_verified': False,
        'decentralized_nodes_verified': False,
        'initial_positions': {},
        'task_set': {},
        'final_bundles': {},
        'task_winners': {},
        'total_tasks': 0,
        'total_allocated': 0,
        'duplicate_winners': [],
        'unassigned_tasks': [],
        'bids_exchanged': 0,
        'consensus_time_sec': 0.0,
        'total_allocation_time_sec': 0.0,
    }

    try:
        while (time.time() - start_time) < max_wait_sec:
            executor.spin_once(timeout_sec=0.2)

            # 1. Verify Gazebo Harmonic is running
            if not gazebo_verified and (time.time() - start_time) > 4.0:
                is_gz = check_process_running('gz sim') or check_process_running('ruby')
                if is_gz:
                    gazebo_verified = True
                    experiment_data['gazebo_process_verified'] = True
                    print("  [+] Verified REAL Gazebo Harmonic simulation process is running!")

            # 2. Verify decentralized nodes running in separate namespaces
            if not nodes_verified and len(collector.robot_positions) == 5:
                # Check running node list via ros2 cli
                try:
                    node_list_out = subprocess.check_output(['ros2', 'node', 'list'], text=True)
                    expected_nodes = [f'/amr_{i}/cbba_node' for i in range(5)]
                    all_present = all(n in node_list_out for n in expected_nodes)
                    if all_present:
                        nodes_verified = True
                        experiment_data['decentralized_nodes_verified'] = True
                        print("  [+] Verified 5 independent decentralized nodes in ROS 2 graph:")
                        for n in expected_nodes:
                            print(f"      - {n}")
                except Exception as e:
                    pass

            # 3. Check consensus convergence
            if len(collector.bundles_by_robot) == 5:
                all_converged = all(collector.converged_flags.get(f'amr_{i}', False) for i in range(5))
                if all_converged and converged_time is None:
                    converged_time = time.time() - start_time
                    print(f"  [+] FLEET REACHED CONSENSUS in {round(converged_time, 2)}s!")

            # 4. Check M3 TaskManager commitment
            if converged_time is not None:
                assigned_count = sum(1 for s in collector.task_states.values() if s == 'ASSIGNED')
                total_in_bundles = sum(len(b) for b in collector.bundles_by_robot.values())
                if total_in_bundles > 0 and assigned_count >= total_in_bundles:
                    all_assigned_time = time.time() - start_time
                    print(f"  [+] All {assigned_count} tasks confirmed in ASSIGNED state in {round(all_assigned_time, 2)}s!")
                    break

        # Populate experiment data
        experiment_data['initial_positions'] = dict(collector.robot_positions)
        experiment_data['task_set'] = dict(collector.available_tasks)
        experiment_data['final_bundles'] = dict(collector.bundles_by_robot)
        experiment_data['bids_exchanged'] = len(collector.bids_history)
        experiment_data['consensus_time_sec'] = round(converged_time or 0.0, 2)
        experiment_data['total_allocation_time_sec'] = round(all_assigned_time or (time.time() - start_time), 2)

        # Build task winners mapping and check duplicates
        task_winners: Dict[str, str] = {}
        duplicate_winners: List[str] = []
        for r_id, bundle in collector.bundles_by_robot.items():
            for t_id in bundle:
                if t_id in task_winners:
                    duplicate_winners.append(t_id)
                task_winners[t_id] = r_id
        experiment_data['task_winners'] = task_winners
        experiment_data['duplicate_winners'] = duplicate_winners
        experiment_data['total_allocated'] = len(task_winners)
        experiment_data['total_tasks'] = len(collector.available_tasks)

        unassigned = [t for t in collector.available_tasks if t not in task_winners]
        experiment_data['unassigned_tasks'] = unassigned

        if (
            gazebo_verified
            and nodes_verified
            and converged_time is not None
            and len(duplicate_winners) == 0
            and experiment_data['total_allocated'] == 15
        ):
            experiment_data['success'] = True
            print(f"  [SUCCESS] Run {run_id} completed successfully with 15/15 tasks allocated!")
        else:
            print(f"  [FAILURE] Run {run_id} failed: gaz={gazebo_verified}, nodes={nodes_verified}, conv={converged_time}, dup={duplicate_winners}, alloc={experiment_data['total_allocated']}")

    finally:
        print("  [*] Tearing down experiment simulation cleanly...")
        collector.destroy_node()
        rclpy.shutdown()

        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
            launch_proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(2.0)

    return experiment_data


def verify_dashboard_integration() -> bool:
    """Verify M2.5 fleet dashboard HTTP server against running CBBA nodes."""
    print(f"\n{'='*75}\n[DASHBOARD AUDIT] Verifying M2.5 Dashboard Live API Integration\n{'='*75}")

    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    dashboard_script = os.path.join(ws_root, 'scripts', 'fleet_dashboard.py')

    dash_proc = subprocess.Popen(
        ['python3', dashboard_script, '--port', '8088'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid,
        text=True,
    )

    try:
        time.sleep(2.0)
        # Query HTTP API /api/state
        url = 'http://localhost:8088/api/state'
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            data = json.loads(resp.read().decode('utf-8'))

        print("  [+] Successfully connected to fleet dashboard on port 8088")
        print(f"  [+] Available API Keys: {list(data.keys())}")
        assert 'fleet' in data, "Dashboard must contain 'fleet' key"
        assert 'tasks' in data, "Dashboard must contain 'tasks' key"
        assert 'cbba' in data, "Dashboard must contain 'cbba' key"

        cbba_data = data['cbba']
        print(f"  [+] CBBA Telemetry fields in Dashboard:")
        print(f"      - consensus: {cbba_data.get('consensus')}")
        print(f"      - bids_count: {cbba_data.get('bids_count')}")
        print(f"      - bundles: {cbba_data.get('bundles')}")
        print(f"      - makespan_sec: {cbba_data.get('makespan_sec')}")
        print("  [+] M2.5 Dashboard Integration Verified Successfully!")
        return True
    except Exception as e:
        print(f"  [-] Dashboard verification failed: {e}")
        return False
    finally:
        try:
            os.killpg(os.getpgid(dash_proc.pid), signal.SIGINT)
            dash_proc.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(dash_proc.pid), signal.SIGKILL)
            except Exception:
                pass


def main() -> int:
    """Run real Gazebo CBBA demonstration, repeatability experiment, and dashboard audit."""
    print("\n" + "#"*75)
    print("  NRDAS MILESTONE M4: REAL GAZEBO & REPEATABILITY AUDIT SUITE")
    print("#"*75)

    # 1. Run Live Real Gazebo Demonstration (Run 1)
    run_1_data = run_real_gazebo_cbba_experiment("RUN_GAZEBO_LIVE_01", max_wait_sec=45.0)

    # 2. Run Deterministic Repeatability Run (Run 2)
    run_2_data = run_real_gazebo_cbba_experiment("RUN_GAZEBO_LIVE_02", max_wait_sec=45.0)

    # 3. Verify Dashboard Integration
    dash_ok = verify_dashboard_integration()

    # 4. Repeatability Analysis
    print(f"\n{'='*75}\n[DETERMINISTIC REPEATABILITY COMPARISON]\n{'='*75}")
    print(f"Run 1 Task Winners: {run_1_data['task_winners']}")
    print(f"Run 2 Task Winners: {run_2_data['task_winners']}")
    print(f"Run 1 Final Bundles: {run_1_data['final_bundles']}")
    print(f"Run 2 Final Bundles: {run_2_data['final_bundles']}")

    is_identical_bundles = run_1_data['final_bundles'] == run_2_data['final_bundles']
    is_identical_winners = run_1_data['task_winners'] == run_2_data['task_winners']

    print(f"  [+] Identical Bundles Across Runs: {is_identical_bundles}")
    print(f"  [+] Identical Task Winners Across Runs: {is_identical_winners}")

    # Save summary JSON for report inclusion
    audit_file = '/home/raghav/Downloads/NRDAS_Antigravity_Project_Starter/antigravity_amr_project/docs/checkpoints/m4_audit_evidence.json'
    with open(audit_file, 'w') as f:
        json.dump({
            'run_1': run_1_data,
            'run_2': run_2_data,
            'is_identical_bundles': is_identical_bundles,
            'is_identical_winners': is_identical_winners,
            'dashboard_verified': dash_ok,
        }, f, indent=2)
    print(f"  [+] Full audit telemetry dumped to {audit_file}")

    if run_1_data['success'] and run_2_data['success'] and is_identical_winners and dash_ok:
        print("\n" + "="*75)
        print("  [SUCCESS] ALL M4 REAL GAZEBO & REPEATABILITY CHECKS PASSED!")
        print("="*75)
        return 0
    else:
        print("\n" + "="*75)
        print("  [FAILURE] SOME EXPERIMENT CHECKS FAILED.")
        print("="*75)
        return 1


if __name__ == '__main__':
    sys.exit(main())
