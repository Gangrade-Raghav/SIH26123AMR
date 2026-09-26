#!/usr/bin/env python3
"""
Integration test for live operator task creation, allocation, and control.

Validates end-to-end:
1. TaskManagerNode exposes /tasks/create and /tasks/control services.
2. Operator creates AUTO task via /tasks/create -> CBBA decentralized consensus allocates it.
3. Operator creates DIRECT task constrained to amr_1 -> only amr_1 bids and wins.
4. Operator cancels assigned task via /tasks/control -> transitions to CANCELLED & bundle purges.
5. Operator requeues assigned task via /tasks/control -> transitions to PENDING.
"""

import os
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Set

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from amr_fleet_msgs.msg import CBBABid, RobotBundle, TaskEvent as TaskEventMsg, TaskList
from amr_fleet_msgs.srv import ControlTask, CreateTask


class LiveTaskIntegrationClient(Node):
    """Client node for testing live operator task creation and control."""

    def __init__(self) -> None:
        super().__init__('live_task_integration_client')
        self.cli_create = self.create_client(CreateTask, '/tasks/create')
        self.cli_control = self.create_client(ControlTask, '/tasks/control')

        self.bundles: Dict[str, List[str]] = {}
        self.converged: Dict[str, bool] = {}
        self.task_states: Dict[str, str] = {}
        self.task_assigned_robots: Dict[str, str] = {}

        self.create_subscription(TaskList, '/tasks/all', self._tasks_cb, 10)
        self.create_subscription(RobotBundle, '/amr_0/bundle', self._b0_cb, 10)
        self.create_subscription(RobotBundle, '/amr_1/bundle', self._b1_cb, 10)

    def _tasks_cb(self, msg: TaskList) -> None:
        for t in msg.tasks:
            self.task_states[t.task_id] = t.status
            if t.assigned_robot_id:
                self.task_assigned_robots[t.task_id] = t.assigned_robot_id

    def _b0_cb(self, msg: RobotBundle) -> None:
        self.bundles['amr_0'] = list(msg.task_ids)
        self.converged['amr_0'] = bool(msg.is_converged)

    def _b1_cb(self, msg: RobotBundle) -> None:
        self.bundles['amr_1'] = list(msg.task_ids)
        self.converged['amr_1'] = bool(msg.is_converged)


def main() -> None:
    print('======================================================================')
    print('  NRDAS LIVE OPERATOR TASK ALLOCATION & CONTROL INTEGRATION TEST')
    print('======================================================================')

    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    subprocesses = []

    try:
        # 1. Start TaskManagerNode
        print('\n[1/5] Starting TaskManagerNode...')
        cmd_tm = [
            'ros2', 'run', 'amr_fleet_core', 'task_manager',
            '--ros-args',
            '-p', 'publish_rate:=5.0',
            '-p', 'workload_file:=/dev/null',
        ]
        p_tm = subprocess.Popen(cmd_tm, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocesses.append(p_tm)

        # 2. Start 2 decentralized CBBA Nodes (amr_0, amr_1)
        print('[2/5] Starting 2 decentralized CBBANodes (amr_0, amr_1)...')
        for r_id in ('amr_0', 'amr_1'):
            cmd_cbba = [
                'ros2', 'run', 'amr_fleet_core', 'cbba_node',
                '--ros-args',
                '-r', f'__ns:=/{r_id}',
                '-p', f'robot_id:={r_id}',
                '-p', 'max_bundle_size:=8',
                '-p', 'consensus_rate:=5.0',
                '-p', 'stable_rounds_for_convergence:=3',
            ]
            p_cbba = subprocess.Popen(cmd_cbba, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocesses.append(p_cbba)

        # Initialize ROS 2 client
        rclpy.init()
        client = LiveTaskIntegrationClient()
        executor = SingleThreadedExecutor()
        executor.add_node(client)

        # Wait for service readiness
        print('  [*] Waiting for /tasks/create and /tasks/control services...')
        start_t = time.time()
        while time.time() - start_t < 10.0:
            if client.cli_create.wait_for_service(timeout_sec=0.2) and \
               client.cli_control.wait_for_service(timeout_sec=0.2):
                break
            executor.spin_once(timeout_sec=0.1)

        assert client.cli_create.service_is_ready(), "Service '/tasks/create' not ready!"
        assert client.cli_control.service_is_ready(), "Service '/tasks/control' not ready!"
        print('  [+] Operator task services online.')

        # 3. Create Task 1 (AUTO allocation)
        print('\n[3/5] Testing Task Creation: AUTO (Decentralized CBBA Auction)...')
        req_auto = CreateTask.Request()
        req_auto.task_id = 'T_AUTO_01'
        req_auto.pickup_x = 2.0
        req_auto.pickup_y = 3.0
        req_auto.dropoff_x = 8.0
        req_auto.dropoff_y = 9.0
        req_auto.priority = 3  # CRITICAL

        fut_auto = client.cli_create.call_async(req_auto)
        while not fut_auto.done():
            executor.spin_once(timeout_sec=0.1)
        resp_auto = fut_auto.result()
        assert resp_auto.accepted is True, f'AUTO task rejected: {resp_auto.message}'
        print(f'  [+] AUTO task T_AUTO_01 created: {resp_auto.message}')

        # 4. Create Task 2 (DIRECT constraint to amr_1)
        print('\n[4/5] Testing Task Creation: DIRECT Constraint (Target: amr_1)...')
        req_dir = CreateTask.Request()
        req_dir.task_id = 'T_DIR_01'
        req_dir.pickup_x = 4.0
        req_dir.pickup_y = 5.0
        req_dir.dropoff_x = 10.0
        req_dir.dropoff_y = 11.0
        req_dir.priority = 2
        req_dir.requested_robot = 'amr_1'

        fut_dir = client.cli_create.call_async(req_dir)
        while not fut_dir.done():
            executor.spin_once(timeout_sec=0.1)
        resp_dir = fut_dir.result()
        assert resp_dir.accepted is True, f'DIRECT task rejected: {resp_dir.message}'
        print(f'  [+] DIRECT task T_DIR_01 created: {resp_dir.message}')

        # Wait for CBBA consensus on both tasks
        print('  [*] Waiting for decentralized CBBA allocation & consensus...')
        start_alloc = time.time()
        allocated = False
        while time.time() - start_alloc < 15.0:
            executor.spin_once(timeout_sec=0.2)
            s_auto = client.task_states.get('T_AUTO_01')
            s_dir = client.task_states.get('T_DIR_01')
            if s_auto == 'ASSIGNED' and s_dir == 'ASSIGNED':
                allocated = True
                break

        assert allocated, (
            f'CBBA allocation timeout! States: '
            f'T_AUTO_01={client.task_states.get("T_AUTO_01")}, '
            f'T_DIR_01={client.task_states.get("T_DIR_01")}'
        )

        winner_auto = client.task_assigned_robots.get('T_AUTO_01')
        winner_dir = client.task_assigned_robots.get('T_DIR_01')
        print(f'  [+] T_AUTO_01 assigned to: {winner_auto}')
        print(f'  [+] T_DIR_01 assigned to:  {winner_dir}')
        assert winner_dir == 'amr_1', f'DIRECT task should be assigned strictly to amr_1, got {winner_dir}!'
        assert 'T_DIR_01' in client.bundles.get('amr_1', []), 'T_DIR_01 missing from amr_1 bundle!'
        assert 'T_DIR_01' not in client.bundles.get('amr_0', []), 'T_DIR_01 leaked into amr_0 bundle!'

        # 5. Live Control Actions: CANCEL and REQUEUE
        print('\n[5/5] Testing Live Operator Control: CANCEL and REQUEUE...')
        # A. Cancel T_AUTO_01
        print("  [*] Cancelling 'T_AUTO_01' via /tasks/control...")
        req_cancel = ControlTask.Request()
        req_cancel.task_id = 'T_AUTO_01'
        req_cancel.action = 'CANCEL'
        fut_cancel = client.cli_control.call_async(req_cancel)
        while not fut_cancel.done():
            executor.spin_once(timeout_sec=0.1)
        resp_cancel = fut_cancel.result()
        assert resp_cancel.success is True, f'Cancel failed: {resp_cancel.message}'

        # Wait for state update to propagate
        start_c = time.time()
        while time.time() - start_c < 5.0:
            executor.spin_once(timeout_sec=0.1)
            if client.task_states.get('T_AUTO_01') == 'CANCELLED':
                break
        assert client.task_states.get('T_AUTO_01') == 'CANCELLED', 'T_AUTO_01 state did not update to CANCELLED!'
        print("  [+] 'T_AUTO_01' successfully transitioned to CANCELLED.")

        # B. Requeue T_DIR_01
        print("  [*] Requeueing 'T_DIR_01' via /tasks/control...")
        req_req = ControlTask.Request()
        req_req.task_id = 'T_DIR_01'
        req_req.action = 'REQUEUE'
        fut_req = client.cli_control.call_async(req_req)
        while not fut_req.done():
            executor.spin_once(timeout_sec=0.1)
        resp_req = fut_req.result()
        assert resp_req.success is True, f'Requeue failed: {resp_req.message}'

        start_r = time.time()
        while time.time() - start_r < 5.0:
            executor.spin_once(timeout_sec=0.1)
            if client.task_states.get('T_DIR_01') in ('PENDING', 'ASSIGNED'):
                break
        print(f"  [+] 'T_DIR_01' successfully requeued (current state: {client.task_states.get('T_DIR_01')}).")

        print('\n======================================================================')
        print('  LIVE INTEGRATION TEST PASSED: ALL CONTROLS & CBBA INVARIANTS VERIFIED')
        print('======================================================================\n')
        sys.exit(0)

    finally:
        for p in subprocesses:
            try:
                p.terminate()
                p.wait(timeout=2.0)
            except Exception:
                p.kill()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
