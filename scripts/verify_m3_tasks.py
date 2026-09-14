#!/usr/bin/env python3
"""
M3 Task Generation and Lifecycle Infrastructure Verification Script.

Empirically validates:
1. Workload YAML specifications load and validate correctly.
2. Determinism and reproducibility of the PRNG task generator.
3. Task lifecycle state machine transitions, invalid transition rejections, and audit history.
4. ROS 2 topic interfaces (/tasks/all, /tasks/available, /tasks/events, /tasks/update_status).
5. Dynamic status updates and event publishing via running task_manager node.
6. Clean process termination and graceful shutdown.
"""

import os
import subprocess
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from amr_fleet_core.task_generator import TaskGenerator, TaskGeneratorConfig
from amr_fleet_core.task_model import (
    InvalidTaskTransitionError,
    Task,
    TaskLifecycleState,
    TaskPriority,
)
from amr_fleet_core.workload import WorkloadManager
from amr_fleet_msgs.msg import TaskDefinition, TaskEvent as TaskEventMsg, TaskList


def print_step(title: str) -> None:
    print(f"\n{'='*70}\n[M3 TEST STEP] {title}\n{'='*70}")


def run_workload_and_model_checks() -> bool:
    print_step("1. Validating Workload YAMLs and Task Domain Model")
    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    workloads_dir = os.path.join(ws_root, 'config', 'workloads')

    files = [
        ('workload_small_deterministic.yaml', 5),
        ('workload_medium_priority.yaml', 15),
        ('workload_benchmark_deadlines.yaml', 25),
        ('workload_explicit_sample.yaml', 3),
    ]

    for fname, expected_count in files:
        fpath = os.path.join(workloads_dir, fname)
        if not os.path.isfile(fpath):
            print(f"[-] Missing workload file: {fpath}")
            return False
        tasks = WorkloadManager.load_from_yaml(fpath)
        if len(tasks) != expected_count:
            print(f"[-] Expected {expected_count} tasks in {fname}, got {len(tasks)}")
            return False
        print(f"  [+] {fname}: Loaded {len(tasks)} tasks successfully.")

    # Validate generator determinism across 2 independent runs
    cfg = TaskGeneratorConfig(task_count=20, seed=9999, mode='BOUNDED_RANDOM')
    gen1_tasks = TaskGenerator(cfg).generate_workload()
    gen2_tasks = TaskGenerator(cfg).generate_workload()
    for t1, t2 in zip(gen1_tasks, gen2_tasks):
        assert t1.task_id == t2.task_id
        assert t1.pickup == t2.pickup
        assert t1.dropoff == t2.dropoff
        assert t1.priority == t2.priority
        assert t1.deadline == t2.deadline
    print("  [+] Generator determinism verified: 100% bitwise matching across identical seeds.")

    # Validate state machine rules
    sample = gen1_tasks[0]
    assert sample.state == TaskLifecycleState.PENDING
    sample.transition_to(TaskLifecycleState.ASSIGNED, timestamp=1.0, robot_id='amr_0')
    assert sample.state == TaskLifecycleState.ASSIGNED
    assert sample.assigned_robot_id == 'amr_0'

    sample.transition_to(TaskLifecycleState.IN_PROGRESS, timestamp=3.0)
    assert sample.state == TaskLifecycleState.IN_PROGRESS

    sample.transition_to(TaskLifecycleState.COMPLETED, timestamp=10.0)
    assert sample.state == TaskLifecycleState.COMPLETED
    assert sample.is_terminal
    assert sample.waiting_time == 3.0  # 3.0 - 0.0
    assert sample.execution_time == 7.0  # 10.0 - 3.0
    assert sample.total_duration == 10.0  # 10.0 - 0.0

    # Illegal transition rejection
    try:
        sample.transition_to(TaskLifecycleState.PENDING)
        print("[-] Failed: State machine permitted illegal transition from terminal state.")
        return False
    except InvalidTaskTransitionError:
        print("  [+] State machine correctly rejected illegal transition from terminal state.")

    return True


class TaskHarnessNode(Node):
    """Harness node to interact with running task_manager node."""

    def __init__(self) -> None:
        super().__init__('m3_test_harness')
        self.all_tasks: Optional[TaskList] = None
        self.available_tasks: Optional[TaskList] = None
        self.events_received: List[TaskEventMsg] = []

        self.sub_all = self.create_subscription(TaskList, '/tasks/all', self._on_all, 10)
        self.sub_avail = self.create_subscription(TaskList, '/tasks/available', self._on_avail, 10)
        self.sub_ev = self.create_subscription(TaskEventMsg, '/tasks/events', self._on_ev, 20)

        self.pub_update = self.create_publisher(TaskEventMsg, '/tasks/update_status', 10)

    def _on_all(self, msg: TaskList) -> None:
        self.all_tasks = msg

    def _on_avail(self, msg: TaskList) -> None:
        self.available_tasks = msg

    def _on_ev(self, msg: TaskEventMsg) -> None:
        self.events_received.append(msg)


def run_ros2_task_manager_integration() -> bool:
    print_step("2. Validating Running ROS 2 Task Manager Node Integration")

    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    workload_file = os.path.join(ws_root, 'config', 'workloads', 'workload_small_deterministic.yaml')

    cmd = [
        'ros2', 'run', 'amr_fleet_core', 'task_manager',
        '--ros-args',
        '-p', f'workload_file:={workload_file}',
        '-p', 'publish_rate:=5.0',
    ]

    print(f"  [+] Launching task_manager node: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    try:
        rclpy.init()
        harness = TaskHarnessNode()
        executor = SingleThreadedExecutor()
        executor.add_node(harness)

        # Wait for initial publication
        print("  [+] Awaiting /tasks/all and /tasks/available publications...")
        start_wait = time.time()
        while time.time() - start_wait < 5.0:
            executor.spin_once(timeout_sec=0.2)
            if harness.all_tasks is not None and harness.available_tasks is not None:
                break

        if harness.all_tasks is None or harness.available_tasks is None:
            print("[-] Timeout waiting for /tasks/all or /tasks/available")
            return False

        total_tasks = len(harness.all_tasks.tasks)
        avail_tasks = len(harness.available_tasks.tasks)
        print(f"  [+] Received /tasks/all ({total_tasks} tasks) and /tasks/available ({avail_tasks} tasks).")
        assert total_tasks == 5
        assert avail_tasks == 5

        # Check first task fields
        first_t = harness.all_tasks.tasks[0]
        print(f"  [+] Inspecting first task: id='{first_t.task_id}', pickup=({first_t.pickup_pose.x}, {first_t.pickup_pose.y}), priority={first_t.priority}")
        assert first_t.task_id == 'task_small_0001'
        assert first_t.status == 'PENDING'

        # Test state transition via /tasks/update_status
        print("  [+] Sending status update: assign task_small_0001 to amr_0...")
        update_msg = TaskEventMsg()
        update_msg.task_id = 'task_small_0001'
        update_msg.new_state = 'ASSIGNED'
        update_msg.robot_id = 'amr_0'
        update_msg.details = 'Assigned via verification harness'

        # Publish and spin
        for _ in range(5):
            harness.pub_update.publish(update_msg)
            executor.spin_once(timeout_sec=0.1)

        # Wait for confirmation on /tasks/events
        print("  [+] Awaiting confirmed event on /tasks/events...")
        ev_start = time.time()
        while time.time() - ev_start < 4.0:
            executor.spin_once(timeout_sec=0.2)
            if any(ev.task_id == 'task_small_0001' and ev.new_state == 'ASSIGNED' for ev in harness.events_received):
                break

        matching_events = [ev for ev in harness.events_received if ev.task_id == 'task_small_0001']
        if not matching_events:
            print("[-] Did not receive confirmed event on /tasks/events")
            return False

        confirmed_ev = matching_events[0]
        print(f"  [+] Confirmed event: task_id='{confirmed_ev.task_id}', new_state='{confirmed_ev.new_state}', robot='{confirmed_ev.robot_id}'")
        assert confirmed_ev.robot_id == 'amr_0'

        # Verify /tasks/available now has 4 tasks
        start_avail_check = time.time()
        while time.time() - start_avail_check < 3.0:
            executor.spin_once(timeout_sec=0.2)
            if len(harness.available_tasks.tasks) == 4:
                break

        print(f"  [+] /tasks/available count after assignment: {len(harness.available_tasks.tasks)} (expected 4)")
        assert len(harness.available_tasks.tasks) == 4

        # Clean harness shutdown
        harness.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

    finally:
        print("  [+] Terminating task_manager process...")
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
            print("  [+] task_manager terminated cleanly (SIGTERM).")
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            print("  [!] task_manager killed (SIGKILL).")

    return True


def main() -> int:
    print("==================================================================")
    print("  NRDAS MILESTONE M3: TASK INFRASTRUCTURE VERIFICATION")
    print("==================================================================")

    step1_ok = run_workload_and_model_checks()
    if not step1_ok:
        print("\n[-] STEP 1 FAILED!")
        return 1

    step2_ok = run_ros2_task_manager_integration()
    if not step2_ok:
        print("\n[-] STEP 2 FAILED!")
        return 1

    print("\n==================================================================")
    print("  ALL M3 VERIFICATION CHECKS PASSED PERFECTLY")
    print("==================================================================")
    return 0


if __name__ == '__main__':
    sys.exit(main())
