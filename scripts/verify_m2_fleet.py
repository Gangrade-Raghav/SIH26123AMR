#!/usr/bin/env python3
"""Automated verification suite for Milestone M2 parameterized multi-robot fleet simulation."""

import argparse
import os
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Set

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class FleetVerificationNode(Node):
    """ROS 2 node auditing multi-robot telemetry, isolation, and TF trees."""

    def __init__(self, robot_names: List[str]):
        super().__init__('m2_fleet_verification_node')
        self.robot_names = robot_names

        self.odom_received: Dict[str, bool] = {r: False for r in robot_names}
        self.scan_received: Dict[str, bool] = {r: False for r in robot_names}
        self.initial_poses: Dict[str, Dict[str, float]] = {}
        self.latest_poses: Dict[str, Dict[str, float]] = {}
        self.scan_ray_counts: Dict[str, int] = {r: 0 for r in robot_names}
        self.scan_valid_counts: Dict[str, int] = {r: 0 for r in robot_names}
        self.scan_frame_ids: Dict[str, str] = {r: '' for r in robot_names}
        self.tf_frames: Set[tuple] = set()

        self.odom_subs = {}
        self.scan_subs = {}
        self.cmd_vel_pubs = {}

        for r_name in robot_names:
            self._setup_robot_subscribers(r_name)

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            100,
        )

        tf_static_qos = QoSProfile(
            depth=200,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_static_callback,
            tf_static_qos,
        )

    def _setup_robot_subscribers(self, r_name: str):
        def make_odom_cb(name):
            def cb(msg: Odometry):
                pos = msg.pose.pose.position
                p = {'x': pos.x, 'y': pos.y, 'z': pos.z}
                self.latest_poses[name] = p
                if name not in self.initial_poses:
                    self.initial_poses[name] = p
                self.odom_received[name] = True
            return cb

        def make_scan_cb(name):
            def cb(msg: LaserScan):
                self.scan_received[name] = True
                self.scan_ray_counts[name] = len(msg.ranges)
                self.scan_valid_counts[name] = sum(
                    1 for r in msg.ranges if msg.range_min <= r <= msg.range_max
                )
                self.scan_frame_ids[name] = msg.header.frame_id.strip('/')
            return cb

        self.odom_subs[r_name] = self.create_subscription(
            Odometry,
            f'/{r_name}/odom',
            make_odom_cb(r_name),
            10,
        )
        self.scan_subs[r_name] = self.create_subscription(
            LaserScan,
            f'/{r_name}/scan',
            make_scan_cb(r_name),
            10,
        )
        self.cmd_vel_pubs[r_name] = self.create_publisher(
            Twist,
            f'/{r_name}/cmd_vel',
            10,
        )

    def tf_callback(self, msg: TFMessage):
        for t in msg.transforms:
            parent = t.header.frame_id.strip('/')
            child = t.child_frame_id.strip('/')
            self.tf_frames.add((parent, child))

    def tf_static_callback(self, msg: TFMessage):
        for t in msg.transforms:
            parent = t.header.frame_id.strip('/')
            child = t.child_frame_id.strip('/')
            self.tf_frames.add((parent, child))

    def publish_cmd_vel(self, robot_name: str, linear_x: float, angular_z: float = 0.0):
        if robot_name in self.cmd_vel_pubs:
            t = Twist()
            t.linear.x = float(linear_x)
            t.angular.z = float(angular_z)
            self.cmd_vel_pubs[robot_name].publish(t)


def get_system_metrics() -> Dict[str, Any]:
    """Capture host resource utilization (CPU, memory, process counts)."""
    metrics = {
        'cpu_percent': 0.0,
        'ram_used_mb': 0.0,
        'ram_total_mb': 0.0,
        'gz_processes': 0,
    }
    try:
        ps_out = subprocess.run(
            ['ps', 'aux'],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        metrics['gz_processes'] = sum(1 for line in ps_out.splitlines() if 'gz sim' in line)

        free_out = subprocess.run(
            ['free', '-m'],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.splitlines()
        for line in free_out:
            if line.startswith('Mem:'):
                parts = line.split()
                metrics['ram_total_mb'] = float(parts[1])
                metrics['ram_used_mb'] = float(parts[2])
                break
    except Exception as e:
        print(f'[WARN] Could not sample system metrics: {e}')
    return metrics


def run_fleet_test(robot_count: int, test_cross_talk: bool = False) -> Dict[str, Any]:
    """Execute end-to-end verification of an N-robot fleet."""
    print('=' * 70)
    print(f' NRDAS AMR FLEET — M2 VERIFICATION FOR {robot_count} ROBOTS')
    print('=' * 70)

    # Clean up any lingering simulation processes
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)
    time.sleep(1.0)

    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    install_setup = os.path.join(workspace_dir, 'install', 'setup.bash')

    robot_names = [f'amr_{i}' for i in range(robot_count)]

    launch_cmd = (
        f'source /opt/ros/jazzy/setup.bash && '
        f'source {install_setup} && '
        f'ros2 launch amr_fleet_bringup fleet.launch.py '
        f'robot_count:={robot_count} headless:=true'
    )

    print(f'[INFO] Launching Gazebo Harmonic + ROS 2 fleet with {robot_count} robots...')
    t_start = time.time()
    sim_proc = subprocess.Popen(
        launch_cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )

    rclpy.init()
    node = FleetVerificationNode(robot_names)
    results = {}
    diagnostics = {}

    try:
        # Phase 1: Wait for all robots to publish odom and scan
        print(f'[INFO] Waiting for all {robot_count} robots to spawn and publish telemetry...')
        timeout = 35.0
        all_ready = False

        while time.time() - t_start < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (
                all(node.odom_received.values()) and
                all(node.scan_received.values())
            ):
                all_ready = True
                break

        startup_time = time.time() - t_start
        diagnostics['startup_time_sec'] = startup_time
        diagnostics['resource_metrics'] = get_system_metrics()

        results['all_robots_spawned'] = all_ready
        results['odometry_isolation'] = all(node.odom_received.values())
        results['sensor_isolation'] = all(node.scan_received.values())

        if not all_ready:
            print(f'[FAIL] Fleet failed to spawn or publish within {timeout}s')
            for r in robot_names:
                print(f'       - {r}: odom={node.odom_received[r]}, scan={node.scan_received[r]}')
            sys.exit(1)

        print(f'[PASS] All {robot_count} robots spawned in {startup_time:.2f}s')

        # Phase 2: Verify Topics
        print('[INFO] Verifying ROS 2 topic discovery across fleet...')
        t_top_start = time.time()
        missing_topics = []
        expected_topics = ['/clock', '/tf', '/tf_static']
        for r in robot_names:
            expected_topics.extend([
                f'/{r}/cmd_vel',
                f'/{r}/odom',
                f'/{r}/scan',
            ])

        while time.time() - t_top_start < 10.0:
            topic_check = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                check=True,
            ).stdout.splitlines()
            missing_topics = [t for t in expected_topics if t not in topic_check]
            if not missing_topics:
                break
            time.sleep(0.5)

        results['namespace_isolation'] = len(missing_topics) == 0
        if results['namespace_isolation']:
            print(f'[PASS] All {len(expected_topics)} operational topics discovered in ROS 2 graph')
        else:
            print(f'[FAIL] Missing topics after timeout: {missing_topics}')

        # Phase 3: Verify Sensor Stream Isolation and Frame IDs
        sensor_frames_valid = True
        for r in robot_names:
            expected_frame = f'{r}/laser_frame'
            actual_frame = node.scan_frame_ids.get(r, '')
            if actual_frame != expected_frame:
                sensor_frames_valid = False
                print(
                    f'[FAIL] {r} scan frame mismatch: '
                    f'expected {expected_frame}, got {actual_frame}'
                )
            valid_returns = node.scan_valid_counts.get(r, 0)
            if valid_returns == 0:
                sensor_frames_valid = False
                print(f'[FAIL] {r} has 0 valid obstacle returns')

        results['sensor_frames_isolated'] = sensor_frames_valid
        if sensor_frames_valid:
            print(f'[PASS] All {robot_count} LiDAR sensor streams isolated with correct frame IDs')

        # Phase 4: Verify TF Isolation
        print('[INFO] Verifying TF frame hierarchy across fleet...')
        t_tf_start = time.time()
        while time.time() - t_tf_start < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            all_odom_tfs = all(
                any(c == f'{r}/base_footprint' for _, c in node.tf_frames)
                for r in robot_names
            )
            if all_odom_tfs:
                break

        print(f'[INFO] Discovered {len(node.tf_frames)} total TF frame pairs')

        tf_valid = True
        for r in robot_names:
            odom_tf = any(c == f'{r}/base_footprint' for _, c in node.tf_frames)
            base_tf = any(c == f'{r}/base_link' for _, c in node.tf_frames)
            laser_tf = any(c == f'{r}/laser_frame' for _, c in node.tf_frames)
            if not (odom_tf and base_tf and laser_tf):
                tf_valid = False
                print(
                    f'[FAIL] Incomplete TF tree for {r} '
                    f'(odom={odom_tf}, base={base_tf}, laser={laser_tf})'
                )

        results['tf_isolation'] = tf_valid
        if tf_valid:
            print(
                f'[PASS] TF trees isolated for all {robot_count} robots '
                f'({len(node.tf_frames)} total frame pairs)'
            )

        # Phase 5: Cross-talk Command Isolation (if requested)
        if test_cross_talk and robot_count >= 2:
            print('[INFO] Executing cross-talk command isolation test...')
            # Reset/sample current positions
            r0 = 'amr_0'
            r1 = 'amr_1'

            p0_start = node.latest_poses[r0]['x']
            p1_start = node.latest_poses[r1]['x']

            # Test A: Command AMR_0 forward, AMR_1 stationary
            print(
                f'[INFO] Command Phase A: Driving {r0} forward (0.5 m/s) '
                f'while {r1} stays stationary...'
            )
            t_cmd = time.time()
            while time.time() - t_cmd < 2.0:
                node.publish_cmd_vel(r0, 0.5, 0.0)
                node.publish_cmd_vel(r1, 0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.1)

            # Settle
            t_stop = time.time()
            while time.time() - t_stop < 1.0:
                node.publish_cmd_vel(r0, 0.0, 0.0)
                node.publish_cmd_vel(r1, 0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.1)

            p0_after_a = node.latest_poses[r0]['x']
            p1_after_a = node.latest_poses[r1]['x']

            d0_a = p0_after_a - p0_start
            d1_a = p1_after_a - p1_start

            print(f'[INFO] Phase A displacements: {r0} moved {d0_a:.3f}m | {r1} moved {d1_a:.3f}m')
            phase_a_passed = (d0_a > 0.4) and (abs(d1_a) < 0.03)

            # Test B: Command AMR_1 forward, AMR_0 stationary
            print(
                f'[INFO] Command Phase B: Driving {r1} forward (0.5 m/s) '
                f'while {r0} stays stationary...'
            )
            t_cmd = time.time()
            while time.time() - t_cmd < 2.0:
                node.publish_cmd_vel(r0, 0.0, 0.0)
                node.publish_cmd_vel(r1, 0.5, 0.0)
                rclpy.spin_once(node, timeout_sec=0.1)

            # Settle
            t_stop = time.time()
            while time.time() - t_stop < 1.0:
                node.publish_cmd_vel(r0, 0.0, 0.0)
                node.publish_cmd_vel(r1, 0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.1)

            p0_after_b = node.latest_poses[r0]['x']
            p1_after_b = node.latest_poses[r1]['x']

            d0_b = p0_after_b - p0_after_a
            d1_b = p1_after_b - p1_after_a

            print(f'[INFO] Phase B displacements: {r0} moved {d0_b:.3f}m | {r1} moved {d1_b:.3f}m')
            phase_b_passed = (d1_b > 0.4) and (abs(d0_b) < 0.03)

            results['command_isolation'] = phase_a_passed and phase_b_passed
            diagnostics['cross_talk'] = {
                'phase_a': {'moved_robot': r0, 'disp_r0': d0_a, 'disp_r1': d1_a},
                'phase_b': {'moved_robot': r1, 'disp_r0': d0_b, 'disp_r1': d1_b},
            }

            if results['command_isolation']:
                print('[PASS] Cross-talk test PASSED: Independent command isolation confirmed!')
            else:
                print('[FAIL] Cross-talk detected or robot failed to move as commanded')

    finally:
        print('[INFO] Terminating simulation cleanly...')
        node.destroy_node()
        rclpy.shutdown()

        os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        try:
            sim_proc.wait(timeout=10)
            results['clean_shutdown'] = True
            print('[PASS] Simulation process terminated cleanly on SIGINT')
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
            results['clean_shutdown'] = False
            print('[WARN] Simulation required SIGKILL after timeout')

        subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)
        time.sleep(1.0)

    print('-' * 70)
    print(f' SUMMARY FOR {robot_count}-ROBOT FLEET')
    print('-' * 70)
    passed_all = True
    for k, v in results.items():
        tag = '[PASS]' if v else '[FAIL]'
        print(f'{tag} {k}: {v}')
        if not v:
            passed_all = False

    print(f"Startup Time: {diagnostics.get('startup_time_sec', 0.0):.2f}s")
    res = diagnostics.get('resource_metrics', {})
    gz_proc = res.get('gz_processes', 0)
    ram_u = res.get('ram_used_mb', 0)
    ram_t = res.get('ram_total_mb', 0)
    print(f'Gazebo Processes: {gz_proc} | RAM: {ram_u:.0f}MB / {ram_t:.0f}MB')
    print('=' * 70)

    return {
        'robot_count': robot_count,
        'passed': passed_all,
        'results': results,
        'diagnostics': diagnostics,
    }


def main():
    desc = 'Verify multi-robot fleet simulation in Gazebo Harmonic'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument(
        '--count', type=int, default=2, help='Robot count to verify (2, 5, or 10)'
    )
    parser.add_argument('--all', action='store_true', help='Run full sweep: 2, 5, and 10 robots')
    parser.add_argument(
        '--cross-talk', action='store_true', help='Execute cross-talk isolation test'
    )
    args = parser.parse_args()

    counts = [2, 5, 10] if args.all else [args.count]

    overall_success = True
    for count in counts:
        do_cross_talk = args.cross_talk or (count == 2)
        res = run_fleet_test(count, test_cross_talk=do_cross_talk)
        if not res['passed']:
            overall_success = False

    if overall_success:
        print('[FINAL RESULT] ALL FLEET VERIFICATION TESTS PASSED SUCCESSFULLY')
        sys.exit(0)
    else:
        print('[FINAL RESULT] FLEET VERIFICATION HAD FAILURES')
        sys.exit(1)


if __name__ == '__main__':
    main()
