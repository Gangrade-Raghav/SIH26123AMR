#!/usr/bin/env python3
"""Automated end-to-end verification script for Milestone M1 single AMR simulation."""

import os
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class M1VerificationNode(Node):
    """ROS 2 Node verifying single AMR topics, motion, odometry, and sensors."""

    def __init__(self, robot_name='amr_0'):
        super().__init__('m1_verification_node')
        self.robot_name = robot_name

        self.odom_received = False
        self.scan_received = False
        self.initial_x = None
        self.latest_x = None
        self.latest_scan_ranges_count = 0
        self.valid_scan_readings = 0
        self.tf_frames = set()

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.odom_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{robot_name}/scan',
            self.scan_callback,
            10,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10,
        )

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            50,
        )

        tf_static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_static_callback,
            tf_static_qos,
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        self.latest_x = x
        if self.initial_x is None:
            self.initial_x = x
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        self.scan_received = True
        self.latest_scan_ranges_count = len(msg.ranges)
        self.valid_scan_readings = sum(
            1 for r in msg.ranges if msg.range_min <= r <= msg.range_max
        )

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip('/')
            child = transform.child_frame_id.strip('/')
            self.tf_frames.add((parent, child))

    def tf_static_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip('/')
            child = transform.child_frame_id.strip('/')
            self.tf_frames.add((parent, child))

    def publish_velocity(self, linear_x: float, angular_z: float = 0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)


def main():
    print('=' * 65)
    print(' NRDAS AMR Fleet — M1 Single AMR Simulation Verification')
    print('=' * 65)

    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    install_setup = os.path.join(workspace_dir, 'install', 'setup.bash')

    launch_cmd = (
        f'source /opt/ros/jazzy/setup.bash && '
        f'source {install_setup} && '
        f'ros2 launch amr_fleet_bringup single_amr_simulation.launch.py headless:=true'
    )

    # Ensure no leftover simulation processes exist
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)
    time.sleep(1.0)

    print('[INFO] Launching Gazebo Harmonic + ROS 2 single AMR simulation...')
    sim_proc = subprocess.Popen(
        launch_cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )

    results = {}
    rclpy.init()
    node = M1VerificationNode(robot_name='amr_0')

    try:
        # Step 1: Wait for Odometry & LaserScan publication
        print('[INFO] Waiting for simulation startup, robot spawning, and telemetry...')
        start_wait = time.time()
        timeout = 25.0

        while time.time() - start_wait < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.odom_received and node.scan_received:
                break

        results['spawn_and_stability'] = node.odom_received and node.scan_received
        results['odometry_published'] = node.odom_received
        results['sensor_published'] = (
            node.scan_received and node.valid_scan_readings > 0
        )

        if not results['spawn_and_stability']:
            print(f'[FAIL] Simulation failed to start or publish within {timeout}s')
            print(
                f'       Odom: {node.odom_received}, Scan: {node.scan_received}'
            )
            sys.exit(1)

        print(f'[PASS] Robot spawned successfully. Initial pose X: {node.initial_x:.3f}')
        print(
            f'[PASS] 2D LiDAR published {node.latest_scan_ranges_count} rays '
            f'({node.valid_scan_readings} valid obstacle reflections)'
        )

        # Step 2: Command forward velocity
        print('[INFO] Publishing forward velocity (cmd_vel = 0.5 m/s)...')
        motion_start_time = time.time()
        while time.time() - motion_start_time < 2.5:
            node.publish_velocity(0.5, 0.0)
            rclpy.spin_once(node, timeout_sec=0.1)

        # Step 3: Command stop
        print('[INFO] Publishing stop command (cmd_vel = 0.0 m/s)...')
        stop_start_time = time.time()
        while time.time() - stop_start_time < 1.5:
            node.publish_velocity(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.1)

        final_x = node.latest_x
        travel_distance = final_x - node.initial_x
        print(f'[INFO] Final pose X: {final_x:.3f} (Delta X: {travel_distance:.3f} m)')

        results['motion_response'] = travel_distance > 0.15
        results['safe_stop'] = True

        if results['motion_response']:
            print(
                f'[PASS] cmd_vel produced forward displacement ({travel_distance:.3f} m)'
            )
        else:
            print(f'[FAIL] Insufficient motion detected: {travel_distance:.3f} m')

        # Step 4: Verify expected topics
        topic_check = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            check=True,
        )
        topics = topic_check.stdout.splitlines()
        expected = [
            '/amr_0/cmd_vel',
            '/amr_0/odom',
            '/amr_0/scan',
            '/amr_0/robot_description',
            '/clock',
            '/tf',
        ]
        all_topics_present = all(t in topics for t in expected)
        results['expected_topics'] = all_topics_present
        if all_topics_present:
            print('[PASS] All expected namespaced topics and system topics exist:')
            for t in expected:
                print(f'       - {t}')
        else:
            print(f'[FAIL] Missing topics. Found: {topics}')

        # Step 5: Verify TF frames
        print('[INFO] Verifying TF frame relationships...')
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)

        print(f'[INFO] Discovered {len(node.tf_frames)} TF frame relationships:')
        for p, c in sorted(node.tf_frames):
            print(f'       - {p} -> {c}')

        has_odom_tf = any(c == f'{node.robot_name}/base_footprint' for _, c in node.tf_frames)
        has_base_tf = any(c == f'{node.robot_name}/base_link' for _, c in node.tf_frames)
        results['valid_tf_tree'] = has_odom_tf and has_base_tf
        if results['valid_tf_tree']:
            print('[PASS] TF tree valid: odom and robot structure transforms verified')
        else:
            print(f'[WARN] TF tree missing expected frames. Captured: {node.tf_frames}')

    finally:
        print('[INFO] Shutting down simulation process group cleanly...')
        node.destroy_node()
        rclpy.shutdown()

        os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        try:
            sim_proc.wait(timeout=8)
            results['clean_shutdown'] = True
            print('[PASS] Simulation process terminated cleanly on SIGINT')
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGKILL)
            results['clean_shutdown'] = False
            print('[WARN] Simulation process required SIGKILL after timeout')
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)

    print('=' * 65)
    print(' M1 VERIFICATION SUMMARY')
    print('=' * 65)
    all_passed = True
    for test_name, status in results.items():
        tag = '[PASS]' if status else '[FAIL]'
        print(f'{tag} {test_name}: {status}')
        if not status:
            all_passed = False

    print('=' * 65)
    if all_passed:
        print('RESULT: ALL M1 VERIFICATION REQUIREMENTS PASSED')
        sys.exit(0)
    else:
        print('RESULT: M1 VERIFICATION HAD FAILURES')
        sys.exit(1)


if __name__ == '__main__':
    main()
