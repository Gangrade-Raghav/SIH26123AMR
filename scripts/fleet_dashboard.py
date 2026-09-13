#!/usr/bin/env python3
"""NRDAS Fleet Observability Dashboard.

Decoupled, read-only monitoring suite for multi-AMR fleet coordination.
Consumes ground-truth telemetry directly from ROS 2 topics and graph.

Exposes:
1. Zero-dependency Web Dashboard (http://localhost:8080) with 2D warehouse mini-map.
2. Rich Terminal TUI mode for live CLI telemetry.
3. REST JSON endpoint (/api/state) for external observability.
"""

import argparse
from datetime import datetime
import http.server
import json
import math
import os
import socketserver
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock

try:
    from amr_fleet_msgs.msg import (
        CBBABid,
        RobotBundle,
        TaskDefinition,
        TaskEvent as TaskEventMsg,
        TaskList,
    )
    HAVE_TASK_MSGS = True
except ImportError:
    HAVE_TASK_MSGS = False


class SystemMetricsReader:
    """Reads host CPU and memory usage from Linux /proc filesystem."""

    def __init__(self) -> None:
        self._prev_idle = 0
        self._prev_total = 0

    def read_cpu_percent(self) -> float:
        try:
            with open('/proc/stat', 'r', encoding='utf-8') as f:
                fields = [float(x) for x in f.readline().strip().split()[1:8]]
            idle = fields[3] + fields[4]
            total = sum(fields)
            diff_idle = idle - self._prev_idle
            diff_total = total - self._prev_total
            self._prev_idle = idle
            self._prev_total = total
            if diff_total == 0:
                return 0.0
            return max(0.0, min(100.0, (1.0 - diff_idle / diff_total) * 100.0))
        except Exception:
            return 0.0

    def read_ram_usage_mb(self) -> Dict[str, float]:
        try:
            mem_total = 0.0
            mem_avail = 0.0
            with open('/proc/meminfo', 'r', encoding='utf-8') as f:
                for line in f:
                    if line.startswith('MemTotal:'):
                        mem_total = float(line.split()[1]) / 1024.0
                    elif line.startswith('MemAvailable:'):
                        mem_avail = float(line.split()[1]) / 1024.0
            mem_used = mem_total - mem_avail
            return {
                'total_mb': round(mem_total, 1),
                'used_mb': round(mem_used, 1),
                'percent': round((mem_used / mem_total) * 100.0, 1) if mem_total > 0 else 0.0,
            }
        except Exception:
            return {'total_mb': 0.0, 'used_mb': 0.0, 'percent': 0.0}


class RobotTelemetryTracker:
    """Tracks live state, kinematics, and sensor health for a single AMR."""

    def __init__(self, robot_id: str, spawn_x: float = 0.0, spawn_y: float = 0.0) -> None:
        self.robot_id = robot_id
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.x = spawn_x
        self.y = spawn_y
        self.yaw = 0.0
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.last_odom_time = 0.0
        self.total_distance = 0.0
        self._prev_x: Optional[float] = None
        self._prev_y: Optional[float] = None
        
        # Sensor
        self.lidar_count = 0
        self.lidar_rate_hz = 0.0
        self.min_scan_range = float('inf')
        self._last_scan_times: List[float] = []

    def update_odometry(self, msg: Odometry) -> None:
        now = time.time()
        self.last_odom_time = now
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        
        # Orientation quaternion -> Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Cumulative distance
        if self._prev_x is not None and self._prev_y is not None:
            d = math.hypot(px - self._prev_x, py - self._prev_y)
            if 0.0001 < d < 2.0:  # ignore teleports
                self.total_distance += d
        self._prev_x = px
        self._prev_y = py

        self.x = round(self.spawn_x + px, 2)
        self.y = round(self.spawn_y + py, 2)
        self.linear_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.angular_speed = msg.twist.twist.angular.z

    def update_scan(self, msg: LaserScan) -> None:
        now = time.time()
        self.lidar_count += 1
        self._last_scan_times.append(now)
        if len(self._last_scan_times) > 20:
            self._last_scan_times.pop(0)

        if len(self._last_scan_times) >= 2:
            dt = self._last_scan_times[-1] - self._last_scan_times[0]
            if dt > 0.0:
                self.lidar_rate_hz = round((len(self._last_scan_times) - 1) / dt, 1)

        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > msg.range_min]
        self.min_scan_range = min(valid_ranges) if valid_ranges else float('inf')

    @property
    def status(self) -> str:
        if time.time() - self.last_odom_time > 3.0:
            return 'OFFLINE'
        if self.linear_speed > 0.02 or abs(self.angular_speed) > 0.05:
            return 'ACTIVE'
        return 'IDLE'

    def to_dict(self) -> Dict[str, Any]:
        return {
            'robot_id': self.robot_id,
            'status': self.status,
            'x': round(self.x, 2),
            'y': round(self.y, 2),
            'yaw_deg': round(math.degrees(self.yaw), 1),
            'linear_speed': round(self.linear_speed, 2),
            'angular_speed': round(self.angular_speed, 2),
            'distance_m': round(self.total_distance, 2),
            'lidar_hz': self.lidar_rate_hz,
            'min_obstacle_m': round(self.min_scan_range, 2) if self.min_scan_range != float('inf') else None,
            'last_update_sec_ago': round(time.time() - self.last_odom_time, 1) if self.last_odom_time > 0 else None,
        }


class FleetMonitorNode(Node):
    """ROS 2 Node collecting ground-truth multi-robot telemetry."""

    def __init__(self, target_robot_count: int = 10) -> None:
        super().__init__('amr_fleet_monitor')
        self.target_robot_count = target_robot_count
        self.robots: Dict[str, RobotTelemetryTracker] = {}
        self.metrics_reader = SystemMetricsReader()
        self.sim_time_sec = 0.0
        self._last_sim_time = 0.0
        self._last_wall_time = time.time()
        self.real_time_factor = 1.0

        # Load spawn configurations
        self.spawn_poses: Dict[str, Tuple[float, float]] = {}
        ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg_paths = [
            os.path.join(ws_root, 'config', 'robots', f'fleet_{self.target_robot_count}_robots.yaml'),
            os.path.join(ws_root, 'config', 'robots', 'fleet_5_robots.yaml'),
            os.path.join(ws_root, 'config', 'robots', 'fleet_default.yaml'),
        ]
        for cp in cfg_paths:
            if os.path.isfile(cp):
                try:
                    with open(cp, 'r', encoding='utf-8') as f:
                        cfg_data = yaml.safe_load(f)
                    for r_cfg in cfg_data.get('fleet', {}).get('robots', []):
                        r_id = r_cfg.get('id')
                        if r_id:
                            self.spawn_poses[r_id] = (float(r_cfg.get('x', 0.0)), float(r_cfg.get('y', 0.0)))
                    if self.spawn_poses:
                        break
                except Exception:
                    pass

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscribe to simulation clock
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)

        # M3 Task State Tracking
        self.tasks_data: Dict[str, Any] = {
            'status': 'AWAITING /tasks/all',
            'total': 0,
            'pending': 0,
            'assigned': 0,
            'in_progress': 0,
            'completed': 0,
            'failed': 0,
            'cancelled': 0,
            'tasks': [],
            'recent_events': [],
        }

        # CBBA State Tracking
        self.cbba_data: Dict[str, Any] = {
            'status': 'IDLE — Awaiting CBBA Auction',
            'is_converged': False,
            'bids_count': 0,
            'winning_allocations': {},
            'bundles': {},
            'makespan_sec': 0.0,
            'last_bid_time': 0.0,
        }
        self._bundle_subs: set = set()

        if HAVE_TASK_MSGS:
            self.create_subscription(TaskList, '/tasks/all', self._tasks_cb, 10)
            self.create_subscription(TaskEventMsg, '/tasks/events', self._task_event_cb, 20)
            self.create_subscription(CBBABid, '/fleet/cbba_bids', self._cbba_bid_cb, 50)

        # Dynamic topic discovery timer
        self.create_timer(1.0, self._discover_fleet_topics)

    def _tasks_cb(self, msg: 'TaskList') -> None:
        tasks_list = []
        pending = 0
        assigned = 0
        in_progress = 0
        completed = 0
        failed = 0
        cancelled = 0

        for t in msg.tasks:
            status = t.status.upper()
            if status == 'PENDING':
                pending += 1
            elif status == 'ASSIGNED':
                assigned += 1
            elif status == 'IN_PROGRESS':
                in_progress += 1
            elif status == 'COMPLETED':
                completed += 1
            elif status == 'FAILED':
                failed += 1
            elif status == 'CANCELLED':
                cancelled += 1

            tasks_list.append({
                'id': t.task_id,
                'pickup': [round(t.pickup_pose.x, 2), round(t.pickup_pose.y, 2)],
                'dropoff': [round(t.dropoff_pose.x, 2), round(t.dropoff_pose.y, 2)],
                'priority': t.priority,
                'status': t.status,
                'robot': t.assigned_robot_id or None,
            })

        self.tasks_data = {
            'status': 'ONLINE — M3 Lifecycle Active',
            'total': len(msg.tasks),
            'pending': pending,
            'assigned': assigned,
            'in_progress': in_progress,
            'completed': completed,
            'failed': failed,
            'cancelled': cancelled,
            'tasks': tasks_list,
            'recent_events': self.tasks_data.get('recent_events', []),
        }

    def _task_event_cb(self, msg: 'TaskEventMsg') -> None:
        ev = {
            'task_id': msg.task_id,
            'event': msg.event_type,
            'from': msg.previous_state,
            'to': msg.new_state,
            'robot': msg.robot_id or 'none',
            'details': msg.details,
        }
        recent = self.tasks_data.get('recent_events', [])
        recent.insert(0, ev)
        self.tasks_data['recent_events'] = recent[:5]

    def _cbba_bid_cb(self, msg: 'CBBABid') -> None:
        self.cbba_data['bids_count'] += 1
        self.cbba_data['last_bid_time'] = time.time()
        for idx, t_id in enumerate(msg.task_ids):
            bid = msg.winning_bids[idx] if idx < len(msg.winning_bids) else 0.0
            winner = msg.winning_robots[idx] if idx < len(msg.winning_robots) else ''
            self.cbba_data['winning_allocations'][t_id] = {
                'winner': winner,
                'bid': round(bid, 2),
            }

    def _bundle_cb(self, r_id: str, msg: 'RobotBundle') -> None:
        self.cbba_data['bundles'][r_id] = list(msg.task_ids)
        if msg.is_converged:
            self.cbba_data['is_converged'] = True
            self.cbba_data['status'] = 'CONVERGED (Consensus Reached)'
        else:
            self.cbba_data['status'] = 'NEGOTIATING (Bids Exchanging)'

    def _clock_cb(self, msg: Clock) -> None:
        now_sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time_sec = now_sim
        now_wall = time.time()
        wall_dt = now_wall - self._last_wall_time
        sim_dt = now_sim - self._last_sim_time
        if wall_dt >= 1.0:
            if wall_dt > 0:
                self.real_time_factor = round(sim_dt / wall_dt, 2)
            self._last_wall_time = now_wall
            self._last_sim_time = now_sim

    def _discover_fleet_topics(self) -> None:
        topic_names_and_types = self.get_topic_names_and_types()
        existing_topics = {t[0] for t in topic_names_and_types}

        for i in range(self.target_robot_count):
            r_id = f'amr_{i}'
            odom_topic = f'/{r_id}/odom'
            scan_topic = f'/{r_id}/scan'
            bundle_topic = f'/{r_id}/bundle'

            if odom_topic in existing_topics and r_id not in self.robots:
                spawn_x, spawn_y = self.spawn_poses.get(r_id, (2.0, 2.0 + i * 3.0))
                tracker = RobotTelemetryTracker(r_id, spawn_x, spawn_y)
                self.robots[r_id] = tracker

                # Subscribe to odom
                def make_odom_cb(t):
                    return lambda msg: t.update_odometry(msg)
                self.create_subscription(Odometry, odom_topic, make_odom_cb(tracker), 10)

                # Subscribe to scan
                def make_scan_cb(t):
                    return lambda msg: t.update_scan(msg)
                self.create_subscription(LaserScan, scan_topic, make_scan_cb(tracker), 10)

            if bundle_topic in existing_topics and r_id not in self._bundle_subs:
                def make_bundle_cb(rid):
                    return lambda msg: self._bundle_cb(rid, msg)
                self.create_subscription(RobotBundle, bundle_topic, make_bundle_cb(r_id), 10)
                self._bundle_subs.add(r_id)

    def get_fleet_summary(self) -> Dict[str, Any]:
        node_names = self.get_node_names()
        topics = [t[0] for t in self.get_topic_names_and_types()]
        robot_data = [self.robots[k].to_dict() for k in sorted(self.robots.keys())]

        active_count = sum(1 for r in robot_data if r['status'] == 'ACTIVE')
        idle_count = sum(1 for r in robot_data if r['status'] == 'IDLE')
        offline_count = sum(1 for r in robot_data if r['status'] == 'OFFLINE')

        # Safety determination
        min_distance = min([r['min_obstacle_m'] for r in robot_data if r['min_obstacle_m'] is not None], default=99.0)
        safety_status = 'CLEAR'
        if min_distance < 0.25:
            safety_status = 'CRITICAL PROXIMITY'
        elif min_distance < 0.45:
            safety_status = 'PROXIMITY WARNING'

        ram = self.metrics_reader.read_ram_usage_mb()
        cpu = self.metrics_reader.read_cpu_percent()

        # Makespan calculation from CBBA bundles
        max_dist = 0.0
        task_dict = {t['id']: t for t in self.tasks_data.get('tasks', [])}
        for r_id, b_tasks in self.cbba_data['bundles'].items():
            r_tracker = self.robots.get(r_id)
            cur_pos = (r_tracker.x, r_tracker.y) if r_tracker else (0.0, 0.0)
            d = 0.0
            for tid in b_tasks:
                if tid in task_dict:
                    pk = task_dict[tid]['pickup']
                    dp = task_dict[tid]['dropoff']
                    d += math.hypot(pk[0] - cur_pos[0], pk[1] - cur_pos[1])
                    d += math.hypot(dp[0] - pk[0], dp[1] - pk[1])
                    cur_pos = (dp[0], dp[1])
            if d > max_dist:
                max_dist = d
        self.cbba_data['makespan_sec'] = round(max_dist / 0.5, 1) if max_dist > 0 else 0.0

        return {
            'timestamp': datetime.now().isoformat(),
            'simulation': {
                'sim_time_sec': round(self.sim_time_sec, 2),
                'real_time_factor': self.real_time_factor,
                'status': 'RUNNING' if self.sim_time_sec > 0 else 'INITIALIZING',
            },
            'fleet': {
                'total_discovered': len(robot_data),
                'active_count': active_count,
                'idle_count': idle_count,
                'offline_count': offline_count,
                'robots': robot_data,
            },
            'tasks': {
                'status': self.tasks_data['status'],
                'total': self.tasks_data['total'],
                'pending': self.tasks_data['pending'],
                'active': self.tasks_data['assigned'] + self.tasks_data['in_progress'],
                'completed': self.tasks_data['completed'],
                'failed': self.tasks_data['failed'],
                'cancelled': self.tasks_data['cancelled'],
                'tasks': self.tasks_data.get('tasks', []),
                'recent_events': self.tasks_data.get('recent_events', []),
            },
            'cbba': self.cbba_data,
            'network': {
                'ros2_nodes_count': len(node_names),
                'ros2_topics_count': len(topics),
                'middleware': 'ROS 2 Jazzy (Zenoh / Cyclone DDS)',
                'transport_state': 'HEALTHY' if len(robot_data) > 0 else 'AWAITING_FLEET',
            },
            'performance': {
                'host_cpu_percent': round(cpu, 1),
                'host_ram_used_mb': ram['used_mb'],
                'host_ram_total_mb': ram['total_mb'],
                'host_ram_percent': ram['percent'],
                'mapf_planning_latency': 'N/A — Pending M6 PIBT/RHCR',
                'fleet_throughput': (
                    f"{self.tasks_data['completed']} / {self.tasks_data['total']} completed"
                    if self.tasks_data['total'] > 0 else '0 tasks/hr'
                ),
            },
            'safety': {
                'estop_status': 'NORMAL — DISENGAGED',
                'active_safety_zone': safety_status,
                'deadlock_detection': 'N/A — Pending M8 Wait-For-Graph',
            },
        }


# =====================================================================
# EMBEDDED DASHBOARD HTML/CSS/JS INTERFACE
# =====================================================================
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NRDAS // INDUSTRIAL FLEET OPERATIONS</title>
  <style>
    :root {
      --c-black: #000000;
      --c-bg: #0a0a0a;
      --c-surface: #141414;
      --c-card: #1c1c1c;
      --c-card-header: #000000;
      --c-border: #333333;
      --c-border-strong: #000000;
      --c-border-focus: #ff5500;
      --c-orange: #ff5500;
      --c-orange-dim: #cc4400;
      --c-orange-bg: rgba(255, 85, 0, 0.12);
      --c-grey-muted: #888888;
      --c-grey-light: #cccccc;
      --c-white: #ffffff;
      --shadow-brutal: 4px 4px 0px #000000;
      --shadow-brutal-sm: 2px 2px 0px #000000;
      --shadow-orange: 4px 4px 0px #ff5500;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      border-radius: 0px !important;
    }

    body {
      font-family: 'JetBrains Mono', 'Space Mono', 'Consolas', 'Courier New', monospace;
      background-color: var(--c-bg);
      color: var(--c-white);
      padding: 16px;
      line-height: 1.4;
    }

    /* Hazard Warning Top Stripe */
    .hazard-stripe {
      height: 8px;
      background: repeating-linear-gradient(45deg, var(--c-orange), var(--c-orange) 14px, var(--c-black) 14px, var(--c-black) 28px);
      border: 1px solid var(--c-black);
      margin-bottom: 14px;
    }

    /* Header Bar */
    header {
      background: var(--c-surface);
      border: 2px solid var(--c-black);
      box-shadow: var(--shadow-brutal);
      padding: 14px 18px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 16px;
      border-left: 6px solid var(--c-orange);
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 14px;
    }
    .brand h1 {
      font-size: 1.25rem;
      font-weight: 900;
      text-transform: uppercase;
      letter-spacing: 1px;
      color: var(--c-white);
    }
    .badge-sub {
      background: var(--c-orange);
      color: var(--c-black);
      font-weight: 900;
      font-size: 0.72rem;
      padding: 3px 8px;
      border: 1px solid var(--c-black);
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    .header-stats {
      display: flex;
      gap: 10px;
    }
    .stat-pill {
      background: var(--c-card);
      border: 2px solid var(--c-black);
      box-shadow: var(--shadow-brutal-sm);
      padding: 6px 12px;
      font-size: 0.8rem;
      display: flex;
      gap: 8px;
      align-items: center;
    }
    .stat-label {
      color: var(--c-grey-muted);
      font-weight: 700;
      text-transform: uppercase;
      font-size: 0.72rem;
    }
    .stat-value {
      color: var(--c-orange);
      font-weight: 900;
    }

    /* Main Grid Layout */
    .grid {
      display: grid;
      grid-template-columns: 1.25fr 0.75fr;
      gap: 16px;
    }

    /* Brutalist Cards */
    .card {
      background: var(--c-surface);
      border: 2px solid var(--c-black);
      box-shadow: var(--shadow-brutal);
      padding: 16px;
      margin-bottom: 16px;
    }
    .card-title {
      font-size: 0.85rem;
      font-weight: 900;
      text-transform: uppercase;
      letter-spacing: 1.2px;
      color: var(--c-white);
      border-bottom: 2px solid var(--c-border);
      padding-bottom: 10px;
      margin-bottom: 14px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .card-title .tag {
      color: var(--c-orange);
      font-size: 0.75rem;
    }

    /* Robot Grid */
    .robot-grid {
      display: grid;
      grid-template-columns: repeat(auto-fill, minmax(180px, 1fr));
      gap: 10px;
    }
    .robot-card {
      background: var(--c-card);
      border: 2px solid var(--c-black);
      box-shadow: var(--shadow-brutal-sm);
      padding: 10px;
    }
    .robot-header {
      background: var(--c-card-header);
      padding: 6px 8px;
      margin: -10px -10px 10px -10px;
      border-bottom: 2px solid var(--c-orange);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .robot-name {
      color: var(--c-orange);
      font-weight: 900;
      font-size: 0.9rem;
      letter-spacing: 0.5px;
    }
    .status-badge {
      font-size: 0.65rem;
      font-weight: 900;
      padding: 2px 6px;
      text-transform: uppercase;
      border: 1px solid var(--c-black);
    }
    .status-IDLE {
      background: var(--c-surface);
      color: var(--c-grey-light);
      border-color: var(--c-border);
    }
    .status-ACTIVE {
      background: var(--c-orange);
      color: var(--c-black);
    }
    .status-OFFLINE {
      background: #444444;
      color: var(--c-grey-muted);
    }
    .robot-kv {
      display: flex;
      justify-content: space-between;
      font-size: 0.75rem;
      margin: 3px 0;
    }
    .robot-kv .k {
      color: var(--c-grey-muted);
      font-weight: 700;
    }
    .robot-kv .v {
      color: var(--c-white);
      font-weight: 700;
    }

    /* Map Canvas */
    .map-container {
      background: #0d0d0d;
      border: 2px solid var(--c-black);
      box-shadow: var(--shadow-brutal);
      width: 100%;
      height: 420px;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
      margin-bottom: 8px;
    }
    canvas {
      width: 100%;
      height: 100%;
    }
    .map-legend {
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      font-size: 0.72rem;
      padding: 8px 12px;
      background: var(--c-card);
      border: 1px solid var(--c-border);
      color: var(--c-grey-light);
    }
    .legend-item {
      display: flex;
      align-items: center;
      gap: 6px;
      font-weight: 700;
    }
    .leg-amr { color: var(--c-orange); }
    .leg-pickup { color: var(--c-orange); }
    .leg-dropoff { color: var(--c-white); }
    .leg-route { color: var(--c-orange); }

    /* Metric Table Rows */
    .metric-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 7px 0;
      border-bottom: 1px solid var(--c-border);
      font-size: 0.8rem;
    }
    .metric-row:last-child {
      border-bottom: none;
    }
    .metric-key {
      color: var(--c-grey-muted);
      font-weight: 700;
      text-transform: uppercase;
      font-size: 0.74rem;
    }
    .metric-val {
      color: var(--c-white);
      font-weight: 900;
    }

    /* Big Callout Box */
    .callout-box {
      border: 2px solid var(--c-black);
      box-shadow: var(--shadow-brutal-sm);
      padding: 10px 14px;
      margin-bottom: 12px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .callout-converged {
      background: var(--c-orange);
      color: var(--c-black);
    }
    .callout-converged .title {
      font-weight: 900;
      font-size: 0.92rem;
      letter-spacing: 0.5px;
    }
    .callout-converged .meta {
      font-weight: 800;
      font-size: 0.75rem;
      background: var(--c-black);
      color: var(--c-orange);
      padding: 2px 8px;
    }

    /* Task Table */
    .task-table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.74rem;
      margin-top: 10px;
    }
    .task-table th {
      background: var(--c-black);
      color: var(--c-grey-muted);
      text-align: left;
      padding: 6px 8px;
      border: 1px solid var(--c-border);
      text-transform: uppercase;
      font-weight: 800;
    }
    .task-table td {
      padding: 6px 8px;
      border: 1px solid var(--c-border);
      background: var(--c-card);
      font-weight: 700;
    }
    .task-table tr:hover td {
      background: #252525;
    }
    .task-id {
      color: var(--c-white);
    }
    .task-winner {
      color: var(--c-orange);
      font-weight: 900;
    }
    .priority-CRITICAL { color: var(--c-orange); font-weight: 900; }
    .priority-HIGH { color: var(--c-white); font-weight: 800; }
    .priority-NORMAL { color: var(--c-grey-light); }
    .priority-LOW { color: var(--c-grey-muted); }
  </style>
</head>
<body>

  <!-- Hazard Header Stripe -->
  <div class="hazard-stripe"></div>

  <header>
    <div class="brand">
      <h1>NRDAS // AMR FLEET OPERATIONS</h1>
      <span class="badge-sub">BRUTALIST CONSOLE</span>
    </div>
    <div class="header-stats">
      <div class="stat-pill"><span class="stat-label">SIM:</span><span id="sim-time" class="stat-value">0.0s</span></div>
      <div class="stat-pill"><span class="stat-label">RTF:</span><span id="rtf-val" class="stat-value">1.00x</span></div>
      <div class="stat-pill"><span class="stat-label">CPU:</span><span id="cpu-val" class="stat-value">0.0%</span></div>
      <div class="stat-pill"><span class="stat-label">RAM:</span><span id="ram-val" class="stat-value">0 MB</span></div>
    </div>
  </header>

  <div class="grid">
    <!-- Left Column: Spatial Radar & Fleet Telemetry -->
    <div>
      <div class="card">
        <div class="card-title">
          <span>// 01. WAREHOUSE 2D RADAR (16M x 16M)</span>
          <span class="tag">TOPOLOGY ACTIVE</span>
        </div>
        <div class="map-container">
          <canvas id="warehouse-canvas" width="640" height="640"></canvas>
        </div>
        <div class="map-legend">
          <div class="legend-item"><span class="leg-amr">● [R0..R4]</span> AMR Robot</div>
          <div class="legend-item"><span class="leg-pickup">■ [P]</span> Pickup Bay</div>
          <div class="legend-item"><span class="leg-dropoff">□ [D]</span> Dropoff Hub</div>
          <div class="legend-item"><span class="leg-route">- - -</span> CBBA Assigned Route</div>
        </div>
      </div>

      <div class="card">
        <div class="card-title">
          <span>// 02. ACTIVE AMR TELEMETRY MATRIX</span>
          <span id="fleet-count-badge" class="badge-sub">0 AMRs</span>
        </div>
        <div id="robot-container" class="robot-grid">
          <div style="color: var(--c-grey-muted); padding: 12px;">Awaiting fleet odometry...</div>
        </div>
      </div>
    </div>

    <!-- Right Column: CBBA Consensus, Tasks, Diagnostics -->
    <div>
      <!-- CBBA Decentralized Consensus (M4) -->
      <div class="card">
        <div class="card-title">
          <span>// 03. CBBA CONSENSUS ALLOCATOR (M4)</span>
          <span class="tag">DECENTRALIZED</span>
        </div>

        <div id="cbba-callout" class="callout-box callout-converged">
          <span class="title" id="cbba-status">CONVERGED // CONSENSUS REACHED</span>
          <span class="meta" id="cbba-bids-badge">0 BIDS</span>
        </div>

        <div class="metric-row">
          <span class="metric-key">Total Bids Exchanged:</span>
          <span class="metric-val" id="cbba-bids">0</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Estimated Makespan:</span>
          <span class="metric-val" id="perf-makespan" style="color: var(--c-orange);">0.0s</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Convergence Mode:</span>
          <span class="metric-val" style="color: var(--c-white);">18-RULE GOSSIP CBBA</span>
        </div>
        <div class="metric-row" style="flex-direction: column; align-items: flex-start; gap: 4px;">
          <span class="metric-key">Allocated Bundles:</span>
          <span class="metric-val" id="cbba-bundles" style="font-size: 0.74rem; color: var(--c-grey-light); word-break: break-all;">None</span>
        </div>
      </div>

      <!-- Task Lifecycle (M3) -->
      <div class="card">
        <div class="card-title">
          <span>// 04. TASK LIFECYCLE & DISPATCH (M3)</span>
          <span id="task-status-badge" class="tag">ONLINE</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Pool Status:</span>
          <span class="metric-val" id="task-status" style="color: var(--c-orange);">M3 Lifecycle Active</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Allocation Progress:</span>
          <span class="metric-val"><span id="task-active" style="color: var(--c-orange);">0</span> / <span id="task-total">0</span> Assigned</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Pending Tasks:</span>
          <span class="metric-val" id="task-pending" style="color: var(--c-white);">0</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Completed Tasks:</span>
          <span class="metric-val" id="task-completed" style="color: var(--c-grey-muted);">0</span>
        </div>

        <!-- Task Table -->
        <table class="task-table">
          <thead>
            <tr>
              <th>TASK ID</th>
              <th>PRIORITY</th>
              <th>PICKUP</th>
              <th>DROPOFF</th>
              <th>WINNER</th>
            </tr>
          </thead>
          <tbody id="task-table-body">
            <tr><td colspan="5" style="text-align: center; color: var(--c-grey-muted);">Loading task registry...</td></tr>
          </tbody>
        </table>
      </div>

      <!-- Diagnostics & Safety Interlocks -->
      <div class="card">
        <div class="card-title">
          <span>// 05. SAFETY & TRANSPORT TELEMETRY</span>
          <span class="tag">INTERLOCKS</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Emergency Stop:</span>
          <span class="metric-val" id="safe-estop" style="color: var(--c-white);">[ NORMAL // DISENGAGED ]</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Safety Proximity Zone:</span>
          <span class="metric-val" id="safe-zone" style="color: var(--c-orange);">[ CLEAR // NO BREACH ]</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Transport State:</span>
          <span class="metric-val" id="net-state" style="color: var(--c-white);">ROS 2 Jazzy (HEALTHY)</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Discovered Nodes / Topics:</span>
          <span class="metric-val"><span id="net-nodes">0</span> Nodes / <span id="net-topics">0</span> Topics</span>
        </div>
      </div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('warehouse-canvas');
    const ctx = canvas.getContext('2d');

    function drawWarehouseMap(robots, tasks, cbba) {
      const W = canvas.width;
      const H = canvas.height;
      ctx.clearRect(0, 0, W, H);

      // Transform: Warehouse is 0..16m x 0..16m.
      const pad = 36;
      const mapW = W - 2 * pad;
      const mapH = H - 2 * pad;
      function toX(x) { return pad + (x / 16.0) * mapW; }
      function toY(y) { return H - pad - (y / 16.0) * mapH; }

      // Outer Perimeter Wall (Heavy Brutalist White/Grey Outline)
      ctx.fillStyle = '#0a0a0a';
      ctx.fillRect(pad, pad, mapW, mapH);
      ctx.strokeStyle = '#333333';
      ctx.lineWidth = 3;
      ctx.strokeRect(pad, pad, mapW, mapH);

      // Floor Grid Lines
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
      ctx.lineWidth = 1;
      for (let i = 1; i < 16; i++) {
        ctx.beginPath();
        ctx.moveTo(toX(i), toY(0));
        ctx.lineTo(toX(i), toY(16));
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(toX(0), toY(i));
        ctx.lineTo(toX(16), toY(i));
        ctx.stroke();
      }

      // Warehouse Aisle Lanes (Industrial Orange Dashed Lines)
      ctx.strokeStyle = 'rgba(255, 85, 0, 0.35)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 6]);
      [1.0, 3.0, 6.8, 9.2, 13.0, 15.0].forEach(lx => {
        ctx.beginPath();
        ctx.moveTo(toX(lx), toY(1.0));
        ctx.lineTo(toX(lx), toY(15.0));
        ctx.stroke();
      });
      ctx.setLineDash([]);

      // Storage Racks (Deep Charcoal with stark borders)
      ctx.fillStyle = '#1c1c1c';
      ctx.strokeStyle = '#444444';
      ctx.lineWidth = 1.5;
      [[4.5, 5.5], [4.5, 10.5], [11.5, 5.5], [11.5, 10.5]].forEach(r => {
        const rw = (1.2 / 16.0) * mapW;
        const rh = (3.0 / 16.0) * mapH;
        ctx.fillRect(toX(r[0]) - rw/2, toY(r[1]) - rh/2, rw, rh);
        ctx.strokeRect(toX(r[0]) - rw/2, toY(r[1]) - rh/2, rw, rh);

        // Rack cross hatching
        ctx.strokeStyle = '#2d2d2d';
        ctx.beginPath();
        ctx.moveTo(toX(r[0]) - rw/2, toY(r[1]) - rh/2);
        ctx.lineTo(toX(r[0]) + rw/2, toY(r[1]) + rh/2);
        ctx.moveTo(toX(r[0]) + rw/2, toY(r[1]) - rh/2);
        ctx.lineTo(toX(r[0]) - rw/2, toY(r[1]) + rh/2);
        ctx.stroke();
        ctx.strokeStyle = '#444444';
      });

      // Operational Zones: Pickups (Industrial Orange Boxes)
      ctx.fillStyle = 'rgba(255, 85, 0, 0.15)';
      ctx.strokeStyle = '#ff5500';
      ctx.lineWidth = 2;
      [[2, 2], [2, 13], [13, 2], [13, 13]].forEach(p => {
        const sz = (1.6 / 16.0) * mapW;
        ctx.fillRect(toX(p[0]) - sz/2, toY(p[1]) - sz/2, sz, sz);
        ctx.strokeRect(toX(p[0]) - sz/2, toY(p[1]) - sz/2, sz, sz);

        ctx.fillStyle = '#ff5500';
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('P', toX(p[0]), toY(p[1]) + 3);
        ctx.fillStyle = 'rgba(255, 85, 0, 0.15)';
      });

      // Dropoff Hub (Solid White/Grey Hub)
      ctx.fillStyle = '#1c1c1c';
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      const hubSz = (2.6 / 16.0) * mapW;
      ctx.fillRect(toX(8) - hubSz/2, toY(8) - hubSz/2, hubSz, hubSz);
      ctx.strokeRect(toX(8) - hubSz/2, toY(8) - hubSz/2, hubSz, hubSz);

      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 10px monospace';
      ctx.textAlign = 'center';
      ctx.fillText('DROPOFF', toX(8), toY(8) + 4);

      // Draw Tasks on Map
      if (tasks && tasks.tasks && tasks.tasks.length > 0) {
        tasks.tasks.forEach(t => {
          const px = toX(t.pickup[0]);
          const py = toY(t.pickup[1]);
          const dx = toX(t.dropoff[0]);
          const dy = toY(t.dropoff[1]);

          // Pickup marker (Orange dot)
          ctx.beginPath();
          ctx.arc(px, py, 4, 0, 2 * Math.PI);
          ctx.fillStyle = '#ff5500';
          ctx.fill();
          ctx.strokeStyle = '#000000';
          ctx.lineWidth = 1;
          ctx.stroke();

          // Dropoff marker (White square)
          ctx.fillStyle = '#ffffff';
          ctx.fillRect(dx - 3.5, dy - 3.5, 7, 7);
          ctx.strokeStyle = '#000000';
          ctx.lineWidth = 1;
          ctx.strokeRect(dx - 3.5, dy - 3.5, 7, 7);
        });
      }

      // Draw CBBA Allocation Route Lines (Bold Dashed Orange Lines)
      if (cbba && cbba.bundles && robots) {
        const botMap = {};
        robots.forEach(b => { botMap[b.robot_id] = b; });
        const taskMap = {};
        if (tasks && tasks.tasks) {
          tasks.tasks.forEach(t => { taskMap[t.id] = t; });
        }

        ctx.lineWidth = 2;
        ctx.setLineDash([5, 4]);

        Object.entries(cbba.bundles).forEach(([rId, bundle]) => {
          const bot = botMap[rId];
          if (bot && bundle.length > 0) {
            let startX = toX(bot.x);
            let startY = toY(bot.y);
            bundle.forEach(tId => {
              const t = taskMap[tId];
              if (t) {
                const targetX = toX(t.pickup[0]);
                const targetY = toY(t.pickup[1]);
                ctx.strokeStyle = '#ff5500';
                ctx.beginPath();
                ctx.moveTo(startX, startY);
                ctx.lineTo(targetX, targetY);
                ctx.stroke();
                startX = toX(t.dropoff[0]);
                startY = toY(t.dropoff[1]);
              }
            });
          }
        });
        ctx.setLineDash([]);
      }

      // Draw Robots (Heavy Brutalist Industrial Tokens)
      if (robots && robots.length > 0) {
        robots.forEach(bot => {
          const rx = toX(bot.x);
          const ry = toY(bot.y);
          const rRadius = 12;

          // Shadow
          ctx.beginPath();
          ctx.arc(rx + 2, ry + 2, rRadius, 0, 2 * Math.PI);
          ctx.fillStyle = '#000000';
          ctx.fill();

          // Body (Solid Industrial Orange)
          ctx.beginPath();
          ctx.arc(rx, ry, rRadius, 0, 2 * Math.PI);
          ctx.fillStyle = '#ff5500';
          ctx.fill();
          ctx.strokeStyle = '#000000';
          ctx.lineWidth = 2.5;
          ctx.stroke();

          // Orientation Heading Needle (Bright White line)
          const yaw = (bot.yaw_deg * Math.PI) / 180.0;
          ctx.beginPath();
          ctx.moveTo(rx, ry);
          ctx.lineTo(rx + Math.cos(yaw) * 17, ry - Math.sin(yaw) * 17);
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 2.5;
          ctx.stroke();

          // Center Label (R0..R4 in bold white)
          ctx.fillStyle = '#ffffff';
          ctx.font = 'bold 9px monospace';
          ctx.textAlign = 'center';
          ctx.fillText(bot.robot_id.replace('amr_', 'R'), rx, ry + 3);
        });
      }
    }

    async function pollState() {
      try {
        const res = await fetch('/api/state');
        if (!res.ok) return;
        const data = await res.json();

        // Header Metrics
        document.getElementById('sim-time').textContent = data.simulation.sim_time_sec + 's';
        document.getElementById('rtf-val').textContent = data.simulation.real_time_factor + 'x';
        document.getElementById('cpu-val').textContent = data.performance.host_cpu_percent + '%';
        document.getElementById('ram-val').textContent = data.performance.host_ram_used_mb + ' MB';

        // Fleet Telemetry Matrix
        const robots = data.fleet.robots || [];
        document.getElementById('fleet-count-badge').textContent = robots.length + ' AMRs ONLINE';

        if (robots.length > 0) {
          const container = document.getElementById('robot-container');
          container.innerHTML = robots.map(r => `
            <div class="robot-card">
              <div class="robot-header">
                <span class="robot-name">${r.robot_id.toUpperCase()}</span>
                <span class="status-badge status-${r.status}">[ ${r.status} ]</span>
              </div>
              <div class="robot-kv"><span class="k">POSE:</span><span class="v">(${r.x}, ${r.y})</span></div>
              <div class="robot-kv"><span class="k">HEADING:</span><span class="v">${r.yaw_deg}&deg;</span></div>
              <div class="robot-kv"><span class="k">SPEED:</span><span class="v">${r.linear_speed} m/s</span></div>
              <div class="robot-kv"><span class="k">ODOM:</span><span class="v">${r.distance_m} m</span></div>
              <div class="robot-kv"><span class="k">LIDAR:</span><span class="v">${r.lidar_hz} Hz</span></div>
            </div>
          `).join('');
        }

        // 2D Warehouse Map
        drawWarehouseMap(robots, data.tasks, data.cbba);

        // CBBA Consensus (M4)
        if (data.cbba) {
          const cb = data.cbba;
          const statusEl = document.getElementById('cbba-status');
          const calloutEl = document.getElementById('cbba-callout');
          const bidsBadge = document.getElementById('cbba-bids-badge');

          if (cb.is_converged) {
            statusEl.textContent = 'CONVERGED // CONSENSUS REACHED';
            calloutEl.className = 'callout-box callout-converged';
          } else {
            statusEl.textContent = 'NEGOTIATING // BIDS EXCHANGING';
            calloutEl.className = 'callout-box';
            calloutEl.style.background = '#333333';
            calloutEl.style.color = '#ffffff';
          }

          bidsBadge.textContent = (cb.bids_count || 0) + ' BIDS';
          document.getElementById('cbba-bids').textContent = cb.bids_count || 0;
          document.getElementById('perf-makespan').textContent = (cb.makespan_sec || 0.0) + 's';

          const bundlesStr = Object.entries(cb.bundles || {})
            .map(([r, b]) => `[${r.toUpperCase()}: ${b.join(', ')}]`)
            .join(' ') || 'None';
          document.getElementById('cbba-bundles').textContent = bundlesStr;
        }

        // Task Lifecycle (M3)
        if (data.tasks) {
          const ts = data.tasks;
          document.getElementById('task-status').textContent = ts.status;
          document.getElementById('task-total').textContent = ts.total || 0;
          document.getElementById('task-pending').textContent = ts.pending || 0;
          document.getElementById('task-active').textContent = ts.active || 0;
          document.getElementById('task-completed').textContent = ts.completed || 0;

          // Populate Task Table
          const tbody = document.getElementById('task-table-body');
          if (ts.tasks && ts.tasks.length > 0) {
            tbody.innerHTML = ts.tasks.map(t => {
              const pStr = t.priority === 3 ? 'CRITICAL' : (t.priority === 2 ? 'NORMAL' : 'LOW');
              return `
                <tr>
                  <td class="task-id">${t.id}</td>
                  <td class="priority-${pStr}">${pStr}</td>
                  <td>(${t.pickup[0]}, ${t.pickup[1]})</td>
                  <td>(${t.dropoff[0]}, ${t.dropoff[1]})</td>
                  <td class="task-winner">${(t.robot || 'UNASSIGNED').toUpperCase()}</td>
                </tr>
              `;
            }).join('');
          }
        }

        // Network
        document.getElementById('net-nodes').textContent = data.network.ros2_nodes_count;
        document.getElementById('net-topics').textContent = data.network.ros2_topics_count;
        document.getElementById('net-state').textContent = data.network.transport_state;

        // Safety
        document.getElementById('safe-zone').textContent = '[ ' + data.safety.active_safety_zone + ' ]';
      } catch (err) {
        console.error('Telemetry fetch error:', err);
      }
    }

    setInterval(pollState, 500);
    pollState();
  </script>
</body>
</html>"""


class DashboardHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
    """Zero-dependency HTTP request handler for the fleet dashboard."""

    monitor_node: Optional[FleetMonitorNode] = None

    def log_message(self, format, *args):
        # Suppress routine GET logs to keep console clean
        pass

    def do_GET(self) -> None:
        if self.path == '/' or self.path.startswith('/index'):
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode('utf-8'))
        elif self.path == '/api/state':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            data = self.monitor_node.get_fleet_summary() if self.monitor_node else {}
            self.wfile.write(json.dumps(data).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()


def run_tui(node: FleetMonitorNode) -> None:
    """Rich terminal TUI loop."""
    try:
        from rich.console import Console
        from rich.table import Table
        from rich.live import Live
        from rich.panel import Panel
        from rich.layout import Layout
    except ImportError:
        print("[WARN] 'rich' is not available. Falling back to simple stdout print.")
        while rclpy.ok():
            data = node.get_fleet_summary()
            print(f"\r[FLEET] Robots: {data['fleet']['total_discovered']} | Sim Time: {data['simulation']['sim_time_sec']}s | RTF: {data['simulation']['real_time_factor']}x | CPU: {data['performance']['host_cpu_percent']}%", end="")
            time.sleep(1.0)
        return

    console = Console()
    with Live(console=console, screen=True, refresh_per_second=2) as live:
        while rclpy.ok():
            data = node.get_fleet_summary()
            
            layout = Layout()
            layout.split_column(
                Layout(name="header", size=3),
                Layout(name="main"),
                Layout(name="footer", size=5),
            )

            # Header
            header_text = f"[bold cyan]NRDAS AMR FLEET MONITOR[/bold cyan] | Sim Time: [green]{data['simulation']['sim_time_sec']}s[/green] | RTF: [green]{data['simulation']['real_time_factor']}x[/green] | CPU: {data['performance']['host_cpu_percent']}% | RAM: {data['performance']['host_ram_used_mb']}MB"
            layout["header"].update(Panel(header_text, border_style="blue"))

            # Table of Robots
            table = Table(title=f"Discovered AMRs ({data['fleet']['total_discovered']})", expand=True)
            table.add_column("Robot ID", style="cyan", justify="left")
            table.add_column("Status", justify="center")
            table.add_column("Pose (X, Y)", justify="right")
            table.add_column("Yaw", justify="right")
            table.add_column("Speed (m/s)", justify="right")
            table.add_column("Dist (m)", justify="right")
            table.add_column("LiDAR (Hz)", justify="right")
            table.add_column("Min Range", justify="right")

            for r in data['fleet']['robots']:
                st_color = "green" if r['status'] == 'ACTIVE' else ("yellow" if r['status'] == 'IDLE' else "red")
                table.add_row(
                    r['robot_id'],
                    f"[{st_color}]{r['status']}[/{st_color}]",
                    f"({r['x']}, {r['y']})",
                    f"{r['yaw_deg']}°",
                    f"{r['linear_speed']}",
                    f"{r['distance_m']}",
                    f"{r['lidar_hz']}",
                    f"{r['min_obstacle_m'] if r['min_obstacle_m'] else 'N/A'}",
                )
            layout["main"].update(table)

            # Footer
            t_info = (
                f"{data['tasks']['status']} "
                f"(Total: {data['tasks']['total']}, Pending: {data['tasks']['pending']}, "
                f"Active: {data['tasks']['active']}, Done: {data['tasks']['completed']})"
            )
            footer_text = (
                f"[bold]TASKS:[/bold] {t_info} | "
                f"[bold]MAPF:[/bold] {data['performance']['mapf_planning_latency']}\n"
                f"[bold]SAFETY:[/bold] [green]{data['safety']['active_safety_zone']}[/green] | "
                f"[bold]E-STOP:[/bold] {data['safety']['estop_status']} | "
                f"[bold]MIDDLEWARE:[/bold] {data['network']['ros2_topics_count']} topics "
                f"across {data['network']['ros2_nodes_count']} nodes"
            )
            layout["footer"].update(Panel(footer_text, border_style="green"))

            live.update(layout)
            time.sleep(0.5)


def main() -> None:
    parser = argparse.ArgumentParser(description="NRDAS Fleet Observability Dashboard")
    parser.add_argument('--port', type=int, default=8080, help="Web dashboard port (default: 8080)")
    parser.add_argument('--tui', action='store_true', help="Run terminal Rich TUI in current terminal")
    parser.add_argument('--no-web', action='store_true', help="Disable web dashboard server")
    args = parser.parse_args()

    rclpy.init()
    node = FleetMonitorNode()
    DashboardHTTPRequestHandler.monitor_node = node

    # Start Web Server if requested
    if not args.no_web:
        class ThreadedTCPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        try:
            httpd = ThreadedTCPServer(('0.0.0.0', args.port), DashboardHTTPRequestHandler)
            server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
            server_thread.start()
            print(f"[INFO] Fleet Dashboard running at http://localhost:{args.port}")
        except Exception as e:
            print(f"[WARN] Failed to bind Web Dashboard on port {args.port}: {e}")

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if args.tui:
        run_tui(node)
    else:
        print("[INFO] Fleet Monitor Node active. Press Ctrl+C to exit.")
        try:
            while rclpy.ok():
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
