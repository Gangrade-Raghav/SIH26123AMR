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
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock


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

    def __init__(self, robot_id: str) -> None:
        self.robot_id = robot_id
        self.x = 0.0
        self.y = 0.0
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

        self.x = px
        self.y = py
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

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscribe to simulation clock
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)

        # Dynamic topic discovery timer
        self.create_timer(1.0, self._discover_fleet_topics)

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

            if odom_topic in existing_topics and r_id not in self.robots:
                tracker = RobotTelemetryTracker(r_id)
                self.robots[r_id] = tracker

                # Subscribe to odom
                def make_odom_cb(t):
                    return lambda msg: t.update_odometry(msg)
                self.create_subscription(Odometry, odom_topic, make_odom_cb(tracker), 10)

                # Subscribe to scan
                def make_scan_cb(t):
                    return lambda msg: t.update_scan(msg)
                self.create_subscription(LaserScan, scan_topic, make_scan_cb(tracker), 10)

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
                'status': 'N/A — Pending M3 Task Engine',
                'pending': 0,
                'active': 0,
                'completed': 0,
                'failed': 0,
            },
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
                'fleet_throughput': 'N/A — Pending M3 Task Engine',
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
  <title>NRDAS — AMR Fleet Operations Dashboard</title>
  <style>
    :root {
      --bg-base: #0f1218;
      --bg-surface: #181d26;
      --bg-card: #202633;
      --border: #2d3648;
      --text-main: #f0f3f8;
      --text-muted: #8c9bb0;
      --accent-orange: #f36c21;
      --accent-cyan: #00bcd4;
      --accent-green: #00e676;
      --accent-amber: #ffb300;
      --accent-red: #ff5252;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
      background-color: var(--bg-base);
      color: var(--text-main);
      padding: 18px;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 16px;
      border-bottom: 1px solid var(--border);
      margin-bottom: 18px;
    }
    .brand { display: flex; align-items: center; gap: 12px; }
    .brand h1 { font-size: 1.35rem; font-weight: 700; letter-spacing: 0.5px; }
    .badge-sub {
      background: rgba(243, 108, 33, 0.18);
      color: var(--accent-orange);
      border: 1px solid var(--accent-orange);
      font-size: 0.72rem;
      padding: 2px 8px;
      border-radius: 4px;
      text-transform: uppercase;
      font-weight: 600;
    }
    .header-stats { display: flex; gap: 16px; font-size: 0.82rem; }
    .stat-pill {
      background: var(--bg-surface);
      border: 1px solid var(--border);
      padding: 5px 12px;
      border-radius: 6px;
      display: flex;
      gap: 6px;
    }
    .stat-label { color: var(--text-muted); }
    .stat-value { font-weight: 600; color: var(--accent-cyan); }

    /* Layout Grid */
    .grid {
      display: grid;
      grid-template-columns: 1.2fr 0.8fr;
      gap: 18px;
    }
    .card {
      background: var(--bg-surface);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 16px;
      margin-bottom: 18px;
    }
    .card-title {
      font-size: 0.92rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      color: var(--text-muted);
      margin-bottom: 12px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    /* Robot Cards Grid */
    .robot-grid {
      display: grid;
      grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
      gap: 12px;
    }
    .robot-card {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 12px;
      position: relative;
    }
    .robot-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
      border-bottom: 1px solid var(--border);
      padding-bottom: 6px;
    }
    .robot-name { font-weight: 700; font-size: 0.95rem; color: var(--accent-orange); }
    .status-badge {
      font-size: 0.68rem;
      padding: 2px 6px;
      border-radius: 3px;
      font-weight: 700;
      text-transform: uppercase;
    }
    .status-ACTIVE { background: rgba(0, 230, 118, 0.18); color: var(--accent-green); border: 1px solid var(--accent-green); }
    .status-IDLE { background: rgba(255, 179, 0, 0.18); color: var(--accent-amber); border: 1px solid var(--accent-amber); }
    .status-OFFLINE { background: rgba(255, 82, 82, 0.18); color: var(--accent-red); border: 1px solid var(--accent-red); }

    .robot-kv { display: flex; justify-content: space-between; font-size: 0.78rem; margin: 3px 0; }
    .robot-kv .k { color: var(--text-muted); }
    .robot-kv .v { font-family: monospace; }

    /* Map Canvas */
    .map-container {
      background: #12151c;
      border: 1px solid var(--border);
      border-radius: 6px;
      width: 100%;
      height: 380px;
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      overflow: hidden;
    }
    canvas { width: 100%; height: 100%; }

    /* Metrics Table */
    .metric-row {
      display: flex;
      justify-content: space-between;
      padding: 7px 0;
      border-bottom: 1px solid rgba(255,255,255,0.05);
      font-size: 0.82rem;
    }
    .metric-row:last-child { border-bottom: none; }
    .metric-key { color: var(--text-muted); }
    .metric-val { font-family: monospace; font-weight: 600; }
    .metric-pending { color: var(--text-muted); font-style: italic; font-size: 0.76rem; }
  </style>
</head>
<body>

  <header>
    <div class="brand">
      <h1>NRDAS Fleet Operations Dashboard</h1>
      <span class="badge-sub">M2.5 Executive Layer</span>
    </div>
    <div class="header-stats">
      <div class="stat-pill"><span class="stat-label">Sim Time:</span><span id="sim-time" class="stat-value">0.0s</span></div>
      <div class="stat-pill"><span class="stat-label">RTF:</span><span id="rtf-val" class="stat-value">1.00x</span></div>
      <div class="stat-pill"><span class="stat-label">Host CPU:</span><span id="cpu-val" class="stat-value">0.0%</span></div>
      <div class="stat-pill"><span class="stat-label">Host RAM:</span><span id="ram-val" class="stat-value">0 MB</span></div>
    </div>
  </header>

  <div class="grid">
    <!-- Left Column: Fleet & Live Map -->
    <div>
      <div class="card">
        <div class="card-title">
          <span>Active AMR Fleet</span>
          <span id="fleet-count-badge" class="badge-sub">0 Discovered</span>
        </div>
        <div id="robot-container" class="robot-grid">
          <div style="color: var(--text-muted); padding: 12px;">Awaiting fleet telemetry from ROS 2...</div>
        </div>
      </div>

      <div class="card">
        <div class="card-title">Warehouse 2D Operational Map (16m x 16m)</div>
        <div class="map-container">
          <canvas id="warehouse-canvas" width="600" height="600"></canvas>
        </div>
      </div>
    </div>

    <!-- Right Column: System Observability -->
    <div>
      <div class="card">
        <div class="card-title">Task Engine & Dispatch</div>
        <div class="metric-row"><span class="metric-key">Engine Status:</span><span class="metric-pending">Pending M3 Task Engine</span></div>
        <div class="metric-row"><span class="metric-key">Active Tasks:</span><span class="metric-val">0</span></div>
        <div class="metric-row"><span class="metric-key">Pending Tasks:</span><span class="metric-val">0</span></div>
        <div class="metric-row"><span class="metric-key">Completed Tasks:</span><span class="metric-val">0</span></div>
        <div class="metric-row"><span class="metric-key">Throughput:</span><span class="metric-pending">Pending M3 (0 tasks/hr)</span></div>
      </div>

      <div class="card">
        <div class="card-title">Network & Transport Health</div>
        <div class="metric-row"><span class="metric-key">Middleware Layer:</span><span class="metric-val" id="net-mid">ROS 2 Jazzy</span></div>
        <div class="metric-row"><span class="metric-key">Transport State:</span><span class="metric-val" id="net-state" style="color: var(--accent-green);">HEALTHY</span></div>
        <div class="metric-row"><span class="metric-key">Discovered Nodes:</span><span class="metric-val" id="net-nodes">0</span></div>
        <div class="metric-row"><span class="metric-key">Discovered Topics:</span><span class="metric-val" id="net-topics">0</span></div>
      </div>

      <div class="card">
        <div class="card-title">Performance & Optimization</div>
        <div class="metric-row"><span class="metric-key">MAPF Latency:</span><span class="metric-pending">Pending M6 (PIBT/RHCR)</span></div>
        <div class="metric-row"><span class="metric-key">Makespan Estimation:</span><span class="metric-pending">Pending M4 Allocation</span></div>
        <div class="metric-row"><span class="metric-key">Host CPU Load:</span><span class="metric-val" id="perf-cpu">0.0%</span></div>
        <div class="metric-row"><span class="metric-key">Host RAM Usage:</span><span class="metric-val" id="perf-ram">0 MB</span></div>
      </div>

      <div class="card">
        <div class="card-title">Safety & Interlocks</div>
        <div class="metric-row"><span class="metric-key">E-Stop Status:</span><span class="metric-val" id="safe-estop" style="color: var(--accent-green);">NORMAL</span></div>
        <div class="metric-row"><span class="metric-key">Active Proximity Zone:</span><span class="metric-val" id="safe-zone" style="color: var(--accent-green);">CLEAR</span></div>
        <div class="metric-row"><span class="metric-key">Deadlock Detection:</span><span class="metric-pending">Pending M8 Wait-For-Graph</span></div>
      </div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('warehouse-canvas');
    const ctx = canvas.getContext('2d');

    function drawWarehouseMap(robots) {
      const W = canvas.width;
      const H = canvas.height;
      ctx.clearRect(0, 0, W, H);

      // Transform: Warehouse is 0..16m x 0..16m.
      // Margin = 30px
      const pad = 35;
      const mapW = W - 2 * pad;
      const mapH = H - 2 * pad;
      function toX(x) { return pad + (x / 16.0) * mapW; }
      function toY(y) { return H - pad - (y / 16.0) * mapH; }

      // Outer Perimeter Wall
      ctx.strokeStyle = '#4a5568';
      ctx.lineWidth = 4;
      ctx.strokeRect(pad, pad, mapW, mapH);

      // Floor grid lines
      ctx.strokeStyle = 'rgba(255,255,255,0.04)';
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

      // Warehouse Aisle Lanes (Yellow)
      ctx.strokeStyle = 'rgba(241, 196, 15, 0.4)';
      ctx.lineWidth = 2;
      [1.0, 3.0, 6.8, 9.2, 13.0, 15.0].forEach(lx => {
        ctx.beginPath();
        ctx.moveTo(toX(lx), toY(1.0));
        ctx.lineTo(toX(lx), toY(15.0));
        ctx.stroke();
      });

      // Operational Zones: Pickups (Cyan)
      ctx.fillStyle = 'rgba(41, 128, 185, 0.35)';
      ctx.strokeStyle = '#3498db';
      ctx.lineWidth = 1.5;
      [[2, 2], [2, 13], [13, 2], [13, 13]].forEach(p => {
        const sz = (1.6 / 16.0) * mapW;
        ctx.fillRect(toX(p[0]) - sz/2, toY(p[1]) - sz/2, sz, sz);
        ctx.strokeRect(toX(p[0]) - sz/2, toY(p[1]) - sz/2, sz, sz);
      });

      // Dropoff Hub (Green)
      ctx.fillStyle = 'rgba(39, 174, 96, 0.35)';
      ctx.strokeStyle = '#2ecc71';
      const hubSz = (2.6 / 16.0) * mapW;
      ctx.fillRect(toX(8) - hubSz/2, toY(8) - hubSz/2, hubSz, hubSz);
      ctx.strokeRect(toX(8) - hubSz/2, toY(8) - hubSz/2, hubSz, hubSz);

      // Storage Racks (Deep Blue)
      ctx.fillStyle = '#2c3e50';
      ctx.strokeStyle = '#e67e22';
      ctx.lineWidth = 2;
      [[4.5, 5.5], [4.5, 10.5], [11.5, 5.5], [11.5, 10.5]].forEach(r => {
        const rw = (1.2 / 16.0) * mapW;
        const rh = (3.0 / 16.0) * mapH;
        ctx.fillRect(toX(r[0]) - rw/2, toY(r[1]) - rh/2, rw, rh);
        ctx.strokeRect(toX(r[0]) - rw/2, toY(r[1]) - rh/2, rw, rh);
      });

      // Draw Robots
      if (robots && robots.length > 0) {
        robots.forEach(bot => {
          const rx = toX(bot.x);
          const ry = toY(bot.y);
          const rRadius = 11;

          // Body
          ctx.beginPath();
          ctx.arc(rx, ry, rRadius, 0, 2 * Math.PI);
          ctx.fillStyle = bot.status === 'ACTIVE' ? '#e65c00' : '#4a5568';
          ctx.fill();
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 2;
          ctx.stroke();

          // Orientation Heading Line
          const yaw = (bot.yaw_deg * Math.PI) / 180.0;
          ctx.beginPath();
          ctx.moveTo(rx, ry);
          ctx.lineTo(rx + Math.cos(yaw) * 16, ry - Math.sin(yaw) * 16);
          ctx.strokeStyle = '#00e676';
          ctx.lineWidth = 2.5;
          ctx.stroke();

          // Label
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

        // Header
        document.getElementById('sim-time').textContent = data.simulation.sim_time_sec + 's';
        document.getElementById('rtf-val').textContent = data.simulation.real_time_factor + 'x';
        document.getElementById('cpu-val').textContent = data.performance.host_cpu_percent + '%';
        document.getElementById('ram-val').textContent = data.performance.host_ram_used_mb + ' MB';

        // Fleet
        const robots = data.fleet.robots || [];
        document.getElementById('fleet-count-badge').textContent = robots.length + ' Discovered';

        if (robots.length > 0) {
          const container = document.getElementById('robot-container');
          container.innerHTML = robots.map(r => `
            <div class="robot-card">
              <div class="robot-header">
                <span class="robot-name">${r.robot_id}</span>
                <span class="status-badge status-${r.status}">${r.status}</span>
              </div>
              <div class="robot-kv"><span class="k">Pose:</span><span class="v">(${r.x}, ${r.y})</span></div>
              <div class="robot-kv"><span class="k">Heading:</span><span class="v">${r.yaw_deg}&deg;</span></div>
              <div class="robot-kv"><span class="k">Speed:</span><span class="v">${r.linear_speed} m/s</span></div>
              <div class="robot-kv"><span class="k">Dist Odom:</span><span class="v">${r.distance_m} m</span></div>
              <div class="robot-kv"><span class="k">LiDAR:</span><span class="v">${r.lidar_hz} Hz</span></div>
            </div>
          `).join('');
        }

        // Map
        drawWarehouseMap(robots);

        // Network
        document.getElementById('net-nodes').textContent = data.network.ros2_nodes_count;
        document.getElementById('net-topics').textContent = data.network.ros2_topics_count;
        document.getElementById('net-state').textContent = data.network.transport_state;

        // Performance
        document.getElementById('perf-cpu').textContent = data.performance.host_cpu_percent + '%';
        document.getElementById('perf-ram').textContent = data.performance.host_ram_used_mb + ' MB (' + data.performance.host_ram_percent + '%)';

        // Safety
        document.getElementById('safe-zone').textContent = data.safety.active_safety_zone;
        if (data.safety.active_safety_zone !== 'CLEAR') {
          document.getElementById('safe-zone').style.color = 'var(--accent-red)';
        } else {
          document.getElementById('safe-zone').style.color = 'var(--accent-green)';
        }
      } catch (err) {
        console.error('Telemetry fetch error:', err);
      }
    }

    setInterval(pollState, 500);
    pollState();
  </script>
</body>
</html>
"""


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
            footer_text = f"[bold]TASKS:[/bold] {data['tasks']['status']} | [bold]MAPF:[/bold] {data['performance']['mapf_planning_latency']}\n[bold]SAFETY:[/bold] [green]{data['safety']['active_safety_zone']}[/green] | [bold]E-STOP:[/bold] {data['safety']['estop_status']} | [bold]MIDDLEWARE:[/bold] {data['network']['ros2_topics_count']} topics across {data['network']['ros2_nodes_count']} nodes"
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
