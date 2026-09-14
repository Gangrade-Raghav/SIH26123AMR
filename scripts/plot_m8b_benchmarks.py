#!/usr/bin/env python3
"""
Plot M8B Adaptive Compute & Degradation-Aware Coordination Analytical Figures.

Reads raw and aggregated benchmark results from `results/m8b/` and generates
a publication-grade 6-panel analytical figure evaluating:
1. Fleet Throughput (tasks/min) across Compute Modes & Comm Profiles
2. Replan Volume across Compute Modes
3. Planning Latency (Mean & P95 ms)
4. Host CPU & Memory Utilization
5. Mode Occupancy Distribution (% time in LOW / NORMAL / HIGH)
6. Spatial Safety: Minimum Separation Distance (m) & Safety Interventions
"""

import argparse
import glob
import json
import os
import sys
from typing import Any, Dict, List

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_all_m8b_trials(results_dir: str) -> List[Dict[str, Any]]:
    """Load all raw and aggregated trial files."""
    raw_files = glob.glob(os.path.join(results_dir, "raw", "exp_m8b_*.json"))
    trials = []
    for f in sorted(raw_files):
        try:
            with open(f, 'r', encoding='utf-8') as fp:
                data = json.load(fp)
                trials.append(data)
        except Exception as e:
            print(f"[WARN] Failed to load {f}: {e}")
    return trials


def plot_m8b_analysis(trials: List[Dict[str, Any]], output_path: str) -> None:
    """Generate 6-panel analytical evaluation figure."""
    if not trials:
        print("[ERROR] No trial data available for M8B plotting.")
        return

    # Group trials by condition: (workload, comm_profile, compute_mode)
    groups: Dict[str, List[Dict[str, Any]]] = {}
    for tr in trials:
        meta = tr.get('metadata', {})
        w = meta.get('workload_size', 15)
        comm = meta.get('communication_profile', 'NORMAL')
        mode = meta.get('compute_mode', 'NORMAL')
        key = f"W{w}_{comm}_{mode}"
        if key not in groups:
            groups[key] = []
        groups[key].append(tr)

    labels = sorted(list(groups.keys()))
    n_groups = len(labels)
    x_pos = np.arange(n_groups)

    # Compute averages per condition
    throughputs = []
    completed = []
    replans = []
    lat_means = []
    lat_p95s = []
    cpu_means = []
    ram_means = []
    min_dists = []
    safety_brakes = []
    occ_low = []
    occ_norm = []
    occ_high = []

    for k in labels:
        trs = groups[k]
        m_list = [t.get('metrics', {}) for t in trs]
        n = len(m_list)

        throughputs.append(np.mean([m.get('throughput_tasks_per_min', 0.0) for m in m_list]))
        completed.append(np.mean([m.get('completed_tasks', 0) for m in m_list]))
        replans.append(np.mean([m.get('replan_count', 0) for m in m_list]))
        lat_means.append(np.mean([m.get('planning_latency_mean_ms', 0.0) or 0.0 for m in m_list]))
        lat_p95s.append(np.mean([m.get('planning_latency_p95_ms', 0.0) or 0.0 for m in m_list]))
        cpu_means.append(np.mean([m.get('cpu_utilization_mean_pct', 0.0) or 0.0 for m in m_list]))
        ram_means.append(np.mean([m.get('ram_utilization_mean_mb', 0.0) or 0.0 for m in m_list]))
        min_dists.append(np.mean([m.get('minimum_inter_robot_distance_m', 1.5) for m in m_list]))
        safety_brakes.append(np.mean([m.get('safety_brake_interventions', 0) for m in m_list]))
        occ_low.append(np.mean([m.get('mode_occupancy_low_pct', 0.0) for m in m_list]))
        occ_norm.append(np.mean([m.get('mode_occupancy_normal_pct', 100.0) for m in m_list]))
        occ_high.append(np.mean([m.get('mode_occupancy_high_pct', 0.0) for m in m_list]))

    fig = plt.figure(figsize=(20, 12))
    fig.suptitle(
        "NRDAS M8B: Adaptive Compute & Degradation-Aware Coordination Performance\n"
        "(Real Gazebo Harmonic 5-AMR Simulation — Comparing Fixed LOW, NORMAL, HIGH, and ADAPTIVE Policy)",
        fontsize=14,
        fontweight='bold',
    )

    clean_labels = [
        k.replace('W15_', '15T: ').replace('W30_', '30T: ').replace('_', ' ')
        for k in labels
    ]

    # Panel 1: Throughput & Task Completion
    ax1 = fig.add_subplot(2, 3, 1)
    color1 = '#2ecc71'
    ax1.bar(x_pos - 0.2, throughputs, width=0.4, label='Throughput (tasks/min)', color=color1, alpha=0.85)
    ax1.set_ylabel('Throughput (tasks/min)', color=color1, fontweight='bold')
    ax1.set_title('Fleet Throughput & Delivered Tasks', fontweight='bold')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(clean_labels, rotation=25, ha='right', fontsize=8)
    ax1.grid(axis='y', alpha=0.3, linestyle='--')

    ax1_twin = ax1.twinx()
    color2 = '#3498db'
    ax1_twin.bar(x_pos + 0.2, completed, width=0.4, label='Completed Tasks', color=color2, alpha=0.85)
    ax1_twin.set_ylabel('Completed Tasks', color=color2, fontweight='bold')

    # Panel 2: Replan Count & Frequency
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.bar(x_pos, replans, color='#9b59b6', alpha=0.85)
    ax2.set_title('Total Rolling Horizon Replanning Cycles', fontweight='bold')
    ax2.set_ylabel('Replan Cycles across Fleet')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(clean_labels, rotation=25, ha='right', fontsize=8)
    ax2.grid(axis='y', alpha=0.3, linestyle='--')
    for i, v in enumerate(replans):
        ax2.text(i, v + 2, f"{int(v)}", ha='center', fontsize=8)

    # Panel 3: Planning Latency
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.bar(x_pos - 0.2, lat_means, width=0.4, label='Mean Latency (ms)', color='#e67e22', alpha=0.85)
    ax3.bar(x_pos + 0.2, lat_p95s, width=0.4, label='P95 Latency (ms)', color='#d35400', alpha=0.85)
    ax3.set_title('Single-Robot Planning Latency', fontweight='bold')
    ax3.set_ylabel('Latency (ms)')
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(clean_labels, rotation=25, ha='right', fontsize=8)
    ax3.legend(loc='upper left', fontsize=8)
    ax3.grid(axis='y', alpha=0.3, linestyle='--')

    # Panel 4: Host CPU & RAM Utilization
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.bar(x_pos - 0.2, cpu_means, width=0.4, label='CPU Mean (%)', color='#e74c3c', alpha=0.85)
    ax4.set_ylabel('CPU Utilization (%)', color='#e74c3c', fontweight='bold')
    ax4.set_title('Host System Compute Utilization', fontweight='bold')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(clean_labels, rotation=25, ha='right', fontsize=8)
    ax4.grid(axis='y', alpha=0.3, linestyle='--')

    ax4_twin = ax4.twinx()
    ax4_twin.plot(x_pos, ram_means, color='#1abc9c', marker='o', linewidth=2, label='RAM (MB)')
    ax4_twin.set_ylabel('RAM Utilization (MB)', color='#1abc9c', fontweight='bold')

    # Panel 5: Mode Occupancy Distribution (Stacked Bar)
    ax5 = fig.add_subplot(2, 3, 5)
    p_low = ax5.bar(x_pos, occ_low, label='LOW Mode', color='#34495e', alpha=0.85)
    p_norm = ax5.bar(x_pos, occ_norm, bottom=occ_low, label='NORMAL Mode', color='#3498db', alpha=0.85)
    bot_high = [l + n for l, n in zip(occ_low, occ_norm)]
    p_high = ax5.bar(x_pos, occ_high, bottom=bot_high, label='HIGH Mode', color='#e74c3c', alpha=0.85)
    ax5.set_title('Compute Mode Occupancy (%)', fontweight='bold')
    ax5.set_ylabel('Occupancy (% of mission time)')
    ax5.set_ylim(0, 105)
    ax5.set_xticks(x_pos)
    ax5.set_xticklabels(clean_labels, rotation=25, ha='right', fontsize=8)
    ax5.legend(loc='upper right', fontsize=8)
    ax5.grid(axis='y', alpha=0.3, linestyle='--')

    # Panel 6: Spatial Safety Distance & Interventions
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.bar(x_pos - 0.2, min_dists, width=0.4, label='Min Distance (m)', color='#16a085', alpha=0.85)
    ax6.axhline(0.35, color='red', linestyle='--', linewidth=1.5, label='Safety Threshold (0.35m)')
    ax6.set_ylabel('Minimum Separation (m)', color='#16a085', fontweight='bold')
    ax6.set_title('Spatial Safety & Reactive Interventions', fontweight='bold')
    ax6.set_xticks(x_pos)
    ax6.set_xticklabels(clean_labels, rotation=25, ha='right', fontsize=8)
    ax6.grid(axis='y', alpha=0.3, linestyle='--')

    ax6_twin = ax6.twinx()
    ax6_twin.plot(x_pos, safety_brakes, color='#e67e22', marker='s', linewidth=2, label='Safety Brakes')
    ax6_twin.set_ylabel('Safety Brake Interventions', color='#e67e22', fontweight='bold')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    plt.savefig(output_path, dpi=200)
    plt.close()
    print(f"[PLOT GENERATED] Saved M8B analytical figure to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Plot M8B Adaptive Compute Benchmarks")
    parser.add_argument('--results-dir', type=str, default='results/m8b', help="Directory containing m8b results")
    parser.add_argument('--output', type=str, default='docs/images/m8b_adaptive_compute_scaling.png', help="Output path")
    args = parser.parse_args()

    trials = load_all_m8b_trials(args.results_dir)
    print(f"Loaded {len(trials)} M8B trial files from {args.results_dir}.")
    plot_m8b_analysis(trials, args.output)


if __name__ == '__main__':
    main()
