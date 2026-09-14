#!/usr/bin/env python3
"""
Plot M7 Communication Degradation and Fleet Resilience Empirical Curves.

Reads empirical Gazebo simulation data from `gazebo_m7_results.json` and generates
a publication-quality 6-panel analytical figure visualizing fleet resilience:
1. Fleet Throughput (tasks/min) vs Packet Loss
2. Mission Makespan (sec) vs Latency & Delay Variation
3. Packet Delivery Ratio (%) across Impairment Profiles
4. Replanning and Space-Time Conflict Frequency
5. Minimum Observed Physical Distance vs 0.35m Collision Threshold
6. Stale Information Transitions & Reconnection Reconciliation Events
"""

import argparse
import json
import os
import sys
from typing import Any, Dict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_resilience_curves(data: Dict[str, Any], output_path: str) -> None:
    """Generate 6-panel visualization of communication degradation metrics."""
    profiles = data.get('profiles', {})
    if not profiles:
        print("[ERROR] No profile data found in input json")
        return

    profile_names = list(profiles.keys())
    p_data = list(profiles.values())

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(
        "NRDAS M7: Empirical Fleet Resilience Under Communication Degradation\n"
        "(5-AMR Gazebo Harmonic Suite Across Bounded 55.0s Horizons — Discrete Observed Events)",
        fontsize=14,
        fontweight='bold',
    )

    colors = [
        '#2ecc71', '#3498db', '#2980b9', '#9b59b6', '#e67e22',
        '#d35400', '#e74c3c', '#c0392b', '#1abc9c', '#34495e',
    ]

    # Panel 1: Throughput vs Profile
    ax1 = axes[0, 0]
    throughputs = [p['fleet_throughput_tasks_per_min'] for p in p_data]
    bars1 = ax1.bar(profile_names, throughputs, color=colors[:len(profile_names)])
    ax1.set_title("Fleet Throughput (Tasks / Min, 55s Horizon)", fontweight='bold')
    ax1.set_ylabel("Tasks / Min")
    ax1.tick_params(axis='x', rotation=45)
    ax1.grid(axis='y', linestyle='--', alpha=0.6)
    for bar in bars1:
        yval = bar.get_height()
        ax1.text(
            bar.get_x() + bar.get_width() / 2.0,
            yval + 0.05,
            f"{yval:.2f}",
            ha='center',
            va='bottom',
            fontsize=8,
        )

    # Panel 2: Makespan vs Latency / Jitter
    ax2 = axes[0, 1]
    makespans = [p['makespan_sec'] for p in p_data]
    ax2.plot(profile_names, makespans, marker='o', color='#e74c3c', linewidth=2, markersize=6)
    ax2.set_title("Workload Makespan / Cutoff Horizon (s)", fontweight='bold')
    ax2.set_ylabel("Makespan (s)")
    ax2.tick_params(axis='x', rotation=45)
    ax2.grid(True, linestyle='--', alpha=0.6)
    for i, txt in enumerate(makespans):
        ax2.annotate(
            f"{txt:.1f}s",
            (profile_names[i], makespans[i]),
            textcoords="offset points",
            xytext=(0, 7),
            ha='center',
            fontsize=8,
        )

    # Panel 3: Packet Delivery Ratio
    ax3 = axes[0, 2]
    pdr = [100.0 - p['observed_loss_rate_pct'] for p in p_data]
    bars3 = ax3.bar(profile_names, pdr, color='#3498db')
    ax3.set_title("Packet Delivery Ratio (%)", fontweight='bold')
    ax3.set_ylabel("PDR (%)")
    ax3.set_ylim(0, 110)
    ax3.tick_params(axis='x', rotation=45)
    ax3.grid(axis='y', linestyle='--', alpha=0.6)
    for bar in bars3:
        yval = bar.get_height()
        ax3.text(
            bar.get_x() + bar.get_width() / 2.0,
            yval + 1.0,
            f"{yval:.1f}%",
            ha='center',
            va='bottom',
            fontsize=8,
        )

    # Panel 4: Replanning Frequency & Conflicts
    ax4 = axes[1, 0]
    replans = [p.get('total_replan_delta', p.get('replanning_count', 0)) for p in p_data]
    conflicts = [p.get('conflicts_resolved', 0) for p in p_data]
    x_indices = range(len(profile_names))
    width = 0.35
    ax4.bar([x - width / 2 for x in x_indices], replans, width, label='Replans', color='#f39c12')
    ax4.bar([x + width / 2 for x in x_indices], conflicts, width, label='Conflicts', color='#e74c3c')
    ax4.set_title("Replanning & Space-Time Conflicts", fontweight='bold')
    ax4.set_ylabel("Count")
    ax4.set_xticks(x_indices)
    ax4.set_xticklabels(profile_names, rotation=45)
    ax4.legend(loc='upper left')
    ax4.grid(axis='y', linestyle='--', alpha=0.6)

    # Panel 5: Minimum Inter-Robot Physical Distance (Safety Clearance)
    ax5 = axes[1, 1]
    min_dists = [p['min_inter_robot_distance_m'] for p in p_data]
    ax5.plot(
        profile_names,
        min_dists,
        marker='s',
        color='#27ae60',
        linewidth=2,
        markersize=6,
        label='Min Observed Distance (m)',
    )
    ax5.axhline(
        y=0.35,
        color='red',
        linestyle='--',
        linewidth=1.5,
        label='Safety Critical Limit (0.35m)',
    )
    ax5.set_title("Minimum Clearance vs Safety Invariant", fontweight='bold')
    ax5.set_ylabel("Distance (meters)")
    ax5.set_ylim(0.0, max(min_dists) * 1.3)
    ax5.tick_params(axis='x', rotation=45)
    ax5.legend(loc='upper right')
    ax5.grid(True, linestyle='--', alpha=0.6)
    for i, txt in enumerate(min_dists):
        ax5.annotate(
            f"{txt:.2f}m",
            (profile_names[i], min_dists[i]),
            textcoords="offset points",
            xytext=(0, 6),
            ha='center',
            fontsize=8,
        )

    # Panel 6: Stale State Transitions & Expired Pruning
    ax6 = axes[1, 2]
    stale_counts = [p['stale_state_transitions'] for p in p_data]
    pruned_counts = [p['expired_reservations_pruned'] for p in p_data]
    ax6.bar(
        [x - width / 2 for x in x_indices],
        stale_counts,
        width,
        label='Stale Transitions',
        color='#8e44ad',
    )
    ax6.bar(
        [x + width / 2 for x in x_indices],
        pruned_counts,
        width,
        label='Pruned Expired',
        color='#d35400',
    )
    ax6.set_title("Stale Tracking & Reservation Pruning", fontweight='bold')
    ax6.set_ylabel("Occurrences")
    ax6.set_xticks(x_indices)
    ax6.set_xticklabels(profile_names, rotation=45)
    ax6.legend(loc='upper right')
    ax6.grid(axis='y', linestyle='--', alpha=0.6)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    plt.savefig(output_path, dpi=200)
    plt.close()
    print(f"[PLOT] Generated degradation curves figure at: {output_path}")


def main() -> int:
    """Entry point."""
    parser = argparse.ArgumentParser(description="Plot M7 Degradation Curves")
    parser.add_argument(
        '--input',
        default='gazebo_m7_results.json',
        help='Input JSON results file',
    )
    parser.add_argument(
        '--output',
        default='docs/images/m7_degradation_curves.png',
        help='Output PNG path',
    )
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"[ERROR] Input results file not found: {args.input}")
        return 1

    with open(args.input, 'r', encoding='utf-8') as f:
        data = json.load(f)

    plot_resilience_curves(data, args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
