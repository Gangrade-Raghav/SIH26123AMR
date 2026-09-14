#!/usr/bin/env python3
"""Export structured CSV and markdown tables from empirical results."""

import csv
import json
import os


def export_m9_tables():
    os.makedirs('results/final/tables', exist_ok=True)
    os.makedirs('results/final/comparisons', exist_ok=True)

    files = {
        'M9-V1 (Expanded 5 AMR)': 'results/final/canonical/m9_v1_canonical_raw.json',
        'M9-V2 (Congested 10 AMR)': 'results/final/canonical/m9_v2_canonical_raw.json',
        'M9-V3-D (Dynamic Arrival)': 'results/final/canonical/m9_v3_d_canonical_raw.json',
        'M9-V3-A (Dynamic Blockage)': 'results/final/canonical/m9_v3_a_canonical_raw.json',
        'M9-V3-E (Combined Stress)': 'results/final/canonical/m9_v3_e_canonical_raw.json',
    }

    rows = []
    for scenario_name, path in files.items():
        if not os.path.isfile(path):
            continue
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        meta = data.get('metadata', {})
        m = data.get('metrics', {})

        min_dist = m.get('minimum_center_to_center_distance_m')
        if min_dist is None:
            min_dist = m.get('minimum_inter_robot_distance_m', 0.0)

        row = {
            'scenario': scenario_name,
            'world': meta.get('world'),
            'fleet_size': meta.get('fleet_size'),
            'workload': meta.get('workload_size'),
            'horizon_s': meta.get('mission_horizon_sec'),
            'seed': meta.get('seed'),
            'comm_profile': meta.get('communication_profile'),
            'compute_mode': meta.get('compute_mode'),
            'cbba_initial_ms': m.get('cbba_initial_convergence_time_ms', m.get('cbba_convergence_time_ms', 0.0)),
            'cbba_dynamic_ms': m.get('cbba_dynamic_convergence_time_ms', 0.0),
            'replans': m.get('replan_count', 0),
            'planning_latency_mean_ms': m.get('planning_latency_mean_ms', 0.0),
            'planning_latency_p95_ms': m.get('planning_latency_p95_ms', 0.0),
            'min_distance_m': min_dist,
            'proximity_breaches': m.get('proximity_breaches', 0),
            'gazebo_physical_contacts': m.get('physical_gazebo_contacts', m.get('collision_contact_events', 0)),
            'obb_overlap_samples': m.get('obb_chassis_overlap_samples', 0),
            'safety_brakes': m.get('safety_brake_interventions', 0),
            'safety_aborts': m.get('safety_aborts', 0),
            'packets_sent': m.get('comm_packets_sent', 0),
            'packets_delivered': m.get('comm_packets_delivered', 0),
            'packets_dropped': m.get('comm_packets_dropped', 0),
            'observed_loss_rate_pct': m.get('comm_observed_loss_rate_pct', 0.0),
            'tasks_generated': m.get('generated_tasks', meta.get('workload_size', 0)),
            'tasks_assigned': m.get('assigned_tasks', 0),
            'tasks_in_progress': m.get('in_progress_tasks', 0),
            'tasks_completed': m.get('completed_tasks', 0),
            'tasks_remaining': m.get('remaining_tasks', meta.get('workload_size', 0)),
            'cpu_mean_pct': m.get('cpu_utilization_mean_pct', 0.0),
            'ram_mean_mb': m.get('ram_utilization_mean_mb', 0.0),
        }
        rows.append(row)

    # Write CSV
    csv_path = 'results/final/tables/m9_scenarios_comparison.csv'
    if rows:
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"Exported {csv_path}")

    # Write Markdown Table
    md_path = 'results/final/comparisons/m9_cross_scenario_summary.md'
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write("# M9 Multi-Scenario Empirical Comparison Summary\n\n")
        f.write("| Scenario | Fleet | Workload | Blockage | Comm Loss | Initial CBBA | Dynamic CBBA | Replans | Mean Latency | Min Dist | Contacts | OBB Overlaps | Brakes | Tasks (A / IP / C) |\n")
        f.write("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
        for r in rows:
            block = "Yes [45-90s]" if "Blockage" in r['scenario'] or "Stress" in r['scenario'] else "None"
            comm = f"{r['observed_loss_rate_pct']:.2f}%" if r['observed_loss_rate_pct'] > 0 else "0.0%"
            dyn_cbba = f"{r['cbba_dynamic_ms']:.1f} ms" if r['cbba_dynamic_ms'] else "N/A"
            f.write(
                f"| {r['scenario']} | {r['fleet_size']} | {r['workload']} | {block} | {comm} | "
                f"{r['cbba_initial_ms']:.1f} ms | {dyn_cbba} | {r['replans']} | "
                f"{r['planning_latency_mean_ms']:.2f} ms | {r['min_distance_m']:.2f} m | "
                f"{r['gazebo_physical_contacts']} | {r['obb_overlap_samples']} | {r['safety_brakes']} | "
                f"{r['tasks_assigned']}A / {r['tasks_in_progress']}IP / {r['tasks_completed']}C |\n"
            )
    print(f"Exported {md_path}")


if __name__ == '__main__':
    export_m9_tables()
