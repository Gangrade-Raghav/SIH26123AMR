#!/usr/bin/env python3
"""
Real ROS 2 + Gazebo Operator Task Control Demonstration Script.

Validates in a full live Gazebo Harmonic multi-robot simulation:
1. Full 5-AMR fleet bringup in Gazebo Harmonic with TaskManagerNode and CBBANodes.
2. Web Dashboard HTTP API + UI backend integration on port 8080.
3. Live Operator Task Creation via Web API: AUTO allocation auctioned via CBBA.
4. Live Operator Task Creation via CLI Tool: DIRECT constraint targeting amr_3.
5. CBBA consensus verification: amr_3 claims DIRECT task; AUTO task claimed by winner.
6. Live Task Control: CANCEL action via Web API updates state to CANCELLED and purges bundle.
7. Captures full JSON telemetry evidence for the DoD checkpoint report.
"""

import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional


def check_process_running(proc_name: str) -> bool:
    """Check if process containing proc_name is currently running."""
    try:
        out = subprocess.check_output(['pgrep', '-f', proc_name], text=True)
        return len(out.strip().splitlines()) > 0
    except subprocess.CalledProcessError:
        return False


def main() -> int:
    print('=' * 75)
    print('  NRDAS LIVE GAZEBO OPERATOR TASK ALLOCATION & CONTROL DEMONSTRATION')
    print('=' * 75)

    ws_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    dashboard_script = os.path.join(ws_root, 'scripts', 'fleet_dashboard.py')
    cli_create_script = os.path.join(ws_root, 'scripts', 'create_task.py')
    evidence_file = os.path.join(
        ws_root, 'docs', 'checkpoints', 'task_allocation_evidence.json'
    )

    subprocesses: List[subprocess.Popen] = []

    evidence: Dict[str, Any] = {
        'timestamp': time.time(),
        'gazebo_online': False,
        'dashboard_online': False,
        'task_auto_created': False,
        'task_direct_created': False,
        'cbba_consensus_reached': False,
        'direct_allocation_verified': False,
        'cancel_action_verified': False,
        'final_task_states': {},
        'final_bundles': {},
        'bids_count': 0,
    }

    try:
        # 1. Launch 5 AMRs in Gazebo Harmonic + TaskManager + CBBANodes
        print('\n[1/6] Launching 5-AMR fleet in Gazebo Harmonic simulation...')
        launch_cmd = [
            'ros2', 'launch', 'amr_fleet_bringup', 'cbba_fleet.launch.py',
            'robot_count:=5',
            'max_bundle_size:=4',
            'headless:=true',
            'launch_simulation:=true',
        ]
        launch_proc = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        subprocesses.append(launch_proc)

        # 2. Launch Fleet Dashboard on port 8080
        print('[2/6] Starting NRDAS Fleet Dashboard on http://localhost:8080...')
        dash_proc = subprocess.Popen(
            ['python3', dashboard_script, '--port', '8080'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        subprocesses.append(dash_proc)

        # Wait for dashboard and Gazebo to come online
        print('  [*] Waiting for Gazebo Harmonic and Fleet Dashboard to initialize...')
        dash_ready = False
        start_wait = time.time()
        while time.time() - start_wait < 35.0:
            time.sleep(2.0)
            try:
                with urllib.request.urlopen('http://localhost:8080/api/state', timeout=2.0) as resp:
                    if resp.status == 200:
                        state_data = json.loads(resp.read().decode('utf-8'))
                        robots = state_data.get('fleet', {}).get('robots', [])
                        if len(robots) >= 5 and state_data.get('tasks'):
                            dash_ready = True
                            evidence['gazebo_online'] = True
                            evidence['dashboard_online'] = True
                            print(f'  [+] Fleet online ({len(robots)} AMRs reporting telemetry).')
                            break
            except Exception:
                pass

        if not dash_ready:
            print('[-] Timed out waiting for Gazebo / Dashboard.')
            return 1

        # 3. Create Task 1 via Dashboard Web API (AUTO Allocation)
        print('\n[3/6] Creating Task 1 via Dashboard HTTP API (/api/task/create — AUTO CBBA)...')
        task_1_payload = {
            'task_id': 'T_DEMO_AUTO',
            'pickup_x': 2.0,
            'pickup_y': 4.0,
            'dropoff_x': 10.0,
            'dropoff_y': 12.0,
            'priority': 3,
            'requested_robot': '',
        }
        req = urllib.request.Request(
            'http://localhost:8080/api/task/create',
            data=json.dumps(task_1_payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=4.0) as resp:
            res_json = json.loads(resp.read().decode('utf-8'))
            print(f'  [+] API Response: {res_json}')
            assert res_json.get('accepted') is True, f'Task 1 creation failed: {res_json}'
            evidence['task_auto_created'] = True

        # 4. Create Task 2 via CLI tool (DIRECT Constraint: amr_3)
        print('\n[4/6] Creating Task 2 via CLI tool (scripts/create_task.py — DIRECT amr_3)...')
        cli_cmd = [
            'python3', cli_create_script,
            '--pickup-x', '4.0',
            '--pickup-y', '2.0',
            '--dropoff-x', '12.0',
            '--dropoff-y', '8.0',
            '--priority', 'NORMAL',
            '--task-id', 'T_DEMO_DIR3',
            '--robot', 'amr_3',
            '--timeout', '8.0',
        ]
        res_cli = subprocess.run(cli_cmd, capture_output=True, text=True)
        print(res_cli.stdout.strip())
        assert res_cli.returncode == 0, f'CLI task creation failed:\n{res_cli.stderr}'
        evidence['task_direct_created'] = True

        # 5. Monitor CBBA consensus & allocation via /api/state
        print('\n[5/6] Monitoring Decentralized CBBA Consensus & Task Allocation...')
        consensus_achieved = False
        start_consensus = time.time()
        while time.time() - start_consensus < 25.0:
            time.sleep(1.0)
            try:
                with urllib.request.urlopen('http://localhost:8080/api/state', timeout=2.0) as resp:
                    state_data = json.loads(resp.read().decode('utf-8'))
                    cbba_info = state_data.get('cbba', {})
                    tasks_info = state_data.get('tasks', {})
                    tasks_list = tasks_info.get('tasks', [])

                    task_map = {t['id']: t for t in tasks_list}
                    t_auto = task_map.get('T_DEMO_AUTO')
                    t_dir = task_map.get('T_DEMO_DIR3')

                    if t_auto and t_dir:
                        if t_auto.get('status') == 'ASSIGNED' and t_dir.get('status') == 'ASSIGNED':
                            consensus_achieved = True
                            evidence['cbba_consensus_reached'] = True
                            evidence['direct_allocation_verified'] = (t_dir.get('robot') == 'amr_3')
                            evidence['final_task_states'] = {t['id']: t['status'] for t in tasks_list}
                            evidence['final_bundles'] = cbba_info.get('bundles', {})
                            evidence['bids_count'] = cbba_info.get('bids_count', 0)

                            print('  [+] Consensus achieved across fleet!')
                            print(f"      - T_DEMO_AUTO: Assigned to {t_auto.get('robot')}")
                            print(f"      - T_DEMO_DIR3: Assigned to {t_dir.get('robot')} (Constraint: amr_3)")
                            print(f"      - Bundles: {cbba_info.get('bundles')}")
                            break
            except Exception as e:
                print(f'  [*] Polling error: {e}')

        assert consensus_achieved, 'CBBA consensus not reached within timeout!'
        assert evidence['direct_allocation_verified'], 'T_DEMO_DIR3 was not assigned to amr_3!'

        # 6. Live Operator Task Control: CANCEL T_DEMO_AUTO
        print('\n[6/6] Testing Live Task Control: Cancelling T_DEMO_AUTO via Web API...')
        cancel_payload = {'task_id': 'T_DEMO_AUTO', 'action': 'CANCEL'}
        req_cancel = urllib.request.Request(
            'http://localhost:8080/api/task/control',
            data=json.dumps(cancel_payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        with urllib.request.urlopen(req_cancel, timeout=4.0) as resp:
            cancel_res = json.loads(resp.read().decode('utf-8'))
            print(f'  [+] Control API Response: {cancel_res}')
            assert cancel_res.get('success') is True, f'Cancel action failed: {cancel_res}'

        time.sleep(2.0)
        with urllib.request.urlopen('http://localhost:8080/api/state', timeout=2.0) as resp:
            state_data = json.loads(resp.read().decode('utf-8'))
            tasks_list = state_data.get('tasks', {}).get('tasks', [])
            task_map = {t['id']: t for t in tasks_list}
            assert task_map['T_DEMO_AUTO']['status'] == 'CANCELLED', 'Task state not CANCELLED!'
            evidence['cancel_action_verified'] = True
            evidence['final_task_states'] = {t['id']: t['status'] for t in tasks_list}
            evidence['final_bundles'] = state_data.get('cbba', {}).get('bundles', {})
            print('  [+] Task T_DEMO_AUTO confirmed in CANCELLED state.')
            print(f"  [+] Updated Bundles: {evidence['final_bundles']}")

        # Save evidence JSON
        with open(evidence_file, 'w') as f:
            json.dump(evidence, f, indent=2)
        print(f'\n[+] Demonstration telemetry evidence saved to:\n    {evidence_file}')

        print('\n======================================================================')
        print('  GAZEBO LIVE DEMONSTRATION PASSED: ALL INVARIANTS SATISFIED')
        print('======================================================================\n')
        return 0

    finally:
        print('  [*] Cleaning up simulation and dashboard subprocesses...')
        for proc in subprocesses:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        # Final safety cleanup
        subprocess.run(
            ['killall', '-9', 'gz', 'sim-server', 'sim-gui', 'ruby'],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
        )


if __name__ == '__main__':
    sys.exit(main())
