"""Run existing opt-in tests against a fresh installed system for every case.

Usage: source env.sh; python3 src/vehicle_simulation/test/run_backend_regression.py
       --output evaluation_runs/005_separation/regression --cases s01 s02 s03 s05 s06
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from vehicle_interfaces.msg import NavigationRecoveryStatus
from vehicle_simulation.process_cleanup import stop_system


GOALS = {'s01': [1., 0., 0.], 's02': [0., 1., math.pi / 2],
         's03': [4., 0., 0.], 's05': [4., 1.1, 0.], 's06': [1., 0., 0.]}


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.)


def wait_for_navigation(environment, timeout, *, require_manager_observation=True, process=None):
    os.environ['ROS_DOMAIN_ID'] = environment['ROS_DOMAIN_ID']
    os.environ['ROS_LOG_DIR'] = environment['ROS_LOG_DIR']
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('regression_startup_probe', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    scans, statuses = [], []
    def receive_scan(message):
        scans.append((time.monotonic(), message))
    node.create_subscription(LaserScan, '/scan', receive_scan, qos_profile_sensor_data)
    node.create_subscription(NavigationRecoveryStatus, '/navigation_manager/recovery_status',
        statuses.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        client = node.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.monotonic() + timeout
        active, stable_since = False, None
        while time.monotonic() < deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'System launch exited before readiness: code {process.returncode}')
            if not active and client.wait_for_service(timeout_sec=.2):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
                active = future.done() and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            rclpy.spin_once(node, timeout_sec=.05)
            fresh = False
            if len(scans) >= 20:
                received, scan = scans[-1]
                stamp = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
                age = (node.get_clock().now().nanoseconds - stamp) / 1e9
                fresh = time.monotonic() - received <= .5 and -.1 <= age <= .5
            manager_ready = not require_manager_observation or (
                statuses and statuses[-1].data_ready and statuses[-1].control_ready
                and statuses[-1].stop_confirmed)
            if active and fresh and manager_ready:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 2.:
                    return
            else:
                stable_since = None
        raise RuntimeError(f'Nav2, sensors, and stopped state did not stabilize within {timeout} seconds')
    finally:
        startup = {
            'domain': environment['ROS_DOMAIN_ID'], 'nav2_active': active,
            'scan_count': len(scans), 'ros_now_ns': node.get_clock().now().nanoseconds,
            'recovery_status': str(statuses[-1]) if statuses else None,
        }
        if scans:
            received, scan = scans[-1]
            startup.update(last_scan_stamp_ns=scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec,
                           last_scan_received_age_sec=time.monotonic() - received)
        (Path(environment['ROS_LOG_DIR']).parent / 'startup.json').write_text(json.dumps(startup, indent=2))
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--startup-timeout', type=float, default=120.,
                        help='Startup limit, separate from the existing 120-second goal limit')
    parser.add_argument('--cases', nargs='+', choices=[*GOALS, 'safety', 'recovery', 'sensors', 'watchdog'],
                        default=['s01', 's02', 's03'] * 3 + ['s05', 's06', 'safety', 'recovery', 'sensors', 'watchdog'])
    args = parser.parse_args()
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('--startup-timeout must be finite and positive')
    workspace = Path(__file__).resolve().parents[3]
    output = args.output.resolve()
    if output.is_relative_to(workspace / 'src'):
        parser.error('--output must be outside src to avoid recursive input snapshots')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'COLCON_IGNORE').touch()
    snapshot = output / 'inputs/src'
    shutil.copytree(workspace / 'src', snapshot, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '*.egg-info'))
    manifest = {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(snapshot.rglob('*')) if p.is_file()}
    (output / 'source_manifest.json').write_text(json.dumps(manifest, indent=2))
    (output / 'changes.diff').write_bytes(subprocess.check_output(['git', 'diff', 'main'], cwd=workspace))
    test_directory = Path(__file__).resolve().parent
    results = []
    for index, case in enumerate(args.cases):
        run = output / f'{index + 1:02d}_{case}'
        run.mkdir()
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID=str(60 + index % 20), GZ_PARTITION=f'backend_regression_{os.getpid()}_{index}',
                           ROS_LOG_DIR=str(run / 'ros_log'), GZ_HOMEDIR=str(run / 'gz_home'))
        environment.pop('DISPLAY', None)
        (run / 'environment.json').write_text(json.dumps({name: environment[name] for name in (
            'ROS_DOMAIN_ID', 'GZ_PARTITION', 'ROS_LOG_DIR', 'GZ_HOMEDIR')}, indent=2))
        launch = ['ros2', 'launch', 'vehicle_bringup', 'system.launch.py',
                  'rviz:=false', 'gazebo_gui:=false', 'headless_rendering:=true']
        if case == 'recovery':
            launch = ['ros2', 'launch', str(test_directory / 'recovery_system.launch.py')]
        process = bag = None
        print(f'Starting {index + 1}/{len(args.cases)}: {case}; artifacts: {run}', flush=True)
        with (run / 'system.log').open('w') as launch_log, (run / 'test.log').open('w') as test_log, (run / 'bag.log').open('w') as bag_log:
            try:
                if case != 'watchdog':
                    process = subprocess.Popen(launch, env=environment, stdout=launch_log,
                                               stderr=subprocess.STDOUT, start_new_session=True)
                    # The recovery test starts the manager-only odom relay itself.
                    wait_for_navigation(environment, args.startup_timeout,
                                        require_manager_observation=case != 'recovery', process=process)
                if case in GOALS:
                    environment.update(RUN_NAVIGATION_INTEGRATION='1', NAVIGATION_SCENARIO=case)
                    tests = [test_directory / 'test_navigation_integration.py']
                    bag_command = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(run / 'bag'),
                        '--qos-profile-overrides-path', str(snapshot / 'vehicle_simulation/config/recording_qos.yaml'),
                        '--topics', '/clock', '/scan', '/odom', '/ground_truth/odom', '/tf', '/tf_static',
                        '/cmd_vel', '/actuator/command', '/actuator/status', '/navigation_manager/events']
                    bag = subprocess.Popen(bag_command, env=environment, stdout=bag_log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                    # Wait for the recorder's discovery before submitting a goal.
                    os.environ['ROS_DOMAIN_ID'] = environment['ROS_DOMAIN_ID']
                    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
                    node = rclpy.create_node('regression_recording_probe')
                    try:
                        end = time.monotonic() + 15.
                        while time.monotonic() < end:
                            rclpy.spin_once(node, timeout_sec=.1)
                            if node.count_subscribers('/navigation_manager/events') >= 1:
                                break
                        else:
                            raise RuntimeError('Recorder failed to subscribe to navigation events')
                    finally:
                        node.destroy_node()
                        rclpy.shutdown()
                    metadata = {'mode': 'navigation', 'backend': 'simulation', 'scenario': case,
                                'planned_goal_map': GOALS[case], 'expected_start_map': [0., 0., 0.],
                                'launch_command': launch, 'bag_command': bag_command}
                    (run / 'run.json').write_text(json.dumps(metadata, indent=2))
                elif case == 'safety':
                    environment['RUN_SAFETY_SYNC_INTEGRATION'] = '1'
                    tests = [test_directory / 'test_safety_navigation_sync.py']
                elif case == 'recovery':
                    environment['RUN_RECOVERY_GAZEBO'] = '1'
                    tests = [test_directory / 'test_navigation_recovery_gazebo.py']
                elif case == 'sensors':
                    environment.update(RUN_LIDAR_INTEGRATION='1', RUN_ODOMETRY_INTEGRATION='1', RUN_ODOMETRY_MOTION='1')
                    tests = [test_directory / 'test_lidar_integration.py', test_directory / 'test_odometry_integration.py']
                else:
                    environment['RUN_ACTUATOR_WATCHDOG_INTEGRATION'] = '1'
                    tests = [test_directory / 'test_actuator_watchdog.py']
                command = [sys.executable, '-m', 'pytest', *map(str, tests), '-q', '-s',
                           f'--basetemp={run / "pytest"}', f'--junitxml={run / "junit.xml"}']
                result = subprocess.run(command, env=environment, stdout=test_log,
                                        stderr=subprocess.STDOUT, timeout=240.)
                stop(bag)
                if result.returncode:
                    raise RuntimeError(f'{case} failed; inspect {run / "test.log"}')
                if case in GOALS:
                    analysis = subprocess.run(['ros2', 'run', 'vehicle_simulation', 'evaluate_run',
                        '--run-dir', str(run), '--map-from-world', '2.036', '0.020', '0',
                        '--alignment-note', 'Existing grid-center world/map alignment; no collision inference.'],
                        env=environment, stdout=test_log, stderr=subprocess.STDOUT, timeout=30.)
                    if analysis.returncode:
                        raise RuntimeError(f'Bag evaluation failed: {run}')
                    report = json.loads((run / 'analysis.json').read_text())
                    if case in ('s01', 's02', 's03'):
                        assert report['position_error_m'] <= .10, report
                        assert abs(report['yaw_error_deg']) <= 10., report
                results.append({'case': case, 'result': 'passed', 'directory': str(run)})
                print(f'Passed: {case}', flush=True)
            except Exception as error:
                results.append({'case': case, 'result': 'failed', 'error': str(error), 'directory': str(run)})
                print(f'Failed: {case}: {error}', flush=True)
            finally:
                stop(bag)
                if process is not None:
                    stop_system(process, environment['GZ_PARTITION'], simulation=True, environment=environment)
                (output / 'results.json').write_text(json.dumps(results, indent=2))
    if any(result['result'] != 'passed' for result in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
