"""Opt-in, self-contained Gazebo fault injection; only owned processes are signalled."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from vehicle_interfaces.msg import ActuatorCommand, ActuatorStatus, ControlState
from vehicle_simulation.process_cleanup import cleanup_partition, stop_gazebo


@pytest.mark.skipif(os.environ.get('RUN_ACTUATOR_WATCHDOG_INTEGRATION') != '1',
                    reason='Opt in to launch an isolated headless Gazebo system')
def test_receiver_stops_independently_and_requires_explicit_activation(tmp_path):
    env = os.environ.copy()
    env['GZ_PARTITION'] = f'watchdog_{uuid.uuid4().hex}'
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(
        Path(get_package_prefix('vehicle_actuator_sim')) / 'lib')
    env['ROS_LOG_DIR'] = str(tmp_path / 'ros_log')
    # Avoid touching the user's GUI, home config, or running Gazebo transport partition.
    env['GZ_HOMEDIR'] = str(tmp_path / 'gz_home')
    description = Path(get_package_share_directory('vehicle_description'))
    urdf = subprocess.check_output(['xacro', str(description / 'urdf/vehicle.urdf.xacro'), 'simulation:=true'],
                                   env=env, text=True)
    urdf_path = tmp_path / 'vehicle.urdf'
    urdf_path.write_text(urdf)
    sdf = subprocess.check_output(['gz', 'sdf', '-p', str(urdf_path)], env=env, text=True)
    model = ET.fromstring(sdf).find('model')
    ET.SubElement(model, 'pose').text = '0 0 0.02 0 0 0'
    # Visual meshes are irrelevant to physical stopping and need no resource download.
    for link in model.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    world = ET.fromstring('''<sdf version="1.9"><world name="watchdog">
      <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size>
        <real_time_factor>1</real_time_factor></physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <model name="ground"><static>true</static><link name="ground">
        <collision name="ground"><geometry><plane><normal>0 0 1</normal>
        <size>100 100</size></plane></geometry></collision></link></model>
    </world></sdf>''')
    world.find('world').append(model)
    world_path = tmp_path / 'watchdog.sdf'
    ET.ElementTree(world).write(world_path)
    processes, logs = [], []

    def start(args, name):
        log = open(tmp_path / f'{name}.log', 'w')
        logs.append(log)
        proc = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append(proc)
        return proc

    rclpy.init()
    node = rclpy.create_node('watchdog_fault_probe')
    statuses, truth, controls = [], [], []
    node.create_subscription(ControlState, '/control_state', controls.append, 10)
    node.create_subscription(ActuatorStatus, '/actuator/status', statuses.append, 100)
    node.create_subscription(Odometry, '/ground_truth/odom',
                             lambda m: truth.append((time.monotonic(), m)), 100)
    manual = node.create_publisher(Twist, '/cmd_vel_manual', 1)
    raw = node.create_publisher(ActuatorCommand, '/actuator/command', 1)
    mode = node.create_client(SetBool, '/set_auto_mode')
    stop = node.create_client(SetBool, '/set_emergency_stop')
    drive = Twist()
    drive.linear.x = 0.1
    producer = node.create_timer(0.05, lambda: manual.publish(drive))
    report = []

    def wait(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return
        tails = '\n'.join(p.read_text()[-2000:] for p in tmp_path.glob('*.log'))
        raise AssertionError(f'Timed out; status={statuses[-1:]}; logs={tails}')

    def observe(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def call(client, req, success=True):
        wait(client.service_is_ready)
        future = client.call_async(req)
        wait(future.done, 5)
        response = future.result()
        assert response.success is success, response.message
        return response

    def activate():
        wait(lambda: statuses and statuses[-1].stationary and not statuses[-1].armed)
        call(mode, SetBool.Request(data=False))
        wait(lambda: statuses[-1].armed and abs(statuses[-1].left_output_rps) > 0.1)
        wait(lambda: truth and abs(truth[-1][1].twist.twist.linear.x) > .03)

    def stopped(name, expired=True):
        wait(lambda: statuses and not statuses[-1].armed and
             statuses[-1].left_output_rps == statuses[-1].right_output_rps == 0)
        status = statuses[-1]
        if expired:
            assert status.reason == 'watchdog_expired'
            delay = (status.expired_ns - status.last_valid_ns) / 1e9
            assert .3 <= delay < (1.2 if name == 'simulator_sigstop' else .4)
        else:
            delay = None
        wait(lambda: statuses[-1].stationary)
        settled = time.monotonic()
        observe(1.1)
        samples = [m for t, m in truth if t >= settled]
        assert len(samples) >= 20
        assert all(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) <= .02
                   and abs(m.twist.twist.angular.z) <= .05 for m in samples)
        # Linux C++ steady_clock and Python monotonic share CLOCK_MONOTONIC.
        # Ground-truth receipt time includes transport latency; report it as an observation.
        after = [(t, m) for t, m in truth if t >= status.last_valid_ns / 1e9]
        stationary_start = None
        previous = None
        for t, m in after:
            stationary = (math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) <= .02
                          and abs(m.twist.twist.angular.z) <= .05)
            if not stationary or (previous is not None and t - previous > .2):
                stationary_start = None
            if stationary and stationary_start is None:
                stationary_start = t
            previous = t
        assert stationary_start is not None and after[-1][0] - stationary_start >= 1.0
        traveled = 0.0
        for (_, a), (t, b) in zip(after, after[1:]):
            if t > stationary_start:
                break
            traveled += math.hypot(b.pose.pose.position.x - a.pose.pose.position.x,
                                   b.pose.pose.position.y - a.pose.pose.position.y)
        report.append(dict(scenario=name, expiry_sec=delay,
                           observed_stop_sec=stationary_start - status.last_valid_ns / 1e9,
                           observed_travel_m=traveled,
                           zero_after_last_valid_sec=(status.zero_applied_ns - status.last_valid_ns) / 1e9,
                           max_update_gap_sec=status.max_update_gap_ns / 1e9,
                           stationary_samples=len(samples)))

    def pause(value):
        result = subprocess.run([
            'gz', 'service', '-s', '/world/watchdog/control', '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
            '--req', f'pause: {str(value).lower()}',
        ], env=env, text=True, capture_output=True, timeout=5)
        assert result.returncode == 0 and 'true' in result.stdout, result.stderr

    try:
        sim = start(['gz', 'sim', '-s', '-r', str(world_path)], 'gazebo')
        start(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
               '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'], 'bridge')
        guard_executable = str(Path(get_package_prefix('vehicle_control')) / 'lib/vehicle_control/command_guard')
        guard = start([guard_executable, '--ros-args', '-p', 'initial_mode:=manual'], 'guard')
        wait(lambda: statuses and statuses[-1].stationary, 25)
        observe(.5)
        assert not statuses[-1].armed, 'Initial mode/continuous input must not arm'
        activate()

        # Input producer outage: guard stays alive, then outputs zero after input expiry.
        producer.cancel()
        producer_stopped = time.monotonic()
        wait(lambda: statuses[-1].left_output_rps == statuses[-1].right_output_rps == 0)
        assert statuses[-1].armed
        report.append(dict(scenario='producer_exit', stopped_by_guard=True,
                           observed_zero_sec=time.monotonic() - producer_stopped))
        producer.reset()
        wait(lambda: abs(statuses[-1].left_output_rps) > .1)

        for sig, name in [(signal.SIGSTOP, 'guard_sigstop'),
                          (signal.SIGKILL, 'guard_sigkill'),
                          (signal.SIGINT, 'guard_sigint')]:
            os.killpg(guard.pid, sig)
            stopped(name)
            if sig == signal.SIGSTOP:
                os.killpg(guard.pid, signal.SIGCONT)
            else:
                guard.wait(timeout=5)
                old_session = controls[-1].guard_session
                guard = start([guard_executable, '--ros-args', '-p', 'initial_mode:=manual'],
                              f'guard_after_{name}')
                wait(lambda: controls and controls[-1].guard_session != old_session)
            observe(.5)
            assert not statuses[-1].armed
            activate()

        # Hold short/long simulation pauses: even continued input cannot restart.
        for duration in [.05, .5]:
            pause(True)
            wait(lambda: statuses[-1].paused and not statuses[-1].armed)
            observe(duration)
            pause(False)
            wait(lambda: not statuses[-1].paused)
            stopped(f'pause_{duration}', expired=False)
            activate()

        # Freeze the entire simulator while the PC producer/guard remain alive.
        os.killpg(sim.pid, signal.SIGSTOP)
        observe(.8)
        os.killpg(sim.pid, signal.SIGCONT)
        stopped('simulator_sigstop')
        observe(.3)
        assert not statuses[-1].armed
        activate()

        # Explicit estop clear leaves disarmed; mode alone cannot clear an active estop.
        call(stop, SetBool.Request(data=True))
        wait(lambda: statuses[-1].emergency_stop and not statuses[-1].armed)
        call(mode, SetBool.Request(data=False), success=False)
        assert statuses[-1].emergency_stop
        call(stop, SetBool.Request(data=False))
        wait(lambda: not statuses[-1].emergency_stop)
        observe(.4)
        assert not statuses[-1].armed
        activate()

        # Simulate loss of the PC command channel: stop guard, replay its last packet.
        status = statuses[-1]
        replay = ActuatorCommand(
            actuator_session=status.actuator_session, generation=status.generation,
            guard_session=status.guard_session, sequence=status.accepted_sequence,
            heartbeat=status.heartbeat, velocity=drive,
        )
        os.killpg(guard.pid, signal.SIGSTOP)
        replay_timer = node.create_timer(.02, lambda: raw.publish(replay))
        stopped('duplicate_replay_during_channel_loss')
        replay_timer.cancel()
        os.killpg(guard.pid, signal.SIGCONT)
        observe(.4)
        assert not statuses[-1].armed
        activate()

        # A malformed packet in the current session must stop actual wheel outputs.
        status = statuses[-1]
        invalid = ActuatorCommand(
            actuator_session=status.actuator_session, generation=status.generation,
            guard_session=status.guard_session, sequence=status.accepted_sequence + 100000,
            heartbeat=status.heartbeat, velocity=Twist(),
        )
        invalid.velocity.linear.x = float('nan')
        raw.publish(invalid)
        wait(lambda: statuses[-1].reason == 'invalid_command')
        stopped('invalid_nan', expired=False)
        activate()

        # Reboot the simulated receiver while the PC keeps producing commands.
        old_session = statuses[-1].actuator_session
        stop_gazebo(env['GZ_PARTITION'], env)
        if sim.poll() is None:
            os.killpg(sim.pid, signal.SIGINT)
        sim.wait(timeout=8)
        cleanup_partition(env['GZ_PARTITION'], command_names={'ruby'})
        observe(.3)
        sim = start(['gz', 'sim', '-s', '-r', str(world_path)], 'gazebo_restarted')
        old_timer = node.create_timer(.02, lambda: raw.publish(replay))
        wait(lambda: statuses[-1].actuator_session != old_session and statuses[-1].stationary, 20)
        observe(.5)
        assert not statuses[-1].armed
        assert statuses[-1].left_output_rps == statuses[-1].right_output_rps == 0
        old_timer.cancel()
        report.append(dict(scenario='receiver_restart_and_old_session_replay',
                           new_session=True, automatic_restart=False))
        activate()
        call(stop, SetBool.Request(data=True))
        report_path = Path(os.environ.get('ACTUATOR_WATCHDOG_REPORT', tmp_path / 'report.json'))
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        producer.cancel()
        stop_gazebo(env['GZ_PARTITION'], env)
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGCONT)
                os.killpg(proc.pid, signal.SIGINT)
        for proc in reversed(processes):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        cleanup_partition(env['GZ_PARTITION'])
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
