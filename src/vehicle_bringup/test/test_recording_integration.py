"""Opt-in real rosbag recording and backend smoke tests; no retained bags."""
import os
from pathlib import Path
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
from uuid import uuid4

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import yaml


pytestmark = pytest.mark.skipif(
    os.environ.get('RUN_RECORDING_INTEGRATION') != '1',
    reason='Opt in to rosbag and isolated backend recording tests',
)


def wait_for(node, condition, timeout=20.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return
        rclpy.spin_once(node, timeout_sec=.05)
    assert condition(), 'Timed out waiting for recording condition'


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=25.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.)
            pytest.fail('Launch did not stop after SIGINT')


def assert_bag(path, expected_topics):
    import rosbag2_py
    info = subprocess.run(['ros2', 'bag', 'info', str(path)], capture_output=True, text=True, timeout=15.)
    assert info.returncode == 0, info.stderr
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts = {item['topic_metadata']['name']: item['message_count']
              for item in metadata['topics_with_message_count']}
    assert all(counts.get(topic, 0) > 0 for topic in expected_topics), counts
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    decoded = set()
    try:
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic == '/scan':
                assert deserialize_message(data, LaserScan).ranges
                decoded.add(topic)
            elif topic == '/tf_static':
                assert deserialize_message(data, TFMessage).transforms
                decoded.add(topic)
    finally:
        del reader
    assert {'/scan', '/tf_static'} <= decoded


@pytest.mark.parametrize('ending', ['timeout', 'interrupt', 'failure'])
def test_recorder_lifetime_and_late_static_tf(tmp_path, monkeypatch, ending):
    with TemporaryDirectory(dir=tmp_path) as temporary:
        work = Path(temporary)
        monkeypatch.setenv('ROS_DOMAIN_ID', '81')
        monkeypatch.setenv('ROS_LOG_DIR', str(work / 'ros_log'))
        rclpy.init()
        node = rclpy.create_node('recording_probe')
        scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        static_pub = node.create_publisher(TFMessage, '/tf_static', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'recording_test_sensor'
        transform.transform.rotation.w = 1.
        # Publish once BEFORE recorder discovery; only durable subscription can retrieve it.
        static_pub.publish(TFMessage(transforms=[transform]))
        scan = LaserScan(angle_min=0., angle_max=0., angle_increment=1.,
                         range_min=.1, range_max=10., ranges=[1.])
        node.create_timer(.05, lambda: scan_pub.publish(scan))
        bag = work / 'bag'
        harness = work / 'launch_recorder.py'
        harness.write_text('''import runpy, sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
module = runpy.run_path(get_package_share_directory('vehicle_bringup') + '/launch/system.launch.py')
def configure(context):
    context.launch_configurations.update(record='true', record_dir=sys.argv[1], record_duration_sec='8')
    return module['recording_actions'](context, simulation=True)
service = LaunchService(noninteractive=True)
service.include_launch_description(LaunchDescription([
    *[action for action in module['generate_launch_description']().entities if isinstance(action, DeclareLaunchArgument)],
    OpaqueFunction(function=configure),
    ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(120)']),
]))
raise SystemExit(service.run())
''')
        environment = os.environ.copy()
        if ending == 'failure':
            fake = work / 'bin'
            fake.mkdir()
            executable = fake / 'ros2'
            executable.write_text('#!/bin/sh\nexit 7\n')
            executable.chmod(0o755)
            environment['PATH'] = str(fake) + os.pathsep + environment['PATH']
        process = None
        try:
            with (work / 'launch.log').open('w') as log:
                started = time.monotonic()
                process = subprocess.Popen([sys.executable, str(harness), str(bag)], env=environment,
                                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                if ending == 'failure':
                    wait_for(node, lambda: 'Recorder exited with code 7' in (work / 'launch.log').read_text())
                    # Wait beyond the original limit: canceled timer must not affect the survivor.
                    wait_for(node, lambda: time.monotonic() - started > 10.)
                    assert process.poll() is None
                    assert str(bag) in (work / 'launch.log').read_text()
                    assert 'Recording time limit reached' not in (work / 'launch.log').read_text()
                else:
                    wait_for(node, lambda: scan_pub.get_subscription_count() > 0, timeout=6.)
                    if ending == 'interrupt':
                        wait_for(node, lambda: time.monotonic() - started > 3.)
                        stop(process)
                    else:
                        wait_for(node, lambda: (bag / 'metadata.yaml').exists(), timeout=15.)
                        assert time.monotonic() - started < 18.
                        assert process.poll() is None, 'Recorder timeout stopped other launch processes'
                    wait_for(node, lambda: scan_pub.get_subscription_count() == 0)
                    assert_bag(bag, {'/scan', '/tf_static'})
        finally:
            if process is not None:
                stop(process)
            node.destroy_node()
            rclpy.shutdown()


@pytest.mark.parametrize('backend', ['hardware', 'simulation'])
def test_backend_recording_smoke(tmp_path, monkeypatch, backend):
    from rosgraph_msgs.msg import Clock
    from vehicle_simulation.process_cleanup import stop_system

    with TemporaryDirectory(dir=tmp_path) as temporary:
        work = Path(temporary)
        simulation = backend == 'simulation'
        monkeypatch.setenv('ROS_DOMAIN_ID', '83' if simulation else '82')
        monkeypatch.setenv('ROS_LOG_DIR', str(work / 'ros_log'))
        environment = os.environ.copy()
        environment.update(GZ_PARTITION='recording_' + uuid4().hex, GZ_HOMEDIR=str(work / 'gz_home'))
        environment.pop('DISPLAY', None)
        bag = work / 'bag'
        command = ['ros2', 'launch', 'vehicle_bringup', 'system.launch.py',
                   f'backend:={backend}', 'hardware_driver:=mock', 'mode:=manual',
                   'rviz:=false', 'gazebo_gui:=false', 'headless_rendering:=true',
                   'record:=true', f'record_dir:={bag}', 'record_duration_sec:=30']
        rclpy.init()
        node = rclpy.create_node('recording_backend_probe')
        scans, clocks = [], []
        node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
        node.create_subscription(Clock, '/clock', clocks.append, qos_profile_sensor_data)
        process = None
        try:
            with (work / 'system.log').open('w') as log:
                process = subprocess.Popen(command, env=environment, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
                wait_for(node, lambda: len(scans) >= 5, timeout=25.)
                if simulation:
                    wait_for(node, lambda: bool(clocks), timeout=5.)
                    paused = subprocess.run(['gz', 'service', '-s', '/world/evaluation/control',
                        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', 'pause: true'],
                        env=environment, capture_output=True, text=True, timeout=10.)
                    assert paused.returncode == 0 and 'true' in paused.stdout, paused
                    # Drain in-flight clock messages and verify simulation time actually stops.
                    deadline = time.monotonic() + 1.
                    wait_for(node, lambda: time.monotonic() > deadline)
                    stamp = clocks[-1].clock
                    deadline = time.monotonic() + 1.
                    wait_for(node, lambda: time.monotonic() > deadline)
                    assert clocks[-1].clock == stamp
                wait_for(node, lambda: (bag / 'metadata.yaml').exists(), timeout=40.)
                assert process.poll() is None, (work / 'system.log').read_text()[-4000:]
                wait_for(node, lambda: 'vehicle_recorder' not in node.get_node_names())
                assert 'command_guard' in node.get_node_names()
                expected = {'/scan', '/tf_static', '/odom', '/actuator/status'}
                if simulation:
                    expected |= {'/clock', '/ground_truth/odom'}
                assert_bag(bag, expected)
        finally:
            if process is not None:
                stop_system(process, environment['GZ_PARTITION'], simulation=simulation, environment=environment)
            node.destroy_node()
            rclpy.shutdown()
