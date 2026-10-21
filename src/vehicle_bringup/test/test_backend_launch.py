"""Validate backend isolation and nested time overrides before starting processes."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.utilities import perform_substitutions
import pytest
import yaml

from vehicle_planning.launch_config import timed_params


@pytest.fixture(autouse=True)
def launch_logs(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))


def system():
    filename = Path(get_package_share_directory('vehicle_bringup')) / 'launch/system.launch.py'
    spec = importlib.util.spec_from_file_location('system_launch_test', filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return context


@pytest.mark.parametrize('mode', ['manual', 'mapping', 'navigation'])
def test_hardware_backend_never_looks_up_simulator_packages(mode):
    module = system()
    context = context_for(module, backend='hardware', hardware_driver='mock', mode=mode, rviz='false')
    looked_up = []
    def lookup(name):
        looked_up.append(name)
        assert name not in ('vehicle_simulation', 'vehicle_actuator_sim', 'ros_gz_sim', 'ros_gz_bridge')
        return get_package_share_directory(name)
    with patch.object(module, 'share', lookup):
        assert len(module.configure(context)) == 2
    assert 'vehicle_drivers' in looked_up


@pytest.mark.parametrize('backend,clock', [('hardware', 'true'), ('simulation', 'false')])
def test_mismatched_clock_fails_before_backend_resolution(backend, clock):
    module = system()
    context = context_for(module, backend=backend, use_sim_time=clock)
    with patch.object(module, 'share', side_effect=AssertionError('Unexpected package lookup')):
        with pytest.raises(ValueError, match='requires use_sim_time'):
            module.configure(context)


def test_real_hardware_fails_without_falling_back():
    module = system()
    context = context_for(module, backend='hardware')
    with patch.object(module, 'share', side_effect=AssertionError('Unexpected package lookup')):
        with pytest.raises(RuntimeError, match='not implemented'):
            module.configure(context)


def test_missing_configuration_fails_before_process_creation(tmp_path):
    module = system()
    context = context_for(module, backend='hardware', hardware_driver='mock',
                          safety_params_file=str(tmp_path / 'missing.yaml'))
    with pytest.raises(ValueError, match='safety_params_file file does not exist'):
        module.configure(context)


def test_recording_disabled_does_not_read_config_or_create_directory(tmp_path):
    module = system()
    output = tmp_path / 'unused' / 'bag'
    context = context_for(module, record_dir=str(output), record_duration_sec='invalid')
    with patch.object(module, 'share', side_effect=AssertionError('Recording config lookup')):
        assert module.recording_actions(context, simulation=True) == []
    assert not output.parent.exists()


@pytest.mark.parametrize('backend', ['simulation', 'hardware'])
def test_recording_command_and_backend_isolation(tmp_path, backend):
    module = system()
    output = tmp_path / 'recordings with spaces' / 'bag'
    context = context_for(module, record='true', backend=backend, hardware_driver='mock',
                          mode='manual', rviz='false', record_dir=str(output))
    def lookup(name):
        if backend == 'hardware':
            assert name not in ('vehicle_simulation', 'vehicle_actuator_sim', 'ros_gz_sim')
        return get_package_share_directory(name)
    with patch.object(module, 'share', lookup):
        actions = module.configure(context)
    recorder, = [action for action in actions if isinstance(action, ExecuteProcess)]
    command = [perform_substitutions(context, part) for part in recorder.cmd]
    assert command[:5] == ['ros2', 'bag', 'record', '-s', 'mcap']
    assert command[command.index('-o') + 1] == str(output)
    assert command[command.index('--max-bag-size') + 1] == '268435456'
    assert command[command.index('--max-cache-size') + 1] == '16777216'
    assert '--use-sim-time' not in command and '--no-discovery' not in command
    assert '-a' not in command and '--disable-keyboard-controls' in command
    assert ('/clock' in command) is (backend == 'simulation')
    assert ('/ground_truth/odom' in command) is (backend == 'simulation')
    assert '/scan' in command and '/tf_static' in command
    assert output.parent.is_dir() and not output.exists()


@pytest.mark.parametrize('argument,value', [
    ('record_duration_sec', 'nan'), ('record_duration_sec', 'inf'),
    ('record_duration_sec', '0'), ('record_duration_sec', '-1'),
    ('record_max_bag_size', '0'), ('record_max_bag_size', '-1'),
    ('record_max_bag_size', '1.5'),
])
def test_invalid_recording_limits_have_no_filesystem_side_effect(tmp_path, argument, value):
    module = system()
    output = tmp_path / 'new' / 'bag'
    context = context_for(module, record='true', record_dir=str(output), **{argument: value})
    with pytest.raises(ValueError):
        module.recording_actions(context, simulation=True)
    assert not output.parent.exists()


def test_recording_rejects_reused_directory_and_file_parent(tmp_path):
    module = system()
    marker = tmp_path / 'keep.txt'
    marker.write_text('preserve')
    context = context_for(module, record='true', record_dir=str(tmp_path))
    with pytest.raises(ValueError, match='already exists'):
        module.recording_actions(context, simulation=True)
    assert marker.read_text() == 'preserve'
    context.launch_configurations['record_dir'] = str(marker / 'bag')
    with pytest.raises(OSError):
        module.recording_actions(context, simulation=True)


@pytest.mark.parametrize('clock', ['true', 'false'])
@pytest.mark.parametrize('existing', [True, False])
def test_time_override_reaches_nested_costmap_with_and_without_keys(tmp_path, clock, existing):
    parameters = {'frequency': 20.}
    if existing:
        parameters['use_sim_time'] = clock != 'true'
    source = tmp_path / 'parameters.yaml'
    source.write_text(yaml.safe_dump({'controller_server': {'ros__parameters': parameters},
        'local_costmap': {'local_costmap': {'ros__parameters': parameters.copy()}}}))
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = clock
    rewritten = timed_params(str(source), 'controller_server', 'local_costmap.local_costmap')
    try:
        result = yaml.safe_load(Path(rewritten.evaluate(context)).read_text())
        assert result['controller_server']['ros__parameters']['use_sim_time'] is (clock == 'true')
        assert result['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] is (clock == 'true')
        assert result['controller_server']['ros__parameters']['frequency'] == 20.
    finally:
        rewritten.cleanup()
