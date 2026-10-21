"""Validate backend isolation and nested time overrides before starting processes."""
import importlib.util
from pathlib import Path
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
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
