import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import pytest


def load_launch():
    filename = Path(get_package_share_directory('vehicle_simulation')) / 'launch/gazebo.launch.py'
    spec = importlib.util.spec_from_file_location('gazebo_launch_test', filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('gui', ['true', 'false'])
@pytest.mark.parametrize('rendering', ['true', 'false'])
def test_gui_and_headless_rendering_are_independent(tmp_path, monkeypatch, gui, rendering):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    module = load_launch()
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(gazebo_gui=gui, headless_rendering=rendering)
    for action in module.launch_backend(context):
        if isinstance(action, IncludeLaunchDescription):
            arguments = dict(action.launch_arguments)
            if 'gz_args' in arguments:
                flags = perform_substitutions(context, normalize_to_list_of_substitutions(arguments['gz_args']))
                assert (' -s' in flags) is (gui == 'false')
                assert ('--headless-rendering' in flags) is (rendering == 'true')
                break
    else:
        pytest.fail('Gazebo server include is missing')


def test_invalid_spawn_fails_before_starting_any_process(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    module = load_launch()
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations['spawn_x'] = 'nan'
    with pytest.raises(ValueError, match='finite'):
        module.launch_backend(context)
