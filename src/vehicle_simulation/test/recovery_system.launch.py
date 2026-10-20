"""Isolated headless test system; only manager odometry passes through the probe."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def description(package, filename):
    path = Path(share(package)) / 'launch' / filename
    spec = importlib.util.spec_from_file_location(package + '_test_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def generate_launch_description():
    actions = [DeclareLaunchArgument('manager_odom_topic', default_value='recovery/odom')]
    for action in description('vehicle_description', 'gazebo.launch.py').entities:
        if isinstance(action, IncludeLaunchDescription):
            action = IncludeLaunchDescription(PythonLaunchDescriptionSource(
                str(Path(share('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
                launch_arguments={'gz_args': [
                    '-s -r --headless-rendering "', LaunchConfiguration('world'), '"',
                ]}.items())
        actions.append(action)
    actions.append(Node(
        package='vehicle_control', executable='command_guard',
        parameters=[str(Path(share('vehicle_bringup')) / 'config' / 'safety.yaml'),
                    {'use_sim_time': True, 'initial_mode': 'auto'}], output='screen'))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(share('vehicle_localization')) / 'launch' / 'localization.launch.py'))))
    for action in description('vehicle_bringup', 'nav2.launch.py').entities:
        if isinstance(action, Node) and action.node_executable == 'navigation_manager':
            action = GroupAction([
                SetRemap(src='odom', dst=LaunchConfiguration('manager_odom_topic')), action,
            ])
        actions.append(action)
    return LaunchDescription(actions)
