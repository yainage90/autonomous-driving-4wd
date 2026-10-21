"""Gazebo backend: model, simulator, spawn, and one-way sensor bridges."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_backend(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    world = Path(value('world'))
    if not world.is_file():
        raise ValueError(f'Gazebo world does not exist: {world}')
    poses = ['spawn_x', 'spawn_y', 'spawn_z', 'spawn_roll', 'spawn_pitch', 'spawn_yaw']
    if not all(math.isfinite(float(value(name))) for name in poses):
        raise ValueError('Spawn pose values must be finite')
    simulation = Path(share('vehicle_simulation'))
    flags = '-r'
    if value('gazebo_gui') == 'false':
        flags += ' -s'
    if value('headless_rendering') == 'true':
        flags += ' --headless-rendering'
    return [
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                  str(Path(get_package_prefix('vehicle_actuator_sim')) / 'lib')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(Path(share('vehicle_description')).parent)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            f"{share('vehicle_description')}/launch/description.launch.py"), launch_arguments={
                'simulation': 'true', 'use_sim_time': 'true',
                'safety_config_file': value('safety_params_file'),
                'lidar_config_file': value('lidar_config_file'),
                'simulation_config_file': value('simulation_config_file'),
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            f"{share('ros_gz_sim')}/launch/gz_sim.launch.py"),
            launch_arguments={'gz_args': f'{flags} "{world}"'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='vehicle_bridge',
             parameters=[{'config_file': str(simulation / 'config/bridge.yaml'), 'use_sim_time': True}],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='scan_bridge',
             parameters=[{'config_file': str(simulation / 'config/scan_bridge.yaml'), 'use_sim_time': True}],
             output='screen'),
        Node(package='ros_gz_sim', executable='create', name='spawn_vehicle',
             arguments=['-topic', 'robot_description', '-name', 'vehicle',
                        *[item for flag, name in zip(
                            ['-x', '-y', '-z', '-R', '-P', '-Y'], poses)
                          for item in (flag, value(name))]], output='screen'),
    ]


def generate_launch_description():
    simulation = Path(share('vehicle_simulation'))
    defaults = {
        'world': str(simulation / 'worlds/evaluation.sdf'),
        'safety_params_file': f"{share('vehicle_actuator_sim')}/config/safety.yaml",
        'lidar_config_file': str(simulation / 'config/lidar.yaml'),
        'simulation_config_file': str(simulation / 'config/simulation.yaml'),
        'spawn_x': '-2.0', 'spawn_y': '0.0', 'spawn_z': '0.02',
        'spawn_roll': '0.0', 'spawn_pitch': '0.0', 'spawn_yaw': '0.0',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('headless_rendering', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_backend),
    ])
