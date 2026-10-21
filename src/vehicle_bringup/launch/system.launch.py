"""Select an environment independently of the driving mode."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def configure(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    backend, mode = value('backend'), value('mode')
    simulation = backend == 'simulation'
    use_sim_time = 'true' if simulation else 'false'
    if value('use_sim_time') not in ('auto', use_sim_time):
        raise ValueError(f'backend={backend} requires use_sim_time={use_sim_time}')
    if not simulation and value('hardware_driver') != 'mock':
        raise RuntimeError('Hardware driver real is not implemented; explicitly select hardware_driver:=mock for tests')
    if simulation and not all(math.isfinite(float(value(name))) for name in (
            'spawn_x', 'spawn_y', 'spawn_z', 'spawn_roll', 'spawn_pitch', 'spawn_yaw')):
        raise ValueError('Spawn pose values must be finite')

    defaults = {'safety_params_file': f"{share('vehicle_bringup')}/config/safety.yaml"}
    if mode == 'navigation' or not simulation:
        defaults['map'] = f"{share('vehicle_localization')}/maps/evaluation.yaml"
    if mode == 'mapping':
        defaults['slam_params_file'] = f"{share('vehicle_localization')}/config/slam.yaml"
    if mode == 'navigation':
        defaults.update({
            'amcl_params_file': f"{share('vehicle_localization')}/config/amcl.yaml",
            'nav2_params_file': f"{share('vehicle_planning')}/config/nav2.yaml",
            'controller_params_file': f"{share('vehicle_control')}/config/controller.yaml",
        })
    if simulation:
        defaults.update({
            'world': f"{share('vehicle_simulation')}/worlds/evaluation.sdf",
            'lidar_config_file': f"{share('vehicle_simulation')}/config/lidar.yaml",
            'simulation_config_file': f"{share('vehicle_simulation')}/config/simulation.yaml",
        })
    resolved = {}
    for name, default in defaults.items():
        resolved[name] = value(name) or default
        if not Path(resolved[name]).is_file():
            raise ValueError(f'{name} file does not exist: {resolved[name]}')
    if 'map' in resolved:
        import yaml
        map_path = Path(resolved['map'])
        metadata = yaml.safe_load(map_path.read_text())
        image = map_path.parent / metadata['image']
        if not image.is_file():
            raise ValueError(f'Map image does not exist: {image}')

    def include(package, filename, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            f'{share(package)}/launch/{filename}'), launch_arguments=arguments.items())

    if simulation:
        backend_args = {name: resolved[name] for name in (
            'world', 'safety_params_file', 'lidar_config_file', 'simulation_config_file')}
        backend_args.update({name: value(name) for name in (
            'gazebo_gui', 'headless_rendering', 'spawn_x', 'spawn_y', 'spawn_z',
            'spawn_roll', 'spawn_pitch', 'spawn_yaw')})
        actions = [include('vehicle_simulation', 'gazebo.launch.py', backend_args)]
    else:
        actions = [include('vehicle_drivers', 'hardware.launch.py', {
            'hardware_driver': value('hardware_driver'), 'map': resolved['map'],
            'safety_params_file': resolved['safety_params_file'],
        })]
    common_args = {**resolved, 'mode': mode, 'use_sim_time': use_sim_time,
                   'manager_odom_topic': value('manager_odom_topic')}
    actions.append(include('vehicle_bringup', 'common.launch.py', common_args))
    rviz = simulation if value('rviz') == 'auto' else value('rviz') == 'true'
    if rviz:
        actions.append(include('vehicle_description', 'rviz.launch.py', {
            'use_sim_time': use_sim_time,
            'fixed_frame': 'odom' if mode == 'manual' else 'map',
        }))
    return actions


def generate_launch_description():
    choices = {
        'backend': ('simulation', ['simulation', 'hardware']),
        'mode': ('navigation', ['manual', 'mapping', 'navigation']),
        'hardware_driver': ('real', ['real', 'mock']),
        'use_sim_time': ('auto', ['auto', 'true', 'false']),
        'rviz': ('auto', ['auto', 'true', 'false']),
        'gazebo_gui': ('true', ['true', 'false']),
        'headless_rendering': ('false', ['true', 'false']),
    }
    defaults = dict.fromkeys([
        'world', 'map', 'safety_params_file', 'slam_params_file', 'amcl_params_file',
        'nav2_params_file', 'controller_params_file', 'lidar_config_file',
        'simulation_config_file',
    ], '')
    defaults.update({
        'spawn_x': '-2.0', 'spawn_y': '0.0', 'spawn_z': '0.02',
        'spawn_roll': '0.0', 'spawn_pitch': '0.0', 'spawn_yaw': '0.0',
        'manager_odom_topic': 'odom',
    })
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, choices=allowed)
          for name, (default, allowed) in choices.items()],
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=configure),
    ])
