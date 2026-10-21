"""Select an environment independently of the driving mode."""
import math
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration


def recording_actions(context, simulation):
    """Record selected topics without coupling recorder exit to vehicle shutdown."""
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('record') == 'false':
        return []
    if value('record') != 'true':
        raise ValueError('record must be true or false')
    duration = float(value('record_duration_sec'))
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('record_duration_sec must be finite and positive')
    max_size = int(value('record_max_bag_size'))
    if max_size <= 0:
        raise ValueError('record_max_bag_size must be positive')

    import yaml
    config_dir = Path(share('vehicle_bringup')) / 'config'
    settings = yaml.safe_load((config_dir / 'recording.yaml').read_text())
    topics = settings['common_topics'] + (settings['simulation_topics'] if simulation else [])
    if not topics or any(not isinstance(topic, str) or not topic.startswith('/') for topic in topics):
        raise ValueError('Recording topics must be nonempty absolute topic names')
    cache_size = settings['max_cache_size']
    if type(cache_size) is not int or cache_size <= 0:
        raise ValueError('Recording max_cache_size must be a positive integer')
    qos_file = config_dir / 'recording_qos.yaml'
    if not qos_file.is_file():
        raise ValueError(f'Recording QoS file does not exist: {qos_file}')

    name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_' + uuid4().hex[:8]
    output = Path(value('record_dir') or Path.cwd() / 'log' / 'bags' / name).expanduser()
    if output.exists() or output.is_symlink():
        raise ValueError(f'Recording directory already exists: {output}')
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(output),
             '--node-name', 'vehicle_recorder', '--disable-keyboard-controls',
             '--max-bag-size', str(max_size), '--max-cache-size', str(cache_size),
             '--qos-profile-overrides-path', str(qos_file), '--topics', *dict.fromkeys(topics)],
        name='vehicle_recorder', output='screen', sigterm_timeout='15', sigkill_timeout='5',
    )
    expired = False

    def stop_recording(context):
        nonlocal expired
        if recorder.return_code is not None or context.is_shutdown:
            return []
        expired = True
        # ShutdownProcess sends SIGINT first, with bounded escalation for this action only.
        return [LogInfo(msg=f'Recording time limit reached: {output}'),
                EmitEvent(event=ShutdownProcess(process_matcher=matches_action(recorder)))]

    timer = TimerAction(period=duration, actions=[OpaqueFunction(function=stop_recording)])

    def exited(event, context):
        timer.cancel()
        message = f'Recorder exited with code {event.returncode}: {output}'
        if event.returncode != 0 or (not expired and not context.is_shutdown):
            get_logger('vehicle_recording').warning(
                message + '; recording stopped, vehicle processes continue')
            return []
        return [LogInfo(msg=message)]

    return [
        RegisterEventHandler(OnProcessStart(target_action=recorder, on_start=[
            LogInfo(msg=f'Recording to {output}; wall-clock limit {duration:g} s'), timer,
        ])),
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=exited)),
        recorder,
    ]


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
    return recording_actions(context, simulation) + actions


def generate_launch_description():
    choices = {
        'backend': ('simulation', ['simulation', 'hardware']),
        'mode': ('navigation', ['manual', 'mapping', 'navigation']),
        'hardware_driver': ('real', ['real', 'mock']),
        'use_sim_time': ('auto', ['auto', 'true', 'false']),
        'rviz': ('auto', ['auto', 'true', 'false']),
        'gazebo_gui': ('true', ['true', 'false']),
        'headless_rendering': ('false', ['true', 'false']),
        'record': ('false', ['true', 'false']),
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
        'record_dir': '', 'record_duration_sec': '300',
        'record_max_bag_size': '268435456',
    })
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, choices=allowed)
          for name, (default, allowed) in choices.items()],
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=configure),
    ])
