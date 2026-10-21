"""Shared driving functions; no simulator or driver selection here."""
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    values = dict(context.launch_configurations)
    value = lambda name: values[name]
    mode = value('mode')
    clock = value('use_sim_time')
    if mode == 'navigation':
        defaults = {
            'map': ('vehicle_localization', 'maps/evaluation.yaml'),
            'amcl_params_file': ('vehicle_localization', 'config/amcl.yaml'),
            'nav2_params_file': ('vehicle_planning', 'config/nav2.yaml'),
            'controller_params_file': ('vehicle_control', 'config/controller.yaml'),
        }
    elif mode == 'mapping':
        defaults = {'slam_params_file': ('vehicle_localization', 'config/slam.yaml')}
    else:
        defaults = {}
    for name, (package, relative) in defaults.items():
        if not values[name]:
            values[name] = f'{share(package)}/{relative}'
    actions = [Node(
        package='vehicle_control', executable='command_guard',
        parameters=[value('safety_params_file'), {
            'initial_mode': 'auto' if mode == 'navigation' else 'manual',
            'use_sim_time': clock == 'true',
        }], output='screen',
    )]

    def include(package, filename, names):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            f'{share(package)}/launch/{filename}'), launch_arguments={
                'use_sim_time': clock, **{name: value(name) for name in names},
            }.items())

    if mode == 'mapping':
        actions.append(include('vehicle_localization', 'mapping.launch.py', ['slam_params_file']))
    elif mode == 'navigation':
        actions.extend([
            include('vehicle_localization', 'localization.launch.py', ['map', 'amcl_params_file']),
            include('vehicle_bringup', 'nav2.launch.py', [
                'nav2_params_file', 'controller_params_file', 'manager_odom_topic']),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='navigation', choices=['manual', 'mapping', 'navigation']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('safety_params_file', default_value=f"{share('vehicle_bringup')}/config/safety.yaml"),
        *[DeclareLaunchArgument(name, default_value='') for name in (
            'map', 'amcl_params_file', 'slam_params_file', 'nav2_params_file', 'controller_params_file')],
        DeclareLaunchArgument('manager_odom_topic', default_value='odom'),
        OpaqueFunction(function=configure),
    ])
