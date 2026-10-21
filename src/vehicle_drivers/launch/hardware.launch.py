"""Hardware boundary. A mock is selected explicitly and never as a fallback."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('hardware_driver') != 'mock':
        raise RuntimeError('Hardware driver real is not implemented; explicitly select hardware_driver:=mock for tests')
    if not value('map'):
        raise ValueError('Mock hardware requires map:=<map.yaml> for consistent laser scans')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            f"{get_package_share_directory('vehicle_description')}/launch/description.launch.py"),
            launch_arguments={'simulation': 'false', 'use_sim_time': 'false'}.items()),
        Node(package='vehicle_drivers', executable='mock_hardware', name='mock_hardware',
             parameters=[value('safety_params_file'), {'use_sim_time': False, 'map_file': value('map')}],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_driver', default_value='real', choices=['real', 'mock']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('safety_params_file', default_value=''),
        OpaqueFunction(function=configure),
    ])
