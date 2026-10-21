"""Publish the common model; simulator extensions are explicitly opt-in."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def publish_model(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    model = Path(get_package_share_directory('vehicle_description')) / 'urdf/vehicle.urdf.xacro'
    mappings = {name: value(name) for name in (
        'simulation', 'safety_config_file', 'lidar_config_file', 'simulation_config_file',
    ) if value(name)}
    description = xacro.process_file(str(model), mappings=mappings).toxml()
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description,
                     'use_sim_time': value('use_sim_time').lower() == 'true'}],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('safety_config_file', default_value=''),
        DeclareLaunchArgument('lidar_config_file', default_value=''),
        DeclareLaunchArgument('simulation_config_file', default_value=''),
        OpaqueFunction(function=publish_model),
    ])
