from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    localization_share = Path(
        get_package_share_directory('vehicle_localization')
    )

    map_argument = DeclareLaunchArgument(
        'map',
        default_value=str(localization_share / 'maps' / 'evaluation.yaml'),
        description='Saved map YAML file path',
    )

    sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the simulation clock',
    )

    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool,
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'frame_id': 'map',
            'topic_name': 'map',
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            str(localization_share / 'config' / 'amcl.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return LaunchDescription([
        map_argument,
        sim_time_argument,
        map_server,
        amcl,
        lifecycle_manager,
    ])
