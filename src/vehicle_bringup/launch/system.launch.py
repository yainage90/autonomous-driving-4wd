from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    package_share = get_package_share_directory('vehicle_bringup')
    safety_config = f'{package_share}/config/safety.yaml'

    simulation_share = get_package_share_directory('vehicle_simulation')
    world_argument = DeclareLaunchArgument(
        'world',
        default_value=f'{simulation_share}/worlds/evaluation.sdf',
        description='Gazebo world file path',
    )

    world = LaunchConfiguration('world')

    command_guard = Node(
        package = 'vehicle_control',
        executable='command_guard',
        parameters=[safety_config],
        output='screen',
    )

    description_share = get_package_share_directory('vehicle_description')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{description_share}/launch/gazebo.launch.py',
        ),
        launch_arguments={'world': world}.items(),
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{description_share}/launch/rviz.launch.py'
        )
    )

    localization_share = get_package_share_directory('vehicle_localization')

    mode_argument = DeclareLaunchArgument(
        'mode',
        default_value='localization',
        choices=['localization', 'mapping'],
        description='Localization or mapping mode',
    )

    map_argument = DeclareLaunchArgument(
        'map',
        default_value=f'{localization_share}/maps/evaluation.yaml',
        description='Saved map YAML file path',
    )

    slam_params_argument = DeclareLaunchArgument(
        'slam_params_file',
        default_value=f'{localization_share}/config/slam.yaml',
        description='SLAM Toolbox parameter file',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{localization_share}/launch/localization.launch.py'
        ),
        condition=LaunchConfigurationEquals('mode', 'localization'),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
        }.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{localization_share}/launch/mapping.launch.py'
        ),
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription(
        [
            world_argument,
            mode_argument,
            map_argument,
            slam_params_argument,
            simulation,
            visualization,
            command_guard,
            localization,
            mapping,
        ]
    )
