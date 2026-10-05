from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    localization_share = Path(
        get_package_share_directory('vehicle_localization')
    )
    slam_share = Path(
        get_package_share_directory('slam_toolbox')
    )

    params_argument = DeclareLaunchArgument(
        'slam_params_file',
        default_value=str(localization_share / 'config' / 'slam.yaml'),
        description='SLAM Toolbox parameter file',
    )

    sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the simulation clock',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(slam_share / 'launch' / 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    return LaunchDescription([
        params_argument,
        sim_time_argument,
        slam,
    ])
