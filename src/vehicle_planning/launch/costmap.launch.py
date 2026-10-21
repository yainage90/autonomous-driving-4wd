from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from vehicle_planning.launch_config import sim_time, timed_params
from launch_ros.actions import Node


def generate_launch_description():
    planning_share = Path(
        get_package_share_directory('vehicle_planning')
    )
    params_file = planning_share / 'config' / 'nav2.yaml'

    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='global_costmap',
        name='global_costmap',
        parameters=[timed_params(LaunchConfiguration('nav2_params_file'), 'global_costmap.global_costmap', 'local_costmap.local_costmap'), {'use_sim_time': sim_time()}],
        output='screen',
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        parameters=[timed_params(LaunchConfiguration('nav2_params_file'), 'global_costmap.global_costmap', 'local_costmap.local_costmap'), {'use_sim_time': sim_time()}],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        parameters=[{
            'use_sim_time': sim_time(),
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                '/global_costmap/global_costmap', 
                '/local_costmap/local_costmap',
            ],
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params_file', default_value=str(params_file)),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        global_costmap,
        local_costmap,
        lifecycle_manager,
    ])