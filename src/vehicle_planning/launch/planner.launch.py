from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planning_share = Path(
        get_package_share_directory('vehicle_planning')
    )
    params_file = planning_share / 'config' / 'nav2.yaml'

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[str(params_file)],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['planner_server'],
        }],
        output='screen',
    )

    return LaunchDescription([
        planner_server,
        lifecycle_manager,
    ])