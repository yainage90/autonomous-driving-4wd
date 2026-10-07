
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planning_share = Path(
        get_package_share_directory('vehicle_planning')
    )
    params_file = planning_share / 'config' / 'nav2.yaml'

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        parameters=[str(params_file)],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_footprint',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['/local_costmap/local_costmap'],
        }],
        output='screen',
    )

    return LaunchDescription([
        local_costmap,
        lifecycle_manager,
    ])