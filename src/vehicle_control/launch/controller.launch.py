from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_share = Path(
        get_package_share_directory('vehicle_control')
    )
    planning_share = Path(
        get_package_share_directory('vehicle_planning')
    )

    controller_params = control_share / 'config' / 'controller.yaml'
    costmap_params = planning_share / 'config' / 'nav2.yaml'

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[
            str(costmap_params),
            str(controller_params),
            {'enable_stamped_cmd_vel': False},
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel_auto'),
        ],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_control',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
        output='screen',
    )

    return LaunchDescription([
        controller_server,
        lifecycle_manager,
    ])