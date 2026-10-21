from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from vehicle_planning.launch_config import sim_time
from vehicle_control.nav2_nodes import controller_node
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

    controller_server = controller_node(LaunchConfiguration('nav2_params_file'), LaunchConfiguration('controller_params_file'))

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_control',
        parameters=[{
            'use_sim_time': sim_time(),
            'autostart': True,
            'node_names': ['controller_server'],
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller_params_file', default_value=str(controller_params)),
        DeclareLaunchArgument('nav2_params_file', default_value=str(costmap_params)),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        controller_server,
        lifecycle_manager,
    ])