from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from vehicle_planning.launch_config import sim_time
from vehicle_planning.nav2_nodes import planner_node
from launch_ros.actions import Node


def generate_launch_description():
    planning_share = Path(
        get_package_share_directory('vehicle_planning')
    )
    params_file = planning_share / 'config' / 'nav2.yaml'

    planner_server = planner_node(LaunchConfiguration('nav2_params_file'))

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        parameters=[{
            'use_sim_time': sim_time(),
            'autostart': True,
            'node_names': ['planner_server'],
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params_file', default_value=str(params_file)),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        planner_server,
        lifecycle_manager,
    ])