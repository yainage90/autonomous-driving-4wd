from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planning_share = Path(
        get_package_share_directory('vehicle_planning')
    )
    control_share = Path(
        get_package_share_directory('vehicle_control')
    )

    nav2_params = str(planning_share / 'config' / 'nav2.yaml')
    controller_params = str(control_share / 'config' / 'controller.yaml')
    bt_xml = str(
        planning_share / 'behavior_trees' / 'navigate_to_pose.xml'
    )

    autostart_argument = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate navigation servers',
    )
    autostart = ParameterValue(
        LaunchConfiguration('autostart'),
        value_type=bool,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params],
        output='screen',
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[
            nav2_params,
            controller_params,
            {'enable_stamped_cmd_vel': False},
        ],
        remappings=[('cmd_vel', 'cmd_vel_auto')],
        output='screen',
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[
            nav2_params,
            {'default_nav_to_pose_bt_xml': bt_xml},
        ],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': autostart,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
            ],
        }],
        output='screen',
    )

    return LaunchDescription([
        autostart_argument,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager,
    ])