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

    nav2_params_argument = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=str(planning_share / 'config' / 'nav2.yaml'),
        description='Nav2 parameter file',
    )

    controller_params_argument = DeclareLaunchArgument(
        'controller_params_file',
        default_value=str(control_share / 'config' / 'controller.yaml'),
        description='Controller parameter file',
    )

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
        parameters=[LaunchConfiguration('nav2_params_file')],
        output='screen',
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            LaunchConfiguration('controller_params_file'),
            {'enable_stamped_cmd_vel': True},
        ],
        remappings=[('cmd_vel', 'cmd_vel_auto')],
        output='screen',
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'enable_stamped_cmd_vel': True},
        ],
        remappings=[('cmd_vel', 'cmd_vel_auto')],
        output='screen',
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'default_nav_to_pose_bt_xml': bt_xml},
        ],
        remappings=[
            ('goal_pose', 'nav2_internal/goal_pose')
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
                'behavior_server',
                'bt_navigator',
            ],
        }],
        output='screen',
    )

    navigation_manager = Node(
        package='vehicle_planning',
        executable='navigation_manager',
        name='navigation_manager',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        autostart_argument,
        nav2_params_argument,
        controller_params_argument,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        navigation_manager,
        lifecycle_manager,
    ])