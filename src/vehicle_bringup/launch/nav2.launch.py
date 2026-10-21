from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_planning.launch_config import sim_time, timed_params
from vehicle_planning.nav2_nodes import planner_node
from vehicle_control.nav2_nodes import controller_node


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

    planner_server = planner_node(LaunchConfiguration('nav2_params_file'))
    controller_server = controller_node(
        LaunchConfiguration('nav2_params_file'), LaunchConfiguration('controller_params_file'))

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[
            timed_params(LaunchConfiguration('nav2_params_file'), 'behavior_server', 'bt_navigator'),
            {'enable_stamped_cmd_vel': True, 'use_sim_time': sim_time()},
        ],
        remappings=[('cmd_vel', 'cmd_vel_auto')],
        output='screen',
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[
            timed_params(LaunchConfiguration('nav2_params_file'), 'behavior_server', 'bt_navigator'),
            {'default_nav_to_pose_bt_xml': bt_xml, 'use_sim_time': sim_time()},
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
            'use_sim_time': sim_time(),
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
        parameters=[{'use_sim_time': sim_time()}],
        remappings=[('odom', LaunchConfiguration('manager_odom_topic'))],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('manager_odom_topic', default_value='odom'),
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
