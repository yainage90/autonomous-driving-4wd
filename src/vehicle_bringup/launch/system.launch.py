from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    package_share = get_package_share_directory('vehicle_bringup')
    safety_config = f'{package_share}/config/safety.yaml'

    simulation_share = get_package_share_directory('vehicle_simulation')
    world_argument = DeclareLaunchArgument(
        'world',
        default_value=f'{simulation_share}/worlds/evaluation.sdf',
        description='Gazebo world file path',
    )

    world = LaunchConfiguration('world')

    initial_mode = PythonExpression([
        "'auto' if '",
        LaunchConfiguration('mode'),
        "' == 'navigation' else 'manual'",
    ])

    command_guard = Node(
        package = 'vehicle_control',
        executable='command_guard',
        parameters=[
            LaunchConfiguration('safety_params_file'),
            {'initial_mode': initial_mode, 'use_sim_time': True},
        ],
        output='screen',
    )

    description_share = get_package_share_directory('vehicle_description')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{description_share}/launch/gazebo.launch.py',
        ),
        launch_arguments={
            'world': world,
            'safety_params_file': LaunchConfiguration('safety_params_file'),
        }.items(),
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{description_share}/launch/rviz.launch.py'
        ),
        launch_arguments={
            'fixed_frame': PythonExpression([
                "'odom' if '", LaunchConfiguration('mode'), "' == 'manual' else 'map'",
            ]),
        }.items()
    )

    localization_share = get_package_share_directory('vehicle_localization')

    mode_argument = DeclareLaunchArgument(
        'mode',
        default_value='navigation',
        choices=['manual', 'mapping', 'navigation'],
        description='Manual driving, mapping or navigation mode',
    )

    map_argument = DeclareLaunchArgument(
        'map',
        default_value=f'{localization_share}/maps/evaluation.yaml',
        description='Saved map YAML file path',
    )

    slam_params_argument = DeclareLaunchArgument(
        'slam_params_file',
        default_value=f'{localization_share}/config/slam.yaml',
        description='SLAM Toolbox parameter file',
    )

    planning_share = get_package_share_directory('vehicle_planning')
    control_share = get_package_share_directory('vehicle_control')

    safety_params_argument = DeclareLaunchArgument(
        'safety_params_file',
        default_value=safety_config,
        description='Command guard parameter file',
    )

    amcl_params_argument = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=f'{localization_share}/config/amcl.yaml',
        description='AMCL parameter file',
    )

    nav2_params_argument = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=f'{planning_share}/config/nav2.yaml',
        description='Nav2 parameter file',
    )

    controller_params_argument = DeclareLaunchArgument(
        'controller_params_file',
        default_value=f'{control_share}/config/controller.yaml',
        description='Controller parameter file',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{localization_share}/launch/localization.launch.py'
        ),
        condition=LaunchConfigurationEquals('mode', 'navigation'),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'amcl_params_file': LaunchConfiguration('amcl_params_file'),
            'use_sim_time': 'true',
        }.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{localization_share}/launch/mapping.launch.py'
        ),
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': 'true',
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{package_share}/launch/nav2.launch.py'
        ),
        condition=LaunchConfigurationEquals('mode', 'navigation'),
        launch_arguments={
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'controller_params_file': LaunchConfiguration('controller_params_file'),
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription(
        [
            world_argument,
            mode_argument,
            map_argument,
            slam_params_argument,
            safety_params_argument,
            amcl_params_argument,
            nav2_params_argument,
            controller_params_argument,
            simulation,
            visualization,
            command_guard,
            localization,
            mapping,
            nav2,
        ]
    )
