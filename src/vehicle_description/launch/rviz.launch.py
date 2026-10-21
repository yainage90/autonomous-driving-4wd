import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('vehicle_description')

    rviz_config = os.path.join(
        package_share, 'rviz', 'vehicle.rviz'
    )

    fixed_frame_argument = DeclareLaunchArgument(
        'fixed_frame',
        default_value='map',
        description='RViz fixed frame',
    )

    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_config,
            '-f', LaunchConfiguration('fixed_frame')
        ],
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        fixed_frame_argument,
        rviz,
    ])
    
    