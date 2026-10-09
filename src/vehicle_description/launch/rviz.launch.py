import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vehicle_description')

    rviz_config = os.path.join(
        package_share, 'rviz', 'vehicle.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([rviz])
    
    