"""Exercise the production system with only manager odometry remapped."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('vehicle_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('manager_odom_topic', default_value='recovery/odom'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(f'{share}/launch/system.launch.py'),
                                 launch_arguments={
                                     'rviz': 'false', 'gazebo_gui': 'false',
                                     'headless_rendering': 'true',
                                     'manager_odom_topic': LaunchConfiguration('manager_odom_topic'),
                                 }.items()),
    ])
