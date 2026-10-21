"""Compatibility entry point; install vehicle_simulation to use Gazebo."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('vehicle_simulation')
    return LaunchDescription([
        LogInfo(msg='Gazebo moved to: ros2 launch vehicle_simulation gazebo.launch.py'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(f'{share}/launch/gazebo.launch.py')),
    ])
