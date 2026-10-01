from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory('vehicle_bringup')
    safety_config = f'{package_share}/config/safety.yaml'

    command_guard = Node(
        package = 'vehicle_control',
        executable='command_guard',
        parameters=[safety_config],
        output='screen',
    )

    description_share = get_package_share_directory('vehicle_description')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=f'{description_share}/launch/gazebo.launch.py',
        )
    )



    return LaunchDescription(
        [
            simulation,
            command_guard,
        ]
    )
