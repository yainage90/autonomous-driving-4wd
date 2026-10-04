import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'vehicle_description'
    pkg_share = get_package_share_directory(pkg_name)

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    simulation_share = get_package_share_directory('vehicle_simulation')

    default_world = f'{simulation_share}/worlds/evaluation.sdf'

    world_argument = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Gazebo world file path',
    )

    world = LaunchConfiguration('world')

    # xacro file path
    xacro_file = os.path.join(pkg_share, 'urdf', 'vehicle.urdf.xacro')

    # xacro 명령을 실행하여 URDF 생성
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Robot State Publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # Gazebo의 관절 상태와 시뮬레이션 시계를 ROS 2에 전달
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # Gazebo Harmonic 실행 (empty.sdf 월드)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r "', world, '"']}.items()
    )

    # Gazebo에 로봇 모델 스폰 (Spawn)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'vehicle',
            '-x', '-2.0',
            '-y', '0.0',
            '-z', '0.02',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    return LaunchDescription([
        world_argument,
        robot_state_publisher_node,
        bridge,
        gazebo,
        spawn_entity,
    ])
