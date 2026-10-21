"""Controller definition shared by integrated and standalone execution."""
from launch_ros.actions import Node
from vehicle_planning.launch_config import sim_time, timed_params


def controller_node(nav2_params, controller_params):
    return Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        parameters=[
            timed_params(nav2_params, 'controller_server', 'local_costmap.local_costmap'),
            timed_params(controller_params, 'controller_server', 'local_costmap.local_costmap'),
            {'use_sim_time': sim_time(), 'enable_stamped_cmd_vel': True},
        ], remappings=[('cmd_vel', 'cmd_vel_auto')], output='screen',
    )
