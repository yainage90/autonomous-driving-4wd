"""Planner definition shared by integrated and standalone execution."""
from launch_ros.actions import Node
from vehicle_planning.launch_config import sim_time, timed_params


def planner_node(params):
    return Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        parameters=[timed_params(params, 'planner_server', 'global_costmap.global_costmap'),
                    {'use_sim_time': sim_time()}], output='screen',
    )
