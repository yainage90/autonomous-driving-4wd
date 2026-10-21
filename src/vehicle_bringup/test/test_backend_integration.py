"""Opt-in production launch matrix in isolated ROS domains/Gazebo partitions."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from uuid import uuid4

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterType
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from vehicle_interfaces.msg import ActuatorStatus, NavigationRecoveryStatus
from vehicle_interfaces.srv import SetNavigationGoal
from vehicle_simulation.process_cleanup import stop_system


pytestmark = pytest.mark.skipif(os.environ.get('RUN_BACKEND_INTEGRATION') != '1',
                                reason='Opt in to isolated production backend launches')


@pytest.mark.parametrize('backend', ['hardware', 'simulation'])
@pytest.mark.parametrize('mode', ['manual', 'mapping', 'navigation'])
def test_backend_mode_contract(tmp_path, monkeypatch, backend, mode):
    simulation = backend == 'simulation'
    domain = str((40 if simulation else 30) + ['manual', 'mapping', 'navigation'].index(mode))
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    environment = os.environ.copy()
    environment['GZ_PARTITION'] = f'separation_{uuid4().hex}'
    environment['GZ_HOMEDIR'] = str(tmp_path / 'gz_home')
    environment.pop('DISPLAY', None)
    command = ['ros2', 'launch', 'vehicle_bringup', 'system.launch.py',
               f'backend:={backend}', f'mode:={mode}', 'hardware_driver:=mock',
               'rviz:=false', 'gazebo_gui:=false', 'headless_rendering:=true']
    log = (tmp_path / 'system.log').open('w')
    process = subprocess.Popen(command, env=environment, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    report = {'backend': backend, 'mode': mode, 'domain': domain, 'command': command}
    try:
        rclpy.init()
        node = rclpy.create_node('backend_contract_probe', parameter_overrides=[
            Parameter('use_sim_time', value=simulation)])
        scans, odometry, maps, statuses, recoveries = [], [], [], [], []
        transforms = set()
        node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/odom', odometry.append, qos_profile_sensor_data)
        node.create_subscription(ActuatorStatus, '/actuator/status', statuses.append, 1)
        node.create_subscription(OccupancyGrid, '/map', maps.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(NavigationRecoveryStatus, '/navigation_manager/recovery_status',
            recoveries.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def record_tf(message):
            for transform in message.transforms:
                key = (transform.header.frame_id, transform.child_frame_id)
                transforms.add(key)
        node.create_subscription(TFMessage, '/tf', record_tf, qos_profile_sensor_data)
        node.create_subscription(TFMessage, '/tf_static', record_tf,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def wait(predicate, timeout=30.):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return
                assert process.poll() is None, (tmp_path / 'system.log').read_text()[-6000:]
            assert predicate(), ((tmp_path / 'system.log').read_text()[-6000:],
                                str(recoveries[-1:]))

        def observe(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)

        def call(client, request, success=True):
            wait(client.service_is_ready)
            future = client.call_async(request)
            wait(future.done)
            result = future.result()
            if hasattr(result, 'success'):
                assert result.success is success, result.message
            return result

        wait(lambda: len(scans) >= 5 and len(odometry) >= 5 and len(statuses) >= 5, 60.)
        expected = {'command_guard', 'robot_state_publisher'}
        forbidden = {'rviz2', 'joint_state_publisher_gui'}
        if mode == 'navigation':
            expected |= {'navigation_manager', 'amcl', 'map_server', 'planner_server',
                         'controller_server', 'behavior_server', 'bt_navigator',
                         'lifecycle_manager_navigation', 'lifecycle_manager_localization'}
            wait(lambda: recoveries and recoveries[-1].data_ready and
                 recoveries[-1].control_ready and recoveries[-1].stop_confirmed, 60.)
            lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
            activation_deadline = time.monotonic() + 60.
            while call(lifecycle, GetState.Request()).current_state.id != State.PRIMARY_STATE_ACTIVE:
                assert time.monotonic() < activation_deadline, 'Nav2 did not activate'
                observe(.1)
            wait(lambda: ('map', 'odom') in transforms)
        elif mode == 'mapping':
            expected.add('slam_toolbox')
            forbidden |= {'navigation_manager', 'amcl', 'planner_server', 'controller_server'}
            wait(lambda: bool(maps), 60.)
        else:
            forbidden |= {'navigation_manager', 'amcl', 'slam_toolbox', 'planner_server', 'controller_server'}
        wait(lambda: expected <= {name for name, namespace in node.get_node_names_and_namespaces()})
        names = [name for name, namespace in node.get_node_names_and_namespaces()]
        assert expected <= set(names), names
        assert forbidden.isdisjoint(names), names
        for name in expected:
            assert names.count(name) == 1, names
        parameter_nodes = {name: '/' + name for name in expected}
        if mode == 'navigation':
            parameter_nodes.update(global_costmap='/global_costmap/global_costmap',
                                   local_costmap='/local_costmap/local_costmap')
        clocks = {}
        for name, path in parameter_nodes.items():
            client = node.create_client(GetParameters, path + '/get_parameters')
            response = call(client, GetParameters.Request(names=['use_sim_time']))
            assert response.values[0].type == ParameterType.PARAMETER_BOOL, (path, response)
            assert response.values[0].bool_value is simulation, (path, response)
            clocks[path] = response.values[0].bool_value
            node.destroy_client(client)
        assert node.count_publishers('/scan') == 1, [info.node_name for info in node.get_publishers_info_by_topic('/scan')]
        assert node.count_publishers('/odom') == 1
        assert node.count_publishers('/actuator/status') == 1
        assert node.count_publishers('/actuator/command') == 1
        assert node.count_publishers('/cmd_vel') == 1
        assert node.count_publishers('/clock') == int(simulation)
        assert node.count_publishers('/ground_truth/odom') == int(simulation)
        assert ('odom', 'base_footprint') in transforms
        assert ('base_footprint', 'base_link') in transforms
        assert ('camera_mount_link', 'camera_pan_link') in transforms
        assert ('camera_pan_link', 'camera_tilt_link') in transforms
        assert ('camera_tilt_link', 'camera_optical_frame') in transforms
        tf_publishers = node.get_publishers_info_by_topic('/tf')
        expected_tf = {'robot_state_publisher', 'vehicle_bridge' if simulation else 'mock_hardware'}
        if mode == 'mapping':
            expected_tf.add('slam_toolbox')
        elif mode == 'navigation':
            expected_tf.add('amcl')
        assert {info.node_name for info in tf_publishers} == expected_tf
        if mode == 'manual':
            assert len(tf_publishers) == 2
        assert node.count_publishers('/tf_static') == 1
        for message in (scans[-1], odometry[-1]):
            stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            age = (node.get_clock().now().nanoseconds - stamp_ns) / 1e9
            assert -.1 <= age < .5, age
        if not simulation:
            assert not any('bridge' in name or 'spawn' in name for name in names)
            if mode == 'navigation':
                goal = node.create_client(SetNavigationGoal, '/navigation_manager/set_goal')
                call(goal, SetNavigationGoal.Request(x=.3, y=0., yaw=0.))
                wait(lambda: recoveries[-1].navigation_state == 'succeeded', 60.)
                assert abs(odometry[-1].twist.twist.linear.x) <= .02
                settings = node.create_client(SetParameters, '/mock_hardware/set_parameters')
                recover = node.create_client(Trigger, '/navigation_manager/recover')
                restart = node.create_client(Trigger, '/mock_hardware/restart_actuator')
                checked_faults = []
                for fault in ('publish_scan', 'publish_odom', 'publish_status', 'restart_actuator'):
                    target = odometry[-1].pose.pose.position.x + .35
                    call(goal, SetNavigationGoal.Request(x=target, y=0., yaw=0.))
                    wait(lambda: odometry[-1].twist.twist.linear.x > .03)
                    session = statuses[-1].actuator_session
                    if fault == 'restart_actuator':
                        call(restart, Trigger.Request())
                        wait(lambda: statuses[-1].actuator_session != session)
                    else:
                        response = call(settings, SetParameters.Request(parameters=[
                            Parameter(fault, value=False).to_parameter_msg()]))
                        assert all(result.successful for result in response.results)
                    if fault == 'publish_odom':
                        wait(lambda: statuses[-1].stationary)
                        observe(.7)
                    else:
                        wait(lambda: odometry[-1].twist.twist.linear.x == 0.)
                        observe(.7)
                    if fault != 'restart_actuator':
                        response = call(settings, SetParameters.Request(parameters=[
                            Parameter(fault, value=True).to_parameter_msg()]))
                        assert all(result.successful for result in response.results)
                    wait(lambda: recoveries[-1].data_ready and recoveries[-1].control_ready
                         and recoveries[-1].stop_confirmed)
                    wait(lambda: recoveries[-1].navigation_state == 'failed')
                    observe(.7)
                    assert odometry[-1].twist.twist.linear.x == 0.
                    assert statuses[-1].stationary
                    assert statuses[-1].left_output_rps == statuses[-1].right_output_rps == 0.
                    if recoveries[-1].recovery_state == 'required':
                        call(recover, Trigger.Request())
                        wait(lambda: recoveries[-1].navigation_state == 'waiting')
                    observe(.3)
                    assert odometry[-1].twist.twist.linear.x == 0.
                    checked_faults.append(fault)
                report['mock_goal_and_faults'] = checked_faults
            else:
                manual = node.create_publisher(Twist, '/cmd_vel_manual', 1)
                velocity = Twist()
                velocity.linear.x = .1
                timer = node.create_timer(.05, lambda: manual.publish(velocity))
                client = node.create_client(SetBool, '/set_auto_mode')
                call(client, SetBool.Request(data=False))
                wait(lambda: abs(odometry[-1].twist.twist.linear.x) > .05)
                node.destroy_timer(timer)
                wait(lambda: odometry[-1].twist.twist.linear.x == 0.)
                stop = node.create_client(SetBool, '/set_emergency_stop')
                call(stop, SetBool.Request(data=True))
                wait(lambda: statuses[-1].emergency_stop and not statuses[-1].armed)
                report['manual_motion_timeout_estop'] = True
        report.update(nodes=names, clocks=clocks,
                      tf_edges=[list(edge) for edge in transforms],
                      scan_count=len(scans), odom_count=len(odometry))
        (tmp_path / 'report.json').write_text(json.dumps(report, indent=2))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stop_system(process, environment['GZ_PARTITION'], simulation=simulation, environment=environment)
        log.close()
