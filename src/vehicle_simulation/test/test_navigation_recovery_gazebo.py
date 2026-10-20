"""Opt-in recovery test against recovery_system.launch.py in a fresh ROS domain."""
import json
import os
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from vehicle_interfaces.msg import NavigationRecoveryStatus
from vehicle_interfaces.srv import SetNavigationGoal


@pytest.mark.skipif(os.environ.get('RUN_RECOVERY_GAZEBO') != '1',
                    reason='Requires fresh recovery_system.launch.py')
def test_stop_timeout_recovery_then_new_goal(tmp_path):
    rclpy.init()
    node = rclpy.create_node('recovery_gazebo_probe', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    statuses, truth, allowed = [], [], []
    feed = [True]
    odom = node.create_publisher(Odometry, '/recovery/odom', qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom',
                             lambda msg: odom.publish(msg) if feed[0] else None,
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/ground_truth/odom', truth.append, 100)
    node.create_subscription(Bool, '/navigation_allowed', allowed.append, 100)
    node.create_subscription(
        NavigationRecoveryStatus, '/navigation_manager/recovery_status', statuses.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    goal = node.create_client(SetNavigationGoal, '/navigation_manager/set_goal')
    cancel = node.create_client(Trigger, '/navigation_manager/cancel')
    recover = node.create_client(Trigger, '/navigation_manager/recover')
    stop = node.create_client(SetBool, '/set_emergency_stop')
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')

    def wait(predicate, timeout=15.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        assert predicate(), repr(statuses[-1] if statuses else 'No recovery status')

    def observe(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def call(client, request, success=True):
        assert client.wait_for_service(timeout_sec=10.)
        future = client.call_async(request)
        wait(future.done)
        response = future.result()
        assert response.success == success, response.message
        return response

    def stationary():
        if not truth:
            return False
        twist = truth[-1].twist.twist
        return (twist.linear.x ** 2 + twist.linear.y ** 2) ** .5 <= .02 and abs(twist.angular.z) <= .05

    report = {}
    try:
        assert lifecycle.wait_for_service(timeout_sec=30.)
        activation_deadline = time.monotonic() + 60.
        while True:
            future = lifecycle.call_async(GetState.Request())
            wait(future.done)
            if future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                break
            assert time.monotonic() < activation_deadline, 'Nav2 did not activate'
            observe(.2)
        wait(lambda: statuses and statuses[-1].data_ready and statuses[-1].control_ready
             and statuses[-1].stop_confirmed, 60.)
        call(goal, SetNavigationGoal.Request(x=1., y=0., yaw=0.))
        wait(lambda: statuses[-1].navigation_state == 'running')
        wait(lambda: not stationary())
        call(cancel, Trigger.Request())
        # Deny the stop observation only after the actual action has terminated.
        wait(lambda: statuses[-1].navigation_state == 'stopping')
        feed[0] = False
        wait(lambda: statuses[-1].recovery_state == 'required', 10.)
        assert statuses[-1].action_state == 'terminal'
        assert stationary()
        report['fault'] = statuses[-1].fault_reason
        feed[0] = True
        wait(lambda: statuses[-1].data_ready and statuses[-1].stop_confirmed)
        call(goal, SetNavigationGoal.Request(x=1., y=0., yaw=0.), success=False)
        assert statuses[-1].recovery_state == 'required'
        allowed.clear()
        call(recover, Trigger.Request())
        wait(lambda: statuses[-1].last_result == 'succeeded')
        assert statuses[-1].navigation_state == 'waiting'
        observe(2.)
        assert stationary()
        assert allowed and not any(msg.data for msg in allowed)
        report['recovered_stationary_without_goal'] = True
        call(goal, SetNavigationGoal.Request(x=1., y=0., yaw=0.))
        wait(lambda: not stationary())
        wait(lambda: statuses[-1].navigation_state == 'succeeded', 40.)
        assert stationary()
        report['new_goal'] = statuses[-1].navigation_state
        (tmp_path / 'recovery_report.json').write_text(json.dumps(report, indent=2))
    finally:
        try:
            call(stop, SetBool.Request(data=True))
        finally:
            node.destroy_node()
            rclpy.shutdown()
