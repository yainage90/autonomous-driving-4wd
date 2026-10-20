"""Opt-in test against a fresh evaluation Gazebo/Nav2 system."""
import json
import os
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from vehicle_interfaces.srv import SetNavigationGoal


@pytest.mark.skipif(
    os.environ.get('RUN_SAFETY_SYNC_INTEGRATION') != '1',
    reason='Requires a fresh evaluation Gazebo/Nav2 system in the selected ROS domain',
)
def test_mode_and_emergency_transitions_require_new_goal(tmp_path):
    rclpy.init()
    node = rclpy.create_node(
        'safety_sync_probe',
        parameter_overrides=[Parameter('use_sim_time', value=True)],
    )
    events, commands, raw, truth = [], [], [], []
    node.create_subscription(
        String, '/navigation_manager/events',
        lambda msg: events.append(json.loads(msg.data)), 100,
    )
    node.create_subscription(
        Twist, '/cmd_vel',
        lambda msg: commands.append((time.monotonic(), msg.linear.x, msg.angular.z)),
        100,
    )
    node.create_subscription(TwistStamped, '/cmd_vel_auto', raw.append, 100)
    node.create_subscription(
        Odometry, '/ground_truth/odom',
        lambda msg: truth.append((time.monotonic(), msg)), 100,
    )
    goal = node.create_client(SetNavigationGoal, '/navigation_manager/set_goal')
    mode = node.create_client(SetBool, '/set_auto_mode')
    stop = node.create_client(SetBool, '/set_emergency_stop')
    cancel = node.create_client(Trigger, '/navigation_manager/cancel')

    def wait(test, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if test():
                return
        raise AssertionError('Timed out: ' + repr(events[-5:]))

    def observe(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def call(client, request):
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(request)
        wait(future.done)
        response = future.result()
        assert response.success, response.message
        return response

    def drive():
        commands.clear()
        events.clear()
        call(goal, SetNavigationGoal.Request(x=1.0, y=0.0, yaw=0.0))
        wait(lambda: (
            truth
            and abs(truth[-1][1].twist.twist.linear.x) > 0.03
            and any(abs(x) > 0.03 for _, x, _ in commands)
        ), 15)
        assert raw, 'No stamped Nav2 commands'

    def has_state(state):
        return any(
            event['event'] == 'state_changed' and event['data']['state'] == state
            for event in events
        )

    report = []
    try:
        wait(lambda: len(truth) >= 20, 25)
        call(mode, SetBool.Request(data=True))
        call(stop, SetBool.Request(data=False))
        observe(2)
        for name in ['manual', 'emergency_stop']:
            drive()
            commands.clear()
            started = time.monotonic()
            if name == 'manual':
                call(mode, SetBool.Request(data=False))
                observe(0.2)
                call(mode, SetBool.Request(data=True))
            else:
                call(stop, SetBool.Request(data=True))
                observe(0.2)
                call(stop, SetBool.Request(data=False))
            wait(lambda: has_state('failed'), 10)
            wait(lambda: truth and abs(truth[-1][1].twist.twist.linear.x) <= .02, 5)
            settled = time.monotonic()
            observe(3)
            held_commands = [(x, w) for t, x, w in commands if t >= settled]
            held_truth = [m for t, m in truth if t >= settled]
            assert len(held_commands) >= 10
            assert all(x == 0 and w == 0 for x, w in held_commands)
            assert len(held_truth) >= 30
            assert all(
                abs(msg.twist.twist.linear.x) <= 0.02
                and abs(msg.twist.twist.angular.z) <= 0.05
                for msg in held_truth
            )
            report.append(dict(
                scenario=name, terminal='failed',
                terminal_and_stop_sec=round(settled - started, 3), hold_sec=3,
                zero_commands=len(held_commands), stationary_samples=len(held_truth),
                states=[event['data']['state'] for event in events
                        if event['event'] == 'state_changed'],
            ))
        drive()
        call(cancel, Trigger.Request())
        wait(lambda: has_state('canceled'), 10)
        report.append(dict(scenario='new_goal_after_transitions', moved=True, final='canceled'))
        events.clear()
        call(goal, SetNavigationGoal.Request(x=0.5, y=0.0, yaw=0.0))
        wait(lambda: has_state('succeeded'), 30)
        report.append(dict(scenario='normal_goal_after_transitions', final='succeeded'))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        (tmp_path / 'safety_sync_result.json').write_text(json.dumps(report, indent=2))
    finally:
        if stop.service_is_ready():
            try:
                call(stop, SetBool.Request(data=True))
            except Exception as exc:
                node.get_logger().error(f'Test cleanup stop failed: {exc}')
        node.destroy_node()
        rclpy.shutdown()
