"""Real ROS service/action transport with controllable Nav2 acceptance/results."""
import time
from types import SimpleNamespace
from uuid import uuid4

import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from std_srvs.srv import Trigger
from vehicle_interfaces.msg import ControlState, NavigationRecoveryStatus
from vehicle_interfaces.srv import PrepareActuator, SetNavigationGoal
from vehicle_planning.navigation_manager import (
    NavigationManager,
    NavigationState,
    RecoveryState,
    ActionState,
    RecoveryResult,
)


def wait(system, predicate, timeout=4.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        system.executor.spin_once(timeout_sec=.01)
        if predicate():
            return
    assert predicate(), 'Timed out waiting for ROS callback'


def call(system, client):
    assert client.wait_for_service(timeout_sec=2.)
    future = client.call_async(Trigger.Request())
    wait(system, future.done)
    return future.result()


@pytest.fixture
def system(monkeypatch):
    rclpy.init(args=['--ros-args', '-r', f'__ns:=/recovery_{uuid4().hex}'])
    manager, probe = NavigationManager(), Node('nav2_double')
    executor = SingleThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(probe)
    system = SimpleNamespace(
        manager=manager, probe=probe, executor=executor, feed=True,
        delay_acceptance=False, acceptance_gate=Future(), result_gate=Future(),
        received=False, handle=None, statuses=[], cancels=0,
    )
    monkeypatch.setattr(manager, '_readiness_error', lambda: None)
    control = probe.create_publisher(ControlState, 'control_state', 1)
    odom = probe.create_publisher(Odometry, 'odom', qos_profile_sensor_data)

    def publish():
        system.heartbeat += 1
        control.publish(ControlState(
            guard_session='guard', manager_session=manager._manager_session,
            epoch=1, heartbeat=system.heartbeat, auto_mode=True,
        ))
        if system.feed:
            msg = Odometry()
            msg.header.stamp = probe.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_footprint'
            msg.pose.pose.orientation.w = 1.
            odom.publish(msg)

    system.heartbeat = 0
    timer = probe.create_timer(.03, publish)

    async def accept(_goal):
        system.received = True
        if system.delay_acceptance:
            await system.acceptance_gate
        from rclpy.action import GoalResponse
        return GoalResponse.ACCEPT

    async def execute(handle):
        system.handle = handle
        await system.result_gate
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def cancel(_handle):
        system.cancels += 1
        return CancelResponse.ACCEPT

    action = ActionServer(probe, NavigateToPose, 'navigate_to_pose',
                          execute_callback=execute, goal_callback=accept,
                          cancel_callback=cancel, callback_group=ReentrantCallbackGroup())

    def prepare(_request, response):
        response.success = True
        return response

    prepare_service = probe.create_service(PrepareActuator, 'prepare_actuator', prepare)
    system.recover = probe.create_client(Trigger, 'navigation_manager/recover')
    system.goal = probe.create_client(SetNavigationGoal, 'navigation_manager/set_goal')
    subscription = probe.create_subscription(
        NavigationRecoveryStatus, 'navigation_manager/recovery_status',
        system.statuses.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        wait(system, lambda: manager._control_error() is None and manager.is_stop_confirmed())
        yield system
    finally:
        # Release any execute callback before destroying its action server.
        system.feed = True
        if not system.acceptance_gate.done():
            system.acceptance_gate.set_result(True)
        if not system.result_gate.done():
            system.result_gate.set_result(True)
        for _ in range(20):
            executor.spin_once(timeout_sec=.01)
        action.destroy()
        executor.remove_node(manager)
        executor.remove_node(probe)
        manager.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def submit(system):
    future = system.goal.call_async(SetNavigationGoal.Request(x=1., y=0., yaw=0.))
    wait(system, future.done)
    assert future.result().success


def test_stop_timeout_service_recovery_and_new_goal(system):
    manager = system.manager
    submit(system)
    wait(system, lambda: manager._state == NavigationState.RUNNING)
    old_handle = manager._goal_handle
    system.feed = False
    system.result_gate.set_result(True)
    wait(system, lambda: manager._state == NavigationState.STOPPING)
    wait(system, lambda: manager._recovery_state == RecoveryState.REQUIRED, timeout=6.)
    system.feed = True
    wait(system, manager.is_stop_confirmed)
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert call(system, system.recover).success
    deadline = manager._recovery_deadline
    assert call(system, system.recover).success
    assert manager._recovery_deadline == deadline
    wait(system, lambda: manager._state == NavigationState.WAITING)
    wait(system, lambda: system.statuses[-1].last_result == 'succeeded')
    assert system.statuses[-1].recovery_state == 'none'
    assert system.statuses[-1].action_state == 'terminal'
    assert manager._command_start_ns is None
    # Late-joining observers receive the durable current snapshot.
    snapshots = []
    late = system.probe.create_subscription(
        NavigationRecoveryStatus, 'navigation_manager/recovery_status', snapshots.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait(system, lambda: snapshots and snapshots[-1].last_result == 'succeeded')
    system.probe.destroy_subscription(late)
    system.result_gate = Future()
    submit(system)
    wait(system, lambda: manager._state == NavigationState.RUNNING)
    assert manager._goal_handle is not old_handle
    system.result_gate.set_result(True)
    wait(system, lambda: manager._state == NavigationState.SUCCEEDED)


def test_late_acceptance_requires_targeted_cancel_and_explicit_recovery(system):
    manager = system.manager
    system.delay_acceptance = True
    submit(system)
    wait(system, lambda: system.received)
    # Expire only the manager deadline; action transport still holds its response.
    manager._goal_response_deadline = time.monotonic() - .1
    wait(system, lambda: manager._recovery_state == RecoveryState.REQUIRED)
    assert not call(system, system.recover).success
    system.acceptance_gate.set_result(True)
    wait(system, lambda: system.cancels == 1)
    wait(system, lambda: system.handle is not None and system.handle.is_cancel_requested)
    assert call(system, system.recover).success
    assert call(system, system.recover).success
    assert manager._action_state == ActionState.ACTIVE
    system.result_gate.set_result(True)
    wait(system, lambda: manager._action_state == ActionState.TERMINAL)
    assert manager._recovery_state == RecoveryState.RECOVERING
    wait(system, lambda: manager._state == NavigationState.WAITING)
    assert manager._recovery_result == RecoveryResult.SUCCEEDED
    assert system.cancels <= 2  # one original safety request and one recovery attempt
