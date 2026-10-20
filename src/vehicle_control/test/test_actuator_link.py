"""Real guard/link transport against a controllable receiver (no production bypass)."""
import copy
import time
import uuid
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.task import Future
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool
from vehicle_interfaces.msg import ActuatorCommand, ActuatorStatus
from vehicle_interfaces.srv import ArmActuator, PrepareActuator, RegisterNavigation

from vehicle_control.command_guard import CommandGuard
from vehicle_planning.navigation_manager import NavigationManager, NavigationState


@pytest.fixture
def linked():
    rclpy.init(args=['--ros-args', '-r', f'__ns:=/actuator_test_{uuid.uuid4().hex}',
                     '-p', 'initial_mode:=manual'])
    executor = SingleThreadedExecutor()
    guard = CommandGuard()
    receiver = Node('receiver')
    status = ActuatorStatus(actuator_session='receiver', generation=1, stationary=True)
    messages, operations = [], []
    publisher = receiver.create_publisher(ActuatorStatus, 'actuator/status', 1)
    receiver.create_subscription(ActuatorCommand, 'actuator/command',
                                 lambda m: messages.append(copy.deepcopy(m)), 100)

    def heartbeat():
        status.heartbeat += 1
        publisher.publish(status)

    def arm(request, response):
        operations.append(request.operation)
        response.success = (request.actuator_session == status.actuator_session
                            and request.generation == status.generation)
        if response.success and request.operation == ArmActuator.Request.ARM:
            response.success = status.stationary and not status.paused and not status.emergency_stop
        if response.success:
            status.generation += 1
            status.armed = request.operation == ArmActuator.Request.ARM
            status.guard_session = request.guard_session if status.armed else ''
            if request.operation == ArmActuator.Request.ESTOP:
                status.emergency_stop = True
            elif request.operation == ArmActuator.Request.CLEAR_ESTOP:
                status.emergency_stop = False
        response.generation = status.generation
        response.message = 'receiver test response'
        return response

    receiver.create_service(ArmActuator, 'actuator/arm', arm)
    heartbeat_timer = receiver.create_timer(.05, heartbeat)
    mode = receiver.create_client(SetBool, 'set_auto_mode')
    stop = receiver.create_client(SetBool, 'set_emergency_stop')
    prepare = receiver.create_client(PrepareActuator, 'prepare_actuator')
    for node in (guard, receiver):
        executor.add_node(node)

    def wait(predicate, timeout=3):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.005)
            if predicate():
                return
        assert predicate(), 'Timed out'

    def call(client, request):
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done, timeout=5)
        return future.result()

    wait(guard.actuator.fresh)
    try:
        yield guard, status, messages, operations, heartbeat_timer, wait, call, mode, stop, prepare
    finally:
        for node in (guard, receiver):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def command(guard):
    twist = Twist()
    twist.linear.x = .1
    guard.on_command_manual(twist)


def test_startup_and_repeated_inputs_cannot_activate(linked):
    guard, status, messages, _, _, wait, *_ = linked
    for _ in range(5):
        command(guard)
        guard.publish_output()
    assert not status.armed and guard.last_command_time is None
    assert not messages


def test_periodic_output_keeps_original_input_deadline(linked):
    guard, status, messages, _, _, wait, call, mode, *_ = linked
    assert call(mode, SetBool.Request(data=False)).success
    assert guard.last_command_time is None
    command(guard)
    received = guard.last_command_time
    wait(lambda: len([m for m in messages if m.velocity.linear.x > 0]) >= 3)
    assert guard.last_command_time == received
    wait(lambda: guard.last_command_time is None)
    wait(lambda: messages[-1].velocity == Twist())
    assert status.armed
    assert all(a.sequence < b.sequence for a, b in zip(messages, messages[1:]))


def test_receiver_fault_revokes_epoch_and_recovery_requires_mode_selection(linked):
    guard, status, messages, _, _, wait, call, mode, *_ = linked
    assert call(mode, SetBool.Request(data=False)).success
    command(guard)
    wait(lambda: messages and messages[-1].velocity.linear.x > 0)
    epoch = guard._epoch
    status.generation += 1
    status.armed = False
    wait(lambda: guard._epoch > epoch)
    count = len(messages)
    for _ in range(5):
        command(guard)
        guard.publish_output()
    assert not guard.actuator.ready() and len(messages) == count
    assert call(mode, SetBool.Request(data=False)).success
    assert guard.last_command_time is None
    command(guard)
    wait(lambda: len(messages) > count and messages[-1].velocity.linear.x > 0)


def test_status_loss_and_old_session_never_restore_token(linked):
    guard, status, _, _, timer, wait, call, mode, *_ = linked
    assert call(mode, SetBool.Request(data=False)).success
    epoch = guard._epoch
    timer.cancel()
    wait(lambda: guard._epoch > epoch)
    timer.reset()
    wait(guard.actuator.fresh)
    assert not guard.actuator.ready()
    old = copy.deepcopy(status)
    status.actuator_session = 'new_receiver'
    status.generation = 1
    status.armed = False
    wait(lambda: guard.actuator.status.actuator_session == 'new_receiver')
    old.heartbeat += 100
    guard.actuator._status_callback(old)
    assert guard.actuator.status.actuator_session == 'new_receiver'


def test_fast_estop_clear_is_serialized_and_does_not_arm(linked):
    guard, status, _, operations, _, wait, call, mode, stop, _ = linked
    assert call(mode, SetBool.Request(data=False)).success
    assert call(stop, SetBool.Request(data=True)).success
    assert call(stop, SetBool.Request(data=False)).success
    wait(lambda: ArmActuator.Request.CLEAR_ESTOP in operations)
    wait(lambda: guard.actuator.fresh() and not guard.actuator.status.emergency_stop)
    assert not guard.actuator.ready() and not status.armed
    command(guard)
    assert guard.last_command_time is None
    assert operations.index(ArmActuator.Request.ESTOP) < operations.index(ArmActuator.Request.CLEAR_ESTOP)
    assert call(mode, SetBool.Request(data=False)).success


def test_manual_activation_failure_preserves_stop(linked):
    guard, status, _, _, _, wait, call, mode, *_ = linked
    status.stationary = False
    wait(lambda: not guard.actuator.status.stationary)
    assert not call(mode, SetBool.Request(data=False)).success
    assert not guard.actuator.ready()
    command(guard)
    assert guard.last_command_time is None


def test_auto_preparation_requires_current_sessions_and_unused_goal(linked):
    guard, status, _, _, _, wait, call, mode, stop, prepare = linked
    guard.on_register_navigation(RegisterNavigation.Request(manager_session='manager'),
                                 RegisterNavigation.Response())
    assert call(mode, SetBool.Request(data=True)).success
    request = PrepareActuator.Request(
        guard_session=guard._guard_session, manager_session='manager',
        epoch=guard._epoch, goal_id='new_goal',
    )
    assert call(prepare, request).success
    assert status.armed and guard.last_command_time is None
    assert not call(prepare, request).success
    request.goal_id = 'another_goal'
    request.epoch -= 1
    assert not call(prepare, request).success


@pytest.fixture
def navigation_link(linked, monkeypatch):
    guard, _, _, _, _, wait, call, mode, *_ = linked
    manager = NavigationManager()
    executor = guard.executor
    executor.add_node(manager)
    monkeypatch.setattr(manager, '_readiness_error', lambda: None)
    monkeypatch.setattr(manager, 'is_stop_confirmed', lambda: True)
    action = Mock()
    action.server_is_ready.return_value = True
    action.send_goal_async.side_effect = lambda *_a, **_k: Future()
    monkeypatch.setattr(manager, '_action_client', action)
    assert call(mode, SetBool.Request(data=True)).success
    wait(lambda: manager._control_error() is None)
    try:
        yield manager, action
    finally:
        executor.remove_node(manager)
        manager.destroy_node()


def test_auto_goal_prepares_before_submission_and_fault_requires_new_goal(linked, navigation_link):
    guard, status, messages, _, _, wait, *_ = linked
    manager, action = navigation_link
    future = manager.send_goal(1., 0., 0.)
    assert not action.send_goal_async.called
    wait(lambda: action.send_goal_async.called)
    assert status.armed and guard.last_command_time is None
    handle = Mock(accepted=True)
    handle.cancel_goal_async.return_value = Future()
    handle.get_result_async.return_value = Future()
    future.set_result(handle)
    wait(lambda: manager._state == NavigationState.RUNNING)

    def drive():
        command = TwistStamped()
        command.header.stamp = manager.get_clock().now().to_msg()
        command.twist.linear.x = .1
        manager._auto_command_callback(command)
    timer = manager.create_timer(.05, drive)
    try:
        wait(lambda: messages and messages[-1].velocity.linear.x > 0)
        status.generation += 1
        status.armed = False
        wait(lambda: manager._state == NavigationState.CANCELING)
        handle.cancel_goal_async.assert_called_once()
        with pytest.raises(RuntimeError, match='busy'):
            manager.send_goal(2., 0., 0.)
        handle.get_result_async.return_value.set_result(SimpleNamespace(
            status=GoalStatus.STATUS_CANCELED, result=NavigateToPose.Result(),
        ))
        wait(lambda: manager._state == NavigationState.FAILED)
        assert not guard.actuator.ready()
        action.send_goal_async.reset_mock()
        manager.send_goal(2., 0., 0.)
        wait(lambda: action.send_goal_async.called)
        assert status.armed
    finally:
        manager.destroy_timer(timer)


def test_control_change_during_preparation_never_submits_nav2(linked, navigation_link):
    guard, status, _, _, _, wait, call, mode, *_ = linked
    manager, action = navigation_link
    status.stationary = False
    wait(lambda: not guard.actuator.status.stationary)
    future = manager.send_goal(1., 0., 0.)
    wait(lambda: guard.actuator.busy)
    assert call(mode, SetBool.Request(data=True)).success
    status.stationary = True
    wait(lambda: future.done())
    assert manager._state == NavigationState.FAILED
    assert future.result() is None
    action.send_goal_async.assert_not_called()
    assert not guard.actuator.ready()


def test_lost_control_response_does_not_permanently_block_reactivation(linked, monkeypatch):
    guard, _, _, _, _, _, call, mode, *_ = linked
    link = guard.actuator
    never_responds = Future()
    with monkeypatch.context() as patch:
        patch.setattr(link, '_request', lambda _operation: never_responds)
        result = link.block()
        operation, result, _ = link.controls[0]
        link.controls[0] = (operation, result, time.monotonic() - 1)
        link._pump_controls()
    assert result.done()
    with pytest.raises(TimeoutError):
        result.result()
    assert never_responds.cancelled()
    assert not link.controls
    assert call(mode, SetBool.Request(data=False)).success
