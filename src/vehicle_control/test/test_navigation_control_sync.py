"""Guard/manager protocol tests over ROS topics with controllable Nav2 responses."""
import copy
import time
import uuid
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool

from vehicle_control.command_guard import CommandGuard
from vehicle_interfaces.msg import NavigationCommand
from vehicle_planning.navigation_manager import NavigationManager, NavigationState


def wait(executor, predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return True
    return predicate()


@pytest.fixture
def system(monkeypatch, selection_actuator):
    rclpy.init(args=['--ros-args', '-r', f'__ns:=/sync_{uuid.uuid4().hex}',
                     '-p', 'initial_mode:=auto'])
    executor = SingleThreadedExecutor()
    guard = CommandGuard()
    manager = NavigationManager()
    probe = Node('probe')
    nodes = [guard, manager, probe]
    for node in nodes:
        executor.add_node(node)
    monkeypatch.setattr(manager, '_readiness_error', lambda: None)
    monkeypatch.setattr(manager, 'is_stop_confirmed', lambda: True)
    action = Mock()
    action.server_is_ready.return_value = True
    action.send_goal_async.side_effect = lambda *_args, **_kwargs: Future()
    monkeypatch.setattr(manager, '_action_client', action)
    outputs = []
    probe.create_subscription(Twist, 'cmd_vel', outputs.append, 10)
    auto = probe.create_publisher(TwistStamped, 'cmd_vel_auto', 10)
    command = TwistStamped()
    command.twist.linear.x = 0.1

    def publish():
        command.header.stamp = probe.get_clock().now().to_msg()
        auto.publish(command)

    timer = probe.create_timer(0.03, publish)
    fixture = SimpleNamespace(
        guard=guard, manager=manager, probe=probe, executor=executor,
        outputs=outputs, timer=timer, nodes=nodes, command=command,
    )
    try:
        assert wait(executor, lambda: manager._control_error() is None)
        yield fixture
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def submit(system, accept=True):
    future = system.manager.send_goal(1.0, 0.0, 0.0)
    assert wait(system.executor, lambda: not system.manager._preparing_goal)
    handle = Mock()
    handle.accepted = True
    handle.cancel_goal_async.return_value = Future()
    handle.get_result_async.return_value = Future()
    if accept:
        future.set_result(handle)
    return future, handle


def moving(system):
    assert wait(system.executor, lambda: any(msg.linear.x > 0 for msg in system.outputs))


def stopped(system):
    system.outputs.clear()
    count = system.guard._heartbeat
    assert wait(system.executor, lambda: system.guard._heartbeat >= count + 4)
    assert system.outputs
    assert all(msg == Twist() for msg in system.outputs)


def change(guard, method, value):
    response = method(SetBool.Request(data=value), SetBool.Response())
    assert response.success


@pytest.mark.parametrize('kind', ['manual', 'emergency'])
def test_fast_stop_release_cancels_old_goal_and_requires_new_goal(system, kind):
    manager, guard = system.manager, system.guard
    _, handle = submit(system)
    moving(system)
    old_permission = copy.deepcopy(guard._permission)
    method = guard.on_auto_mode if kind == 'manual' else guard.on_emergency_stop
    # Do not spin between transitions: even a missed intermediate state must revoke.
    change(guard, method, kind == 'emergency')
    assert not guard._navigation_is_allowed()
    change(guard, method, kind != 'emergency')
    assert wait(system.executor, lambda: manager._state == NavigationState.CANCELING)
    handle.cancel_goal_async.assert_called_once()
    stopped(system)

    # Delayed permit and command from the previous epoch cannot restart the robot.
    guard.on_navigation_allowed(old_permission)
    guard.on_command_auto(NavigationCommand(permission=old_permission, command=system.command))
    assert not guard._navigation_is_allowed()
    with pytest.raises(RuntimeError, match='busy'):
        manager.send_goal(2.0, 0.0, 0.0)

    handle.get_result_async.return_value.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED, result=NavigateToPose.Result(),
    ))
    assert wait(system.executor, lambda: manager._state == NavigationState.FAILED)
    stopped(system)
    submit(system)
    moving(system)


def test_late_acceptance_after_mode_switch_is_canceled(system):
    future, handle = submit(system, accept=False)
    change(system.guard, system.guard.on_auto_mode, False)
    change(system.guard, system.guard.on_auto_mode, True)
    assert wait(system.executor, lambda: system.manager._navigation_fault is not None)
    future.set_result(handle)
    assert wait(system.executor, lambda: system.manager._state == NavigationState.CANCELING)
    handle.cancel_goal_async.assert_called_once()
    stopped(system)


def test_cancel_rejection_and_timeout_keep_goal_blocked(system, monkeypatch):
    _, handle = submit(system)
    moving(system)
    change(system.guard, system.guard.on_auto_mode, False)
    change(system.guard, system.guard.on_auto_mode, True)
    assert wait(system.executor, lambda: system.manager._state == NavigationState.CANCELING)
    handle.cancel_goal_async.return_value.set_result(CancelGoal.Response(
        return_code=CancelGoal.Response.ERROR_REJECTED,
    ))
    assert wait(system.executor, lambda: handle.cancel_goal_async.return_value.done())
    system.manager._cancel_deadline = time.monotonic() - 1
    system.manager._check_state()
    assert system.manager._cancel_deadline is None
    stopped(system)
    with pytest.raises(RuntimeError, match='busy'):
        system.manager.send_goal(2.0, 0.0, 0.0)


@pytest.mark.parametrize('lost', ['control', 'permission'])
def test_heartbeat_loss_latches_block_after_communication_recovers(system, lost):
    _, handle = submit(system)
    moving(system)
    timer = system.guard._control_timer if lost == 'control' else system.manager._safety_timer
    # Commands also carry permits; stop the relay to simulate a manager outage.
    if lost == 'permission':
        system.timer.cancel()
    timer.cancel()
    assert wait(system.executor, lambda: system.manager._navigation_fault is not None)
    timer.reset()
    system.timer.reset()
    assert wait(system.executor, lambda: system.manager._control_received_at is not None)
    stopped(system)
    handle.cancel_goal_async.assert_called_once()


def test_guard_restart_cancels_goal_and_rejects_previous_session(system):
    _, handle = submit(system)
    moving(system)
    old = system.guard
    old_permission = copy.deepcopy(old._permission)
    system.executor.remove_node(old)
    old.destroy_node()
    system.nodes.remove(old)
    system.guard = CommandGuard()
    system.nodes.append(system.guard)
    system.executor.add_node(system.guard)
    assert wait(system.executor, lambda: system.manager._navigation_fault is not None)
    assert wait(system.executor, lambda: system.guard._manager_session == system.manager._manager_session)
    system.guard.on_navigation_allowed(old_permission)
    assert not system.guard._navigation_is_allowed()
    stopped(system)
    handle.cancel_goal_async.assert_called_once()


def test_manager_restart_revokes_old_permit_and_does_not_relay_old_goal(system):
    submit(system)
    moving(system)
    old = system.manager
    old_permission = copy.deepcopy(system.guard._permission)
    old_epoch = system.guard._epoch
    system.executor.remove_node(old)
    old.destroy_node()
    system.nodes.remove(old)
    system.manager = NavigationManager()
    system.nodes.append(system.manager)
    system.executor.add_node(system.manager)
    assert wait(system.executor, lambda: system.guard._manager_session == system.manager._manager_session)
    assert system.guard._epoch > old_epoch
    system.guard.on_navigation_allowed(old_permission)
    assert not system.guard._navigation_is_allowed()
    assert system.guard._navigation_reset_required
    stopped(system)
    assert wait(system.executor, lambda: system.manager._control_state is not None and system.manager._control_state.navigation_reset_required)
    system.manager._readiness_error = lambda: None
    system.manager._action_client.server_is_ready = lambda: True
    with pytest.raises(RuntimeError, match='restart Nav2'):
        system.manager.send_goal(2.0, 0.0, 0.0)


def test_pre_goal_stamped_command_and_out_of_order_permit_are_rejected(system):
    system.timer.cancel()
    old_command = copy.deepcopy(system.command)
    old_command.header.stamp = system.probe.get_clock().now().to_msg()
    submit(system)
    assert wait(system.executor, lambda: system.manager._state == NavigationState.RUNNING)
    assert wait(system.executor, system.guard._navigation_is_allowed)
    system.outputs.clear()
    system.manager._auto_command_callback(old_command)
    stopped(system)
    # A delayed permit from before a revocation cannot restore permission.
    permit = copy.deepcopy(system.guard._permission)
    revoke = copy.deepcopy(permit)
    revoke.sequence += 1
    revoke.allowed = False
    system.guard.on_navigation_allowed(revoke)
    system.guard.on_navigation_allowed(permit)
    assert not system.guard._navigation_is_allowed()


@pytest.mark.parametrize('condition', ['manual', 'emergency_stop', 'stale', 'moving'])
def test_new_goal_requires_fresh_auto_state_and_actual_stop(system, condition):
    if condition == 'manual':
        change(system.guard, system.guard.on_auto_mode, False)
        assert wait(system.executor, lambda: not system.manager._control_state.auto_mode)
    elif condition == 'emergency_stop':
        change(system.guard, system.guard.on_emergency_stop, True)
        assert wait(system.executor, lambda: system.manager._control_state.emergency_stop)
    elif condition == 'stale':
        system.manager._control_received_at = time.monotonic() - 1.0
    else:
        system.manager.is_stop_confirmed = lambda: False
    with pytest.raises(RuntimeError):
        system.manager.send_goal(1.0, 0.0, 0.0)
    system.manager._action_client.send_goal_async.assert_not_called()
