"""Recovery invariants using the actual manager and deterministic sensor clocks."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.task import Future
from rclpy.time import Time
from std_srvs.srv import Trigger
from vehicle_interfaces.msg import ControlState
from vehicle_planning.navigation_manager import (
    NavigationManager,
    NavigationState,
    RecoveryState,
    ActionState,
    RecoveryResult,
)


@pytest.fixture
def rig(monkeypatch):
    rclpy.init()
    manager = NavigationManager()
    clock = Mock(return_value=10.0)
    monkeypatch.setattr('vehicle_planning.navigation_manager.time.monotonic', clock)
    monkeypatch.setattr(manager, 'get_clock', lambda: SimpleNamespace(
        now=lambda: Time(nanoseconds=int(clock.return_value * 10**9))))
    monkeypatch.setattr(manager, '_readiness_error', lambda: None)
    events = Mock()
    monkeypatch.setattr(manager, '_publish_event', events)
    manager._control_state = ControlState(
        guard_session='guard', manager_session=manager._manager_session,
        epoch=1, heartbeat=1, auto_mode=True,
    )
    manager._control_received_at = 10.0
    manager._state = NavigationState.STOPPING
    manager._pending_result_state = NavigationState.SUCCEEDED
    manager._action_state = ActionState.TERMINAL
    manager._stop_deadline = 9.0
    try:
        yield SimpleNamespace(manager=manager, clock=clock, events=events)
    finally:
        manager.destroy_node()
        rclpy.shutdown()


def recover(manager):
    return manager._recover_callback(Trigger.Request(), Trigger.Response())


def observe(rig, seconds, speed=0.0):
    rig.clock.return_value = seconds
    manager = rig.manager
    manager._control_received_at = seconds
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    msg.pose.pose.orientation.w = 1.0
    msg.header.stamp = Time(nanoseconds=int(seconds * 10**9)).to_msg()
    msg.twist.twist.linear.x = speed
    manager._odom_callback(msg)


def stationary(rig, start):
    for index in range(12):
        observe(rig, start + index / 10)
        rig.manager._check_state()


def require(rig):
    rig.manager._check_state()
    assert rig.manager._recovery_state == RecoveryState.REQUIRED


def test_stop_timeout_requires_request_and_new_stationary_window(rig):
    manager = rig.manager
    require(rig)
    reason, stamp = manager._fault_reason, manager._fault_time_ns
    stationary(rig, 10.0)
    assert manager.is_stop_confirmed()
    assert manager._state == NavigationState.STOPPING
    with pytest.raises(RuntimeError, match='recovery'):
        manager.send_goal(1., 0., 0.)
    assert recover(manager).success
    deadline = manager._recovery_deadline
    assert not manager.is_stop_confirmed()
    assert recover(manager).success
    assert manager._recovery_deadline == deadline
    # A queued sample timestamped before the request cannot satisfy the window.
    observe(rig, 11.1)
    assert manager._stationary_since_ns is None
    stationary(rig, 11.2)
    assert manager._state == NavigationState.WAITING
    assert manager._recovery_result == RecoveryResult.SUCCEEDED
    assert manager._fault_reason == ''
    assert reason and stamp > 0
    assert recover(manager).success
    assert not manager._make_permission(False).allowed


@pytest.mark.parametrize('failure', ['moving', 'interrupted', 'stale', 'old', 'reversed'])
def test_invalid_observation_cannot_complete_recovery(rig, failure):
    manager = rig.manager
    require(rig)
    assert recover(manager).success
    if failure == 'moving':
        for index in range(12):
            observe(rig, 10.1 + index / 10, speed=.1)
    elif failure == 'interrupted':
        observe(rig, 10.1)
        observe(rig, 11.2)
    elif failure == 'stale':
        for index in range(12):
            observe(rig, 10.1 + index / 10)
        rig.clock.return_value = 12.0
    elif failure == 'reversed':
        observe(rig, 10.5)
        observe(rig, 10.2)
    else:
        observe(rig, 9.0)
    manager._control_received_at = rig.clock.return_value
    manager._check_state()
    assert manager._recovery_state == RecoveryState.RECOVERING
    assert manager._state != NavigationState.WAITING


@pytest.mark.parametrize('condition', ['emergency', 'manual', 'restart', 'stale_control', 'sensor'])
def test_unsafe_recovery_request_is_rejected_without_resetting_fault(rig, monkeypatch, condition):
    manager = rig.manager
    require(rig)
    original = manager._fault_reason, manager._fault_time_ns
    if condition == 'emergency':
        manager._control_state.emergency_stop = True
    elif condition == 'manual':
        manager._control_state.auto_mode = False
    elif condition == 'restart':
        manager._control_state.navigation_reset_required = True
    elif condition == 'stale_control':
        manager._control_received_at = 9.0
    else:
        monkeypatch.setattr(manager, '_readiness_error', lambda: 'Scan is stale')
    assert not recover(manager).success
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert (manager._fault_reason, manager._fault_time_ns) == original
    assert manager._recovery_result == RecoveryResult.REJECTED


def test_unknown_acceptance_remains_blocked_even_if_stationary(rig):
    manager = rig.manager
    manager._state = NavigationState.SENDING
    manager._action_state = ActionState.ACCEPTANCE_UNKNOWN
    manager._goal_response_deadline = 9.0
    require(rig)
    stationary(rig, 10.0)
    assert not recover(manager).success
    assert 'acceptance is unknown' in manager._recovery_message
    assert manager._action_state == ActionState.ACCEPTANCE_UNKNOWN


def test_late_rejection_still_needs_explicit_recovery(rig):
    manager = rig.manager
    manager._state = NavigationState.SENDING
    manager._action_state = ActionState.ACCEPTANCE_UNKNOWN
    manager._goal_response_deadline = 9.0
    require(rig)
    manager._handle_goal_response(SimpleNamespace(accepted=False))
    stationary(rig, 10.0)
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert recover(manager).success
    stationary(rig, 11.2)
    assert manager._state == NavigationState.WAITING


def test_failed_result_query_retries_only_owned_goal_and_waits_for_termination(rig):
    manager = rig.manager
    handle = Mock()
    handle.cancel_goal_async.side_effect = lambda: Future()
    handle.get_result_async.side_effect = lambda: Future()
    manager._goal_handle = handle
    manager._action_state = ActionState.ACTIVE
    manager._state = NavigationState.CANCELING
    failed = Future()
    failed.set_exception(RuntimeError('Lost result response'))
    manager._result_future = failed
    manager._result_callback(failed)
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert recover(manager).success
    result = manager._result_future
    assert result is not failed
    assert recover(manager).success
    handle.cancel_goal_async.assert_called_once()
    handle.get_result_async.assert_called_once()
    stationary(rig, 10.1)
    assert manager._recovery_state == RecoveryState.RECOVERING
    # Retired query cannot overwrite the current future.
    manager._result_callback(failed)
    assert manager._recovery_state == RecoveryState.RECOVERING
    result.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED, result=NavigateToPose.Result()))
    assert manager._action_state == ActionState.TERMINAL
    assert not manager.is_stop_confirmed()
    stationary(rig, 11.3)
    assert manager._state == NavigationState.WAITING
    manager._result_callback(result)
    assert manager._state == NavigationState.WAITING


@pytest.mark.parametrize('status', [GoalStatus.STATUS_UNKNOWN, GoalStatus.STATUS_EXECUTING])
def test_nonterminal_result_is_not_termination_evidence(rig, status):
    manager = rig.manager
    manager._action_state = ActionState.ACTIVE
    manager._state = NavigationState.CANCELING
    handle = manager._goal_handle = Mock()
    future = manager._result_future = Future()
    future.set_result(SimpleNamespace(status=status, result=NavigateToPose.Result()))
    manager._result_callback(future)
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert manager._goal_handle is handle
    assert manager._action_state == ActionState.ACTIVE


@pytest.mark.parametrize('failure', ['timeout', 'epoch', 'emergency', 'sensor', 'clock'])
def test_recovery_failure_preserves_original_error(rig, monkeypatch, failure):
    manager = rig.manager
    require(rig)
    original = manager._fault_reason, manager._fault_time_ns
    assert recover(manager).success
    if failure == 'timeout':
        rig.clock.return_value = 15.0
        manager._control_received_at = 15.0
    elif failure == 'epoch':
        manager._control_state.epoch += 1
    elif failure == 'emergency':
        manager._control_state.emergency_stop = True
    elif failure == 'sensor':
        monkeypatch.setattr(manager, '_readiness_error', lambda: 'TF is stale')
    else:
        manager._last_ros_time_ns = 11 * 10**9
        monkeypatch.setattr(manager, '_ensure_registration', lambda: None)
        manager._check_navigation_safety()
    manager._check_state()
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert manager._recovery_result == RecoveryResult.FAILED
    assert (manager._fault_reason, manager._fault_time_ns) == original


def test_no_recovery_request_during_normal_operation(rig):
    manager = rig.manager
    manager._state = NavigationState.RUNNING
    assert not recover(manager).success
    assert manager._recovery_state == RecoveryState.NONE


def test_retired_acceptance_preparation_feedback_and_result_cannot_change_new_goal(rig, monkeypatch):
    class RecordingFuture(Future):
        def __init__(self):
            super().__init__()
            self.registered = []

        def add_done_callback(self, callback):
            self.registered.append(callback)
            super().add_done_callback(callback)

    monkeypatch.setattr('vehicle_planning.navigation_manager.Future', RecordingFuture)
    manager = rig.manager
    manager._state = NavigationState.WAITING
    monkeypatch.setattr(manager, 'is_stop_confirmed', lambda: True)
    prepare, action = Mock(), Mock()
    prepare.service_is_ready.return_value = True
    prepare.call_async.side_effect = lambda _request: Future()
    action.server_is_ready.return_value = True
    action.send_goal_async.return_value = Future()
    monkeypatch.setattr(manager, '_prepare_client', prepare)
    monkeypatch.setattr(manager, '_action_client', action)
    feedback = Mock()
    monkeypatch.setattr(manager, '_feedback_callback', feedback)
    public = manager.send_goal(1., 0., 0.)
    prepared = manager._prepare_future
    prepared.set_result(SimpleNamespace(success=True, message=''))
    sent = action.send_goal_async.call_args
    assert bytes(sent.kwargs['goal_uuid'].uuid).hex() == bytes(manager._goal_uuid.uuid).hex()
    stale_feedback = sent.kwargs['feedback_callback']
    handle = Mock(accepted=True)
    handle.get_result_async.side_effect = lambda: Future()
    handle.cancel_goal_async.side_effect = lambda: Future()
    # Resolve the actual transport future via its forwarding callback.
    accepted_callback = public.registered[0]
    action.send_goal_async.return_value.set_result(handle)
    old_result = manager._result_future
    old_result.set_exception(RuntimeError('Lost result'))
    assert recover(manager).success
    manager._result_future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED, result=NavigateToPose.Result()))
    manager._check_state()
    assert manager._state == NavigationState.WAITING
    new_public = manager.send_goal(2., 0., 0.)
    new_attempt = manager._observation_id
    accepted_callback(public)
    manager._actuator_prepared(prepared, NavigateToPose.Goal(), public)
    stale_feedback(object())
    manager._result_callback(old_result)
    assert manager._state == NavigationState.SENDING
    assert manager._public_goal_future is new_public
    assert manager._observation_id == new_attempt
    assert action.send_goal_async.call_count == 1
    feedback.assert_not_called()


def test_cancel_rejection_does_not_complete_recovery_and_retry_can_succeed(rig):
    manager = rig.manager
    manager._action_state = ActionState.ACTIVE
    manager._state = NavigationState.CANCELING
    manager._goal_handle = handle = Mock()
    result = manager._result_future = Future()
    handle.cancel_goal_async.side_effect = lambda: Future()
    manager._require_recovery('Cancellation timed out')
    original = manager._fault_reason, manager._fault_time_ns
    assert recover(manager).success
    # No second result request while one is pending.
    handle.get_result_async.assert_not_called()
    rejection = Future()
    rejection.set_result(CancelGoal.Response(return_code=CancelGoal.Response.ERROR_REJECTED))
    manager._handle_cancel_response(rejection, handle)
    assert manager._state == NavigationState.CANCELING
    stationary(rig, 10.1)
    assert manager._recovery_state == RecoveryState.RECOVERING
    rig.clock.return_value = 15.
    manager._control_received_at = 15.
    manager._check_state()
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert (manager._fault_reason, manager._fault_time_ns) == original
    result.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED, result=NavigateToPose.Result()))
    manager._result_callback(result)
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert recover(manager).success
    stationary(rig, 15.1)
    assert manager._state == NavigationState.WAITING
    assert handle.cancel_goal_async.call_count == 1


def test_fast_control_transition_is_detected_between_recovery_ticks(rig):
    manager = rig.manager
    require(rig)
    assert recover(manager).success
    control = manager._control_state
    manager._control_callback(ControlState(
        guard_session=control.guard_session, manager_session=control.manager_session,
        epoch=control.epoch + 1, heartbeat=control.heartbeat + 1, auto_mode=False))
    assert manager._recovery_state == RecoveryState.REQUIRED
    assert manager._recovery_result == RecoveryResult.FAILED


def test_completed_result_with_queued_callback_is_not_replaced(rig):
    manager = rig.manager
    manager._state = NavigationState.CANCELING
    manager._action_state = ActionState.ACTIVE
    handle = manager._goal_handle = Mock()
    future = manager._result_future = Future()
    future.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED, result=NavigateToPose.Result()))
    manager._require_recovery('Cancellation timed out')
    assert recover(manager).success
    assert manager._result_future is future
    handle.get_result_async.assert_not_called()
    # The already queued original callback remains valid and starts a new stop window.
    manager._result_callback(future)
    assert manager._action_state == ActionState.TERMINAL
    stationary(rig, 10.1)
    assert manager._state == NavigationState.WAITING
