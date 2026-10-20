import math
import pytest
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from vehicle_planning.navigation_manager import (
    NavigationState,
    build_goal,
    classify_result,
    NavigationManager,
    is_stationary,
    goal_pose_to_xy_yaw,
)


class ManagerStub(SimpleNamespace):
    """Legacy callback fixtures with the new recovery hooks stubbed out."""

    def __init__(self, **kwargs):
        defaults = dict(
            _recovery_state='none', _stop_observation_after_ns=None,
            _require_recovery=Mock(), _publish_recovery_status=Mock(),
        )
        defaults.update(kwargs)
        super().__init__(**defaults)


def test_build_goal_sets_map_pose():
    goal = build_goal(1.0, -2.0, math.pi / 2.0)
    pose = goal.pose.pose

    assert goal.pose.header.frame_id == 'map'
    assert pose.position.x == pytest.approx(1.0)
    assert pose.position.y == pytest.approx(-2.0)
    assert pose.position.z == pytest.approx(0.0)

    assert (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) == pytest.approx((
        0.0,
        0.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
    ))


@pytest.mark.parametrize('index', [0, 1, 2])
@pytest.mark.parametrize('value', [
    float('nan'),
    float('inf'),
    float('-inf'),
])
def test_build_goal_rejects_non_finite_input(index, value):
    coordinates = [1.0, 2.0, 0.0]
    coordinates[index] = value

    with pytest.raises(ValueError, match='finite'):
        build_goal(*coordinates)


@pytest.mark.parametrize(
    'status, error_code, expected',
    [
        pytest.param(
            GoalStatus.STATUS_SUCCEEDED,
            NavigateToPose.Result.NONE,
            NavigationState.SUCCEEDED,
            id='succeeded-without-error',
        ),
        pytest.param(
            GoalStatus.STATUS_SUCCEEDED,
            1,
            NavigationState.FAILED,
            id='succeeded-with-error',
        ),
        pytest.param(
            GoalStatus.STATUS_ABORTED,
            NavigateToPose.Result.NONE,
            NavigationState.FAILED,
            id='aborted-without-error-code',
        ),
        pytest.param(
            GoalStatus.STATUS_CANCELED,
            NavigateToPose.Result.NONE,
            NavigationState.CANCELED,
            id='canceled',
        ),
        pytest.param(
            GoalStatus.STATUS_UNKNOWN,
            NavigateToPose.Result.NONE,
            NavigationState.FAILED,
            id='unknown-status',
        ),
    ],
)
def test_classify_result(status, error_code, expected):
    assert classify_result(status, error_code) == expected


@pytest.mark.parametrize('state', [
    NavigationState.SENDING,
    NavigationState.RUNNING,
    NavigationState.CANCELING,
    NavigationState.STOPPING,
])
def test_send_goal_rejects_new_goal_while_busy(state):
    action_client = Mock()
    sender = ManagerStub(
        _state=state,
        _action_client=action_client,
    )

    with pytest.raises(RuntimeError, match='cancel'):
        NavigationManager.send_goal(sender, 1.0, 2.0, 0.0)

    action_client.server_is_ready.assert_not_called()
    action_client.send_goal_async.assert_not_called()
    assert sender._state == state


def test_cancel_goal_requests_cancellation_and_keeps_handle():
    goal_handle = Mock()
    sender = ManagerStub(
        _state=NavigationState.RUNNING,
        _goal_handle=goal_handle,
        _set_state=Mock(),
    )

    cancel_future = NavigationManager.cancel_goal(sender)

    goal_handle.cancel_goal_async.assert_called_once_with()
    assert cancel_future is goal_handle.cancel_goal_async.return_value
    assert sender._goal_handle is goal_handle
    sender._set_state.assert_called_once_with(
        NavigationState.CANCELING
    )

    cancel_future.add_done_callback.assert_called_once()


@pytest.mark.parametrize(
    'result_error',
    [None, RuntimeError('result unavailable')],
    ids=['canceled', 'response-error'],
)
def test_result_callback_preserves_goal_until_result_is_known(
    result_error,
):
    goal_handle = object()
    future = Mock()
    future.result.return_value = SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED,
        result=NavigateToPose.Result(),
    )
    future.result.side_effect = result_error

    sender = ManagerStub(
        _goal_handle=goal_handle,
        _result_future=future,
        _pending_result_state=None,
        _result_error=None,
        _set_state=Mock(),
        get_logger=Mock(),
        _start_stop_check=Mock(),
        _navigation_fault=None,
        _fail_navigation=Mock(),
        _publish_event=Mock(),
    )

    sender._handle_result = lambda response: (
        NavigationManager._handle_result(sender, response)
    )

    NavigationManager._result_callback(sender, future)

    if result_error is None:
        assert sender._pending_result_state == NavigationState.CANCELED
        assert sender._goal_handle is None
        sender._set_state.assert_called_once_with(
            NavigationState.STOPPING
        )
        sender._start_stop_check.assert_called_once_with()
        sender._publish_event.assert_called_once_with(
            'action_result',
            status=GoalStatus.STATUS_CANCELED,
            error_code=NavigateToPose.Result.NONE,
            error_msg='',
            outcome='canceled',
        )
    else:
        assert sender._pending_result_state is None
        assert sender._goal_handle is goal_handle
        assert sender._result_error is result_error
        sender._set_state.assert_not_called()
        sender._start_stop_check.assert_not_called()
        sender._require_recovery.assert_called_once()
        sender._publish_event.assert_not_called()


@pytest.mark.parametrize(
    'state, return_code, expected_state_change',
    [
        (
            NavigationState.CANCELING,
            CancelGoal.Response.ERROR_NONE,
            None,
        ),
        (
            NavigationState.CANCELING,
            CancelGoal.Response.ERROR_REJECTED,
            NavigationState.RUNNING,
        ),
        (
            NavigationState.CANCELED,
            CancelGoal.Response.ERROR_REJECTED,
            None,
        ),
    ],
    ids=['accepted', 'rejected', 'late-rejection'],
)
def test_handle_cancel_response(
    state, return_code, expected_state_change
):
    goal_handle = object()
    sender = ManagerStub(
        _state=state,
        _goal_handle=(
            None if state == NavigationState.CANCELED else goal_handle
        ),
        _set_state=Mock(),
        get_logger=Mock(),
        _navigation_fault=None,
        _publish_event=Mock(),
    )
    response = SimpleNamespace(
        return_code=return_code,
        goals_canceling=(
            [object()] if return_code == CancelGoal.Response.ERROR_NONE
            else []
        ),
    )
    future = Mock()
    future.result.return_value = response

    NavigationManager._handle_cancel_response(sender, future, goal_handle)

    if expected_state_change is None:
        sender._set_state.assert_not_called()
    else:
        sender._set_state.assert_called_once_with(
            expected_state_change
        )


@pytest.mark.parametrize('error', [
    None,
    'No running goal to cancel',
])
def test_cancel_callback_reports_request_outcome(error):
    sender = ManagerStub(
        cancel_goal=Mock(
            side_effect=RuntimeError(error) if error else None
        ),
    )
    response = Trigger.Response()

    returned = NavigationManager._cancel_callback(
        sender, Trigger.Request(), response
    )

    assert returned is response
    sender.cancel_goal.assert_called_once_with()
    assert response.success == (error is None)

    if error:
        assert response.message == error
    else:
        assert 'requested' in response.message


@pytest.mark.parametrize(
    'linear_x, linear_y, angular_z, expected',
    [
        pytest.param(
            0.02, 0.0, 0.05, True,
            id='at-stop-thresholds',
        ),
        pytest.param(
            -0.03, 0.0, 0.0, False,
            id='moving-backward',
        ),
        pytest.param(
            0.015, 0.015, 0.0, False,
            id='combined-planar-speed',
        ),
        pytest.param(
            0.0, 0.0, -0.06, False,
            id='rotating-clockwise',
        ),
        pytest.param(
            float('nan'), 0.0, 0.0, False,
            id='invalid-velocity',
        ),
    ],
)
def test_is_stationary(
    linear_x, linear_y, angular_z, expected
):
    assert is_stationary(
        linear_x, linear_y, angular_z
    ) is expected


@pytest.mark.parametrize(
    'measurement_delay_ns',
    [0, 600_000_000],
    ids=['current-measurements', 'delayed-measurements'],
)
def test_stop_confirmation_requires_continuous_fresh_odometry(
    monkeypatch, measurement_delay_ns,
):
    sender = ManagerStub(
        _stationary_since_ns=None,
        _last_odom_stamp_ns=None,
        _last_odom_received_at=None,
        get_clock=Mock(),
    )
    sender._sensor_is_fresh = lambda msg, received_at: (
        NavigationManager._sensor_is_fresh(sender, msg, received_at)
    )

    clock = Mock(return_value=0.0)
    monkeypatch.setattr(
        'vehicle_planning.navigation_manager.time.monotonic',
        clock,
    )

    def receive(stamp_ns, received_at):
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000,
        )

        clock.return_value = received_at
        sender.get_clock.return_value.now.return_value.nanoseconds = (
            stamp_ns + measurement_delay_ns
        )
        NavigationManager._odom_callback(sender, msg)

    for index in range(11):
        receive(
            1_000_000_000 + index * 100_000_000,
            index / 10,
        )
        expected = index == 10 and measurement_delay_ns == 0
        assert NavigationManager.is_stop_confirmed(sender) is expected

    # 최근 수신했어도 ROS 시간 기준으로 오래되면 확인을 무효로 한다.
    if measurement_delay_ns == 0:
        sender.get_clock.return_value.now.return_value.nanoseconds += (
            600_000_000
        )
        assert not NavigationManager.is_stop_confirmed(sender)

    # 수신이 중단돼도 확인을 무효로 한다.
    clock.return_value = 1.6
    assert not NavigationManager.is_stop_confirmed(sender)

    # 수신 재개 후 정지 유지 시간을 새로 계산한다.
    receive(3_000_000_000, 1.6)
    assert not NavigationManager.is_stop_confirmed(sender)
    

@pytest.mark.parametrize(
    'confirmed, checked_at',
    [(True, 11.0), (False, 15.0)],
    ids=['stop-confirmed', 'stop-timeout'],
)
def test_stop_timer_checks_fresh_observation(
    monkeypatch, confirmed, checked_at
):
    sender = ManagerStub(
        _state=NavigationState.STOPPING,
        _pending_result_state=NavigationState.CANCELED,
        _stationary_since_ns=1_000_000_000,
        _last_odom_stamp_ns=2_000_000_000,
        _last_odom_received_at=0.0,
        _stop_deadline=None,
        _stop_error=None,
        is_stop_confirmed=Mock(return_value=confirmed),
        _set_state=Mock(),
        get_logger=Mock(),
        _publish_event=Mock(),
    )
    clock = Mock(return_value=10.0)
    monkeypatch.setattr(
        'vehicle_planning.navigation_manager.time.monotonic',
        clock,
    )

    NavigationManager._start_stop_check(sender)

    assert sender._stationary_since_ns is None
    assert sender._last_odom_stamp_ns is None
    assert sender._last_odom_received_at is None

    clock.return_value = checked_at
    NavigationManager._check_state(sender)

    assert sender._stop_deadline is None

    if confirmed:
        assert sender._stop_error is None
        sender._set_state.assert_called_once_with(
            NavigationState.CANCELED
        )
    else:
        assert isinstance(sender._stop_error, TimeoutError)
        sender._set_state.assert_not_called()


def test_goal_response_error_keeps_acceptance_unknown():
    sender = ManagerStub(
        _state=NavigationState.SENDING,
        _result_future=None,
        _handle_goal_response=Mock(),
        get_logger=Mock(),
        _fail_navigation=Mock(),
    )
    future = Mock()
    future.result.side_effect = RuntimeError('response unavailable')

    NavigationManager._goal_response_callback(sender, future)

    assert sender._state == NavigationState.SENDING
    assert sender._result_future is None
    sender._require_recovery.assert_called_once()


def test_goal_response_timeout_keeps_new_goals_blocked(monkeypatch):
    sender = ManagerStub(
        _state=NavigationState.SENDING,
        _goal_response_deadline=5.0,
        get_logger=Mock(),
        _fail_navigation=Mock(),
    )
    monkeypatch.setattr(
        'vehicle_planning.navigation_manager.time.monotonic',
        Mock(return_value=5.0),
    )

    NavigationManager._check_state(sender)
    NavigationManager._check_state(sender)

    assert sender._state == NavigationState.SENDING
    assert sender._goal_response_deadline is None
    sender._require_recovery.assert_called_once_with(
        'Goal response timed out; acceptance is unknown'
    )


def test_send_goal_rejects_unready_navigation():
    action_client = Mock()
    action_client.server_is_ready.return_value = True

    sender = ManagerStub(
        _state=NavigationState.WAITING,
        _action_client=action_client,
        _readiness_error=Mock(
            return_value='Navigation TF is unavailable',
        ),
    )

    with pytest.raises(TimeoutError, match='Navigation TF'):
        NavigationManager.send_goal(sender, 1.0, 2.0, 0.0)

    action_client.send_goal_async.assert_not_called()
    assert sender._state == NavigationState.WAITING


def test_safety_fault_keeps_motion_blocked_after_data_recovers():
    reason = 'Scan is unavailable, invalid, or stale'
    sender = ManagerStub(
        _state=NavigationState.RUNNING,
        _navigation_fault=None,
        _goal_handle=Mock(),
        _navigation_pub=Mock(),
        _permission_pub=Mock(),
        _make_permission=Mock(),
        _ensure_registration=Mock(),
        _control_error=Mock(return_value=None),
        _last_ros_time_ns=None,
        get_clock=Mock(return_value=SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9))),
        _readiness_error=Mock(return_value=reason),
        cancel_goal=Mock(),
        get_logger=Mock(),
        _publish_event=Mock(),
    )

    sender._request_safety_cancel = lambda: (
        NavigationManager._request_safety_cancel(sender)
    )
    sender._fail_navigation = lambda error: (
        NavigationManager._fail_navigation(sender, error)
    )

    # 주행 중 이상을 발견하면 허가를 차단하고 취소를 요청한다.
    NavigationManager._check_navigation_safety(sender)

    assert sender._navigation_fault == reason
    sender.cancel_goal.assert_called_once_with()

    # 데이터가 복구돼도 같은 목표의 주행 허가를 복원하지 않는다.
    sender._readiness_error.return_value = None
    NavigationManager._check_navigation_safety(sender)
    NavigationManager._check_navigation_safety(sender)

    assert sender._navigation_fault == reason
    sender.cancel_goal.assert_called_once_with()
    assert sender._navigation_pub.publish.called
    assert all(
        not call.args[0].data
        for call in sender._navigation_pub.publish.call_args_list
    )
    sender._publish_event.assert_called_once_with(
        'safety_failure',
        reason=reason,
    )


def test_late_acceptance_after_fault_waits_for_result_and_stop():
    sender = ManagerStub(
        _state=NavigationState.SENDING,
        _navigation_fault='Scan interrupted',
        _goal_handle=None,
        _navigation_pub=Mock(),
        _permission_pub=Mock(),
        _make_permission=Mock(),
        _ensure_registration=Mock(),
        _control_error=Mock(return_value=None),
        _last_ros_time_ns=None,
        get_clock=Mock(return_value=SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9))),
        is_stop_confirmed=Mock(return_value=False),
        get_logger=Mock(),
        _publish_event=Mock(),
    )
    sender._set_state = lambda state: setattr(
        sender, '_state', state,
    )
    sender.cancel_goal = lambda: NavigationManager.cancel_goal(sender)
    sender._request_safety_cancel = lambda: (
        NavigationManager._request_safety_cancel(sender)
    )

    # 안전 오류 발생 후 늦게 수락된 목표도 취소한다.
    goal_handle = Mock()
    goal_handle.accepted = True

    result_future = NavigationManager._handle_goal_response(
        sender, goal_handle,
    )

    assert result_future is goal_handle.get_result_async.return_value
    assert sender._state == NavigationState.CANCELING
    goal_handle.cancel_goal_async.assert_called_once_with()

    # 취소가 거부돼도 RUNNING으로 복귀하지 않는다.
    cancel_future = Mock()
    cancel_future.result.return_value = SimpleNamespace(
        return_code=CancelGoal.Response.ERROR_REJECTED,
        goals_canceling=[],
    )
    NavigationManager._handle_cancel_response(
        sender, cancel_future, goal_handle,
    )

    assert sender._state == NavigationState.CANCELING
    NavigationManager._check_navigation_safety(sender)

    # Nav2의 취소 결과는 안전 오류 때문에 최종 실패로 분류한다.
    response = SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED,
        result=NavigateToPose.Result(),
    )
    sender._pending_result_state = NavigationManager._handle_result(
        sender, response,
    )

    assert sender._pending_result_state == NavigationState.FAILED
    assert sender._state == NavigationState.STOPPING

    NavigationManager._start_stop_check(sender)
    NavigationManager._check_state(sender)

    assert sender._state == NavigationState.STOPPING

    # 실제 정지를 확인한 뒤에만 FAILED로 종료한다.
    sender.is_stop_confirmed.return_value = True
    NavigationManager._check_state(sender)

    assert sender._state == NavigationState.FAILED
    NavigationManager._check_navigation_safety(sender)
    assert all(
        not call.args[0].data
        for call in sender._navigation_pub.publish.call_args_list
    )


def test_goal_pose_converts_position_and_heading():
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = 1.0
    msg.pose.position.y = -2.0
    msg.pose.orientation.z = math.sin(math.pi / 4.0)
    msg.pose.orientation.w = math.cos(math.pi / 4.0)

    assert goal_pose_to_xy_yaw(msg) == pytest.approx(
        (1.0, -2.0, math.pi / 2.0)
    )


@pytest.mark.parametrize('invalid_case', [
    'wrong_frame',
    'non_finite',
    'zero_quaternion',
    'non_planar',
])
def test_invalid_goal_pose_is_not_submitted(invalid_case):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.orientation.w = 1.0

    if invalid_case == 'wrong_frame':
        msg.header.frame_id = 'odom'
    elif invalid_case == 'non_finite':
        msg.pose.position.x = float('nan')
    elif invalid_case == 'zero_quaternion':
        msg.pose.orientation.w = 0.0
    elif invalid_case == 'non_planar':
        msg.pose.position.z = 1.0

    sender = ManagerStub(
        send_goal=Mock(),
        get_logger=Mock(),
    )

    NavigationManager._goal_pose_callback(sender, msg)

    sender.send_goal.assert_not_called()
    sender.get_logger.return_value.warning.assert_called_once()


def test_goal_pose_busy_rejection_is_logged():
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.orientation.w = 1.0

    sender = ManagerStub(
        send_goal=Mock(side_effect=RuntimeError('Navigation is busy')),
        get_logger=Mock(),
    )

    NavigationManager._goal_pose_callback(sender, msg)

    sender.send_goal.assert_called_once_with(0.0, 0.0, 0.0)
    sender.get_logger.return_value.warning.assert_called_once()


@pytest.mark.parametrize(
    'status, expected_state',
    [
        pytest.param(
            GoalStatus.STATUS_SUCCEEDED,
            NavigationState.SUCCEEDED,
            id='succeeded',
        ),
        pytest.param(
            GoalStatus.STATUS_ABORTED,
            NavigationState.FAILED,
            id='failed',
        ),
        pytest.param(
            GoalStatus.STATUS_CANCELED,
            NavigationState.CANCELED,
            id='canceled',
        ),
    ],
)
def test_result_waits_for_stop_before_terminal_state(
    monkeypatch, status, expected_state,
):
    monkeypatch.setattr(
        'vehicle_planning.navigation_manager.time.monotonic',
        Mock(return_value=10.0),
    )

    future = Mock()
    future.result.return_value = SimpleNamespace(
        status=status,
        result=NavigateToPose.Result(),
    )

    sender = ManagerStub(
        _state=NavigationState.RUNNING,
        _goal_handle=object(),
        _result_future=future,
        _pending_result_state=None,
        _navigation_fault=None,
        is_stop_confirmed=Mock(return_value=False),
        get_logger=Mock(),
        _publish_event=Mock(),
    )

    def set_state(state):
        sender._state = state

    sender._set_state = Mock(side_effect=set_state)
    sender._handle_result = lambda response: (
        NavigationManager._handle_result(sender, response)
    )
    sender._start_stop_check = lambda: (
        NavigationManager._start_stop_check(sender)
    )

    # 결과를 받아도 바로 최종 상태로 전환하면 안 된다.
    NavigationManager._result_callback(sender, future)

    assert sender._pending_result_state == expected_state
    assert sender._goal_handle is None
    assert sender._state == NavigationState.STOPPING
    sender._set_state.assert_called_once_with(
        NavigationState.STOPPING
    )

    # 정지가 확인되지 않으면 STOPPING을 유지한다.
    NavigationManager._check_state(sender)

    assert sender._state == NavigationState.STOPPING
    assert sender._set_state.call_count == 1

    # 정지 확인 후에만 액션 결과에 해당하는 상태로 종료한다.
    sender.is_stop_confirmed.return_value = True
    NavigationManager._check_state(sender)

    assert sender._state == expected_state
    assert sender._stop_deadline is None
    assert sender._set_state.call_count == 2
    sender._set_state.assert_called_with(expected_state)
