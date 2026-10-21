import json
import os
import time
from types import SimpleNamespace
import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from vehicle_interfaces.srv import SetNavigationGoal


pytestmark = pytest.mark.skipif(
    os.environ.get('RUN_NAVIGATION_INTEGRATION') != '1',
    reason='새로 시작한 evaluation navigation 시스템이 필요합니다.',
)


def pose_error(
    actual: tuple[float, float, float],
    goal: tuple[float, float, float],
) -> tuple[float, float]:
    if not all(math.isfinite(v) for v in (*actual, *goal)):
        raise ValueError('Actual and goal poses must be finite')

    position_error = math.hypot(
        actual[0] - goal[0],
        actual[1] - goal[1],
    )
    yaw_difference = actual[2] - goal[2]
    yaw_error = abs(math.atan2(
        math.sin(yaw_difference),
        math.cos(yaw_difference),
    ))
    return position_error, yaw_error


def stamp_ns(sample: Odometry) -> int:
    stamp = sample.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def odometry_pose(sample: Odometry) -> tuple[float, float, float]:
    if (
        sample.header.frame_id != 'world'
        or sample.child_frame_id != 'base_footprint'
    ):
        raise ValueError('Expected world -> base_footprint ground truth')

    pose = sample.pose.pose
    q = pose.orientation
    values = (
        pose.position.x, pose.position.y,
        q.x, q.y, q.z, q.w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Ground-truth pose must be finite')

    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
        raise ValueError('Expected a unit quaternion')

    qx, qy, qz, qw = (
        q.x / norm, q.y / norm, q.z / norm, q.w / norm,
    )
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return pose.position.x, pose.position.y, yaw


def wait_until(node, condition, timeout_sec):
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() < deadline:
        if condition():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)

    return condition()


def call_service(node, client, request, timeout_sec=5.0):
    assert client.wait_for_service(timeout_sec=timeout_sec), (
        f'서비스가 준비되지 않았습니다: {client.srv_name}'
    )

    future = client.call_async(request)
    assert wait_until(node, future.done, timeout_sec), (
        f'서비스 응답 시간이 초과됐습니다: {client.srv_name}'
    )

    response = future.result()
    assert response is not None, '서비스 응답이 없습니다.'
    assert response.success, response.message
    return response


@pytest.fixture
def navigation_probe():
    rclpy.init()
    node = None
    emergency_client = None

    try:
        node = rclpy.create_node(
            'navigation_integration_probe',
            parameter_overrides=[
                Parameter('use_sim_time', value=True),
            ],
        )

        probe = SimpleNamespace(
            node=node,
            events=[],
            truth=[],
            truth_received_at=None,
            goal_client=node.create_client(
                SetNavigationGoal,
                '/navigation_manager/set_goal',
            ),
            cancel_client=node.create_client(
                Trigger,
                '/navigation_manager/cancel',
            ),
        )
        emergency_client = node.create_client(
            SetBool, '/set_emergency_stop',
        )

        def receive_event(msg):
            probe.events.append(json.loads(msg.data))

        def receive_truth(msg):
            probe.truth.append(msg)
            probe.truth_received_at = time.monotonic()

        node.create_subscription(
            String,
            '/navigation_manager/events',
            receive_event,
            100,
        )
        node.create_subscription(
            Odometry,
            '/ground_truth/odom',
            receive_truth,
            qos_profile_sensor_data,
        )

        yield probe

    finally:
        if node is not None:
            try:
                # 성공·실패 여부와 관계없이 종료 시 정지를 요청한다.
                if (
                    emergency_client is not None
                    and emergency_client.service_is_ready()
                ):
                    future = emergency_client.call_async(
                        SetBool.Request(data=True)
                    )
                    if wait_until(node, future.done, 3.0):
                        response = future.result()
                        if response is None or not response.success:
                            node.get_logger().error(
                                '검증 종료 시 비상정지 요청이 실패했습니다.'
                            )
                    else:
                        node.get_logger().error(
                            '검증 종료 시 비상정지 응답이 없습니다.'
                        )
            finally:
                node.destroy_node()

        rclpy.shutdown()


SCENARIOS = {
    's01': ((1.0, 0.0, 0.0), 'succeeded'),
    's02': ((0.0, 1.0, math.pi / 2), 'succeeded'),
    's03': ((4.0, 0.0, 0.0), 'succeeded'),
    's05': ((4.0, 1.1, 0.0), 'failed'),
    's06': ((1.0, 0.0, 0.0), 'canceled'),
}



def fresh_truth(probe):
    if not probe.truth or probe.truth_received_at is None:
        return None

    if time.monotonic() - probe.truth_received_at > 0.5:
        return None

    sample = probe.truth[-1]
    measured_ns = stamp_ns(sample)
    now_ns = probe.node.get_clock().now().nanoseconds

    if (
        now_ns <= 0
        or measured_ns <= 0
        or not -100_000_000 <= now_ns - measured_ns <= 500_000_000
    ):
        return None

    # 기존 함수로 frame, 유한값, quaternion도 검증한다.
    odometry_pose(sample)
    return sample


def truth_is_stationary(sample):
    twist = sample.twist.twist
    velocities = (
        twist.linear.x,
        twist.linear.y,
        twist.angular.z,
    )
    return (
        all(math.isfinite(value) for value in velocities)
        and math.hypot(*velocities[:2]) <= 0.02
        and abs(velocities[2]) <= 0.05
    )


def wait_for_stop(probe, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    started_ros_ns = probe.node.get_clock().now().nanoseconds
    cursor = len(probe.truth)
    stationary_since_ns = None
    previous_ns = None

    while time.monotonic() < deadline:
        rclpy.spin_once(probe.node, timeout_sec=0.05)

        latest = fresh_truth(probe)
        pending = probe.truth[cursor:]
        cursor = len(probe.truth)

        if latest is None:
            stationary_since_ns = None
            previous_ns = None
            continue

        for sample in pending:
            current_ns = stamp_ns(sample)

            if previous_ns is not None:
                assert current_ns > previous_ns, (
                    'Gazebo 오도메트리 시각이 증가하지 않습니다.'
                )
                if current_ns - previous_ns > 500_000_000:
                    stationary_since_ns = None

            previous_ns = current_ns

            if (
                current_ns < started_ros_ns
                or not truth_is_stationary(sample)
            ):
                stationary_since_ns = None
                continue

            if stationary_since_ns is None:
                stationary_since_ns = current_ns

        if (
            stationary_since_ns is not None
            and previous_ns - stationary_since_ns >= 1_000_000_000
        ):
            return latest

    pytest.fail(
        f'{timeout_sec}초 안에 최신 Gazebo 데이터로 '
        '1초 연속 정지를 확인하지 못했습니다.'
    )


def assert_initial_conditions(probe):
    assert wait_until(
        probe.node,
        lambda: (
            fresh_truth(probe) is not None
            and probe.goal_client.service_is_ready()
            and probe.node.count_publishers(
                '/navigation_manager/events'
            ) == 1
            and probe.node.count_publishers(
                '/ground_truth/odom'
            ) == 1
        ),
        timeout_sec=20.0,
    ), '관측 데이터와 목표 서비스 준비 상태를 확인하지 못했습니다.'

    sample = wait_for_stop(probe)
    position_error, yaw_error = pose_error(
        odometry_pose(sample),
        (-2.0, 0.0, 0.0),
    )

    assert position_error <= 0.15, (
        f'Gazebo 시작 위치가 다릅니다: 오차 {position_error:.3f} m'
    )
    assert yaw_error <= math.radians(10.0), (
        f'Gazebo 시작 방향이 다릅니다: 오차 {math.degrees(yaw_error):.2f}도'
    )


@pytest.fixture
def navigation_trace(navigation_probe, tmp_path):
    try:
        yield navigation_probe
    finally:
        output = tmp_path / 'navigation_events.json'
        output.write_text(
            json.dumps(
                navigation_probe.events,
                ensure_ascii=False,
                indent=2,
            ),
            encoding='utf-8',
        )
        print(f'\n이벤트 기록: {output}')


@pytest.mark.parametrize(
    'scenario_id',
    [os.environ.get('NAVIGATION_SCENARIO', 's01')],
)
def test_navigation_scenario(navigation_trace, scenario_id):
    assert scenario_id in SCENARIOS, (
        f'지원하지 않는 시나리오입니다: {scenario_id}'
    )
    goal, expected_state = SCENARIOS[scenario_id]
    probe = navigation_trace

    assert_initial_conditions(probe)
    probe.events.clear()

    call_service(
        probe.node,
        probe.goal_client,
        SetNavigationGoal.Request(
            x=goal[0],
            y=goal[1],
            yaw=goal[2],
        ),
    )
    deadline = time.monotonic() + 120.0

    assert wait_until(
        probe.node,
        lambda: any(
            event['event'] == 'goal_submitted'
            for event in probe.events
        ),
        timeout_sec=5.0,
    ), '목표 제출 이벤트를 받지 못했습니다.'

    submitted = [
        event for event in probe.events
        if event['event'] == 'goal_submitted'
    ]
    assert len(submitted) == 1
    attempt_id = submitted[0]['attempt_id']
    assert attempt_id, '목표 시도 ID가 없습니다.'

    def events_of(kind):
        return [
            event for event in probe.events
            if event['attempt_id'] == attempt_id
            and event['event'] == kind
        ]

    def states():
        return [
            event['data']['state']
            for event in events_of('state_changed')
        ]

    def terminal_received():
        return any(
            state in ('succeeded', 'failed', 'canceled')
            for state in states()
        )

    if scenario_id == 's06':
        def moving_while_running():
            sample = fresh_truth(probe)
            observed_states = states()
            if (
                sample is None
                or not observed_states
                or observed_states[-1] != 'running'
            ):
                return False

            twist = sample.twist.twist
            speed = math.hypot(
                twist.linear.x, twist.linear.y,
            )
            rotation = abs(twist.angular.z)
            return (
                math.isfinite(speed)
                and math.isfinite(rotation)
                and (speed > 0.02 or rotation > 0.05)
            )

        assert wait_until(
            probe.node,
            moving_while_running,
            timeout_sec=10.0,
        ), '취소 전에 running 상태와 실제 움직임을 확인하지 못했습니다.'

        call_service(
            probe.node,
            probe.cancel_client,
            Trigger.Request(),
        )

    assert wait_until(
        probe.node,
        lambda: (
            terminal_received()
            or bool(events_of('safety_failure'))
            or bool(events_of('stop_timeout'))
        ),
        timeout_sec=max(0.0, deadline - time.monotonic()),
    ), '120초 안에 최종 상태를 받지 못했습니다.'

    assert not events_of('safety_failure'), (
        f"주행 중 안전 오류: {events_of('safety_failure')}"
    )
    assert not events_of('stop_timeout'), (
        f"manager 정지 확인 실패: {events_of('stop_timeout')}"
    )

    results = events_of('action_result')
    assert len(results) == 1, '액션 결과가 정확히 하나여야 합니다.'
    assert results[0]['data']['outcome'] == expected_state, (
        f"예상과 다른 액션 결과: {results[0]['data']}"
    )

    expected_states = ['sending', 'running']
    if scenario_id == 's06':
        expected_states.append('canceling')
    expected_states.extend(['stopping', expected_state])

    assert states() == expected_states, (
        f'예상과 다른 상태 전이: {states()}'
    )

    remaining = deadline - time.monotonic()
    assert remaining > 0.0, '실제 정지 확인 전에 제한 시간을 넘겼습니다.'
    wait_for_stop(probe, timeout_sec=min(5.0, remaining))

    assert states() == expected_states
    assert len([
        event for event in probe.events
        if event['event'] == 'goal_submitted'
    ]) == 1, '검증 중 추가 목표가 제출됐습니다.'

    print(
        f'\n{scenario_id}: action={expected_state}, '
        f'final={states()[-1]}, Gazebo 1초 정지 확인'
    )
