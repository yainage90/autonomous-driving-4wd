import math

import pytest

from vehicle_simulation.evaluate_run import (
    pose_error,
    transform_pose,
    sample_at,
    summarize_run,
)

from nav_msgs.msg import Odometry


def test_transform_pose_applies_rotation_then_translation():
    actual = transform_pose(
        (1.0, 0.0, 0.0),
        (2.0, 3.0, math.pi / 2),
    )

    assert actual == pytest.approx(
        (2.0, 4.0, math.pi / 2)
    )


def test_pose_error_uses_shortest_angle_across_pi_boundary():
    position_error, yaw_error = pose_error(
        (0.0, 0.0, math.radians(179)),
        (0.3, 0.4, math.radians(-179)),
    )

    assert position_error == pytest.approx(0.5)
    assert yaw_error == pytest.approx(math.radians(2))


@pytest.mark.parametrize(
    'target_ns, expected_error',
    [
        (1_050_000_000, None),
        (1_100_000_000, None),
        (1_100_000_001, 'older'),
        (900_000_000, 'before'),
    ],
)
def test_sample_at_requires_a_recent_past_sample(
    target_ns, expected_error,
):
    past = Odometry()
    past.header.stamp.sec = 1

    future = Odometry()
    future.header.stamp.sec = 2

    samples = [past, future]

    if expected_error:
        with pytest.raises(ValueError, match=expected_error):
            sample_at(samples, target_ns)
    else:
        assert sample_at(samples, target_ns) is past


def test_sample_at_rejects_simulation_time_reset():
    first = Odometry()
    first.header.stamp.sec = 2
    reset = Odometry()
    reset.header.stamp.sec = 1

    with pytest.raises(ValueError, match='increase'):
        sample_at([first, reset], 2_000_000_000)


def test_summary_preserves_canceled_action_and_failed_final_state():
    def event(kind, sec, data):
        return {
            'schema_version': 1,
            'attempt_id': 'test_attempt',
            'event': kind,
            'ros_time_ns': sec * 1_000_000_000,
            'monotonic_ns': (sec + 10) * 1_000_000_000,
            'data': data,
        }

    events = [
        event('goal_submitted', 1, {
            'frame_id': 'map',
            'x': 1.0, 'y': 0.0, 'yaw': 0.0,
        }),
        event('action_result', 2, {
            'status': 5,
            'error_code': 0,
            'error_msg': '',
            'outcome': 'canceled',
        }),
        event('state_changed', 3, {'state': 'failed'}),
    ]

    samples = []
    for sec, x in [(1, -2.0), (3, -1.0)]:
        sample = Odometry()
        sample.header.stamp.sec = sec
        sample.header.frame_id = 'world'
        sample.child_frame_id = 'base_footprint'
        sample.pose.pose.position.x = x
        sample.pose.pose.orientation.w = 1.0
        samples.append(sample)

    report = summarize_run(events, samples, (2.0, 0.0, 0.0))

    assert report['action_result']['outcome'] == 'canceled'
    assert report['final_state'] == 'failed'
    assert report['position_error_m'] == pytest.approx(0.0)
    assert report['submitted_to_terminal_wall_sec'] == pytest.approx(2.0)
