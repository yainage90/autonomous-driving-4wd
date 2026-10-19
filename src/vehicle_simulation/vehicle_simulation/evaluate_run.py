import math
import json
from pathlib import Path
import argparse

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def transform_pose(
    pose: tuple[float, float, float],
    map_from_world: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, yaw = pose
    tx, ty, rotation = map_from_world

    if not all(math.isfinite(v) for v in (*pose, *map_from_world)):
        raise ValueError('Pose and transform must be finite')

    c = math.cos(rotation)
    s = math.sin(rotation)
    transformed_yaw = yaw + rotation

    return (
        tx + c * x - s * y,
        ty + s * x + c * y,
        math.atan2(
            math.sin(transformed_yaw),
            math.cos(transformed_yaw),
        ),
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


def load_observations(bag_dir: Path) -> tuple[list[dict], list[Odometry]]:
    message_types = {
        '/navigation_manager/events': String,
        '/ground_truth/odom': Odometry,
    }
    expected_types = {
        '/navigation_manager/events': 'std_msgs/msg/String',
        '/ground_truth/odom': 'nav_msgs/msg/Odometry',
    }

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(bag_dir.resolve()),
                storage_id='mcap',
            ),
            rosbag2_py.ConverterOptions('cdr', 'cdr'),
        )

        recorded_types = {
            topic.name: topic.type
            for topic in reader.get_all_topics_and_types()
        }
        for topic, expected in expected_types.items():
            if recorded_types.get(topic) != expected:
                raise ValueError(
                    f'Missing topic or unexpected message type: {topic}'
                )

        reader.set_filter(
            rosbag2_py.StorageFilter(topics=list(message_types))
        )

        events = []
        ground_truth = []

        while reader.has_next():
            topic, data, _received_at_ns = reader.read_next()
            message = deserialize_message(data, message_types[topic])

            if topic == '/navigation_manager/events':
                events.append(json.loads(message.data))
            else:
                ground_truth.append(message)

        if not events or not ground_truth:
            raise ValueError('Events and ground-truth samples are required')

        return events, ground_truth
    finally:
        del reader


def stamp_ns(sample: Odometry) -> int:
    stamp = sample.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def sample_at(
    samples: list[Odometry],
    target_ns: int,
) -> Odometry:
    selected = None
    previous_ns = None

    for sample in samples:
        current_ns = stamp_ns(sample)

        if previous_ns is not None and current_ns <= previous_ns:
            raise ValueError('Ground-truth timestamps must increase')
        previous_ns = current_ns

        if current_ns <= target_ns:
            selected = sample

    if selected is None:
        raise ValueError('No ground-truth sample before the event')

    if target_ns - stamp_ns(selected) > 100_000_000:
        raise ValueError('Ground-truth sample is older than 0.1 s')

    return selected


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


def summarize_run(
    events: list[dict],
    samples: list[Odometry],
    map_from_world: tuple[float, float, float],
) -> dict:
    submitted = [
        event for event in events
        if event['event'] == 'goal_submitted'
    ]
    if len(submitted) != 1:
        raise ValueError('Exactly one submitted goal is required')

    goal_event = submitted[0]
    attempt_id = goal_event['attempt_id']

    if not attempt_id or any(
        event['schema_version'] != 1
        or event['attempt_id'] != attempt_id
        for event in events
    ):
        raise ValueError('Unsupported schema or mixed goal attempts')

    terminals = [
        event for event in events
        if event['event'] == 'state_changed'
        and event['data']['state'] in (
            'succeeded', 'failed', 'canceled',
        )
    ]
    if len(terminals) != 1:
        raise ValueError('Exactly one terminal state is required')
    terminal = terminals[0]

    results = [
        event for event in events
        if event['event'] == 'action_result'
    ]
    if len(results) > 1:
        raise ValueError('Multiple action results are not supported')

    if (
        terminal['ros_time_ns'] < goal_event['ros_time_ns']
        or terminal['monotonic_ns'] < goal_event['monotonic_ns']
    ):
        raise ValueError('Terminal state precedes goal submission')

    data = goal_event['data']
    if data['frame_id'] != 'map':
        raise ValueError('Expected a goal in map frame')
    goal = (data['x'], data['y'], data['yaw'])

    start_sample = sample_at(samples, goal_event['ros_time_ns'])
    final_sample = sample_at(samples, terminal['ros_time_ns'])

    start_world = odometry_pose(start_sample)
    final_world = odometry_pose(final_sample)
    start_map = transform_pose(start_world, map_from_world)
    final_map = transform_pose(final_world, map_from_world)
    position_error, yaw_error = pose_error(final_map, goal)

    return {
        'schema_version': 1,
        'attempt_id': attempt_id,
        'goal_map': goal,
        'action_result': results[0]['data'] if results else None,
        'final_state': terminal['data']['state'],
        'submitted_to_terminal_wall_sec': (
            terminal['monotonic_ns'] - goal_event['monotonic_ns']
        ) / 1_000_000_000,
        'start_sample_ros_ns': stamp_ns(start_sample),
        'final_sample_ros_ns': stamp_ns(final_sample),
        'start_world': start_world,
        'final_world': final_world,
        'start_map': start_map,
        'final_map': final_map,
        'map_from_world': map_from_world,
        'position_error_m': position_error,
        'yaw_error_deg': math.degrees(yaw_error),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument(
        '--map-from-world', type=float, nargs=3, required=True,
        metavar=('TX', 'TY', 'YAW'),
    )
    parser.add_argument('--alignment-note', required=True)
    parser.add_argument('--alignment-verified', action='store_true')
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()

    try:
        metadata = json.loads(
            (run_dir / 'run.json').read_text(encoding='utf-8')
        )
        if metadata['mode'] != 'navigation':
            raise ValueError('This evaluator requires navigation mode')

        events, samples = load_observations(run_dir / 'bag')
        report = summarize_run(
            events,
            samples,
            tuple(args.map_from_world),
        )
        report.update({
            'scenario': metadata['scenario'],
            'planned_goal_map': metadata['planned_goal_map'],
            'expected_start_map': metadata['expected_start_map'],
            'alignment_verified': args.alignment_verified,
            'alignment_note': args.alignment_note,
            'ground_truth_stop_check': 'not_evaluated',
            'collision_check': 'not_evaluated',
        })

        output = json.dumps(
            report,
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
        (run_dir / 'analysis.json').write_text(
            output + '\n',
            encoding='utf-8',
        )
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        parser.error(str(exc))

    print(output)