import math
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



@pytest.mark.skipif(
    os.environ.get('RUN_ODOMETRY_INTEGRATION') != '1',
    reason='실행 중인 Gazebo가 필요한 통합 테스트입니다.',
)
@pytest.mark.parametrize(
    ('topic', 'expected_frame'),
    [
        ('/odom', 'odom'),
        ('/ground_truth/odom', 'world'),
    ],
)
def test_odometry_is_received_through_bridge(topic, expected_frame):
    rclpy.init()
    node = rclpy.create_node('odometry_integration_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    messages = []
    subscription = node.create_subscription(
        Odometry,
        topic,
        messages.append,
        qos_profile_sensor_data,
    )

    try:
        # 수신 대기는 시뮬레이션 정지 여부와 무관한 실제 시간으로 제한한다.
        deadline = time.monotonic() + 20.0

        while len(messages) < 5 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)

        assert len(messages) >= 5, (
            f'20초 안에 {topic} 메시지 5개를 받지 못했습니다: '
            f'{len(messages)}개'
        )

        timestamps = []

        for msg in messages[:5]:
            assert msg.header.frame_id == expected_frame
            assert msg.child_frame_id == 'base_footprint'

            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            linear = msg.twist.twist.linear
            angular = msg.twist.twist.angular

            values = (
                position.x, position.y, position.z,
                orientation.x, orientation.y,
                orientation.z, orientation.w,
                linear.x, linear.y, linear.z,
                angular.x, angular.y, angular.z,
            )
            assert all(math.isfinite(value) for value in values), (
                'odom에 NaN 또는 무한대가 있습니다.'
            )

            norm_squared = (
                orientation.x ** 2
                + orientation.y ** 2
                + orientation.z ** 2
                + orientation.w ** 2
            )
            assert norm_squared == pytest.approx(1.0, abs=1e-6), (
                '방향 quaternion이 정규화되지 않았습니다.'
            )

            stamp = msg.header.stamp
            timestamps.append(
                stamp.sec * 1_000_000_000 + stamp.nanosec
            )

        assert all(
            earlier < later
            for earlier, later in zip(timestamps, timestamps[1:])
        ), 'odom timestamp가 엄격하게 증가하지 않습니다.'

    finally:
        executor.shutdown()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(
    os.environ.get('RUN_ODOMETRY_INTEGRATION') != '1',
    reason='실행 중인 Gazebo가 필요한 통합 테스트입니다.',
)
def test_odometry_matches_tf_at_same_timestamp():
    rclpy.init()
    node = rclpy.create_node('odometry_tf_integration_test')
    node.set_parameters([
        Parameter('use_sim_time', value=True),
    ])

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)

    messages = []
    subscription = node.create_subscription(
        Odometry,
        '/odom',
        messages.append,
        qos_profile_sensor_data,
    )

    matched_stamps = set()
    last_error = '아직 odom을 수신하지 못했습니다.'

    try:
        deadline = time.monotonic() + 20.0

        while len(matched_stamps) < 3 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)

            if not messages:
                continue

            msg = messages[-1]
            stamp = Time.from_msg(msg.header.stamp)

            # 서로 다른 시각의 표본 3개를 검사한다.
            if stamp.nanoseconds in matched_stamps:
                continue

            try:
                transform = buffer.lookup_transform(
                    'odom',
                    'base_footprint',
                    stamp,
                )
            except TransformException as error:
                # odom과 TF의 도착 순서가 다를 수 있으므로 다시 기다린다.
                last_error = str(error)
                continue

            assert transform.header.frame_id == 'odom'
            assert transform.child_frame_id == 'base_footprint'

            position = msg.pose.pose.position
            translation = transform.transform.translation

            assert [
                translation.x, translation.y, translation.z
            ] == pytest.approx(
                [position.x, position.y, position.z],
                rel=0.0,
                abs=1e-6,
            ), '같은 시각의 odom과 TF 위치가 다릅니다.'

            orientation = msg.pose.pose.orientation
            rotation = transform.transform.rotation

            odom_q = (
                orientation.x, orientation.y,
                orientation.z, orientation.w,
            )
            tf_q = (
                rotation.x, rotation.y,
                rotation.z, rotation.w,
            )

            assert all(
                math.isfinite(value) for value in odom_q + tf_q
            ), 'quaternion에 NaN 또는 무한대가 있습니다.'

            # q와 -q는 같은 회전이므로 양쪽 부호를 모두 비교한다.
            same_sign_error = sum(
                (a - b) ** 2 for a, b in zip(odom_q, tf_q)
            )
            opposite_sign_error = sum(
                (a + b) ** 2 for a, b in zip(odom_q, tf_q)
            )

            assert min(
                same_sign_error, opposite_sign_error
            ) <= 1e-10, '같은 시각의 odom과 TF 방향이 다릅니다.'

            matched_stamps.add(stamp.nanoseconds)

        assert len(matched_stamps) == 3, (
            '20초 안에 같은 시각의 odom과 TF를 '
            f'3회 비교하지 못했습니다. 마지막 조회 상태: {last_error}'
        )

    finally:
        executor.shutdown()
        listener.unregister()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(
    os.environ.get('RUN_ODOMETRY_MOTION') != '1',
    reason='차량을 움직이는 Gazebo 검증입니다.',
)
@pytest.mark.parametrize('direction', [1.0, -1.0], ids=['left', 'right'])
def test_rotation_matches_ground_truth(direction):
    rclpy.init()
    node = rclpy.create_node('odometry_rotation_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    latest = {}
    command = Twist()
    publisher = node.create_publisher(Twist, '/cmd_vel_manual', 10)

    # 명령 타임아웃에 걸리지 않도록 20 Hz로 발행한다.
    timer = node.create_timer(
        0.05, lambda: publisher.publish(command)
    )

    def receive(key, msg):
        latest[key] = (msg, time.monotonic())

    subscriptions = [
        node.create_subscription(
            Odometry,
            '/odom',
            lambda msg: receive('odom', msg),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            Odometry,
            '/ground_truth/odom',
            lambda msg: receive('truth', msg),
            qos_profile_sensor_data,
        ),
    ]

    def yaw(msg):
        q = msg.pose.pose.orientation
        angle = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )
        assert math.isfinite(angle)
        return angle

    def angle_difference(end, start):
        return math.atan2(
            math.sin(end - start),
            math.cos(end - start),
        )

    def fresh_pair():
        if 'odom' not in latest or 'truth' not in latest:
            return None

        now = time.monotonic()
        odom, odom_received = latest['odom']
        truth, truth_received = latest['truth']

        if (
            now - odom_received > 0.5
            or now - truth_received > 0.5
        ):
            return None

        odom_stamp = Time.from_msg(odom.header.stamp).nanoseconds
        truth_stamp = Time.from_msg(truth.header.stamp).nanoseconds

        # 두 플러그인의 발행 시각이 조금 다를 수 있다.
        if abs(odom_stamp - truth_stamp) > 50_000_000:
            return None

        return odom, truth

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

    def wait_until_stopped():
        deadline = time.monotonic() + 10.0
        stable_since = None

        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            pair = fresh_pair()

            if pair is None:
                stable_since = None
                continue

            stopped = all(
                abs(msg.twist.twist.linear.x) < 0.02
                and abs(msg.twist.twist.angular.z) < 0.03
                for msg in pair
            )

            if not stopped:
                stable_since = None
                continue

            now = time.monotonic()
            if stable_since is None:
                stable_since = now

            if now - stable_since >= 0.5:
                return pair

        pytest.fail('최신 데이터를 수신한 정지 상태를 확인하지 못했습니다.')

    try:
        start_odom, start_truth = wait_until_stopped()

        assert publisher.get_subscription_count() >= 1, (
            '/cmd_vel_manual을 구독하는 command_guard가 필요합니다.'
        )
        assert node.count_publishers('/cmd_vel_manual') == 1, (
            '키보드 조종 등 다른 수동 명령 발행자를 종료하세요.'
        )

        start_odom_yaw = yaw(start_odom)
        start_truth_yaw = yaw(start_truth)

        command.angular.z = direction * 0.3
        deadline = time.monotonic() + 10.0
        reached_target = False

        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            pair = fresh_pair()

            if pair is None:
                pytest.fail('회전 중 오도메트리 데이터가 중단됐습니다.')

            _, truth = pair
            actual_rotation = angle_difference(
                yaw(truth), start_truth_yaw
            )

            if direction * actual_rotation >= 0.6:
                reached_target = True
                break

        command.angular.z = 0.0

        assert reached_target, (
            '10초 안에 목표 회전량에 도달하지 못했습니다. '
            '수동 모드·비상정지·시뮬레이션 실행 상태를 확인하세요.'
        )

        end_odom, end_truth = wait_until_stopped()

        estimated_rotation = angle_difference(
            yaw(end_odom), start_odom_yaw
        )
        actual_rotation = angle_difference(
            yaw(end_truth), start_truth_yaw
        )

        assert direction * actual_rotation >= 0.6
        assert direction * estimated_rotation > 0.0

        relative_error = abs(
            estimated_rotation - actual_rotation
        ) / abs(actual_rotation)

        print(
            f'\n실제={math.degrees(actual_rotation):.2f}°, '
            f'odom={math.degrees(estimated_rotation):.2f}°, '
            f'오차={relative_error:.2%}'
        )

        assert relative_error <= 0.05, (
            f'회전량 상대 오차가 5%를 넘었습니다: {relative_error:.2%}'
        )

    finally:
        # 검사 실패 시에도 먼저 정지 명령을 반복해서 전달한다.
        command.angular.z = 0.0
        spin_for(0.5)

        node.destroy_timer(timer)
        executor.shutdown()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
