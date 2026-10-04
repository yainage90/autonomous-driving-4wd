import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from vehicle_control.command_guard import CommandGuard


def wait_until(executor, condition, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec

    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.01)

    return condition()



def request_emergency_stop(client, executor, active):
    request = SetBool.Request()
    request.data = active
    future = client.call_async(request)

    responded = wait_until(executor, future.done)
    assert responded, '비상정지 서비스 응답이 도착하지 않았습니다'

    response = future.result()
    assert response is not None
    assert response.success is True


@pytest.fixture
def ros_system():
    # 다른 실행 중인 노드와 토픽·서비스 이름이 겹치지 않도록 분리
    namespace = f'/test_guard_{uuid.uuid4().hex}'
    rclpy.init(args=['--ros-args', '-r', f'__ns:={namespace}'])

    nodes = []
    executor = SingleThreadedExecutor()

    try:
        guard = CommandGuard()
        nodes.append(guard)

        probe = Node('test_probe')
        nodes.append(probe)

        received = []

        command_pub = probe.create_publisher(
            Twist, 'cmd_vel_raw', 10,
        )
        output_sub = probe.create_subscription(
            Twist, 'cmd_vel', received.append, 10,
        )

        for node in nodes:
            executor.add_node(node)

        yield guard, command_pub, output_sub, received, executor, probe
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_normal_command_passes_through_topics(ros_system):
    guard, command_pub, output_sub, received, executor, probe = ros_system

    # 준비: 양방향 토픽 연결이 발견될 때까지 기다림
    connected = wait_until(
        executor,
        lambda: (
            command_pub.get_subscription_count() >= 1
            and guard.cmd_pub.get_subscription_count() >= 1
        ),
    )
    assert connected, '입력·출력 토픽 연결이 발견되지 않았습니다'

    received.clear()

    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.3

    # 실행: 실제 입력 토픽으로 발행
    command_pub.publish(command)

    # 검증: 기대한 주행 명령을 출력 토픽에서 수신
    forwarded = wait_until(
        executor,
        lambda: any(
            msg.linear.x == pytest.approx(0.1)
            and msg.angular.z == pytest.approx(0.3)
            for msg in received
        ),
    )
    assert forwarded, '정상 주행 명령이 출력 토픽에 도착하지 않았습니다'

    published_command = next(
        msg for msg in received
        if msg.linear.x == pytest.approx(0.1)
        and msg.angular.z == pytest.approx(0.3)
    )

    assert published_command.linear.y == 0.0
    assert published_command.linear.z == 0.0
    assert published_command.angular.x == 0.0
    assert published_command.angular.y == 0.0


def test_command_timeout_publishes_stop(ros_system):
    guard, command_pub, output_sub, received, executor, probe = ros_system

    connected = wait_until(
        executor,
        lambda: (
            command_pub.get_subscription_count() >= 1
            and guard.cmd_pub.get_subscription_count() >= 1
        ),
    )
    assert connected, '입력·출력 토픽 연결이 발견되지 않았습니다'

    received.clear()

    # 주행 명령을 한 번만 발행
    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.3
    command_pub.publish(command)

    moving = wait_until(
        executor,
        lambda: any(
            msg.linear.x == pytest.approx(0.1)
            and msg.angular.z == pytest.approx(0.3)
            for msg in received
        ),
    )
    assert moving, '주행 명령이 출력 토픽에 도착하지 않았습니다'

    # 수신 기록에서 주행 명령의 위치를 찾음
    moving_index = next(
        index for index, msg in enumerate(received)
        if msg.linear.x == pytest.approx(0.1)
        and msg.angular.z == pytest.approx(0.3)
    )

    # 추가 명령 없이 실제 타이머에 의한 정지를 기다림
    stopped = wait_until(
        executor,
        lambda: any(
            msg == Twist()
            for msg in received[moving_index + 1:]
        ),
        timeout_sec=guard.command_timeout_sec + 1.0,
    )
    assert stopped, '명령 중단 후 정지 명령이 수신되지 않았습니다'


def test_emergency_stop_service_stops_vehicle(ros_system):
    guard, command_pub, output_sub, received, executor, probe = ros_system

    client = probe.create_client(SetBool, 'set_emergency_stop')

    connected = wait_until(
        executor,
        lambda: (
            command_pub.get_subscription_count() >= 1
            and guard.cmd_pub.get_subscription_count() >= 1
            and client.service_is_ready()
        ),
    )
    assert connected, '토픽 또는 비상정지 서비스가 준비되지 않았습니다'

    received.clear()

    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.3

    # 타임아웃이 발생하지 않도록 0.05초마다 명령 발행
    command_timer = probe.create_timer(
        0.05,
        lambda: command_pub.publish(command),
    )

    try:
        moving = wait_until(
            executor,
            lambda: any(
                msg.linear.x == pytest.approx(0.1)
                and msg.angular.z == pytest.approx(0.3)
                for msg in received
            ),
        )
        assert moving, '주행 명령이 출력 토픽에 도착하지 않았습니다'

        received.clear()

        # 실제 서비스로 비상정지 요청
        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)

        responded = wait_until(executor, future.done)
        assert responded, '비상정지 서비스 응답이 도착하지 않았습니다'

        response = future.result()
        assert response is not None
        assert response.success is True

        # 주행 명령이 계속 들어오는 동안 정지 출력 확인
        stopped = wait_until(
            executor,
            lambda: any(msg == Twist() for msg in received),
        )
        assert stopped, '비상정지 요청 후 정지 명령이 수신되지 않았습니다'

        first_stop_index = next(
            index for index, msg in enumerate(received)
            if msg == Twist()
        )

        # 명령 발행 타이머가 실행되는 동안 후속 출력도 수신
        continued_output = wait_until(
            executor,
            lambda: len(received[first_stop_index + 1:]) >= 3,
        )
        assert continued_output, '비상정지 후 후속 출력이 수신되지 않았습니다'

        # 첫 정지 이후 주행 출력이 다시 나타나면 실패
        following_messages = received[first_stop_index + 1:]
        assert all(msg == Twist() for msg in following_messages), '비상정지 중 주행 명령이 출력되었습니다'

    finally:
        probe.destroy_timer(command_timer)
        probe.destroy_client(client)

        
def test_emergency_stop_release_waits_for_new_command(ros_system):
    guard, command_pub, output_sub, received, executor, probe = ros_system

    client = probe.create_client(SetBool, 'set_emergency_stop')

    try:
        connected = wait_until(
            executor,
            lambda: (
                command_pub.get_subscription_count() >= 1
                and guard.cmd_pub.get_subscription_count() >= 1
                and client.service_is_ready()
            ),
        )
        assert connected, '토픽 또는 비상정지 서비스가 준비되지 않았습니다'

        received.clear()

        # 준비: 기존 주행 명령을 실제 토픽으로 전달
        old_command = Twist()
        old_command.linear.x = 0.1
        old_command.angular.z = 0.3
        command_pub.publish(old_command)

        moving = wait_until(
            executor,
            lambda: any(
                msg.linear.x == pytest.approx(0.1)
                and msg.angular.z == pytest.approx(0.3)
                for msg in received
            ),
        )
        assert moving, '기존 주행 명령이 전달되지 않았습니다'

        # 비상정지 활성화 후 정지 출력 확인
        received.clear()
        request_emergency_stop(client, executor, True)

        stopped = wait_until(
            executor,
            lambda: any(msg == Twist() for msg in received),
        )
        assert stopped, '비상정지 활성화 후 정지하지 않았습니다'

        # 해제 후에는 새 명령을 보내지 않음
        received.clear()
        request_emergency_stop(client, executor, False)

        output_received = wait_until(
            executor,
            lambda: len(received) >= 3,
        )
        assert output_received, '해제 후 출력이 수신되지 않았습니다'
        assert all(msg == Twist() for msg in received), (
            '새 명령 없이 비상정지 해제 후 주행 출력이 발생했습니다'
        )

        # 새 명령을 보내면 주행 출력이 다시 전달되어야 함
        received.clear()

        new_command = Twist()
        new_command.linear.x = -0.1
        new_command.angular.z = -0.3
        command_pub.publish(new_command)

        resumed = wait_until(
            executor,
            lambda: any(
                msg.linear.x == pytest.approx(-0.1)
                and msg.angular.z == pytest.approx(-0.3)
                for msg in received
            ),
        )
        assert resumed, '해제 후 새 주행 명령이 전달되지 않았습니다'
    finally:
        probe.destroy_client(client)
