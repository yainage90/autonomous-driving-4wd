from unittest.mock import patch

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from rclpy.time import Time

from vehicle_control.command_guard import CommandGuard


@pytest.fixture
def guard():
    rclpy.init(args=[])
    node = None

    try:
        node = CommandGuard()

        with patch.object(node.cmd_pub, 'publish') as publish_mock:
            yield node, publish_mock
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    

def test_normal_command_is_forwarded(guard):
    node, publish_mock = guard

    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.3

    node.on_command(command)

    publish_mock.assert_called_once()

    published_command = publish_mock.call_args.args[0]

    assert published_command.linear.x == pytest.approx(0.1)
    assert published_command.angular.z == pytest.approx(0.3)
    


@pytest.mark.parametrize(
    'linear_input, angular_input, expected_linear, expected_angular',
    [
        (0.5, 1.0, 0.2, 0.6),
        (-0.5, -1.0, -0.2, -0.6),
        (0.5, -1.0, 0.2, -0.6),
        (-0.5, 1.0, -0.2, 0.6),
        (0.2, 0.6, 0.2, 0.6),
        (-0.2, -0.6, -0.2, -0.6),
        (0.0, 0.0, 0.0, 0.0),
    ],
)
def test_command_speeds_are_limited(
    guard,
    linear_input,
    angular_input,
    expected_linear,
    expected_angular,
):
    node, publish_mock = guard

    command = Twist()
    command.linear.x = linear_input
    command.angular.z = angular_input

    node.on_command(command)

    publish_mock.assert_called_once()
    published_command = publish_mock.call_args.args[0]

    assert published_command.linear.x == pytest.approx(expected_linear)
    assert published_command.angular.z == pytest.approx(expected_angular)



def test_unused_axes_are_zeroed(guard):
    node, publish_mock = guard

    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.3

    # 사용하지 않는 축에도 값을 넣어 처리 여부를 확인
    command.linear.y = 0.4
    command.linear.z = -0.5
    command.angular.x = 0.7
    command.angular.y = -0.8

    node.on_command(command)

    publish_mock.assert_called_once()
    published_command = publish_mock.call_args.args[0]

    # 사용하는 축은 유지되어야 함
    assert published_command.linear.x == pytest.approx(0.1)
    assert published_command.angular.z == pytest.approx(0.3)

    # 사용하지 않는 축은 제거되어야 함
    assert published_command.linear.y == 0.0
    assert published_command.linear.z == 0.0
    assert published_command.angular.x == 0.0
    assert published_command.angular.y == 0.0


@pytest.mark.parametrize(
    'axis',
    [
        'linear.x',
        'linear.y',
        'linear.z',
        'angular.x',
        'angular.y',
        'angular.z',
    ],
)
@pytest.mark.parametrize(
    'invalid_value',
    [float('nan'), float('inf'), float('-inf')],
    ids=['nan', 'positive_inf', 'negative_inf'],
)
def test_invalid_command_stops_vehicle(guard, axis, invalid_value):
    node, publish_mock = guard

    # 먼저 정상 명령을 받아 이전 명령 시간이 있는 상태를 준비
    valid_command = Twist()
    valid_command.linear.x = 0.1
    valid_command.angular.z = 0.3
    node.on_command(valid_command)

    assert node.last_command_time is not None
    publish_mock.reset_mock()

    # 검증할 축 하나에 비정상 숫자를 넣음
    invalid_command = Twist()
    invalid_command.linear.x = 0.1
    invalid_command.angular.z = 0.3

    vector_name, component_name = axis.split('.')
    vector = getattr(invalid_command, vector_name)
    setattr(vector, component_name, invalid_value)

    node.on_command(invalid_command)

    publish_mock.assert_called_once()
    published_command = publish_mock.call_args.args[0]

    assert published_command.linear.x == 0.0
    assert published_command.linear.y == 0.0
    assert published_command.linear.z == 0.0
    assert published_command.angular.x == 0.0
    assert published_command.angular.y == 0.0
    assert published_command.angular.z == 0.0

    assert node.last_command_time is None


@pytest.mark.parametrize(
    'elapsed_ns, expected_stop',
    [
        (499_999_999, False),
        (500_000_000, True),
        (500_000_001, True),
    ],
    ids=['before_timeout', 'at_timeout', 'after_timeout'],
)
def test_command_timeout_boundary(guard, elapsed_ns, expected_stop):
    node, publish_mock = guard

    # 이번 테스트의 기준 설정
    assert node.command_timeout_sec == pytest.approx(0.5)

    command_time = Time(
        nanoseconds=10_000_000_000,
        clock_type=node.safety_clock.clock_type,
    )
    check_time = Time(
        nanoseconds=10_000_000_000 + elapsed_ns,
        clock_type=node.safety_clock.clock_type,
    )

    with patch.object(node.safety_clock, 'now') as now_mock:
        # 정상 명령을 받은 시각을 고정
        now_mock.return_value = command_time

        command = Twist()
        command.linear.x = 0.1
        command.angular.z = 0.3
        node.on_command(command)

        assert node.last_command_time == command_time
        publish_mock.reset_mock()

        # 지정한 시간이 지난 시점에서 타임아웃 검사
        now_mock.return_value = check_time
        node.check_timeout()

    if expected_stop:
        publish_mock.assert_called_once()
        published_command = publish_mock.call_args.args[0]

        assert published_command.linear.x == 0.0
        assert published_command.linear.y == 0.0
        assert published_command.linear.z == 0.0
        assert published_command.angular.x == 0.0
        assert published_command.angular.y == 0.0
        assert published_command.angular.z == 0.0

        assert node.last_command_time is None
    else:
        publish_mock.assert_not_called()
        assert node.last_command_time == command_time


def test_no_command_publishes_stop(guard):
    node, publish_mock = guard

    # 준비: 시작 후 명령을 받지 않은 상태
    assert node.last_command_time is None

    # 실행
    node.check_timeout()

    # 검증
    publish_mock.assert_called_once()
    published_command = publish_mock.call_args.args[0]

    assert published_command.linear.x == 0.0
    assert published_command.linear.y == 0.0
    assert published_command.linear.z == 0.0
    assert published_command.angular.x == 0.0
    assert published_command.angular.y == 0.0
    assert published_command.angular.z == 0.0

    assert node.last_command_time is None


def test_emergency_stop_activation_stops_vehicle(guard):
    node, publish_mock = guard

    # 준비: 정상 주행 명령을 받은 상태
    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.3
    node.on_command(command)

    assert node.last_command_time is not None
    publish_mock.reset_mock()

    # 실행: 비상정지 활성화 요청
    request = SetBool.Request()
    request.data = True
    response = SetBool.Response()

    result = node.on_emergency_stop(request, response)

    # 검증: 상태 전환과 서비스 응답
    assert node.emergency_stop_active is True
    assert node.last_command_time is None
    assert result.success is True

    # 검증: 즉시 정지 명령 발행
    publish_mock.assert_called_once()
    published_command = publish_mock.call_args.args[0]

    assert published_command.linear.x == 0.0
    assert published_command.linear.y == 0.0
    assert published_command.linear.z == 0.0
    assert published_command.angular.x == 0.0
    assert published_command.angular.y == 0.0
    assert published_command.angular.z == 0.0


def test_emergency_stop_blocks_new_commands(guard):
    node, publish_mock = guard

    # 준비: 서비스 콜백으로 비상정지 활성화
    request = SetBool.Request()
    request.data = True
    node.on_emergency_stop(request, SetBool.Response())

    assert node.emergency_stop_active is True
    publish_mock.reset_mock()

    # 실행: 비상정지 중 정상 범위의 주행 명령 수신
    command = Twist()
    command.linear.x = 0.1
    command.angular.z = 0.3
    node.on_command(command)

    # 검증: 주행 명령 대신 정지 명령 발행
    publish_mock.assert_called_once()
    published_command = publish_mock.call_args.args[0]

    assert published_command.linear.x == 0.0
    assert published_command.linear.y == 0.0
    assert published_command.linear.z == 0.0
    assert published_command.angular.x == 0.0
    assert published_command.angular.y == 0.0
    assert published_command.angular.z == 0.0

    assert node.emergency_stop_active is True
    assert node.last_command_time is None


def test_emergency_stop_release_requires_new_command(guard):
    node, publish_mock = guard

    # 준비: 비상정지 전에 주행 명령을 받은 상태
    old_command = Twist()
    old_command.linear.x = 0.1
    old_command.angular.z = 0.3
    node.on_command(old_command)

    activate_request = SetBool.Request()
    activate_request.data = True
    node.on_emergency_stop(activate_request, SetBool.Response())

    publish_mock.reset_mock()

    # 실행: 비상정지 해제
    release_request = SetBool.Request()
    release_request.data = False
    result = node.on_emergency_stop(
        release_request,
        SetBool.Response(),
    )

    assert result.success is True
    assert node.emergency_stop_active is False
    assert node.last_command_time is None

    # 해제 순간에도 정지 명령을 발행해야 함
    publish_mock.assert_called_once()
    assert publish_mock.call_args.args[0] == Twist()
    publish_mock.reset_mock()

    # 새 명령이 없으면 타이머 검사에서도 정지를 유지해야 함
    node.check_timeout()

    publish_mock.assert_called_once()
    assert publish_mock.call_args.args[0] == Twist()
    assert node.last_command_time is None
    publish_mock.reset_mock()

    # 새 명령을 받으면 해당 명령을 전달해야 함
    new_command = Twist()
    new_command.linear.x = -0.1
    new_command.angular.z = -0.3
    node.on_command(new_command)

    publish_mock.assert_called_once()
    published_command = publish_mock.call_args.args[0]

    assert published_command.linear.x == pytest.approx(-0.1)
    assert published_command.angular.z == pytest.approx(-0.3)
    assert node.last_command_time is not None


def test_manual_mode_accepts_manual_command(guard):
    node, publish_mock = guard
    node.control_mode = 'manual'

    command = Twist()
    command.linear.x = 0.1

    node.on_command_manual(command)

    publish_mock.assert_called_once()
    assert publish_mock.call_args.args[0].linear.x == pytest.approx(0.1)
    assert node.last_command_time is not None


def test_manual_mode_ignores_auto_command(guard):
    node, publish_mock = guard
    node.control_mode = 'manual'

    # 먼저 선택된 입력으로 정상 명령을 전달
    manual_command = Twist()
    manual_command.linear.x = 0.1
    node.on_command_manual(manual_command)

    last_manual_time = node.last_command_time
    publish_mock.reset_mock()

    # 선택되지 않은 입력으로 다른 명령 전달
    auto_command = Twist()
    auto_command.linear.x = -0.1
    node.on_command_auto(auto_command)

    # 출력도 없어야 하고 타임아웃 기준 시각도 유지되어야 함
    publish_mock.assert_not_called()
    assert node.last_command_time == last_manual_time


@pytest.mark.parametrize(
    'auto_enabled',
    [True, False],
    ids=['manual_to_auto', 'auto_to_manual'],
)
def test_mode_switch_stops_and_waits_for_selected_input(
    guard, auto_enabled,
):
    node, publish_mock = guard

    # 전환 전·후의 입력 콜백 선택
    if auto_enabled:
        node.control_mode = 'manual'
        old_command_callback = node.on_command_manual
        new_command_callback = node.on_command_auto
        expected_mode = 'auto'
    else:
        node.control_mode = 'auto'
        old_command_callback = node.on_command_auto
        new_command_callback = node.on_command_manual
        expected_mode = 'manual'

    # 준비: 전환 전 모드로 주행
    old_command = Twist()
    old_command.linear.x = 0.1
    old_command_callback(old_command)

    assert node.last_command_time is not None
    publish_mock.reset_mock()

    # 실행: 모드 전환 요청
    request = SetBool.Request()
    request.data = auto_enabled
    result = node.on_auto_mode(request, SetBool.Response())

    # 전환 순간에 정지하고 기존 명령 시간을 제거
    assert result.success is True
    assert node.control_mode == expected_mode
    assert node.last_command_time is None
    publish_mock.assert_called_once_with(Twist())
    publish_mock.reset_mock()

    # 새 입력이 없으면 정지 유지
    node.check_timeout()

    publish_mock.assert_called_once_with(Twist())
    publish_mock.reset_mock()

    # 이전 모드의 입력은 무시
    old_command_callback(old_command)

    publish_mock.assert_not_called()
    assert node.last_command_time is None

    # 새 모드의 입력은 전달
    new_command = Twist()
    new_command.linear.x = -0.1
    new_command_callback(new_command)

    publish_mock.assert_called_once()
    assert publish_mock.call_args.args[0].linear.x == pytest.approx(-0.1)
    assert node.last_command_time is not None


@pytest.mark.parametrize('auto_enabled', [True, False])
def test_mode_request_preserves_emergency_stop(guard, auto_enabled):
    node, publish_mock = guard

    # 준비: 비상정지 활성화
    stop_request = SetBool.Request()
    stop_request.data = True
    node.on_emergency_stop(stop_request, SetBool.Response())
    publish_mock.reset_mock()

    # 실행: 비상정지 상태에서 모드 선택
    mode_request = SetBool.Request()
    mode_request.data = auto_enabled
    node.on_auto_mode(mode_request, SetBool.Response())

    assert node.emergency_stop_active is True
    publish_mock.assert_called_once_with(Twist())
    publish_mock.reset_mock()

    # 선택된 입력이 들어와도 정지 출력
    command = Twist()
    command.linear.x = 0.1

    if auto_enabled:
        node.on_command_auto(command)
    else:
        node.on_command_manual(command)

    publish_mock.assert_called_once_with(Twist())
    assert node.last_command_time is None
