import math
import copy
from uuid import uuid4
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from vehicle_interfaces.msg import (
    ControlState,
    NavigationCommand,
    NavigationPermission,
)
from vehicle_interfaces.srv import RegisterNavigation, PrepareActuator, ArmActuator
from rclpy.callback_groups import ReentrantCallbackGroup
from vehicle_control.actuator_link import ActuatorLink


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10,
        )

        self.declare_parameter('initial_mode', 'auto')
        self.control_mode = self.get_parameter('initial_mode').value

        if self.control_mode not in ('manual', 'auto'):
            raise ValueError('initial_mode must be manual or auto')

        self.cmd_sub_manual = self.create_subscription(
            Twist,
            'cmd_vel_manual',
            self.on_command_manual,
            10
        )

        self.cmd_sub_auto = self.create_subscription(
            NavigationCommand,
            'navigation_command',
            self.on_command_auto,
            10
        )

        self.declare_parameter('max_linear_speed_mps', 0.2)
        self.max_linear_speed_mps = self.get_parameter('max_linear_speed_mps').value

        self.declare_parameter('max_angular_speed_rps', 0.6)
        self.max_angular_speed_rps = self.get_parameter('max_angular_speed_rps').value

        self.declare_parameter('command_timeout_sec', 0.5)
        self.command_timeout_sec = self.get_parameter('command_timeout_sec').value

        if (
            not math.isfinite(self.command_timeout_sec) or self.command_timeout_sec <= 0.0
        ):
            raise ValueError('command_timeout_sec는 유한한 양수여야 합니다')

        for value in (self.max_linear_speed_mps, self.max_angular_speed_rps):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('Speed limits must be finite and positive')
        self.last_command_time = None
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.emergency_stop_active = False
        self.emergency_stop_service = self.create_service(
            SetBool,
            'set_emergency_stop',
            self.on_emergency_stop,
        )

        self.auto_mode_service = self.create_service(
            SetBool,
            'set_auto_mode',
            self._on_auto_mode_service,
            callback_group=ReentrantCallbackGroup(),
        )

        self.declare_parameter('navigation_timeout_sec', 0.5)
        self.navigation_timeout_sec = self.get_parameter(
            'navigation_timeout_sec'
        ).value

        if (
            not math.isfinite(self.navigation_timeout_sec)
            or self.navigation_timeout_sec <= 0.0
        ):
            raise ValueError('navigation_timeout_sec must be finite and positive')

        self._guard_session = uuid4().hex
        self._manager_session = ''
        self._manager_received_at = None
        self._navigation_reset_required = False
        self._epoch = 0
        self._heartbeat = 0
        self._heartbeat_times = {}
        self._permission = None
        self._permission_sequence = 0
        self._control_pub = self.create_publisher(ControlState, 'control_state', 1)
        self._register_service = self.create_service(
            RegisterNavigation, 'register_navigation', self.on_register_navigation,
        )
        self.navigation_allowed_subscription = self.create_subscription(
            NavigationPermission, 'navigation_permission',
            self.on_navigation_allowed, 1,
        )
        self._control_timer = self.create_timer(
            0.1, self.publish_control_state, clock=self.safety_clock,
        )
        self._cached_command = Twist()
        self._cached_auto_stamp_ns = None
        self._armed_ros_ns = 0
        self._prepared_goals = set()
        self.actuator = ActuatorLink(self, self._actuator_fault)
        self._output_timer = self.create_timer(
            self.actuator.period, self.publish_output, clock=self.safety_clock,
        )
        self._prepare_service = self.create_service(
            PrepareActuator, 'prepare_actuator', self.on_prepare_actuator,
            callback_group=ReentrantCallbackGroup(),
        )
        self.publish_control_state()

    def _actuator_fault(self):
        self._invalidate_navigation(disarm=False)
        self.get_logger().warning('Actuator unavailable or disarmed; explicit activation required')

    async def _on_auto_mode_service(self, request, response):
        self.on_auto_mode(request, response)
        if not request.data:
            try:
                await self.actuator.arm(self._epoch)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
        return response

    async def on_prepare_actuator(self, request, response):
        epoch = self._epoch
        if not (
            self.control_mode == 'auto' and not self.emergency_stop_active
            and not self._navigation_reset_required and request.goal_id
            and request.guard_session == self._guard_session
            and request.manager_session == self._manager_session
            and request.epoch == epoch and request.goal_id not in self._prepared_goals
        ):
            response.message = 'New goal and current control sessions are required'
            return response
        self._prepared_goals.add(request.goal_id)
        self.last_command_time = None
        self._cached_command = Twist()
        self.actuator.block()
        try:
            await self.actuator.arm(epoch)
            response.success = True
            response.message = 'Actuator armed; a new accepted goal command is required'
        except Exception as exc:
            response.message = str(exc)
        return response

    def publish_output(self):
        self._stop_published = False
        self.actuator.check()
        self.check_timeout()
        if self._stop_published:
            return
        command = Twist()
        if self.actuator.ready() and self.last_command_time is not None:
            if self.control_mode == 'manual' or (
                self._navigation_is_allowed()
                and self._cached_auto_stamp_ns is not None
                and 0 <= self.get_clock().now().nanoseconds - self._cached_auto_stamp_ns
                < self.command_timeout_sec * 1e9
            ):
                command = self._cached_command
        self.actuator.send(command)
        self.cmd_pub.publish(copy.deepcopy(command))

    def _publish_stop(self):
        self._stop_published = True
        self._cached_command = Twist()
        self.actuator.send(Twist())
        self.cmd_pub.publish(Twist())

    def publish_control_state(self):
        now = self.safety_clock.now().nanoseconds
        self._heartbeat += 1
        self._heartbeat_times[self._heartbeat] = now
        cutoff = now - int(self.navigation_timeout_sec * 1e9)
        self._heartbeat_times = {
            key: value for key, value in self._heartbeat_times.items()
            if value > cutoff
        }
        self._control_pub.publish(ControlState(
            guard_session=self._guard_session,
            manager_session=self._manager_session,
            epoch=self._epoch, heartbeat=self._heartbeat,
            auto_mode=self.control_mode == 'auto',
            emergency_stop=self.emergency_stop_active,
            navigation_reset_required=self._navigation_reset_required,
        ))

    def _invalidate_navigation(self, disarm=True):
        # Revoke locally before notifying the manager or waiting for cancellation.
        if disarm:
            self.actuator.block()
        self._cached_command = Twist()
        self._cached_auto_stamp_ns = None
        self._epoch += 1
        self._permission = None
        self._permission_sequence = 0
        self._heartbeat_times.clear()
        self.last_command_time = None
        self._publish_stop()
        self.publish_control_state()

    def on_register_navigation(self, request, response):
        response.success = bool(request.manager_session)
        if response.success and request.manager_session != self._manager_session:
            now = self.safety_clock.now().nanoseconds
            if (
                self._manager_received_at is not None
                and now - self._manager_received_at < self.navigation_timeout_sec * 1e9
            ):
                response.success = False
                return response
            # The new process has no handle for a possibly live predecessor goal.
            # Keep admission blocked until Nav2 and guard are restarted together.
            self._navigation_reset_required = bool(self._manager_session)
            self._manager_session = request.manager_session
            self._manager_received_at = None
            self._invalidate_navigation()
        return response

    def _permission_matches(self, permission):
        sent_at = self._heartbeat_times.get(permission.heartbeat)
        return (
            self.control_mode == 'auto'
            and not self.emergency_stop_active
            and not self._navigation_reset_required
            and bool(self._manager_session)
            and permission.guard_session == self._guard_session
            and permission.manager_session == self._manager_session
            and permission.epoch == self._epoch
            and permission.allowed
            and bool(permission.goal_id)
            and sent_at is not None
            and 0 <= self.safety_clock.now().nanoseconds - sent_at
            < self.navigation_timeout_sec * 1e9
        )

    def on_command_manual(self, msg):
        if self.control_mode != 'manual':
            return
        self.on_command(msg)

    def on_command_auto(self, msg):
        if self.control_mode != 'auto':
            return

        if not self._navigation_is_allowed():
            self.last_command_time = None
            self._publish_stop()
            return
        
        if (
            not self._permission_matches(msg.permission)
            or msg.permission.goal_id != self._permission.goal_id
        ):
            self.last_command_time = None
            self._publish_stop()
            return
        stamp = msg.command.header.stamp
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp_ns
        if not (self._armed_ros_ns < stamp_ns and 0 <= age < self.command_timeout_sec * 1e9):
            self.last_command_time = None
            self._publish_stop()
            return
        self._cached_auto_stamp_ns = stamp_ns
        self.on_command(msg.command.twist)

    def on_command(self, msg):
        if self.emergency_stop_active or not self.actuator.ready():
            self._publish_stop()
            return

        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )

        if not all(math.isfinite(value) for value in values):
            self.last_command_time = None
            self._publish_stop()
            self.get_logger().warning('비정상 속도 명령을 거부하고 정지합니다')
            return

        self.last_command_time = self.safety_clock.now()

        linear_speed = msg.linear.x
        if linear_speed > self.max_linear_speed_mps:
            msg.linear.x = self.max_linear_speed_mps
        elif linear_speed < -self.max_linear_speed_mps:
            msg.linear.x = -self.max_linear_speed_mps

        angular_speed = msg.angular.z
        if angular_speed > self.max_angular_speed_rps:
            msg.angular.z = self.max_angular_speed_rps
        elif angular_speed < -self.max_angular_speed_rps:
            msg.angular.z = -self.max_angular_speed_rps

        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        self._cached_command = copy.deepcopy(msg)
        self.cmd_pub.publish(msg)

    def on_emergency_stop(self, request, response):
        self.emergency_stop_active = request.data

        self._invalidate_navigation(disarm=False)
        self.actuator.block(ArmActuator.Request.ESTOP if request.data
                            else ArmActuator.Request.CLEAR_ESTOP)

        response.success = True

        if self.emergency_stop_active:
            response.message = '비상정지를 활성화했습니다'
            self.get_logger().warning(response.message)
        else:
            response.message = '비상정지를 해제했습니다. 수동 재선택 또는 새 목표가 필요합니다'
            self.get_logger().info(response.message)
        
        return response

    def on_auto_mode(self, request, response):
        next_mode = 'auto' if request.data else 'manual'

        self.control_mode = next_mode
        self._invalidate_navigation()

        response.success = True
        response.message = (
            f'{next_mode} 모드를 선택했습니다. 자율주행은 새 목표가 필요합니다.'
        )
        self.get_logger().info(response.message)

        return response


    def check_timeout(self):
        if self.emergency_stop_active:
            self._publish_stop()
            return

        if (
            self.control_mode == 'auto'
            and not self._navigation_is_allowed()
        ):
            if self._permission is not None:
                self._invalidate_navigation()
                return
            self.last_command_time = None
            self._publish_stop()
            return

        if self.last_command_time is None:
            self._publish_stop()
            return

        elapsed = (self.safety_clock.now() - self.last_command_time).nanoseconds / 1_000_000_000
        if elapsed >= self.command_timeout_sec:
            self.last_command_time = None
            self._publish_stop()
            self.get_logger().warning('속도 명령 타임아웃으로 정지합니다')
    
    def on_navigation_allowed(self, msg):
        if (
            msg.guard_session != self._guard_session
            or msg.manager_session != self._manager_session
            or msg.epoch != self._epoch
            or msg.sequence <= self._permission_sequence
            or msg.heartbeat not in self._heartbeat_times
        ):
            return
        self._permission_sequence = msg.sequence
        self._manager_received_at = self.safety_clock.now().nanoseconds
        self._permission = msg if self._permission_matches(msg) else None
        if self._permission is None and self.control_mode == 'auto':
            self.last_command_time = None
            self._publish_stop()

    def _navigation_is_allowed(self) -> bool:
        return (
            self._permission is not None
            and self._permission_matches(self._permission)
        )


def main(args=None):
    rclpy.init(args=args)
    node = CommandGuard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
