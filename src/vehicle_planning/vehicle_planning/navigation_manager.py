import time
import math
from enum import Enum
import json
from uuid import uuid4

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args
from rclpy.qos import qos_profile_sensor_data, DurabilityPolicy, QoSProfile
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from vehicle_interfaces.srv import SetNavigationGoal, RegisterNavigation
from vehicle_interfaces.msg import (
    ControlState,
    NavigationCommand,
    NavigationPermission,
)

from vehicle_planning.readiness import (
    is_data_fresh,
    is_map_valid,
    is_odom_valid,
    is_scan_valid,
)


class NavigationState(Enum):
    WAITING = 'waiting'
    SENDING = 'sending'
    RUNNING = 'running'
    CANCELING = 'canceling'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELED = 'canceled'
    STOPPING = 'stopping'


def classify_result(status: int, error_code: int) -> NavigationState:
    if status == GoalStatus.STATUS_CANCELED:
        return NavigationState.CANCELED

    if (
        status == GoalStatus.STATUS_SUCCEEDED
        and error_code == NavigateToPose.Result.NONE
    ):
        return NavigationState.SUCCEEDED

    return NavigationState.FAILED


def build_goal(x: float, y: float, yaw: float) -> NavigateToPose.Goal:
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError('x, y, yaw must be finite numbers')

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'

    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.y = float(y)

    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

    return goal


def is_stationary(
    linear_x: float,
    linear_y: float,
    angular_z: float,
) -> bool:
    velocities = (linear_x, linear_y, angular_z)

    if not all(math.isfinite(value) for value in velocities):
        return False

    linear_speed = math.hypot(linear_x, linear_y)

    return (
        linear_speed <= 0.02
        and abs(angular_z) <= 0.05
    )
    

def goal_pose_to_xy_yaw(msg: PoseStamped) -> tuple[float, float, float]:
    if msg.header.frame_id != 'map':
        raise ValueError('Goal frame must be map')

    position = msg.pose.position
    orientation = msg.pose.orientation

    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Goal pose must contain finite numbers')

    norm = math.hypot(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
        raise ValueError('Goal orientation must be a unit quaternion')

    qx = orientation.x / norm
    qy = orientation.y / norm
    qz = orientation.z / norm
    qw = orientation.w / norm

    if (
        abs(position.z) > 1e-6
        or abs(qx) > 1e-6
        or abs(qy) > 1e-6
    ):
        raise ValueError('Goal must be a planar pose')

    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )

    return position.x, position.y, yaw


class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        self._state = NavigationState.WAITING
        self._goal_handle = None
        self._result_future = None
        self._pending_result_state = None
        self._result_error = None
        self._stop_deadline = None
        self._stop_error = None

        self._last_feedback_log_time = None
        self._stationary_since_ns = None
        self._last_odom_stamp_ns = None
        self._last_odom_received_at = None
        self._goal_response_deadline = None
        self._cancel_deadline = None

        self._map_valid = False
        self._scan_msg = None
        self._scan_received_at = None
        self._odom_msg = None
        self._odom_received_at = None

        self._navigation_fault = None
        self._manager_session = uuid4().hex
        self._control_state = None
        self._control_received_at = None
        self._retired_guard_sessions = set()
        self._registration_future = None
        self._goal_control = None
        self._command_start_ns = None
        self._last_ros_time_ns = None
        self._permission_sequence = 0

        self._control_subscription = self.create_subscription(
            ControlState, 'control_state', self._control_callback, 1,
        )
        self._register_client = self.create_client(
            RegisterNavigation, 'register_navigation',
        )
        self._permission_pub = self.create_publisher(
            NavigationPermission, 'navigation_permission', 1,
        )
        self._command_pub = self.create_publisher(
            NavigationCommand, 'navigation_command', 1,
        )
        self._command_subscription = self.create_subscription(
            TwistStamped, 'cmd_vel_auto', self._auto_command_callback, 1,
        )

        self._observation_id = None
        self._events_pub = self.create_publisher(
            String,
            '~/events',
            100,
        )

        self.get_logger().info(f'State: {self._state.value}')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        self._goal_service = self.create_service(
            SetNavigationGoal,
            '~/set_goal',
            self._goal_callback,
        )

        self._goal_pose_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self._goal_pose_callback,
            10,
        )
                
        self._cancel_service = self.create_service(
            Trigger,
            '~/cancel',
            self._cancel_callback,
        )

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self._map_callback,
            map_qos,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self._odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            qos_profile_sensor_data,
        )

        self._navigation_pub = self.create_publisher(
            Bool,
            'navigation_allowed',
            1,
        )

        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._state_timer = self.create_timer(
            0.1,
            self._check_state,
            clock=self._steady_clock,
        )

        self._safety_timer = self.create_timer(
            0.1,
            self._check_navigation_safety,
            clock=self._steady_clock,
        )

    def _publish_event(self, event: str, **data) -> None:
        payload = {
            'schema_version': 1,
            'attempt_id': self._observation_id,
            'event': event,
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'wall_time_ns': time.time_ns(),
            'monotonic_ns': time.monotonic_ns(),
            'data': data,
        }
        message = String(
            data=json.dumps(
                payload,
                ensure_ascii=False,
                allow_nan=False,
            ),
        )
        self._events_pub.publish(message)

    def send_goal(self, x: float, y: float, yaw: float):
        if self._state in (
            NavigationState.SENDING,
            NavigationState.RUNNING,
            NavigationState.CANCELING,
            NavigationState.STOPPING,
        ):
            raise RuntimeError(
                'Navigation is busy; wait for the current goal or cancellation'
            )

        goal = build_goal(x, y, yaw)

        if not self._action_client.server_is_ready():
            raise TimeoutError('navigate_to_pose server is unavailable')

        readiness_error = self._readiness_error()
        if readiness_error is not None:
            raise TimeoutError(readiness_error)

        control_error = self._control_error()
        if control_error is not None:
            raise RuntimeError(control_error)
        if not self.is_stop_confirmed():
            raise RuntimeError('Vehicle must be confirmed stationary before a new goal')

        now = self.get_clock().now()
        if (
            self.get_parameter('use_sim_time').value
            and now.nanoseconds == 0
        ):
            raise TimeoutError('Simulation clock is unavailable')

        goal.pose.header.stamp = now.to_msg()

        self._navigation_fault = None
        self._result_future = None
        self._pending_result_state = None
        self._result_error = None
        self._goal_response_deadline = time.monotonic() + 5.0

        self._goal_control = (
            self._control_state.guard_session, self._control_state.epoch,
        )
        self._command_start_ns = None
        self._observation_id = uuid4().hex
        self._publish_event(
            'goal_submitted',
            frame_id='map',
            x=float(x),
            y=float(y),
            yaw=float(yaw),
        )

        self._set_state(NavigationState.SENDING)

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)
        return future

    def _goal_callback(self, request, response):
        try:
            self.send_goal(request.x, request.y, request.yaw)
        except (ValueError, TimeoutError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = 'Goal submitted; waiting for Nav2 acceptance'
        return response

    def _goal_pose_callback(self, msg: PoseStamped):
        try:
            x, y, yaw = goal_pose_to_xy_yaw(msg)
            self.send_goal(x, y, yaw)
        except (ValueError, TimeoutError, RuntimeError) as exc:
            self.get_logger().warning(
                f'Goal pose rejected: {exc}'
            )
            return

        self.get_logger().info(
            'Goal pose submitted; waiting for Nav2 acceptance'
        )

    def cancel_goal(self):
        if (
            self._state != NavigationState.RUNNING
            or self._goal_handle is None
        ):
            raise RuntimeError('No running goal to cancel')

        goal_handle = self._goal_handle
        cancel_future = goal_handle.cancel_goal_async()
        self._set_state(NavigationState.CANCELING)
        self._cancel_deadline = time.monotonic() + 5.0

        cancel_future.add_done_callback(
            lambda future: self._handle_cancel_response(
                future, goal_handle
            )
        )

        return cancel_future

    def _cancel_callback(self, _request, response):
        try:
            self.cancel_goal()
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = (
            'Cancellation requested; wait for the final action result'
        )
        return response
    
    def _set_state(self, state: NavigationState) -> None:
        if self._state == state:
            return

        self._state = state
        if state != NavigationState.RUNNING:
            self._navigation_pub.publish(Bool(data=False))
            self._permission_pub.publish(self._make_permission(False))
        self.get_logger().info(f'State: {state.value}')

        self._publish_event('state_changed', state=state.value)

    def _goal_response_callback(self, future):
        self._goal_response_deadline = None

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_navigation(
                f'Goal response failed; acceptance is unknown: {exc}'
            )
            return

        self._result_future = self._handle_goal_response(goal_handle)

        if self._result_future is not None:
            self._result_future.add_done_callback(
                self._result_callback
            )

    def _handle_goal_response(self, goal_handle):
        self._publish_event(
            'goal_response',
            accepted=bool(goal_handle.accepted),
        )

        if not goal_handle.accepted:
            self._set_state(NavigationState.FAILED)
            self.get_logger().error('Goal rejected')
            return None

        self._goal_handle = goal_handle
        self._last_feedback_log_time = None

        self._command_start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info('Goal accepted')
        self._set_state(NavigationState.RUNNING)

        if self._navigation_fault is not None:
            self._request_safety_cancel()

        try:
            return goal_handle.get_result_async()
        except Exception as exc:
            self._fail_navigation(
                f'Navigation result request failed: {exc}'
            )
            return None

    def _handle_result(self, response) -> NavigationState:
        state = classify_result(
            response.status,
            response.result.error_code,
        )

        self._publish_event(
            'action_result',
            status=int(response.status),
            error_code=int(response.result.error_code),
            error_msg=response.result.error_msg,
            outcome=state.value,
        )

        self._goal_handle = None
        self._cancel_deadline = None
        self.get_logger().info(f'Action result: {state.value}')
        self._set_state(NavigationState.STOPPING)

        if state == NavigationState.FAILED:
            self.get_logger().error(
                f'Navigation failed: status={response.status}, '
                f'error_code={response.result.error_code}, '
                f'error_msg={response.result.error_msg}'
            )

        if self._navigation_fault is not None:
            return NavigationState.FAILED

        return state

    def _handle_cancel_response(self, future, goal_handle):
        if (
            self._goal_handle is not goal_handle
            or self._state != NavigationState.CANCELING
        ):
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Cancel response failed: {exc}; waiting for goal result'
            )
            return

        if response.return_code == CancelGoal.Response.ERROR_REJECTED:
            self.get_logger().warning('Cancellation rejected')
            self._publish_event(
                'cancel_rejected', safety_fault=self._navigation_fault,
            )
            if self._navigation_fault is None:
                self._set_state(NavigationState.RUNNING)
            return

        if (
            response.return_code == CancelGoal.Response.ERROR_NONE
            and response.goals_canceling
        ):
            self.get_logger().info(
                'Cancellation accepted; waiting for goal result'
            )
            return

        self.get_logger().warning(
            f'Cancellation not confirmed: code={response.return_code}; '
            'waiting for goal result'
        )

    def _feedback_callback(self, feedback_msg) -> None:
        now = time.monotonic()

        if (
            self._last_feedback_log_time is not None
            and now - self._last_feedback_log_time < 1.0
        ):
            return

        self._last_feedback_log_time = now
        feedback = feedback_msg.feedback

        elapsed = (
            feedback.navigation_time.sec
            + feedback.navigation_time.nanosec / 1_000_000_000
        )
        remaining = (
            feedback.estimated_time_remaining.sec
            + feedback.estimated_time_remaining.nanosec / 1_000_000_000
        )

        self.get_logger().info(
            f'Progress: distance_remaining={feedback.distance_remaining:.2f} m, '
            f'elapsed={elapsed:.1f} s, '
            f'estimated_remaining={remaining:.1f} s, '
            f'recoveries={feedback.number_of_recoveries}'
        )

    def _map_callback(self, msg):
        self._map_valid = is_map_valid(msg)

    def _scan_callback(self, msg):
        self._scan_msg = msg
        self._scan_received_at = time.monotonic()

    def _odom_callback(self, msg):
        now = time.monotonic()
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        self._odom_msg = msg
        self._odom_received_at = now

        continuous = (
            self._last_odom_received_at is not None
            and now - self._last_odom_received_at <= 0.5
            and self._last_odom_stamp_ns is not None
            and 0 < stamp_ns - self._last_odom_stamp_ns <= 500_000_000
        )

        self._last_odom_received_at = now
        self._last_odom_stamp_ns = stamp_ns

        if not continuous:
            self._stationary_since_ns = None

        twist = msg.twist.twist
        if not (
            is_odom_valid(msg)
            and self._sensor_is_fresh(msg, now)
            and is_stationary(
                twist.linear.x,
                twist.linear.y,
                twist.angular.z,
            )
        ):
            self._stationary_since_ns = None
            return

        if self._stationary_since_ns is None:
            self._stationary_since_ns = stamp_ns

        
    def is_stop_confirmed(self) -> bool:
        if (
            self._stationary_since_ns is None
            or self._last_odom_received_at is None
        ):
            return False

        if not self._sensor_is_fresh(
            self._odom_msg,
            self._odom_received_at,
        ):
            return False

        return (
            self._last_odom_stamp_ns - self._stationary_since_ns
            >= 1_000_000_000
        )

    def _start_stop_check(self):
        self._stationary_since_ns = None
        self._last_odom_stamp_ns = None
        self._last_odom_received_at = None

        self._stop_error = None
        self._stop_deadline = time.monotonic() + 5.0

    def _check_state(self):
        if self._state == NavigationState.CANCELING:
            if (
                self._cancel_deadline is not None
                and time.monotonic() >= self._cancel_deadline
            ):
                self._cancel_deadline = None
                reason = (
                    'Cancellation timed out; waiting for the final action result '
                    '(issue 004)'
                )
                self._publish_event('cancel_timeout', reason=reason)
                self.get_logger().error(reason)
                self._fail_navigation(reason)
            return
        if self._state == NavigationState.SENDING:
            if (
                self._goal_response_deadline is not None
                and time.monotonic() >= self._goal_response_deadline
            ):
                self._goal_response_deadline = None
                self._fail_navigation(
                    'Goal response timed out; acceptance is unknown'
                )
            return

        if (
            self._state != NavigationState.STOPPING
            or self._stop_deadline is None
        ):
            return

        if time.monotonic() >= self._stop_deadline:
            self._stop_error = TimeoutError(
                'Vehicle stop was not confirmed within 5.0 s'
            )
            self.get_logger().error(str(self._stop_error))

            self._publish_event(
                'stop_timeout',
                reason=str(self._stop_error),
            )

            self._stop_deadline = None
            return

        if self.is_stop_confirmed():
            self._stop_deadline = None
            self.get_logger().info('Vehicle stop confirmed')
            self._set_state(self._pending_result_state)

    def _result_callback(self, future):
        if future is not self._result_future:
            return

        try:
            response = future.result()
        except Exception as exc:
            self._result_error = exc
            self._fail_navigation(
                f'Navigation result unavailable: {exc}'
            )
            return

        self._pending_result_state = self._handle_result(response)
        self._start_stop_check()

    def _sensor_is_fresh(self, msg, received_at) -> bool:
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )
        return is_data_fresh(
            received_at=received_at,
            stamp_ns=stamp_ns,
            now_monotonic=time.monotonic(),
            now_ros_ns=self.get_clock().now().nanoseconds,
        )

    def _readiness_error(self) -> str | None:
        if not self._map_valid:
            return 'Map is unavailable or invalid'

        if (
            self._scan_msg is None
            or not is_scan_valid(self._scan_msg)
            or not self._sensor_is_fresh(
                self._scan_msg, self._scan_received_at,
            )
        ):
            return 'Scan is unavailable, invalid, or stale'

        if (
            self._odom_msg is None
            or not is_odom_valid(self._odom_msg)
            or not self._sensor_is_fresh(
                self._odom_msg, self._odom_received_at,
            )
        ):
            return 'Odometry is unavailable, invalid, or stale'

        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(),
            )
            if not self._tf_buffer.can_transform(
                'base_footprint', 'laser_frame', Time(),
            ):
                return 'Laser TF is unavailable'
        except TransformException:
            return 'Navigation TF is unavailable'

        stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
        age_ns = self.get_clock().now().nanoseconds - stamp_ns

        if (
            stamp_ns <= 0
            or not -100_000_000 <= age_ns <= 500_000_000
        ):
            return 'Navigation TF is stale or future-dated'

        return None

    def _control_callback(self, msg):
        if not msg.guard_session or msg.guard_session in self._retired_guard_sessions:
            return
        previous = self._control_state
        if previous is not None:
            if msg.guard_session == previous.guard_session:
                if msg.heartbeat <= previous.heartbeat or msg.epoch < previous.epoch:
                    return
            else:
                self._retired_guard_sessions.add(previous.guard_session)
        self._control_state = msg
        self._control_received_at = time.monotonic()
        reason = self._control_error()
        if self._state in (
            NavigationState.SENDING,
            NavigationState.RUNNING,
            NavigationState.CANCELING,
        ) and reason is not None:
            self._fail_navigation(reason)

    def _control_error(self):
        control = self._control_state
        if (
            control is None or self._control_received_at is None
            or not 0 <= time.monotonic() - self._control_received_at < 0.5
        ):
            return 'Control state is unavailable or stale'
        if control.manager_session != self._manager_session:
            return 'Navigation manager session is not registered'
        if control.navigation_reset_required:
            return 'Manager restarted; restart Nav2 and command_guard together (issue 004)'
        if control.emergency_stop or not control.auto_mode:
            return 'Autonomous control is disabled'
        if self._state in (
            NavigationState.SENDING,
            NavigationState.RUNNING,
            NavigationState.CANCELING,
        ) and self._goal_control != (control.guard_session, control.epoch):
            return 'Control session changed; a new goal is required'
        return None

    def _ensure_registration(self):
        if self._registration_future is not None:
            if not self._registration_future.done():
                return
            try:
                self._registration_future.result()
            except Exception as exc:
                self.get_logger().warning(f'Control registration failed: {exc}')
            self._registration_future = None
        if (
            self._control_state is not None
            and self._control_state.manager_session == self._manager_session
        ):
            return
        if self._register_client.service_is_ready():
            self._registration_future = self._register_client.call_async(
                RegisterNavigation.Request(manager_session=self._manager_session),
            )

    def _make_permission(self, allowed):
        self._permission_sequence += 1
        control = self._control_state
        return NavigationPermission(
            guard_session=control.guard_session if control else '',
            manager_session=self._manager_session,
            epoch=control.epoch if control else 0,
            heartbeat=control.heartbeat if control else 0,
            sequence=self._permission_sequence,
            goal_id=self._observation_id or '',
            allowed=allowed,
        )

    def _auto_command_callback(self, msg):
        if (
            self._state != NavigationState.RUNNING
            or self._navigation_fault is not None
            or self._control_error() is not None
            or self._command_start_ns is None
        ):
            return
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        # Commands queued before this goal was accepted never acquire its session.
        if not self._command_start_ns < stamp_ns <= now_ns:
            return
        if now_ns - stamp_ns >= 500_000_000:
            return
        permission = self._make_permission(True)
        self._permission_pub.publish(permission)
        self._command_pub.publish(NavigationCommand(permission=permission, command=msg))

    def _check_navigation_safety(self):
        self._ensure_registration()
        now_ns = self.get_clock().now().nanoseconds
        clock_reversed = (
            self._last_ros_time_ns is not None
            and now_ns < self._last_ros_time_ns
        )
        self._last_ros_time_ns = now_ns
        if (
            self._state in (
                NavigationState.SENDING,
                NavigationState.RUNNING,
                NavigationState.CANCELING,
            )
            and self._navigation_fault is None
        ):
            reason = self._control_error() or self._readiness_error()
            if clock_reversed:
                reason = 'ROS clock moved backwards; a new goal is required'
            if reason is not None:
                self._fail_navigation(reason)

        allowed = (
            self._state == NavigationState.RUNNING
            and self._navigation_fault is None
            and self._control_error() is None
        )
        self._navigation_pub.publish(Bool(data=allowed))
        self._permission_pub.publish(self._make_permission(allowed))

    def _fail_navigation(self, reason: str):
        if self._navigation_fault is not None:
            return

        self._navigation_fault = reason
        self._navigation_pub.publish(Bool(data=False))
        self._permission_pub.publish(self._make_permission(False))
        self._publish_event(
            'safety_failure',
            reason=reason,
        )
        self.get_logger().error(
            f'Navigation safety failure: {reason}'
        )
        self._request_safety_cancel()

    def _request_safety_cancel(self):
        if (
            self._state != NavigationState.RUNNING
            or self._goal_handle is None
        ):
            return

        try:
            self.cancel_goal()
        except Exception as exc:
            self._set_state(NavigationState.CANCELING)
            self._cancel_deadline = time.monotonic() + 5.0
            self.get_logger().error(
                f'Safety cancellation request failed: {exc}; '
                'waiting for the goal result'
            )


def main():
    rclpy.init()
    node = NavigationManager()

    try:
        rclpy.spin(node)
        return 0

    except (KeyboardInterrupt, ExternalShutdownException):
        return 130

    except (ValueError, TimeoutError, RuntimeError) as exc:
        node.get_logger().error(str(exc))
        return 1

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
