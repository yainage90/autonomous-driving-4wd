"""Guard-side transport. The independent stop decision belongs to the receiver."""
import copy
import math
import time
from collections import deque

from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.task import Future
from vehicle_interfaces.msg import ActuatorCommand, ActuatorStatus
from vehicle_interfaces.srv import ArmActuator


class ActuatorLink:
    def __init__(self, node, fault_callback):
        self.node = node
        self.fault_callback = fault_callback
        self.status = None
        self.received_at = None
        self.token = None
        self.sequence = 0
        self.retired_sessions = set()
        self.pending_disarm = None
        self.busy = False
        self.controls = deque()
        self.control_request = None
        self.wait_generation = None
        self.revision = 0
        self.period = self._parameter('actuator_output_period_sec', 0.05)
        self.timeout = self._parameter('actuator_status_timeout_sec', 0.2)
        heartbeat_age = self._parameter('actuator_heartbeat_timeout_sec', 0.2)
        watchdog = self._parameter('actuator_watchdog_sec', 0.3)
        if not (self.period < heartbeat_age < watchdog and self.period < self.timeout < watchdog):
            raise ValueError('Require output period < freshness windows < watchdog')
        qos = QoSProfile(depth=1)
        self.publisher = node.create_publisher(ActuatorCommand, 'actuator/command', qos)
        self.subscription = node.create_subscription(
            ActuatorStatus, 'actuator/status', self._status_callback, qos,
        )
        self.client = node.create_client(
            ArmActuator, 'actuator/arm', callback_group=ReentrantCallbackGroup(),
        )

        self.control_timer = node.create_timer(
            0.01, self._pump_controls, clock=node.safety_clock,
            callback_group=ReentrantCallbackGroup(),
        )

    def _parameter(self, name, default):
        value = self.node.declare_parameter(name, default).value
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def fresh(self):
        return self.status is not None and self.received_at is not None and (
            time.monotonic() - self.received_at < self.timeout
        )

    def ready(self):
        return self.fresh() and self.token is not None and not self.status.paused

    def _status_callback(self, status):
        if status.actuator_session in self.retired_sessions:
            return
        old = self.status
        if old is not None:
            if old.actuator_session == status.actuator_session:
                if status.heartbeat <= old.heartbeat or status.generation < old.generation:
                    return
            else:
                self.retired_sessions.add(old.actuator_session)
                self.retry_after_heartbeat = 0
                self.wait_generation = None
        self.status = status
        self.received_at = time.monotonic()
        if self.token is not None and (
            self.token != (status.actuator_session, status.generation)
            or not status.armed or status.emergency_stop or status.paused
            or status.guard_session != self.node._guard_session
        ):
            self.token = None
            self.fault_callback()

    def check(self):
        if self.token is not None and not self.fresh():
            self.token = None
            self.fault_callback()

    def send(self, velocity):
        if not self.ready():
            return
        self.sequence += 1
        self.publisher.publish(ActuatorCommand(
            actuator_session=self.token[0], generation=self.token[1],
            guard_session=self.node._guard_session, sequence=self.sequence,
            heartbeat=self.status.heartbeat, velocity=copy.deepcopy(velocity),
        ))

    def _request(self, operation):
        if not self.fresh() or not self.client.service_is_ready():
            return None
        status = self.status
        return self.client.call_async(ArmActuator.Request(
            actuator_session=status.actuator_session, generation=status.generation,
            guard_session=self.node._guard_session, heartbeat=status.heartbeat,
            operation=operation,
        ))

    def block(self, operation=ArmActuator.Request.DISARM):
        self.send(Twist())
        self.token = None
        self.revision += 1
        result = Future()
        # Fire-and-forget stops still report a failed delivery; the receiver watchdog
        # remains responsible for stopping if the transport is unavailable.
        def report(done):
            try:
                done.result()
            except Exception as exc:
                self.node.get_logger().warning(str(exc))
        result.add_done_callback(report)
        # Serialize operations so a fast ESTOP -> CLEAR cannot use the same generation.
        self.controls.append((operation, result, time.monotonic() + 1.0))
        self.pending_disarm = result
        self._pump_controls()
        return result

    def _pump_controls(self):
        if (self.control_request is not None and not self.control_request.done()
                and time.monotonic() >= self.controls[0][2]):
            # A dead receiver must not leave the operation queue blocked forever.
            self.client.remove_pending_request(self.control_request)
            self.control_request.cancel()
            self.control_request = None
            _, result, _ = self.controls.popleft()
            if not result.done():
                result.set_exception(TimeoutError('Actuator control response timed out'))
        if self.control_request is not None:
            if not self.control_request.done():
                return
            try:
                response = self.control_request.result()
                operation, result, deadline = self.controls[0]
                if response.success:
                    self.controls.popleft()
                    self.wait_generation = (self.status.actuator_session, response.generation)
                    if not result.done():
                        result.set_result(response)
                # Stale requests retry only after receiving a newer state heartbeat.
                self.retry_after_heartbeat = self.status.heartbeat
            except Exception as exc:
                _, result, _ = self.controls.popleft()
                if not result.done():
                    result.set_exception(exc)
            self.control_request = None
        if not self.controls:
            return
        operation, result, deadline = self.controls[0]
        if time.monotonic() >= deadline:
            self.controls.popleft()
            if not result.done():
                result.set_exception(TimeoutError('Actuator control operation timed out'))
            return
        if not self.fresh():
            return
        if self.wait_generation is not None:
            if (self.status.actuator_session == self.wait_generation[0]
                    and self.status.generation < self.wait_generation[1]):
                # A receiver restart invalidates the old acknowledgment; the new receiver
                # is disarmed and can accept the next explicit operation.
                if self.status.actuator_session == self.wait_generation[0]:
                    return
            self.wait_generation = None
        if self.status.heartbeat <= getattr(self, 'retry_after_heartbeat', 0):
            return
        self.control_request = self._request(operation)

    async def _bounded(self, future):
        if future is None:
            raise RuntimeError('Actuator state or service is unavailable')
        result = Future()
        def complete(done):
            if result.done():
                return
            try:
                result.set_result(done.result())
            except Exception as exc:
                result.set_exception(exc)
        future.add_done_callback(complete)
        timer = self.node.create_timer(
            1.0, lambda: result.set_exception(TimeoutError('Actuator response timed out'))
            if not result.done() else None, clock=self.node.safety_clock,
            callback_group=ReentrantCallbackGroup(),
        )
        try:
            return await result
        finally:
            self.node.destroy_timer(timer)

    async def _wait_status(self, predicate, timeout=1.0):
        result = Future()
        deadline = time.monotonic() + timeout
        def poll():
            if result.done():
                return
            if self.fresh() and predicate(self.status):
                result.set_result(True)
            elif time.monotonic() >= deadline:
                result.set_exception(TimeoutError('Fresh actuator state was not received'))
        timer = self.node.create_timer(
            0.01, poll, clock=self.node.safety_clock,
            callback_group=ReentrantCallbackGroup(),
        )
        try:
            poll()
            return await result
        finally:
            self.node.destroy_timer(timer)

    async def arm(self, epoch):
        if self.node.emergency_stop_active:
            raise RuntimeError('Emergency stop must be cleared separately')
        if self.busy:
            raise RuntimeError('Actuator activation is already in progress')
        self.busy = True
        revision = self.revision
        try:
            if self.pending_disarm is not None:
                session = self.status.actuator_session if self.status else ''
                response = await self._bounded(self.pending_disarm)
                self.pending_disarm = None
                if not response.success:
                    raise RuntimeError(response.message)
                await self._wait_status(lambda s: s.actuator_session == session and
                                        s.generation == response.generation)
            await self._wait_status(lambda s: not s.armed and s.stationary and
                                    not s.paused and not s.emergency_stop, timeout=3.0)
            if (self.node._epoch != epoch or self.node.emergency_stop_active
                    or self.revision != revision):
                raise RuntimeError('Control changed during actuator activation')
            session = self.status.actuator_session
            response = await self._bounded(self._request(ArmActuator.Request.ARM))
            if not response.success:
                raise RuntimeError(response.message)
            await self._wait_status(lambda s: s.actuator_session == session and
                                    s.generation == response.generation and s.armed)
            if (self.node._epoch != epoch or self.node.emergency_stop_active
                    or self.revision != revision):
                raise RuntimeError('Control changed during actuator activation')
            self.token = (session, response.generation)
            self.sequence = 0
            self.node.last_command_time = None
            self.node._cached_command = Twist()
            self.node._armed_ros_ns = self.node.get_clock().now().nanoseconds
        except Exception:
            self.block()
            raise
        finally:
            self.busy = False
