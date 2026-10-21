"""Explicit test backend: wall-clock sensors and a kinematic actuator receiver.

This models the ROS contract, not physical braking, motor electronics, or collisions.
"""
import copy
import math
from pathlib import Path
import time
from uuid import uuid4

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from vehicle_interfaces.msg import ActuatorCommand, ActuatorStatus
from vehicle_interfaces.srv import ArmActuator
import yaml


class MockReceiver:
    """Receiver-local session/freshness checks matching the actuator interface."""
    def __init__(self, watchdog=.3, heartbeat_age=.2, max_linear=.2, max_angular=.6):
        if not all(math.isfinite(v) and v > 0 for v in (
                watchdog, heartbeat_age, max_linear, max_angular)) or heartbeat_age >= watchdog:
            raise ValueError('Require positive settings and heartbeat age < watchdog')
        self.watchdog, self.heartbeat_age = watchdog, heartbeat_age
        self.max_linear, self.max_angular = max_linear, max_angular
        self.status = ActuatorStatus(actuator_session=uuid4().hex, generation=1,
                                     stationary=True, reason='mock_startup')
        self.linear = self.angular = 0.
        self.heartbeats = {}
        self.last_command = None

    def disarm(self, reason):
        self.status.generation += 1
        self.status.armed = False
        self.status.guard_session = ''
        self.status.accepted_sequence = 0
        self.status.reason = reason
        self.linear = self.angular = 0.
        self.heartbeats.clear()

    def tick(self, now):
        if self.status.armed and now - self.last_command >= self.watchdog:
            self.status.expired_ns = int(now * 1e9)
            self.disarm('watchdog_expired')
        self.status.stationary = self.linear == 0. and self.angular == 0.
        if self.status.stationary:
            self.status.zero_applied_ns = int(now * 1e9)

    def heartbeat(self, now):
        self.tick(now)
        self.status.heartbeat += 1
        self.status.steady_ns = int(now * 1e9)
        self.heartbeats = {key: stamp for key, stamp in self.heartbeats.items()
                           if 0 <= now - stamp < self.heartbeat_age}
        self.heartbeats[self.status.heartbeat] = now
        return copy.deepcopy(self.status)

    def fresh(self, heartbeat, now):
        stamp = self.heartbeats.get(heartbeat)
        return stamp is not None and 0 <= now - stamp < self.heartbeat_age

    def arm(self, request, now):
        self.tick(now)
        status = self.status
        response = ArmActuator.Response(generation=status.generation,
                                        message='Stale session, generation, or heartbeat')
        if (request.actuator_session != status.actuator_session or
                request.generation != status.generation or not request.guard_session or
                not self.fresh(request.heartbeat, now)):
            return response
        if request.operation == request.ARM:
            if status.armed or status.emergency_stop or status.paused or not status.stationary:
                response.message = 'Actuator must be disarmed, stationary, and free of estop'
                return response
            self.disarm('armed_waiting_for_new_command')
            status.armed = True
            status.guard_session = request.guard_session
            self.last_command = now
        elif request.operation in (request.DISARM, request.ESTOP, request.CLEAR_ESTOP):
            if request.operation == request.ESTOP:
                status.emergency_stop = True
            elif request.operation == request.CLEAR_ESTOP:
                status.emergency_stop = False
            self.disarm('emergency_stop' if status.emergency_stop else 'explicit_disarm')
        else:
            response.message = 'Unknown operation'
            return response
        response.success = True
        response.generation = status.generation
        response.message = status.reason
        return response

    def command(self, msg, now):
        self.tick(now)
        status = self.status
        if (not status.armed or msg.actuator_session != status.actuator_session or
                msg.generation != status.generation or msg.guard_session != status.guard_session or
                msg.sequence <= status.accepted_sequence or not self.fresh(msg.heartbeat, now)):
            return False
        v = msg.velocity
        if (not math.isfinite(v.linear.x) or not math.isfinite(v.angular.z) or
                abs(v.linear.x) > self.max_linear or abs(v.angular.z) > self.max_angular or
                any(a != 0. for a in (v.linear.y, v.linear.z, v.angular.x, v.angular.y))):
            self.disarm('invalid_command')
            return False
        self.linear, self.angular = v.linear.x, v.angular.z
        self.last_command = now
        status.last_valid_ns = int(now * 1e9)
        status.accepted_sequence = msg.sequence
        return True


class ScanMap:
    """Raycast a trinary, axis-aligned P5 map used by the mock and map server."""
    def __init__(self, filename):
        filename = Path(filename)
        metadata = yaml.safe_load(filename.read_text())
        self.resolution = float(metadata['resolution'])
        self.origin = metadata['origin']
        if (metadata.get('mode', 'trinary') != 'trinary' or self.origin[2] != 0 or
                not math.isfinite(self.resolution) or self.resolution <= 0):
            raise ValueError('Mock scan requires an axis-aligned trinary PGM map')
        with (filename.parent / metadata['image']).open('rb') as stream:
            tokens = []
            while len(tokens) < 4:
                line = stream.readline()
                if not line:
                    raise ValueError('Incomplete PGM header')
                tokens.extend(line.split(b'#', 1)[0].split())
            if len(tokens) != 4 or tokens[0] != b'P5' or tokens[3] != b'255':
                raise ValueError('Mock scan requires an 8-bit binary PGM map')
            self.width, self.height = map(int, tokens[1:3])
            self.pixels = stream.read()
        if len(self.pixels) != self.width * self.height:
            raise ValueError('PGM pixel count does not match map dimensions')
        self.negate = bool(metadata.get('negate', 0))
        self.threshold = metadata['occupied_thresh']

    def occupied(self, x, y):
        col = math.floor((x - self.origin[0]) / self.resolution)
        row = self.height - 1 - math.floor((y - self.origin[1]) / self.resolution)
        if not (0 <= col < self.width and 0 <= row < self.height):
            return False
        pixel = self.pixels[row * self.width + col] / 255.
        probability = pixel if self.negate else 1. - pixel
        return probability > self.threshold

    def ranges(self, x, y, yaw, count=360):
        result = []
        for index in range(count):
            angle = yaw - math.pi + index * 2 * math.pi / (count - 1)
            dx, dy = math.cos(angle), math.sin(angle)
            distance = .1
            while distance <= 12.:
                if self.occupied(x + dx * distance, y + dy * distance):
                    break
                distance += self.resolution / 2
            result.append(distance if distance <= 12. else math.inf)
        return result


class MockHardware(Node):
    def __init__(self):
        super().__init__('mock_hardware')
        parameter = lambda name, default: self.declare_parameter(name, default).value
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Mock hardware requires wall clock (use_sim_time=false)')
        self.map = ScanMap(parameter('map_file', ''))
        self.receiver = MockReceiver(
            parameter('actuator_watchdog_sec', .3), parameter('actuator_heartbeat_timeout_sec', .2),
            parameter('max_linear_speed_mps', .2), parameter('max_angular_speed_rps', .6))
        for name in ('publish_scan', 'publish_odom', 'publish_status'):
            parameter(name, True)
        self.x = self.y = self.yaw = 0.
        self.wheel_positions = [0., 0.]
        self.previous_time = time.monotonic()
        self.status_pub = self.create_publisher(ActuatorStatus, 'actuator/status', 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.joints_pub = self.create_publisher(JointState, 'joint_states', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(ActuatorCommand, 'actuator/command',
                                 lambda msg: self.receiver.command(msg, time.monotonic()), 1)
        self.create_service(ArmActuator, 'actuator/arm', self.arm)
        self.create_service(Trigger, '~/restart_actuator', self.restart)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.update, clock=self.steady_clock)
        self.create_timer(.1, self.scan, clock=self.steady_clock)
        self.get_logger().warning('Explicit MOCK hardware: kinematics only, no physical actuator')

    def arm(self, request, response):
        return self.receiver.arm(request, time.monotonic())

    def restart(self, request, response):
        old = self.receiver
        self.receiver = MockReceiver(old.watchdog, old.heartbeat_age, old.max_linear, old.max_angular)
        response.success = True
        response.message = 'Mock actuator restarted disarmed with a new session'
        return response

    def enabled(self, name):
        return self.get_parameter(name).value

    def update(self):
        now = time.monotonic()
        dt = now - self.previous_time
        self.previous_time = now
        # A delayed receiver update must not integrate an expired command across the gap.
        self.receiver.tick(now)
        v, w = self.receiver.linear, self.receiver.angular
        self.x += v * math.cos(self.yaw + w * dt / 2) * dt
        self.y += v * math.sin(self.yaw + w * dt / 2) * dt
        self.yaw += w * dt
        status = self.receiver.heartbeat(now)
        status.left_output_rps = (v - w * .1203 / 2) / .0335
        status.right_output_rps = (v + w * .1203 / 2) / .0335
        if self.enabled('publish_status'):
            self.status_pub.publish(status)
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        if self.enabled('publish_odom'):
            self.odom_pub.publish(odom)
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['front_left_wheel_joint', 'rear_left_wheel_joint',
                       'front_right_wheel_joint', 'rear_right_wheel_joint']
        self.wheel_positions[0] += status.left_output_rps * dt
        self.wheel_positions[1] += status.right_output_rps * dt
        joints.name += ['camera_pan_joint', 'camera_tilt_joint']
        joints.position = [self.wheel_positions[0]] * 2 + [self.wheel_positions[1]] * 2 + [0., 0.]
        joints.velocity = [status.left_output_rps] * 2 + [status.right_output_rps] * 2 + [0., 0.]
        self.joints_pub.publish(joints)

    def scan(self):
        if not self.enabled('publish_scan'):
            return
        scan = LaserScan()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2 * math.pi / 359
        scan.range_min, scan.range_max, scan.scan_time = .1, 12., .1
        scan.ranges = self.map.ranges(self.x + .03 * math.cos(self.yaw),
                                      self.y + .03 * math.sin(self.yaw), self.yaw)
        scan.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = MockHardware()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
