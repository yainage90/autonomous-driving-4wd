import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10,
        )

        self.control_mode = 'manual'

        self.cmd_sub_manual = self.create_subscription(
            Twist,
            'cmd_vel_manual',
            self.on_command_manual,
            10
        )

        self.cmd_sub_auto = self.create_subscription(
            Twist,
            'cmd_vel_auto',
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

        self.last_command_time = None
        self.safety_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.check_timeout, clock=self.safety_clock)

        self.emergency_stop_active = False
        self.emergency_stop_service = self.create_service(
            SetBool,
            'set_emergency_stop',
            self.on_emergency_stop,
        )

        self.auto_mode_service = self.create_service(
            SetBool,
            'set_auto_mode',
            self.on_auto_mode,
        )

    def on_command_manual(self, msg):
        if self.control_mode != 'manual':
            return
        self.on_command(msg)

    def on_command_auto(self, msg):
        if self.control_mode != 'auto':
            return
        self.on_command(msg)


    def on_command(self, msg):
        if self.emergency_stop_active:
            self.cmd_pub.publish(Twist())
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
            self.cmd_pub.publish(Twist())
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

        self.cmd_pub.publish(msg)

    def on_emergency_stop(self, request, response):
        self.emergency_stop_active = request.data

        self.last_command_time = None
        self.cmd_pub.publish(Twist())

        response.success = True

        if self.emergency_stop_active:
            response.message = '비상정지를 활성화했습니다'
            self.get_logger().warning(response.message)
        else:
            response.message = '비상정지를 해제했습니다. 새 속도 명령을 기다립니다'
            self.get_logger().info(response.message)
        
        return response

    def on_auto_mode(self, request, response):
        next_mode = 'auto' if request.data else 'manual'

        self.last_command_time = None
        self.cmd_pub.publish(Twist())

        self.control_mode = next_mode

        response.success = True
        response.message = (
            f'{next_mode} 모드를 선택했습니다. 정지하고 새 입력을 기다립니다.'
        )
        self.get_logger().info(response.message)

        return response


    def check_timeout(self):
        if self.emergency_stop_active:
            self.cmd_pub.publish(Twist())
            return

        if self.last_command_time is None:
            self.cmd_pub.publish(Twist())
            return

        elapsed = (self.safety_clock.now() - self.last_command_time).nanoseconds / 1_000_000_000
        if elapsed >= self.command_timeout_sec:
            self.last_command_time = None
            self.cmd_pub.publish(Twist())
            self.get_logger().warning('속도 명령 타임아웃으로 정지합니다')


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
