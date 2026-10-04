import math
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.on_command,
            10
        )

        self.declare_parameter('max_linear_speed_mps', 0.2)
        self.max_linear_speed_mps = self.get_parameter('max_linear_speed_mps').value

        self.declare_parameter('max_angular_speed_rps', 0.6)
        self.max_angular_speed_rps = self.get_parameter('max_angular_speed_rps').value

        self.declare_parameter('command_timeout_sec', 0.5)
        self.command_timeout_sec = self.get_parameter('command_timeout_sec').value

        self.last_command_time = None
        self.timer = self.create_timer(0.05, self.check_timeout)


    def on_command(self, msg):
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

        self.last_command_time = self.get_clock().now()

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

    def check_timeout(self):
        if self.last_command_time is None:
            self.cmd_pub.publish(Twist())
            return

        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1_000_000_000
        if elapsed > self.command_timeout_sec:
            self.cmd_pub.publish(Twist())
            return
        


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
