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

        self.declare_parameter('command_timeout_sec', 0.5)
        self.command_timeout_sec = self.get_parameter('command_timeout_sec').value
        self.last_command_time = None
        self.timer = self.create_timer(0.05, self.check_timeout)


    def on_command(self, msg):
        self.last_command_time = self.get_clock().now()
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
