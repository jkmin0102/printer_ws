"""Single simulation velocity writer; defaults closed, lease and input watchdogs."""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool


class VelocityGate(Node):
    def __init__(self):
        super().__init__('velocity_gate')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('lease_timeout', 0.75)
        self.enabled = False
        self.lease_time = self.command_time = float('-inf')
        self.command = Twist()
        self.output = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_command, 10)
        self.create_subscription(Bool, '/namo/drive_enabled', self.on_lease, 10)
        # Wall timer continues publishing stop if /clock or mission process stops.
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        self.command, self.command_time = msg, time.monotonic()

    def on_lease(self, msg):
        self.enabled, self.lease_time = msg.data, time.monotonic()
        if not self.enabled:
            self.command = Twist()
            self.command_time = float('-inf')
            self.output.publish(Twist())

    def tick(self):
        now = time.monotonic()
        valid = (self.enabled and now-self.lease_time < self.get_parameter('lease_timeout').value
                 and now-self.command_time < self.get_parameter('command_timeout').value)
        self.output.publish(self.command if valid else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
