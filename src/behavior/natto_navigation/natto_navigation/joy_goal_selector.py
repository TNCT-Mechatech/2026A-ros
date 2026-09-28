import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from natto_msgs.msg import NavigationInput

from .config import ButtonEdges, load_bindings


class JoyGoalSelector(Node):
    def __init__(self, **kwargs):
        super().__init__('joy_goal_selector', **kwargs)
        path = self.declare_parameter('bindings_path', '').value
        enable = self.declare_parameter('enable_button', 16).value
        self.timeout = self.declare_parameter('joy_timeout_sec', 0.5).value
        if enable < 0 or self.timeout <= 0:
            raise ValueError('invalid enable button or joy timeout')
        self.edges = ButtonEdges(load_bindings(path, enable), enable)
        self.last_joy = None
        self.connected = False
        self.enabled = False
        self.publisher = self.create_publisher(NavigationInput, 'navigation_input', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, qos_profile_sensor_data)
        self.create_timer(0.02, self.tick)

    def publish(self, destination=''):
        self.publisher.publish(NavigationInput(
            connected=self.connected, enabled=self.enabled, destination=destination))

    def on_joy(self, msg):
        now = time.monotonic()
        if self.last_joy is None or now - self.last_joy > self.timeout:
            self.edges.reset()
        self.last_joy = now
        self.connected, self.enabled, destination = self.edges.sample(msg.buttons)
        self.publish(destination)

    def tick(self):
        if self.last_joy is None or time.monotonic() - self.last_joy > self.timeout:
            self.edges.reset()
            self.connected = self.enabled = False
        self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = JoyGoalSelector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
