"""Own navigation lifecycle and gate all automatic velocity commands."""
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from natto_msgs.msg import (
    NavigationInput, NavigationGoal, NavigationPath,
    NavigationFeedback, NavigationStatus,
)

from .config import load_destinations


class NavigationManager(Node):
    def __init__(self, **kwargs):
        super().__init__('navigation_manager', **kwargs)
        self.frame, self.destinations = load_destinations(
            self.declare_parameter('destinations_path', '').value)
        self.input_timeout = self.declare_parameter('input_timeout_sec', 0.5).value
        self.pose_timeout = self.declare_parameter('pose_timeout_sec', 0.5).value
        self.plan_timeout = self.declare_parameter('plan_timeout_sec', 5.0).value
        self.feedback_timeout = self.declare_parameter('feedback_timeout_sec', 0.5).value
        self.travel_timeout = self.declare_parameter('travel_timeout_sec', 60.0).value
        if min(self.input_timeout, self.pose_timeout, self.plan_timeout,
               self.feedback_timeout, self.travel_timeout) <= 0:
            raise ValueError('timeouts must be positive')
        self.request_id = ''
        self.destination = ''
        self.state = 'idle'
        self.enabled = False
        self.last_input = self.last_pose = self.last_feedback = None
        self.started = 0.0
        self.last_command = None
        self.goal_pub = self.create_publisher(NavigationGoal, 'navigation_goal', 10)
        self.command_pub = self.create_publisher(TwistStamped, 'command_velocity', 10)
        self.allow_pub = self.create_publisher(Bool, 'allow_auto_drive', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(NavigationStatus, 'navigation_status', qos)
        self.create_subscription(NavigationInput, 'navigation_input', self.on_input, 10)
        self.create_subscription(NavigationPath, 'navigation_path', self.on_path, 10)
        self.create_subscription(NavigationFeedback, 'navigation_feedback', self.on_feedback, 10)
        self.create_subscription(PoseStamped, 'current_pose', self.on_pose, qos_profile_sensor_data)
        self.create_timer(0.02, self.tick)
        self.set_status('idle')

    def set_status(self, state, reason=''):
        self.state = state
        self.status_pub.publish(NavigationStatus(
            request_id=self.request_id, destination=self.destination, state=state, reason=reason))
        self.get_logger().info(f'{state}: {self.destination} {reason}')

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command/base_link'
        self.command_pub.publish(msg)
        self.last_command = None

    def finish(self, state, reason=''):
        if self.request_id:
            self.goal_pub.publish(NavigationGoal(request_id=self.request_id, cancel=True))
        self.stop()
        self.set_status(state, reason)
        self.request_id = ''
        self.last_feedback = None

    def on_pose(self, msg):
        pose = msg.pose
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w)
        norm = sum(v * v for v in values[2:])
        if msg.header.frame_id == self.frame and all(math.isfinite(v) for v in values) and abs(norm - 1.0) < 0.01:
            self.last_pose = time.monotonic()
        else:
            self.last_pose = None

    def on_input(self, msg):
        self.last_input = time.monotonic()
        self.enabled = msg.connected and msg.enabled
        # Stop/cancel before exposing a mode change to the downstream selector.
        if not self.enabled and self.request_id:
            self.finish('canceled', 'permission released' if msg.connected else 'controller disconnected')
        self.allow_pub.publish(Bool(data=self.enabled))
        if not self.enabled or not msg.destination:
            return
        if self.request_id and msg.destination == self.destination:
            return
        if msg.destination not in self.destinations:
            self.get_logger().warning(f'Destination disabled: {msg.destination}')
            return
        if self.request_id:
            self.finish('canceled', 'destination changed')
        self.destination = msg.destination
        if self.last_pose is None or time.monotonic() - self.last_pose > self.pose_timeout:
            self.finish('failed', 'current pose unavailable')
            return
        self.request_id = uuid.uuid4().hex
        self.started = time.monotonic()
        self.last_feedback = None
        self.stop()
        destination = self.destinations[self.destination]
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(destination['x'])
        pose.pose.position.y = float(destination['y'])
        yaw = math.radians(destination['yaw_deg'])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.set_status('planning')
        self.goal_pub.publish(NavigationGoal(request_id=self.request_id, pose=pose))

    def on_path(self, msg):
        if not self.request_id or msg.request_id != self.request_id:
            return
        if not msg.valid:
            self.finish('failed', msg.reason or 'no path')
        # Path and feedback have different writers; a matching feedback packet
        # is the acknowledgement that the follower installed this request.

    def on_feedback(self, msg):
        if not self.enabled or not self.request_id or msg.request_id != self.request_id:
            return
        now = time.monotonic()
        if self.last_pose is None or now - self.last_pose > self.pose_timeout:
            self.finish('failed', 'current pose timeout')
            return
        twist = msg.command.twist
        if not all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y, twist.angular.z)):
            self.finish('failed', 'invalid velocity command')
            return
        if msg.reached:
            self.finish('arrived')
            return
        self.last_feedback = now
        self.last_command = msg.command
        if self.state != 'moving':
            self.set_status('moving')

    def tick(self):
        now = time.monotonic()
        if self.last_input is None or now - self.last_input > self.input_timeout:
            self.enabled = False
            if self.request_id:
                self.finish('canceled', 'input timeout')
        if self.request_id:
            if self.last_pose is None or now - self.last_pose > self.pose_timeout:
                self.finish('failed', 'current pose timeout')
            elif now - self.started > self.travel_timeout:
                self.finish('failed', 'travel timeout')
            elif self.state == 'planning' and now - self.started > self.plan_timeout:
                self.finish('failed', 'planning timeout')
            elif self.state == 'moving' and (self.last_feedback is None or now - self.last_feedback > self.feedback_timeout):
                self.finish('failed', 'follower timeout')
        self.allow_pub.publish(Bool(data=self.enabled))
        if self.enabled and self.request_id and self.state == 'moving' and self.last_command is not None:
            self.last_command.header.stamp = self.get_clock().now().to_msg()
            self.command_pub.publish(self.last_command)
        else:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.context.ok():
            node.finish('canceled', 'shutdown')
            node.allow_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()
