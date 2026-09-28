"""Opt-in ROS integration using the real planner/follower and a kinematic robot.

Run in an isolated ROS domain after building all navigation packages:
NATTO_NAVIGATION_INTEGRATION=1 pytest -q test/test_navigation_integration.py
No hardware nodes are started.
"""
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from natto_msgs.msg import NavigationFeedback, NavigationPath, NavigationGoal
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from natto_navigation.joy_goal_selector import JoyGoalSelector
from natto_navigation.navigation_manager import NavigationManager

pytestmark = pytest.mark.skipif(
    os.environ.get('NATTO_NAVIGATION_INTEGRATION') != '1', reason='opt-in ROS integration')


class Harness:
    def __init__(self, directory):
        self.processes = []
        self.logs = []
        self.namespace = '/nav_integration'
        self.executor = SingleThreadedExecutor()
        self.probe = Node('probe', namespace=self.namespace)
        self.executor.add_node(self.probe)
        bindings = directory / 'bindings.yaml'
        bindings.write_text('bindings: {button_0: flag, button_1: home, button_2: blocked, button_3: turn}\n')
        destinations = directory / 'destinations.yaml'
        destinations.write_text('''frame_id: map
destinations:
  flag: {x: 2.0, y: 1.0, yaw_deg: 0.0}
  home: {x: 1.0, y: 2.0, yaw_deg: 90.0}
  blocked: {x: 3.0, y: 3.0, yaw_deg: 0.0}
  turn: {x: 1.0, y: 1.0, yaw_deg: 90.0}
''')
        self.selector = JoyGoalSelector(namespace=self.namespace, parameter_overrides=[
            Parameter('bindings_path', value=str(bindings))])
        self.manager = NavigationManager(namespace=self.namespace, parameter_overrides=[
            Parameter('destinations_path', value=str(destinations))])
        self.executor.add_node(self.selector)
        self.executor.add_node(self.manager)
        self.nodes = [self.probe, self.selector, self.manager]
        self.joy_pub = self.probe.create_publisher(Joy, 'joy', 10)
        self.pose_pub = self.probe.create_publisher(PoseStamped, 'current_pose', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.probe.create_publisher(OccupancyGrid, 'occupancy_grid', qos)
        self.footprint_pub = self.probe.create_publisher(PolygonStamped, 'footprint', 10)
        self.stale_feedback = self.probe.create_publisher(NavigationFeedback, 'navigation_feedback', 10)
        self.legacy_arrival = self.probe.create_publisher(Bool, 'goal_reached', 10)
        self.command = TwistStamped()
        self.paths = []
        self.probe.create_subscription(TwistStamped, 'selected_twist', self.on_command, 10)
        self.probe.create_subscription(NavigationPath, 'navigation_path', self.paths.append, 10)
        self.buttons = [0] * 17
        self.x, self.y, self.yaw = 1.0, 1.0, 0.0
        self.simulate = True
        self.publish_joy = True
        self.publish_pose = True
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.resolution = 0.1
        self.map.info.width = self.map.info.height = 50
        self.map.info.origin.orientation.w = 1.0
        self.map.data = [0] * 2500
        for x in range(28, 33):
            for y in range(28, 33):
                self.map.data[y * 50 + x] = 100
        self.footprint = PolygonStamped()
        self.footprint.header.frame_id = 'base_link'
        self.footprint.polygon.points = [Point32(x=x, y=y) for x, y in
                                        [(-0.05, -0.05), (0.05, -0.05), (0.05, 0.05), (-0.05, 0.05)]]
        self.probe.create_timer(0.02, self.tick)
        self.start(directory, 'natto_astar_planner', 'astar_planner', [
            '-p', 'managed_navigation:=true', '-p', 'xy_inflation:=0.0', '-p', 'xy_offset:=0.0'])
        self.start(directory, 'natto_holonomic_pure_pursuit', 'holonomic_pure_pursuit', [
            '-p', 'managed_navigation:=true', '-p', 'max_speed_xy_m_s:=0.5',
            '-p', 'goal_position_tolerance_m:=0.02', '-p', 'goal_yaw_tolerance_deg:=5.0'])
        self.start(directory, 'natto_command_selector', 'twist_selector', [
            '-r', 'auto_twist:=command_velocity', '-r', 'manual_twist:=manual_velocity'])
        self.map_pub.publish(self.map)
        self.spin(1.0)

    def start(self, directory, package, executable, arguments):
        binary = str(Path(get_package_prefix(package)) / 'lib' / package / executable)
        log = open(directory / f'{len(self.processes)}_{executable}.log', 'w')
        self.logs.append(log)
        self.processes.append(subprocess.Popen(
            [binary, '--ros-args', '-r', f'__ns:={self.namespace}', *arguments],
            stdout=log, stderr=subprocess.STDOUT))

    def on_command(self, msg):
        self.command = msg

    def tick(self):
        if self.simulate:
            twist = self.command.twist
            self.x += (math.cos(self.yaw) * twist.linear.x - math.sin(self.yaw) * twist.linear.y) * 0.02
            self.y += (math.sin(self.yaw) * twist.linear.x + math.cos(self.yaw) * twist.linear.y) * 0.02
            self.yaw += twist.angular.z * 0.02
        if self.publish_joy:
            self.joy_pub.publish(Joy(buttons=self.buttons))
        if self.publish_pose:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.probe.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = self.x, self.y
            pose.pose.orientation.z = math.sin(self.yaw / 2.0)
            pose.pose.orientation.w = math.cos(self.yaw / 2.0)
            self.pose_pub.publish(pose)
        self.footprint_pub.publish(self.footprint)

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        assert all(p.poll() is None for p in self.processes), 'ROS child exited; inspect test logs'

    def until(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.spin(0.02)
        assert predicate(), f'timed out: state={self.manager.state}, request={self.manager.request_id}'

    def press(self, destination):
        self.buttons = [0] * 17
        self.buttons[16] = 1
        self.spin(0.08)
        self.buttons[destination] = 1
        self.spin(0.08)
        self.buttons[destination] = 0

    def is_stopped(self):
        t = self.command.twist
        return (t.linear.x, t.linear.y, t.angular.z) == (0.0, 0.0, 0.0)

    def close(self):
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        for log in self.logs:
            log.close()


@pytest.fixture
def robot(tmp_path):
    rclpy.init()
    harness = Harness(tmp_path)
    try:
        yield harness
    finally:
        harness.close()
        rclpy.shutdown()


def test_real_arrival_retarget_cancel_and_failure(robot):
    robot.press(0)
    robot.until(lambda: robot.manager.state == 'moving')
    first = robot.manager.request_id
    robot.press(1)
    robot.until(lambda: robot.manager.request_id != first and robot.manager.state == 'moving')
    second = robot.manager.request_id
    robot.stale_feedback.publish(NavigationFeedback(request_id=first, reached=True))
    robot.legacy_arrival.publish(Bool(data=True))
    robot.spin(0.15)
    assert robot.manager.request_id == second
    robot.buttons = [0] * 17
    robot.until(lambda: robot.manager.state == 'canceled' and robot.is_stopped())
    robot.buttons[16] = 1
    robot.map_pub.publish(robot.map)
    robot.spin(0.4)
    assert not robot.manager.request_id
    assert robot.is_stopped()
    canceled_count = len(robot.paths)
    robot.map_pub.publish(robot.map)
    robot.spin(0.2)
    assert not any(p.valid for p in robot.paths[canceled_count:])
    robot.press(2)
    robot.until(lambda: robot.manager.state == 'failed' and robot.is_stopped())
    failed_count = len(robot.paths)
    robot.map_pub.publish(robot.map)
    robot.spin(0.2)
    assert not any(p.valid for p in robot.paths[failed_count:])
    robot.press(0)
    robot.until(lambda: robot.manager.state == 'arrived', timeout=15)
    robot.until(robot.is_stopped)
    assert abs(robot.x - 2.0) < 0.04
    assert abs(robot.y - 1.0) < 0.04
    assert not robot.manager.request_id


def test_same_position_rotation_and_disconnection(robot):
    robot.press(3)
    robot.until(lambda: robot.manager.state == 'arrived', timeout=15)
    assert math.hypot(robot.x - 1.0, robot.y - 1.0) < 0.04
    assert abs(robot.yaw - math.pi / 2) < math.radians(6)
    robot.press(0)
    robot.until(lambda: robot.manager.state == 'moving')
    robot.publish_joy = False
    robot.until(lambda: robot.manager.state == 'canceled' and robot.is_stopped(), timeout=2)
    robot.publish_joy = True
    robot.spin(0.3)
    assert not robot.manager.request_id
    assert robot.is_stopped()


def test_pose_timeout_stops_even_when_joy_keeps_arriving(robot):
    robot.press(0)
    robot.until(lambda: robot.manager.state == 'moving')
    robot.publish_pose = False
    robot.until(lambda: robot.manager.state == 'failed' and robot.is_stopped(), timeout=2)
    robot.publish_pose = True
    robot.spin(0.3)
    assert not robot.manager.request_id
    assert robot.is_stopped()


def test_unassigned_power_buttons_emit_no_power_command(robot, tmp_path):
    power_messages, allow_messages = [], []
    robot.probe.create_subscription(Bool, 'power', power_messages.append, 10)
    robot.probe.create_subscription(Bool, 'legacy_allow_auto_drive', allow_messages.append, 10)
    robot.start(tmp_path, 'natto_joy', 'button_manager', [
        '-p', 'num_button:=17', '-p', 'button_16.function:=allow_auto_drive',
        '-p', 'button_16.mode:=hold', '-r', 'allow_auto_drive:=legacy_allow_auto_drive'])
    # DDS discovery may take longer than a fixed startup delay.
    robot.until(lambda: bool(allow_messages), timeout=5)
    robot.buttons[0] = 1
    robot.spin(0.1)
    robot.buttons[0] = 0
    robot.buttons[1] = 1
    robot.spin(0.1)
    assert not allow_messages[-1].data
    robot.buttons[16] = 1
    robot.until(lambda: allow_messages[-1].data, timeout=2)
    robot.buttons[16] = 0
    robot.until(lambda: not allow_messages[-1].data, timeout=2)
    assert power_messages == []


def test_goal_waiting_for_first_pose_can_plan_without_another_map(robot, tmp_path):
    paths = []
    robot.probe.create_subscription(NavigationPath, 'late/path', paths.append, 10)
    goals = robot.probe.create_publisher(NavigationGoal, 'late/goal', 10)
    poses = robot.probe.create_publisher(PoseStamped, 'late/pose', 10)
    robot.start(tmp_path, 'natto_astar_planner', 'astar_planner', [
        '-r', '__node:=late_planner', '-p', 'managed_navigation:=true',
        '-p', 'xy_inflation:=0.0', '-p', 'xy_offset:=0.0',
        '-r', 'navigation_goal:=late/goal', '-r', 'navigation_path:=late/path',
        '-r', 'current_pose:=late/pose'])
    robot.until(lambda: goals.get_subscription_count() == 1 and poses.get_subscription_count() == 1)
    robot.spin(0.2)
    goal = NavigationGoal(request_id='late-pose')
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = 1.5
    goal.pose.pose.position.y = 1.5
    goal.pose.pose.orientation.w = 1.0
    goals.publish(goal)
    robot.spin(0.2)
    assert not paths
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = pose.pose.position.y = 1.0
    pose.pose.orientation.w = 1.0
    poses.publish(pose)
    robot.until(lambda: any(p.valid and p.request_id == 'late-pose' for p in paths))
