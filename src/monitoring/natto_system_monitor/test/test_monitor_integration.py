"""Isolated DDS domain; synthetic inputs only, no robot control or serial writes."""
import os
import signal
import subprocess
import time
import unittest
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '194'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from natto_msgs.msg import NavigationStatus, NavigationPath, NavigationFeedback
from ament_index_python.packages import get_package_prefix, get_package_share_directory


class MonitorIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('monitor_test_probe')
        self.status = {}
        self.agg = {}
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self.on_status, 100)
        self.node.create_subscription(DiagnosticArray, '/diagnostics_agg', self.on_agg, 100)
        self.processes = []
        self.logs = []

    def tearDown(self):
        for process in reversed(self.processes):
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def on_status(self, msg):
        for status in msg.status:
            self.status[status.name] = status

    def on_agg(self, msg):
        for status in msg.status:
            self.agg[status.name] = status

    def spawn(self, package, executable, args):
        log = open(f'/tmp/natto_test_{executable}_{os.getpid()}.log', 'w')
        self.logs.append(log)
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        self.processes.append(subprocess.Popen([str(binary), *args], stdout=log, stderr=log))

    def start(self, managed=False):
        parameters = {
            'diagnostic_updater.period': .1, 'frequency_window_sec': .5,
            'front.min_hz': 15.0, 'front.timeout_sec': .5,
            'rear.enabled': False, 'localization.min_hz': 15.0,
            'localization.node': '', 'path.node': '', 'command.node': '',
            'path.warn_age_sec': .3, 'path.timeout_sec': 1.0,
            'command.min_hz': 15.0, 'command.timeout_sec': .5,
            'managed_navigation': managed, 'navigation_manager_node': '/monitor_test_probe',
            'state_timeout_sec': .5,
        }
        args = ['--ros-args']
        for key, value in parameters.items():
            literal = str(value).lower() if isinstance(value, bool) else str(value)
            args += ['-p', f'{key}:={literal if literal else chr(39)+chr(39)}']
        self.spawn('natto_system_monitor', 'system_monitor', args)
        self.spawn('diagnostic_aggregator', 'aggregator_node', [
            '--ros-args', '-r', '__node:=diagnostic_aggregator', '--params-file',
            str(Path(get_package_share_directory('natto_system_monitor')) / 'config/diagnostics.yaml')])

    def pump(self, duration, callback=lambda: None, period=.025):
        end = time.monotonic() + duration
        next_send = 0
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                callback()
                next_send = time.monotonic() + period
            rclpy.spin_once(self.node, timeout_sec=.005)
        for process in self.processes:
            self.assertIsNone(process.poll(), 'Process exited; inspect /tmp/natto_test_*.log')

    def check(self, suffix, level, state=None):
        name = f'system_monitor: {suffix}'
        self.assertIn(name, self.status)
        s = self.status[name]
        self.assertEqual(int.from_bytes(s.level) if isinstance(s.level, bytes) else s.level, level, (name, s.message, s.values))
        if state:
            self.assertEqual(dict((v.key, v.value) for v in s.values)['operational_state'], state)

    def inputs(self, managed=False):
        scan_pub = self.node.create_publisher(LaserScan, '/sensing/lidar/laserscan/front', qos_profile_sensor_data)
        pose_pub = self.node.create_publisher(PoseStamped, '/localization/current_pose', 10)
        scan = LaserScan()
        scan.header.frame_id = 'lidar_front'
        scan.range_min, scan.range_max = .1, 30.0
        scan.angle_increment, scan.angle_max = .5, 1.0
        scan.ranges = [1.0, float('inf'), 2.0]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        path = RosPath()
        path.header.frame_id = 'map'
        path.poses = [pose, pose]
        twist = TwistStamped()
        twist.twist.linear.x = .2
        def sensing():
            scan.header.stamp = self.node.get_clock().now().to_msg()
            pose.header.stamp = scan.header.stamp
            scan_pub.publish(scan)
            pose_pub.publish(pose)
        return scan, pose, path, twist, sensing

    def test_sequence_health_and_idle(self):
        scan, pose, path, twist, sensing = self.inputs()
        path_pub = self.node.create_publisher(RosPath, '/planning/path', 10)
        command_pub = self.node.create_publisher(TwistStamped, '/control/command_velocity', 10)
        self.start()
        self.pump(2.5, sensing)
        self.check('Sensors/Front LiDAR', 0)
        self.check('Localization/MCL', 0)
        self.check('Planning/Path', 0, 'IDLE')
        self.check('Control/Pure Pursuit', 0, 'IDLE')
        self.pump(1.2, sensing, .1)
        self.check('Sensors/Front LiDAR', 1)
        scan.range_max = 0.0
        self.pump(.6, sensing)
        self.check('Sensors/Front LiDAR', 2)
        self.pump(1.7)
        self.check('Sensors/Front LiDAR', 3)
        self.assertEqual(self.agg['/SYSTEM/Sensors'].level, bytes([3]))
        self.assertEqual(self.agg['/SYSTEM'].level, bytes([3]))
        scan.range_max = 30.0
        pose.pose.orientation.w = 0.0
        self.pump(.7, sensing)
        self.check('Localization/MCL', 2)
        pose.pose.orientation.w = 1.0
        def moving():
            sensing(); path_pub.publish(path); command_pub.publish(twist)
        self.pump(1.5, moving)
        self.check('Planning/Path', 0, 'RUNNING')
        self.check('Control/Pure Pursuit', 0, 'RUNNING')
        def lost_command():
            sensing(); path_pub.publish(path)
        self.pump(.8, lost_command)
        self.check('Control/Pure Pursuit', 3)
        path.poses = []
        self.pump(.3, lambda: (sensing(), path_pub.publish(path)))
        self.pump(1.3, sensing)
        self.check('Planning/Path', 0, 'IDLE')
        self.check('Control/Pure Pursuit', 0, 'IDLE')
        self.node.destroy_publisher(path_pub)
        self.pump(1.0, sensing)
        self.check('Planning/Path', 3)

    def test_managed_latched_state_and_heartbeat(self):
        scan, pose, path, twist, sensing = self.inputs(True)
        status_pub = self.node.create_publisher(NavigationStatus, '/navigation/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        heartbeat = self.node.create_publisher(Bool, '/controller/allow_auto_drive', 10)
        path_pub = self.node.create_publisher(NavigationPath, '/navigation/path', 10)
        command_pub = self.node.create_publisher(NavigationFeedback, '/navigation/feedback', 10)
        status = NavigationStatus(state='idle')
        status_pub.publish(status)
        self.start(True)
        def idle():
            sensing(); heartbeat.publish(Bool(data=False))
        self.pump(2.5, idle)
        self.check('Planning/Path', 0, 'IDLE')
        self.check('Control/Pure Pursuit', 0, 'IDLE')
        status.state, status.request_id = 'moving', 'test'
        status_pub.publish(status)
        def moving():
            idle()
            path_pub.publish(NavigationPath(request_id='test', path=path, valid=True))
            command_pub.publish(NavigationFeedback(request_id='test', command=twist))
        self.pump(1.2, moving)
        self.check('Planning/Path', 0, 'RUNNING')
        self.check('Control/Pure Pursuit', 0, 'RUNNING')
        # Cancellation Path can arrive before its terminal status across writers.
        path_pub.publish(NavigationPath(request_id='test', valid=False, reason='canceled'))
        self.pump(.2, idle)
        status.state = 'arrived'
        status_pub.publish(status)
        self.pump(1.5, idle)
        self.check('Planning/Path', 0, 'IDLE')
        self.check('Control/Pure Pursuit', 0, 'IDLE')
        self.pump(.8, sensing)
        self.check('Planning/Path', 3, 'UNKNOWN')
        status.state, status.reason = 'failed', 'test failure'
        status_pub.publish(status)
        self.pump(.5, idle)
        self.check('Planning/Path', 2, 'STOPPED')


if __name__ == '__main__':
    unittest.main()
