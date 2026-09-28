"""Manual regression: debug enabled/disabled must produce identical commands."""
import math
import os
import signal
import subprocess
import time
import unittest
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path as RosPath
from natto_msgs.msg import PursuitDebug


class PursuitDebugRegression(unittest.TestCase):
    def test_commands_are_identical(self):
        rclpy.init()
        node = rclpy.create_node('pursuit_debug_probe')
        poses = node.create_publisher(PoseStamped, '/probe/pose', 10)
        paths = node.create_publisher(RosPath, '/probe/path', 10)
        outputs = {'on': [], 'off': []}
        debug = []
        for mode in outputs:
            node.create_subscription(TwistStamped, f'/{mode}/command_velocity', lambda m, mode=mode: outputs[mode].append(m), 100)
        node.create_subscription(PursuitDebug, '/on/pursuit_debug', debug.append, qos_profile_sensor_data)
        processes, logs = [], []
        def spin_until(predicate, timeout=5):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)
                if predicate():
                    return
            self.fail('Pursuit did not publish expected output')
        try:
            binary = Path(get_package_prefix('natto_holonomic_pure_pursuit')) / 'lib/natto_holonomic_pure_pursuit/holonomic_pure_pursuit'
            for mode in outputs:
                log = open(f'/tmp/natto_pursuit_debug_{mode}.log', 'w')
                logs.append(log)
                processes.append(subprocess.Popen([
                    str(binary), '--ros-args', '-r', f'__ns:=/{mode}',
                    '-r', 'current_pose:=/probe/pose', '-r', 'path:=/probe/path',
                    '-p', f'publish_debug:={str(mode == "on").lower()}',
                ], stdout=log, stderr=log))
            spin_until(lambda: poses.get_subscription_count() == 2 and paths.get_subscription_count() == 2)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.orientation.w = 1.0
            pose.pose.position.y = .1
            path = RosPath()
            path.header.frame_id = 'map'
            for x in (0.0, 1.0):
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.orientation.w = 1.0
                p.pose.position.x = x
                path.poses.append(p)
            poses.publish(pose)
            paths.publish(path)
            spin_until(lambda: min(len(v) for v in outputs.values()) >= 25 and len(debug) >= 25)
            for a, b in zip(outputs['on'][:25], outputs['off'][:25]):
                self.assertEqual(a.twist, b.twist)
            self.assertEqual(node.count_publishers('/off/pursuit_debug'), 0)
            self.assertAlmostEqual(debug[0].path_error, .1)
            self.assertAlmostEqual(debug[0].goal_distance, math.hypot(1, .1))
            self.assertAlmostEqual(debug[0].goal_yaw_error, 0)
            self.assertGreater(debug[0].target_speed, 0)
        finally:
            for process in processes:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            for log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
