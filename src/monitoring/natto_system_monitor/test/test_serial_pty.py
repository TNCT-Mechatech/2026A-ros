"""Manual integration test: real mechanism executables connected only to PTYs.

Build inject2026A/roll2026A first; run with python3. Never opens a robot device.
"""
import os
import pty
import select
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Joy


class SerialPTY(unittest.TestCase):
    def run_mechanisms(self, package, names):
        rclpy.init()
        node = rclpy.create_node('serial_diagnostic_probe')
        statuses = {}
        def callback(msg):
            for s in msg.status:
                statuses[s.name] = s
        node.create_subscription(DiagnosticArray, '/diagnostics', callback, 100)
        publisher = node.create_publisher(Joy, '/inject/joy', 10)
        pairs = [pty.openpty() for _ in names]
        processes = []
        logs = []
        def wait_for(predicate, timeout=5):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.03)
                if predicate():
                    return
            self.fail(f'Timed out: {statuses}')
        def values(name):
            return {v.key: v.value for v in statuses[name + ': Hardware/Serial'].values}
        def level(name):
            value = statuses[name + ': Hardware/Serial'].level
            return int.from_bytes(value) if isinstance(value, bytes) else value
        try:
            with tempfile.TemporaryDirectory(prefix='natto_serial_test_') as folder:
                config = Path(folder) / 'ports.yaml'
                config.write_text(yaml.safe_dump({name: {'ros__parameters': {
                    'serial_path': os.ttyname(slave), 'diagnostic_updater.period': .05,
                    'serial_diagnostics.recovery_warn_sec': .05,
                }} for name, (_, slave) in zip(names, pairs)}))
                binary = Path(get_package_prefix(package)) / 'lib' / package / 'main'
                log = open(f'/tmp/natto_{package}_pty.log', 'w')
                logs.append(log)
                process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config)], stdout=log, stderr=log)
                processes.append(process)
                wait_for(lambda: all(name + ': Hardware/Serial' in statuses for name in names))
                wait_for(lambda: all('write_success_count' in values(name) for name in names))
                for name, (master, _) in zip(names, pairs):
                    self.assertEqual(level(name), 0)
                    self.assertEqual(values(name)['write_success_count'], '0')
                    self.assertFalse(select.select([master], [], [], 0)[0], 'Diagnostics must not trigger writes')
                wait_for(lambda: publisher.get_subscription_count() == len(names))
                msg = Joy(axes=[0.0] * 8, buttons=[0] * 13)
                publisher.publish(msg)
                wait_for(lambda: all(values(name)['write_success_count'] == '1' for name in names))
                for master, _ in pairs:
                    self.assertTrue(select.select([master], [], [], 1)[0])
                    packet = os.read(master, 4096)
                    self.assertGreater(len(packet), 4)
                    self.assertEqual(packet[0], 0)
                    self.assertEqual(packet[-1], 0)
                # Disconnect one endpoint; no reconnect or additional writes are introduced.
                os.close(pairs[0][0])
                pairs[0] = (-1, pairs[0][1])
                wait_for(lambda: level(names[0]) == 2)
                for _ in range(3):
                    publisher.publish(msg)
                    until = time.monotonic() + .08
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.01)
                wait_for(lambda: int(values(names[0])['write_failure_count']) >= 3)
                self.assertEqual(values(names[0])['consecutive_write_errors'], '3')
                self.assertEqual(values(names[0])['write_success_count'], '1')
                self.assertIsNone(process.poll())
        finally:
            for process in processes:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5)
            for master, slave in pairs:
                if master >= 0:
                    os.close(master)
                os.close(slave)
            for log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()

    def test_inject_and_roll_nodes(self):
        self.run_mechanisms('inject2026A', ['inject_serial_node', 'roll_serial_node'])

    def test_standalone_roll(self):
        self.run_mechanisms('roll2026A', ['serial_node'])


if __name__ == '__main__':
    unittest.main()
