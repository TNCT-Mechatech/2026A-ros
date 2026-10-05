"""Manual integration test using only PTYs, never robot serial devices.

Build natto_system_monitor, inject2026A and natto_robot_hardware, source
install/setup.bash, then run with /usr/bin/python3.
"""
import os
import pty
import re
import xml.etree.ElementTree as ET
import select
import signal
import struct
import subprocess
import tempfile
import time
import tty
import unittest
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Joy, JointState


def encode(raw):
    out = bytearray([0])
    code_index, code = 0, 1
    for byte in raw:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
    out[code_index] = code
    return b'\x00' + out + b'\x00'


def decode(frame):
    out = bytearray()
    offset = 0
    while offset < len(frame):
        code = frame[offset]
        if code == 0 or offset + code > len(frame):
            raise AssertionError('Invalid COBS command')
        out.extend(frame[offset + 1:offset + code])
        offset += code
        if code < 255 and offset < len(frame):
            out.append(0)
    return out


def feedback(frame_id=1, corrupt=False):
    # C++ MotorFeedback is four float32 values, bool, and three padding bytes.
    raw = bytes([frame_id]) + struct.pack('<ffff?3x', 1., 0., 0., 0., False) * 4
    return encode(raw + bytes([(sum(raw) + int(corrupt)) & 255]))


class SerialPTY(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('serial_watchdog_probe')
        self.statuses, self.history = {}, [set() for _ in range(3)]
        self.names = ['inject_serial_node', 'roll_serial_node', 'serial_motor_bridge']
        self.keys = [name + ': Hardware/Serial' for name in self.names[:2]] + [
            'natto_robot_hardware/serial_motor_bridge']
        def diagnostic(msg):
            for status in msg.status:
                self.statuses[status.name] = status
                if status.name in self.keys:
                    self.history[self.keys.index(status.name)].add(status.message)
        self.node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 100)
        self.joy_pub = self.node.create_publisher(Joy, '/inject/joy', 10)
        self.wheel_pub = self.node.create_publisher(JointState, '/command_joint_states', 10)
        self.pairs = [(-1, -1) for _ in range(3)]
        self.buffers = [bytearray() for _ in range(3)]
        self.packets = [[] for _ in range(3)]
        self.processes, self.logs = [], []
        self.folder = tempfile.TemporaryDirectory(prefix='natto_serial_watchdog_')
        self.paths = [Path(self.folder.name) / name for name in self.names]

    def attach(self, i):
        master, slave = pty.openpty()
        tty.setraw(slave)  # Prevent feedback echoes before the node opens the port.
        self.pairs[i] = (master, slave)
        self.paths[i].symlink_to(os.ttyname(slave))
        self.buffers[i].clear()

    def detach(self, i):
        self.paths[i].unlink(missing_ok=True)
        for fd in self.pairs[i]:
            if fd >= 0:
                os.close(fd)
        self.pairs[i] = (-1, -1)
        self.buffers[i].clear()

    def start(self, attached=True, supervised=False, monitor_updates=True):
        if attached:
            for i in range(3):
                self.attach(i)
        config = Path(self.folder.name) / 'ports.yaml'
        config.write_text(yaml.safe_dump({name: {'ros__parameters': {
            'serial_path': str(self.paths[i]), 'diagnostic_updater.period': .01,
            'timeout_ms': 300, 'reconnect_interval_ms': 100,
            **({'pid': False, 'command_mode': 'open_loop_normalized',
                'feedback_timeout_ms': 200, 'startup_feedback_timeout_ms': 300,
                'feedback_recovery_count': 3} if i == 2 else {
                    'serial_update_failure_limit': 20, 'monitor_serial_updates': monitor_updates}),
        }} for i, name in enumerate(self.names)}))
        if supervised:
            # Include the actual mechanism launch; copy the actual wheel node so
            # production respawn settings/parameter forwarding are exercised.
            launch = ET.Element('launch')
            for name, value in [('auto_serial_restart', 'true'), ('serial_path', str(self.paths[2])), ('gain_config_path', '')]:
                ET.SubElement(launch, 'arg', name=name, default=value)
            mechanisms = ET.SubElement(launch, 'include', file=str(Path(get_package_share_directory('inject2026A')) / 'launch/inject.launch.xml'))
            ET.SubElement(mechanisms, 'arg', name='serial_config', value=str(config))
            ET.SubElement(mechanisms, 'arg', name='auto_restart', value='true')
            source = ET.parse(Path(get_package_share_directory('natto_nhk2025_sample')) / 'launch/mecanum_auto_red.launch.xml')
            wheel = next(node for node in source.iter('node') if node.get('exec') == 'serial_motor_bridge')
            ET.SubElement(wheel, 'param', attrib={'from': str(config)})
            launch.append(wheel)
            launch_path = Path(self.folder.name) / 'restart.launch.xml'
            ET.ElementTree(launch).write(launch_path, encoding='unicode')
            self.launch_log = Path(self.folder.name) / 'launch.log'
            log = self.launch_log.open('w')
            self.logs.append(log)
            self.processes.append(subprocess.Popen(['ros2', 'launch', str(launch_path)], stdout=log, stderr=log))
        else:
            for package, executable in [('inject2026A', 'main'), ('natto_robot_hardware', 'serial_motor_bridge')]:
                binary = Path(get_package_prefix(package)) / 'lib' / package / executable
                log = open(f'/tmp/natto_{package}_watchdog_pty.log', 'w')
                self.logs.append(log)
                self.processes.append(subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config)], stdout=log, stderr=log))
        self.wait_for(lambda: self.joy_pub.get_subscription_count() == 2 and self.wheel_pub.get_subscription_count() == 1)

    def pump(self):
        rclpy.spin_once(self.node, timeout_sec=.005)
        for process in self.processes:
            self.assertIsNone(process.poll(), 'Serial node exited unexpectedly')
        for i, (master, _) in enumerate(self.pairs):
            if master < 0:
                continue
            while select.select([master], [], [], 0)[0]:
                self.buffers[i].extend(os.read(master, 65536))
            while 0 in self.buffers[i]:
                end = self.buffers[i].index(0)
                frame = bytes(self.buffers[i][:end])
                del self.buffers[i][:end + 1]
                if frame:
                    raw = decode(frame)
                    self.assertEqual(len(raw), 82)
                    self.assertEqual(raw[0], 0)
                    self.assertEqual(sum(raw[:-1]) & 255, raw[-1])
                    self.packets[i].append([struct.unpack_from('<f', raw, 1 + 20*j)[0] for j in range(4)])

    def value(self, i, key):
        status = self.statuses.get(self.keys[i])
        return next((v.value for v in status.values if v.key == key), None) if status else None

    def attempts(self, i):
        return int(self.value(i, 'reconnect_attempts') or 0)

    def wait_for(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                return
        self.fail(f'Timed out; statuses={self.statuses}')

    def command(self, neutral):
        joy = Joy(axes=[0.] * 8, buttons=[0] * 13)
        if not neutral:
            joy.buttons[0] = 1
            joy.axes[6] = 1.
        self.joy_pub.publish(joy)
        self.wheel_pub.publish(JointState(velocity=[0. if neutral else 1.] * 4))

    def run_for(self, seconds, receive=(), neutral=None, packet=None):
        end, next_send = time.monotonic() + seconds, 0.
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                for i in receive:
                    os.write(self.pairs[i][0], feedback() if packet is None else packet)
                if neutral is not None:
                    self.command(neutral)
                next_send = time.monotonic() + .025
            self.pump()

    def clear(self):
        self.pump()
        for records in self.packets:
            records.clear()

    def all_zero(self, indices=range(3), require_packets=True):
        for i in indices:
            if require_packets:
                self.assertTrue(self.packets[i], f'No commands on port {i}')
            self.assertTrue(all(all(v == 0 for v in p) for p in self.packets[i]), (i, self.packets[i]))

    def moving(self, indices=range(3)):
        for i in indices:
            self.assertTrue(any(any(v != 0 for v in p) for p in self.packets[i]), i)

    def arm_and_move(self):
        self.run_for(.45, range(3), True)
        self.clear()
        self.run_for(.12, range(3), False)
        self.moving()

    def test_silent_and_invalid_feedback_reopens_each_port_and_preserves_stop(self):
        self.start()
        # No response: update() stays -1, even though OS writes succeed.
        self.run_for(.9, neutral=False)
        self.all_zero()
        self.assertTrue(all(self.attempts(i) >= 2 for i in range(3)))
        before = [self.attempts(i) for i in range(3)]
        self.run_for(.45, range(3), False, feedback(corrupt=True))
        self.run_for(.45, range(3), False, feedback(frame_id=9))
        self.all_zero()
        self.assertTrue(all(self.attempts(i) > before[i] for i in range(3)))
        # Reopening alone cannot resume a held command.
        self.run_for(.45, range(3), False)
        self.all_zero()
        self.assertTrue(all(self.value(i, 'watchdog_state') == 'Waiting for neutral command' for i in range(3)))
        self.arm_and_move()
        # A short no-data interval does not trigger reconnect or stop.
        before = [self.attempts(i) for i in range(3)]
        self.clear()
        self.run_for(.07, neutral=False)
        self.moving()
        self.assertEqual(before, [self.attempts(i) for i in range(3)])
        # Inject timeout reopens only inject, even with command messages arriving.
        self.run_for(.5, [1, 2], False)
        self.clear()
        self.run_for(.15, [1, 2], False)
        self.all_zero([0], require_packets=False)
        self.moving([1, 2])
        self.assertGreater(self.attempts(0), before[0])
        self.assertEqual(before[1:], [self.attempts(i) for i in [1, 2]])
        self.arm_and_move()
        # Command loss with healthy serial feedback stops but does not reopen.
        before = [self.attempts(i) for i in range(3)]
        self.run_for(.4, range(3))
        self.clear()
        self.run_for(.1, range(3))
        self.all_zero()
        self.assertTrue(all(self.value(i, 'watchdog_state') == 'Command timeout' for i in range(3)))
        self.assertEqual(before, [self.attempts(i) for i in range(3)])

    def test_missing_devices_at_start_can_be_plugged_in_without_restart(self):
        self.start(attached=False)
        self.run_for(.35, neutral=False)
        self.assertTrue(all(self.attempts(i) >= 2 for i in range(3)))
        for i in range(3):
            self.attach(i)
        self.run_for(.45, range(3), False)
        self.all_zero()
        self.arm_and_move()

    def test_mechanism_mapping_and_invalid_joy_stop(self):
        self.start()
        self.arm_and_move()

        def send_joy(joy, seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                self.joy_pub.publish(joy)
                self.run_for(.025, range(3))

        # Preserve the original button/axis mapping on both mechanism ports.
        joy = Joy(axes=[0., .5, -.5, 0., 0., 0., .75, -.25], buttons=[0] * 13)
        for button in [0, 2, 12]:
            joy.buttons[button] = 1
        self.clear()
        send_joy(joy, .15)
        for i, expected in enumerate([[.4, .4, .2, 0.], [-.75, -.25, .15, .15]]):
            self.assertTrue(self.packets[i])
            for actual, target in zip(self.packets[i][-1], expected):
                self.assertAlmostEqual(actual, target, places=6)

        # Invalid messages must stop both ports and must not refresh the command deadline.
        send_joy(Joy(), .1)
        self.clear()
        send_joy(Joy(), .4)
        self.all_zero([0, 1])
        for i in [0, 1]:
            self.assertEqual(self.value(i, 'watchdog_state'), 'Command timeout')

        for invalid in [float('nan'), float('inf')]:
            with self.subTest(axis_value=invalid):
                self.arm_and_move()
                joy = Joy(axes=[0.] * 8, buttons=[0] * 13)
                joy.axes[6] = invalid
                send_joy(joy, .1)
                self.clear()
                send_joy(joy, .4)
                self.all_zero([0, 1])
                self.assertEqual(self.value(1, 'watchdog_state'), 'Command timeout')

    def test_send_only_silent_devices_allow_commands_and_reconnect_safely(self):
        self.start(monitor_updates=False)
        self.run_for(.4, [2], False)
        self.all_zero([0, 1])  # Neutral is still required after opening.
        self.run_for(.15, [2], True)
        before = [self.attempts(i) for i in [0, 1]]
        self.clear()
        self.run_for(1.3, [2], False)
        self.moving([0, 1])  # Neither mechanism has received any bytes.
        self.assertEqual(before, [self.attempts(i) for i in [0, 1]])
        for i in [0, 1]:
            self.assertEqual(self.value(i, 'watchdog_state'), 'Ready')
            self.assertLess(float(self.value(i, 'feedback_age_sec')), 0)
        self.run_for(.4, [2])
        self.clear()
        self.run_for(.1, [2])
        self.all_zero([0, 1])
        self.assertTrue(all(self.value(i, 'watchdog_state') == 'Command timeout' for i in [0, 1]))
        self.assertEqual(before, [self.attempts(i) for i in [0, 1]])
        self.run_for(.15, [2], True)
        self.detach(0)
        self.run_for(.4, [2], False)
        self.assertGreater(self.attempts(0), before[0])
        self.attach(0)
        self.clear()
        self.run_for(.3, [2], False)
        self.all_zero([0])
        self.moving([1])
        self.run_for(.15, [2], True)
        self.clear()
        self.run_for(.1, [2], False)
        self.moving([0, 1])

    def test_successful_non_feedback_frames_reset_mechanism_failure_count(self):
        self.start()
        # ID 0 is registered but is not the motor-feedback frame. A valid frame
        # of this ID must reset the counter and allow operation after neutral.
        raw = bytes(81)  # ID 0 plus the 80-byte MotorCommandPacket.
        frame = encode(raw + bytes([sum(raw) & 255]))
        def run_commands(seconds, neutral):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                for i in [0, 1]:
                    os.write(self.pairs[i][0], frame)
                self.command(neutral)
                self.run_for(.025, [2])
        run_commands(.45, True)
        before = [self.attempts(i) for i in [0, 1]]
        self.clear()
        run_commands(1.2, False)
        self.moving([0, 1])
        self.assertEqual(before, [self.attempts(i) for i in [0, 1]])
        for i in [0, 1]:
            self.assertEqual(self.value(i, 'watchdog_state'), 'Ready')
            self.assertLess(int(self.value(i, 'serial_update_failure_count')), 20)

    def test_unplug_replug_new_device_on_same_path_recovers_all_three_ports(self):
        self.start()
        self.arm_and_move()
        fd_counts = [len(list(Path(f'/proc/{p.pid}/fd').iterdir())) for p in self.processes]
        for i in range(3):
            with self.subTest(port=i):
                before = [self.attempts(j) for j in range(3)]
                self.detach(i)
                others = [j for j in range(3) if j != i]
                self.run_for(.4, others, False)
                self.assertGreater(self.attempts(i), before[i])
                self.assertTrue(all(self.attempts(j) == before[j] for j in others))
                # Simulate a changed tty name behind a stable /dev/serial/by-id path.
                self.attach(i)
                self.clear()
                self.run_for(.45, range(3), False)
                self.all_zero([i])
                self.moving(others)
                self.arm_and_move()
        # Repeated close/open must not leak descriptors.
        after = [len(list(Path(f'/proc/{p.pid}/fd').iterdir())) for p in self.processes]
        self.assertEqual(after, fd_counts)

    def test_launch_restarts_only_after_sustained_fault_and_requires_neutral(self):
        self.start(supervised=True)
        self.run_for(3.5, range(3), True)
        def starts():
            return re.findall(r'process started with pid \[([0-9]+)\]', self.launch_log.read_text())
        initial = starts()
        self.assertEqual(len(initial), 2)
        # Less than one second of failure must not restart either process.
        self.run_for(.5, neutral=False)
        self.assertEqual(starts(), initial)
        self.run_for(3.5, range(3), False)
        self.clear()
        self.run_for(.2, range(3), False)
        self.all_zero()
        # Persistent loss survives repeated successful opens of a silent PTY.
        deadline = time.monotonic() + 5
        while len(starts()) < 4 and time.monotonic() < deadline:
            self.run_for(.1, neutral=False)
        self.assertEqual(len(starts()), 4, self.launch_log.read_text())
        self.assertEqual(len(set(starts())), 4)
        for executable in ['main', 'serial_motor_bridge']:
            pids = re.findall(
                rf'\[{re.escape(executable)}-\d+\]: process started with pid \[([0-9]+)\]',
                self.launch_log.read_text())
            self.assertEqual(len(pids), 2, self.launch_log.read_text())
            self.assertEqual(len(set(pids)), 2)
        self.clear()
        self.run_for(3.5, range(3), False)
        self.all_zero()
        self.arm_and_move()
        # Healthy feedback without commands is a stop, not a communication fault.
        self.run_for(2.0, range(3))
        self.assertEqual(len(starts()), 4)
        self.clear()
        self.run_for(.2, range(3))
        self.all_zero()

    def tearDown(self):
        for process in self.processes:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for i in range(3):
            self.detach(i)
        for log in self.logs:
            log.close()
        self.folder.cleanup()
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
