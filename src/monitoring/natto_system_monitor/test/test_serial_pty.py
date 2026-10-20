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
from rclpy.qos import qos_profile_sensor_data
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


def packet(frame_id, payload, corrupt=False):
    raw = bytes([frame_id]) + payload
    return encode(raw + bytes([(sum(raw) + int(corrupt)) & 255]))


def feedback(frame_id=None, corrupt=False, rps=1., count=0):
    # ARM/native ABI: raw count at 16, applied IDs at 32/36, total 40 bytes.
    payload = struct.pack('<ffi4xqf?3xII', rps, 0., 0, count, 0., False, 0, 0)
    if frame_id is not None:
        return packet(frame_id, payload, corrupt)
    return b''.join(packet(16 + i, payload, corrupt) for i in range(4)) + packet(20, bytes([1, 0, 1, 0]))


def with_identities(data, identities):
    """Simulate firmware's committed IDs, preserving deliberate checksum faults."""
    result = bytearray()
    for frame in data.split(b'\0'):
        if not frame:
            continue
        raw = bytearray(decode(frame))
        corrupt = sum(raw[:-1]) & 255 != raw[-1]
        if 16 <= raw[0] <= 19 and len(raw) == 42:
            struct.pack_into('<II', raw, 33, *identities[raw[0] - 16])
        result.extend(packet(raw[0], bytes(raw[1:-1]), corrupt))
    return bytes(result)


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
        self.frames = [[] for _ in range(3)]
        self.identities = [[(0, 0)] * 4 for _ in range(3)]
        self.command_batches = [[0.] * 4 for _ in range(3)]
        self.joints, self.switches = [], []
        self.node.create_subscription(JointState, '/joint_states', self.joints.append, qos_profile_sensor_data)
        from std_msgs.msg import UInt8MultiArray
        self.node.create_subscription(UInt8MultiArray, '/switch_feedback', self.switches.append, 100)
        self.processes, self.logs = [], []
        self.folder = tempfile.TemporaryDirectory(prefix='natto_serial_watchdog_')
        self.paths = [Path(self.folder.name) / name for name in self.names]

    def attach(self, i):
        master, slave = pty.openpty()
        tty.setraw(slave)  # Prevent feedback echoes before the node opens the port.
        self.pairs[i] = (master, slave)
        self.paths[i].symlink_to(os.ttyname(slave))
        self.buffers[i].clear()
        self.identities[i] = [(0, 0)] * 4

    def detach(self, i):
        self.paths[i].unlink(missing_ok=True)
        for fd in self.pairs[i]:
            if fd >= 0:
                os.close(fd)
        self.pairs[i] = (-1, -1)
        self.buffers[i].clear()

    def start(self, attached=True, supervised=False, monitor_updates=True, closed_loop=False):
        if attached:
            for i in range(3):
                self.attach(i)
        gains = Path(self.folder.name) / 'gain.yaml'
        gains.write_text(yaml.safe_dump({f'wheel{i}': {'kp': .1 + i, 'ki': .2, 'kd': .3} for i in range(4)}))
        wheel_config = yaml.safe_load((Path(get_package_share_directory('natto_robot_hardware')) /
                                       'config/serial_motor_bridge.yaml').read_text())['serial_motor_bridge']['ros__parameters']
        config = Path(self.folder.name) / 'ports.yaml'
        config.write_text(yaml.safe_dump({name: {'ros__parameters': {
            'serial_path': str(self.paths[i]), 'diagnostic_updater.period': .01,
            'timeout_ms': 300, 'reconnect_interval_ms': 100,
            **({'pid': closed_loop, 'command_mode': 'closed_loop_velocity' if closed_loop else 'open_loop_normalized',
                'gain_config_path': str(gains), 'wheel_polarities': wheel_config['wheel_polarities'],
                'encoder_polarities': wheel_config['encoder_polarities'], 'encoder_ppr': [100, 100, 100, 100],
                'gear_rate': [2., 2., 2., 2.], 'open_loop_slew_rate_per_sec': 100.,
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
                    frame_id = raw[0]
                    sizes = {**{j: 52 for j in range(4)}, **{j: 4 for j in range(4, 8)},
                             **{j: 8 for j in range(8, 12)}, **{j: 4 for j in range(12, 16)}}
                    self.assertIn(frame_id, sizes)
                    self.assertEqual(len(raw), sizes[frame_id] + 2)
                    self.assertEqual(sum(raw[:-1]) & 255, raw[-1])
                    self.frames[i].append((frame_id, bytes(raw[1:-1])))
                    if frame_id < 4:
                        self.identities[i][frame_id] = struct.unpack_from('<II', raw, 45)
                    if 4 <= frame_id <= 7 or 12 <= frame_id <= 15:
                        motor = frame_id % 4
                        self.command_batches[i][motor] = struct.unpack_from('<f', raw, 1)[0]
                        if motor == 3:
                            self.packets[i].append(self.command_batches[i].copy())


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
                    os.write(self.pairs[i][0], with_identities(feedback() if packet is None else packet, self.identities[i]))
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
        self.run_for(.45, range(3), False, feedback(frame_id=99))
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
        # Even with receive watchdog disabled, initial configuration must be confirmed.
        self.run_for(.2, range(3), True)
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
        self.run_for(.2, [0, 2], False)
        self.run_for(.15, [2], True)
        self.clear()
        self.run_for(.1, [2], False)
        self.moving([0, 1])

    def test_successful_non_feedback_frames_reset_mechanism_failure_count(self):
        self.start()
        self.run_for(.3, range(3), True)
        # A registered switch frame counts as a successful update, and the
        # terminal -1 in drain must not overwrite that success.
        frame = packet(20, bytes([0, 1, 0, 1]))
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

    def test_closed_loop_abi_initialization_units_raw_position_and_switches(self):
        self.start(closed_loop=True)
        self.arm_and_move()
        for records in self.frames:
            self.assertEqual([fid for fid, _ in records[:8]], [0, 1, 2, 3, 4, 5, 6, 7] if records is self.frames[2] else [0, 1, 2, 3, 12, 13, 14, 15])
        for i, (_, payload) in enumerate(self.frames[2][:4]):
            self.assertEqual(len(payload), 52)
            # All encoders must agree with motor output inside the STM PID.
            self.assertEqual(payload[28], 1)
            self.assertEqual(payload[30], 1)
            self.assertAlmostEqual(struct.unpack_from('<f', payload, 4)[0], .1 + i, places=5)
            # Even a nonzero saved YAML kd must keep hardware derivative control disabled.
            self.assertEqual(struct.unpack_from('<f', payload, 12)[0], 0.)
            self.assertEqual(struct.unpack_from('<H', payload, 36)[0], 100)
        for (_, payload) in self.frames[2][4:8]:
            self.assertEqual(struct.unpack('<f', payload)[0], 0.)
        for actual, polarity in zip(self.packets[2][-1], [-1, 1, -1, 1]):
            self.assertAlmostEqual(actual, polarity / (2 * 3.141592653589793), places=6)
        for direction in [1, -1]:
            before = len(self.joints)
            # Simulate tracking of ROS wheel motion in STM motor coordinates.
            reply = b''.join(feedback(frame_id=16 + i, rps=direction * polarity * 2.,
                                     count=direction * polarity * 800)
                             for i, polarity in enumerate([-1, 1, -1, 1]))
            self.run_for(.12, [2], False, reply)
            self.wait_for(lambda: len(self.joints) > before and
                          self.joints[-1].position[0] * direction > 0)
            for position, velocity in zip(self.joints[-1].position, self.joints[-1].velocity):
                self.assertAlmostEqual(position, direction * 2 * 3.141592653589793, places=6)
                self.assertAlmostEqual(velocity, direction * 4 * 3.141592653589793, places=6)
        self.assertTrue(self.switches)
        self.assertEqual(list(self.switches[-1].data), [True, False, True, False])
        self.assertEqual(self.value(2, 'unregistered_frames'), '0')

    def test_open_loop_duty_mode_and_reconnect_initialization_order(self):
        self.start()
        self.arm_and_move()
        expected = [-1 / (8 * 3.141592653589793), 1 / (8 * 3.141592653589793)] * 2
        for actual, target in zip(self.packets[2][-1], expected):
            self.assertAlmostEqual(actual, target, places=6)
        for records in self.frames:
            self.assertEqual([fid for fid, _ in records[:8]], [0, 1, 2, 3, 12, 13, 14, 15])
            self.assertTrue(all(payload[30] == 0 for _, payload in records[:4]))
            self.assertFalse(any(4 <= fid <= 11 for fid, _ in records))
        self.detach(2)
        self.run_for(.35, [0, 1], False)
        self.attach(2)
        self.frames[2].clear()
        self.run_for(.45, range(3), False)
        self.assertEqual([fid for fid, _ in self.frames[2][:8]], [0, 1, 2, 3, 12, 13, 14, 15])
        self.assertTrue(all(payload[28] == 1 for _, payload in self.frames[2][:4]))
        self.assertTrue(all(struct.unpack('<f', payload)[0] == 0 for _, payload in self.frames[2][4:8]))

    def test_five_frame_bursts_are_drained_and_partial_feedback_times_out(self):
        self.start()
        self.arm_and_move()
        # 100Hz feedback, exactly the STM's five consecutive frames per cycle.
        for count in range(60):
            os.write(self.pairs[2][0], with_identities(feedback(count=count * 800), self.identities[2]))
            self.run_for(.01, [0, 1], False)
        self.wait_for(lambda: self.joints and self.joints[-1].position[0] <= -59 * 2 * 3.141592653589793, timeout=.15)
        self.assertEqual(self.value(2, 'drain_budget_hits'), '0')
        before = self.attempts(2)
        self.run_for(.6, [2], False, feedback(frame_id=16))
        self.assertGreater(self.attempts(2), before)

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


class ParamCheckerPTY(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('param_checker_pty_probe')
        self.joy_pub = self.node.create_publisher(Joy, '/controller/joy', 10)
        self.folder = tempfile.TemporaryDirectory(prefix='natto_param_pty_')
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.process = None
        self.log = None
        self.identities = [(0, 0)] * 4

    def check_mode(self, pid):
        import csv
        import math
        folder = Path(self.folder.name)
        config = folder / 'config.yaml'
        gains, samples, diagnostic = [folder / name for name in ['gain.yaml', 'samples.csv', 'diagnostic.csv']]
        config.write_text(yaml.safe_dump({'param_check': {'ros__parameters': {
            'serial_path': os.ttyname(self.slave), 'pid': pid, 'measurement_seconds': 1.2,
            'gain_output_path': str(gains), 'vrft_csv_path': str(samples),
            'serial_diagnostic_csv_path': str(diagnostic),
            'initial_kp': .12, 'initial_ki': .34, 'initial_kd': .56,
        }}}))
        binary = Path(get_package_prefix('param_checker')) / 'lib/param_checker/param_checker'
        self.log = (folder / 'node.log').open('w')
        self.process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config)], stdout=self.log, stderr=self.log)
        buffer, frames = bytearray(), []
        deadline, next_send, tick = time.monotonic() + 6, 0., 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.005)
            if time.monotonic() >= next_send and self.process.poll() is None:
                self.joy_pub.publish(Joy(axes=[.5, 0., 0.], buttons=[]))
                # Feedback arrives as five frames, including ID20.
                os.write(self.master, with_identities(feedback(rps=math.sin(tick / 5), count=tick * 100), self.identities))
                tick += 1
                next_send = time.monotonic() + .01
            while select.select([self.master], [], [], 0)[0]:
                buffer.extend(os.read(self.master, 65536))
            while 0 in buffer:
                end = buffer.index(0)
                frame = bytes(buffer[:end])
                del buffer[:end + 1]
                if frame:
                    raw = decode(frame)
                    self.assertEqual(sum(raw[:-1]) & 255, raw[-1])
                    frames.append((raw[0], bytes(raw[1:-1])))
                    if raw[0] < 4:
                        self.identities[raw[0]] = struct.unpack_from('<II', raw, 45)
            if self.process.poll() is not None:
                break
        self.assertEqual(self.process.poll(), 0, (folder / 'node.log').read_text())
        base = 4 if pid else 12
        self.assertEqual([fid for fid, _ in frames[:8]], [0, 1, 2, 3] + list(range(base, base + 4)))
        for _, payload in frames[:4]:
            self.assertEqual(len(payload), 52)
            self.assertEqual(payload[30], int(pid))
            for offset, expected in [(4, .12), (8, .34), (12, .56)]:
                self.assertAlmostEqual(struct.unpack_from('<f', payload, offset)[0], expected, places=6)
        # Gains are initialization-only; Joy must send a single control family.
        self.assertEqual(sum(fid < 4 for fid, _ in frames), 4)
        commands = [(fid, struct.unpack('<f', payload)[0]) for fid, payload in frames if fid >= 4]
        self.assertTrue(all(base <= fid < base + 4 for fid, _ in commands))
        self.assertTrue(any(abs(value) > 0 for _, value in commands))
        self.assertTrue(all(value == 0 for _, value in commands[-4:]))
        saved_gains = yaml.safe_load(gains.read_text())
        self.assertEqual(set(saved_gains), {f'wheel{i}' for i in range(4)})
        output = (folder / 'node.log').read_text()
        self.assertIn('PIDゲインの決定値 (VRFT)', output)
        self.assertNotIn('msg.kp =', output)
        self.assertIn(f'PID YAML: {gains} （保存済み）', output)
        # The summary must label each wheel and match the actual saved result.
        summary_rows = re.findall(r'^wheel([0-3])\s*\|\s*([^|]+)\|\s*([^|]+)\|\s*([^|]+)\|([^\n]+)', output, re.MULTILINE)
        self.assertEqual([row[0] for row in summary_rows], ['0', '1', '2', '3'])
        for wheel, kp, ki, kd, status in summary_rows:
            for key, displayed in zip(['kp', 'ki', 'kd'], [kp, ki, kd]):
                expected = saved_gains[f'wheel{wheel}'][key]
                self.assertAlmostEqual(float(displayed), expected, delta=max(1e-6, abs(expected) * 1e-6))
            self.assertTrue(status.strip())
        with samples.open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual({row['wheel'] for row in rows}, {'0', '1', '2', '3'})
        with diagnostic.open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertTrue(any(row['feedback_updated'] == '1' for row in rows))
        self.assertFalse(any(row['serial_update_result'] in ['-2', '-3'] for row in rows))

    def test_velocity_initialization_feedback_vrft_and_csv(self):
        self.check_mode(True)

    def test_duty_initialization_feedback_vrft_and_csv(self):
        self.check_mode(False)

    def check_position(self, operation='tune', motion=True, target_feedback=True,
                       nested=False, stop_joy=False, missing_gains=False,
                       invalid_units=False, via_launch=False, separate_paths=False):
        import csv
        folder = Path(self.folder.name)
        gains, samples = folder / 'gain.yaml', folder / 'position.csv'
        original = {f'wheel{i}': {'kp': .12 + i, 'ki': .34, 'kd': .56}
                    for i in range(4)}
        if nested:
            original = {f'wheel{i}': {'velocity': {
                'controller': 'PI', 'kp': .12 + i, 'ki': .34, 'kd': 0.,
                'input_unit': 'rps', 'output_unit': 'duty',
                'max_rps': .02, 'response_time_seconds': .3}}
                for i in range(4)}
            original['schema_version'] = 2
        if invalid_units:
            original['wheel2']['velocity']['input_unit'] = 'encoder_count'
        if operation == 'verify':
            original['wheel2']['position'] = {
                'controller': 'P', 'kp': 2.5e-6, 'ki': 0., 'kd': 0.,
                'input_unit': 'encoder_count', 'output_unit': 'rps', 'max_rps': .03}
        if not missing_gains:
            gains.write_text(yaml.safe_dump(original))
        gain_input = ''
        if separate_paths:
            gain_input = str(folder / 'input.yaml')
            Path(gain_input).write_text(yaml.safe_dump(original))
            # Output already has other wheel calibration and a different inner PI.
            destination = yaml.safe_load(yaml.safe_dump(original))
            destination['wheel0']['kp'] = 9.0
            destination['wheel2']['kp'] = 9.9
            gains.write_text(yaml.safe_dump(destination))
            original['wheel0']['kp'] = 9.0
        original_text = gains.read_text() if gains.exists() else None
        config = folder / 'position.yaml'
        config.write_text(yaml.safe_dump({'param_check': {'ros__parameters': {
            'serial_path': os.ttyname(self.slave), 'position_operation': operation,
            'target_motor_index': 2, 'measurement_seconds': 1.2,
            'gain_output_path': str(gains), 'vrft_csv_path': str(samples),
            'gain_input_path': gain_input,
            'position_range_counts': 1000, 'minimum_samples': 20,
        }}}))
        binary = Path(get_package_prefix('param_checker')) / 'lib/param_checker/position_checker'
        self.log = (folder / 'node.log').open('w')
        command = [str(binary), '--ros-args', '--params-file', str(config)]
        if via_launch:
            command = ['ros2', 'launch', 'param_checker', 'param_checker_position.launch.xml',
                       f'serial_path:={os.ttyname(self.slave)}', f'serial_config:={config}',
                       f'gain_output_path:={gains}', 'target_motor_index:=2',
                       f'position_operation:={operation}', 'measurement_seconds:=1.2',
                       'position_range_counts:=1000']
        self.process = subprocess.Popen(command,
                                        stdout=self.log, stderr=self.log)
        buffer, frames = bytearray(), []
        began, previous, next_send = time.monotonic(), time.monotonic(), 0.
        origin, relative, input_rps = 9007199254740993, 0., 0.
        deadline = began + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.003)
            while select.select([self.master], [], [], 0)[0]:
                buffer.extend(os.read(self.master, 65536))
            while 0 in buffer:
                end = buffer.index(0)
                frame = bytes(buffer[:end])
                del buffer[:end + 1]
                if frame:
                    raw = decode(frame)
                    self.assertEqual(sum(raw[:-1]) & 255, raw[-1])
                    fid, payload = raw[0], bytes(raw[1:-1])
                    frames.append((fid, payload, time.monotonic() - began))
                    if fid < 4:
                        self.identities[fid] = struct.unpack_from('<II', payload, 44)
                    if fid == 6:
                        input_rps = struct.unpack('<f', payload)[0]
            instant = time.monotonic()
            if instant >= next_send and self.process.poll() is None:
                if not stop_joy or instant - began < .55:
                    value = .5 if operation == 'verify' or int((instant - began) / .2) % 2 else -.7
                    self.joy_pub.publish(Joy(axes=[0., value, 0.], buttons=[]))
                if motion:
                    relative += input_rps * 200000. * (instant - previous)
                previous = instant
                if target_feedback:
                    os.write(self.master, with_identities(feedback(rps=input_rps, count=origin + round(relative)), self.identities))
                else:
                    # Feedback from other motors must not arm motor2 or extend its deadline.
                    os.write(self.master, with_identities(feedback(frame_id=16, count=origin), self.identities))
                next_send = instant + .008
            if self.process.poll() is not None:
                break
        output = (folder / 'node.log').read_text()
        self.assertIsNotNone(self.process.poll(), output)
        if missing_gains or not target_feedback or invalid_units:
            self.assertNotEqual(self.process.returncode, 0, output)
            self.assertEqual(gains.read_text() if gains.exists() else None, original_text)
            self.assertFalse(any(fid == 6 and abs(struct.unpack('<f', p)[0]) > 0
                                 for fid, p, _ in frames))
            if missing_gains or invalid_units:
                self.assertEqual(frames, [])
            return
        self.assertEqual(self.process.returncode, 0, output)
        self.assertEqual([fid for fid, _, _ in frames[:8]],
                         [0, 1, 2, 3, 12, 13, 6 if operation == 'tune' else 14, 15])
        for i, (_, payload, _) in enumerate(frames[:4]):
            self.assertEqual(payload[30], int(i == 2))
            self.assertEqual(struct.unpack_from('<f', payload, 12)[0], 0.)
        # No non-target motor may receive a nonzero command.
        for fid, payload, _ in frames[4:]:
            if fid not in (6, 10, 14):
                self.assertIn(fid, (12, 13, 15))
                self.assertEqual(struct.unpack('<f', payload)[0], 0.)
        final = frames[-4:]
        self.assertEqual([fid for fid, _, _ in final], [12, 13, 6 if operation == 'tune' else 14, 15])
        self.assertTrue(all(struct.unpack('<f', p)[0] == 0 for _, p, _ in final))
        saved = yaml.safe_load(gains.read_text())
        if operation == 'verify':
            positions = [struct.unpack('<q', p)[0] for fid, p, _ in frames if fid == 10]
            self.assertTrue(positions, output)
            self.assertTrue(all(value == origin + 500 for value in positions))
            self.assertEqual(gains.read_text(), original_text)
            if stop_joy:
                self.assertFalse(any(fid == 10 and elapsed > .95 for fid, _, elapsed in frames))
                self.assertTrue(any(fid == 14 and elapsed > .95 for fid, _, elapsed in frames))
        elif motion:
            position = saved['wheel2'].pop('position')
            self.assertEqual(saved, original)
            self.assertEqual(position['ki'], 0.)
            self.assertEqual(position['kd'], 0.)
            self.assertAlmostEqual(position['kp'], 2.5e-6, delta=1e-6)
            self.assertGreaterEqual(position['fit_r_squared'], .7)
            self.assertIn('位置PIDゲインの決定値', output)
            self.assertIn('保存済み', output)
            with samples.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertGreater(len(rows), 20)
            self.assertGreater(int(rows[0]['encoder_raw_count']), 2**53)
            limit = .02 if nested else .03
            self.assertTrue(any(abs(struct.unpack('<f', p)[0]) > 0 for fid, p, _ in frames if fid == 6))
            self.assertTrue(all(abs(struct.unpack('<f', p)[0]) <= limit + 1e-6
                                for fid, p, _ in frames if fid == 6))
        else:
            self.assertEqual(gains.read_text(), original_text)
            self.assertIn('保存しませんでした', output)

    def test_position_tuning_preserves_velocity_and_other_wheels(self):
        self.check_position()

    def test_position_tuning_accepts_reference_nested_velocity_gains(self):
        self.check_position(nested=True)

    def test_position_separate_output_keeps_other_wheels_and_copies_used_inner_pi(self):
        self.check_position(separate_paths=True)

    def test_position_tuning_no_motion_does_not_overwrite_yaml(self):
        self.check_position(motion=False)

    def test_position_verification_sends_int64_counts_and_stops_on_joy_timeout(self):
        self.check_position(operation='verify', stop_joy=True)

    def test_position_launch_runs_verification(self):
        self.check_position(operation='verify', via_launch=True)

    def test_position_rejects_incompatible_gain_units_before_serial_write(self):
        self.check_position(nested=True, invalid_units=True)

    def test_position_requires_target_motor_feedback(self):
        self.check_position(target_feedback=False)

    def test_position_missing_velocity_gains_fails_before_serial_write(self):
        self.check_position(missing_gains=True)

    def tearDown(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
        if self.log:
            self.log.close()
        os.close(self.master)
        os.close(self.slave)
        self.folder.cleanup()
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
