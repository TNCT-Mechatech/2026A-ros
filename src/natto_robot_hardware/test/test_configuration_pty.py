"""Real wheel bridge against a PTY only; never opens robot device paths."""
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import threading
import time
import tty

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
import yaml


def encode(raw):
    data = bytearray([0])
    start, code = 0, 1
    for value in raw:
        if value == 0:
            data[start] = code
            start, code = len(data), 1
            data.append(0)
        else:
            data.append(value)
            code += 1
    data[start] = code
    return b'\0' + data + b'\0'


def decode(data):
    raw = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if not code or index + code > len(data):
            raise ValueError('invalid COBS')
        raw.extend(data[index + 1:index + code])
        index += code
        if code < 255 and index < len(data):
            raw.append(0)
    return bytes(raw)


class Firmware:
    def __init__(self, drop_first_zero=False, ack=True, reject=None, legacy=False, drop_zero_forever=False):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.drop_first_zero, self.ack, self.reject, self.legacy = drop_first_zero, ack, reject, legacy
        self.drop_zero_forever = drop_zero_forever
        self.config = [None] * 4
        self.ids = [(0, 0)] * 4
        self.requests, self.commands = [], []
        self.applications = [0] * 4
        self.closed = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, fid, data):
        raw = bytes([fid]) + data
        os.write(self.master, encode(raw + bytes([sum(raw) & 255])))

    def reboot(self):
        self.config = [None] * 4
        self.ids = [(0, 0)] * 4

    def run(self):
        buffer = b''
        feedback_at = 0.
        while not self.closed:
            if select.select([self.master], [], [], .003)[0]:
                buffer += os.read(self.master, 65536)
                while b'\0' in buffer:
                    frame, buffer = buffer.split(b'\0', 1)
                    if not frame:
                        continue
                    raw = decode(frame)
                    if raw[-1] != sum(raw[:-1]) & 255:
                        continue
                    fid, payload = raw[0], raw[1:-1]
                    if fid < 4:
                        assert len(payload) == 52
                        self.requests.append((fid, payload))
                        if fid == 0 and self.drop_zero_forever:
                            continue
                        if fid == 0 and self.drop_first_zero:
                            self.drop_first_zero = False
                            continue
                        identity = struct.unpack_from('<II', payload, 44)
                        result = self.reject
                        if result is None:
                            if self.ids[fid] == identity:
                                result = 1 if self.config[fid] == payload else 4
                            elif self.ids[fid][0] == identity[0] and self.ids[fid][1] > identity[1]:
                                result = 3
                            else:
                                result = 0
                                self.config[fid] = payload
                                self.ids[fid] = identity
                                self.applications[fid] += 1
                        if self.ack:
                            self.send(21 + fid, struct.pack('<IIIIB3x', *identity, *self.ids[fid], result))
                    elif 4 <= fid < 8 or 12 <= fid < 16:
                        self.commands.append((time.monotonic(), fid, struct.unpack('<f', payload)[0]))
            if time.monotonic() - feedback_at >= .01:
                feedback_at = time.monotonic()
                for i in range(4):
                    payload = struct.pack('<ffi4xqf?3x', 0., 0., 0, 0, 0., False)
                    if not self.legacy:
                        payload += struct.pack('<II', *self.ids[i])
                    self.send(16 + i, payload)

    def close(self):
        self.closed = True
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


class Driver:
    def __init__(self, folder, initialization_timeout_ms=500, **options):
        self.board = Firmware(**options)
        self.folder = folder
        self.status = None
        self.node = rclpy.create_node('configuration_pty_driver')
        self.publisher = self.node.create_publisher(JointState, '/command_joint_states', 10)
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostic, 100)
        gain = folder / 'gain.yaml'
        gain.write_text(yaml.safe_dump({f'wheel{i}': {'kp': .1, 'ki': .2, 'kd': 0.} for i in range(4)}))
        config = folder / 'params.yaml'
        config.write_text(yaml.safe_dump({'serial_motor_bridge': {'ros__parameters': {
            'serial_path': self.board.path, 'gain_config_path': str(gain),
            'initialization_retry_ms': 100, 'initialization_timeout_ms': initialization_timeout_ms,
            'reconnect_interval_ms': 1000, 'startup_feedback_timeout_ms': 1000,
        }}}))
        self.log = (folder / 'bridge.log').open('w')
        self.process = subprocess.Popen([os.environ['WHEEL_TEST_EXECUTABLE'], '--ros-args', '--params-file', str(config)],
                                        stdout=self.log, stderr=self.log)

    def diagnostic(self, msg):
        for status in msg.status:
            if status.name == 'natto_robot_hardware/serial_motor_bridge':
                self.status = status

    def value(self, name):
        return next((v.value for v in self.status.values if v.key == name), None) if self.status else None

    def pump(self, seconds=.05, neutral=False):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            assert self.process.poll() is None, (self.folder / 'bridge.log').read_text()
            self.publisher.publish(JointState(velocity=[0. if neutral else 1.] * 4))
            rclpy.spin_once(self.node, timeout_sec=.005)

    def wait(self, predicate, timeout=4, neutral=False):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            self.pump(.015, neutral)
            if predicate():
                return
        assert predicate(), (self.status, (self.folder / 'bridge.log').read_text())

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
        self.log.close()
        self.node.destroy_node()
        self.board.close()


@pytest.fixture
def driver(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = '194'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(tmp_path / 'ros_log')
    rclpy.init()
    drivers = []
    def create(**options):
        d = Driver(tmp_path, **options)
        drivers.append(d)
        return d
    yield create
    for d in drivers:
        d.close()
    rclpy.shutdown()


@pytest.mark.parametrize('ack', [True, False])
def test_dropped_motor_zero_init_and_lost_ack_are_recovered_without_early_motion(driver, ack):
    d = driver(drop_first_zero=True, ack=ack)
    d.wait(lambda: d.value('configuration_ready') == 'true')
    requests = [payload for fid, payload in d.board.requests if fid == 0]
    assert len(requests) >= 2
    assert all(payload == requests[0] for payload in requests)
    assert d.board.applications == [1, 1, 1, 1]
    assert not any(value for _, _, value in d.board.commands)
    d.pump(.15, neutral=True)
    d.pump(.1)
    assert any(value for _, _, value in d.board.commands)


def test_stm_reset_revokes_motion_and_requires_neutral_after_reinitialization(driver):
    d = driver()
    d.wait(lambda: d.value('configuration_ready') == 'true', neutral=True)
    d.pump(.1, neutral=True)
    d.pump(.1)
    assert any(value for _, _, value in d.board.commands)
    ids = d.board.ids.copy()
    d.board.reboot()
    d.wait(lambda: d.board.applications == [2, 2, 2, 2])
    assert d.board.ids == ids
    d.wait(lambda: d.value('configuration_ready') == 'true')
    began = time.monotonic()
    d.pump(.12)
    assert all(value == 0 for t, _, value in d.board.commands if t >= began)
    d.pump(.1, neutral=True)
    d.pump(.1)
    assert any(value for t, _, value in d.board.commands if t >= began)


@pytest.mark.parametrize('result', [2, 3, 4])
def test_rejected_configuration_stops_all_wheels_and_reports_error(driver, result):
    d = driver(reject=result)
    d.wait(lambda: d.status and f'result={result}' in d.status.message)
    assert d.value('configuration_ready') == 'false'
    assert d.status.level == b'\x02' or d.status.level == 2
    assert not any(value for _, _, value in d.board.commands)


def test_legacy_feedback_never_confirms_configuration(driver):
    d = driver(ack=False, legacy=True)
    d.wait(lambda: d.status and 'confirmation timed out' in d.status.message)
    assert d.value('configuration_ready') == 'false'
    assert not any(value for _, _, value in d.board.commands)


def test_one_unconfirmed_motor_keeps_all_wheels_stopped_with_fresh_commands(driver):
    d = driver(drop_zero_forever=True, initialization_timeout_ms=2000)
    d.wait(lambda: d.publisher.get_subscription_count() == 1 and d.board.ids[3] != (0, 0), neutral=True)
    d.pump(.1, neutral=True)
    d.pump(.15, neutral=False)
    assert d.value('configuration_ready') == 'false'
    assert d.board.ids[0] == (0, 0)
    assert all(identity != (0, 0) for identity in d.board.ids[1:])
    assert not any(value for _, _, value in d.board.commands)
