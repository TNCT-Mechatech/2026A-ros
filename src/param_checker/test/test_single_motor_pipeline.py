"""Exercise velocity PI -> position P -> verification on an emulated injection STM."""
import math
import importlib.util
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import threading
import time

import pytest
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import yaml


def encode(data):
    result = bytearray([0])
    index, code = 0, 1
    for value in data:
        if value == 0:
            result[index] = code
            index, code = len(result), 1
            result.append(0)
        else:
            result.append(value)
            code += 1
            if code == 255:
                result[index] = code
                index, code = len(result), 1
                result.append(0)
    result[index] = code
    return b'\0' + result + b'\0'


def decode(data):
    result = bytearray()
    cursor = 0
    while cursor < len(data):
        code = data[cursor]
        if not code or cursor + code > len(data):
            raise ValueError('bad COBS')
        result.extend(data[cursor + 1:cursor + code])
        cursor += code
        if code != 255 and cursor < len(data):
            result.append(0)
    return bytes(result)


class MockSTM:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.frames = []
        self.closed = False
        self.initialized = False
        self.identities = [(0, 0)] * 4
        self.contact = False
        self.motor_feedback = True
        self.switch_feedback = True
        self.base_count = 2**40 + 321
        self.delta_count = 0.0
        self.speed = 0.0
        self.velocity_gain = 1.0
        self.command = (14, struct.pack('<f', 0.0))
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    @property
    def raw(self):
        return self.base_count + round(self.delta_count)

    def send(self, frame, payload):
        packet = bytes([frame]) + payload
        os.write(self.master, encode(packet + bytes([sum(packet) % 256])))

    def run(self):
        buffer = b''
        last_feedback = time.monotonic()
        while not self.closed:
            readable, _, _ = select.select([self.master], [], [], 0.003)
            if readable:
                buffer += os.read(self.master, 4096)
                while b'\0' in buffer:
                    frame, buffer = buffer.split(b'\0', 1)
                    if not frame:
                        continue
                    try:
                        packet = decode(frame)
                    except ValueError:
                        continue
                    if len(packet) >= 2 and packet[-1] == sum(packet[:-1]) % 256:
                        fid, payload = packet[0], packet[1:-1]
                        self.frames.append((fid, payload))
                        if fid < 4 and len(payload) == 52:
                            identity = struct.unpack_from('<II', payload, 44)
                            result = int(self.identities[fid] == identity)
                            self.identities[fid] = identity
                            self.send(21 + fid, struct.pack('<IIIIB3x', *identity, *identity, result))
                        if fid in (6, 10, 14):
                            self.command = (fid, payload)
                        self.initialized |= fid == 3
            instant = time.monotonic()
            if self.initialized and instant - last_feedback >= 0.01:
                dt = instant - last_feedback
                last_feedback = instant
                fid, payload = self.command
                if fid == 14:
                    duty = struct.unpack('<f', payload)[0]
                    self.speed += min(dt / 0.05, 1.0) * (0.05 * duty - self.speed)
                else:
                    if fid == 6:
                        desired = struct.unpack('<f', payload)[0]
                    else:
                        desired = max(-0.03, min(0.03, 0.0000015 *
                                                (struct.unpack('<q', payload)[0] - self.raw)))
                    self.speed += min(dt / 0.03, 1.0) * (self.velocity_gain * desired - self.speed)
                    duty = max(-1.0, min(1.0, desired / 0.05))
                self.delta_count += self.speed * (2048 * 4 * (245 / 6)) * dt
                for motor in range(4):
                    if motor == 2 and not self.motor_feedback:
                        continue
                    self.send(16 + motor, struct.pack(
                        '<ffi4xqf?3xII', self.speed if motor == 2 else 0.0, 0.0, 0,
                        self.raw if motor == 2 else 0, abs(duty) if motor == 2 else 0.0,
                        duty < 0 if motor == 2 else False, *self.identities[motor]))
                if self.switch_feedback:
                    self.send(20, bytes([0, 0, self.contact, 0]))

    def close(self):
        self.closed = True
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


class Driver:
    def __init__(self, tmp_path):
        self.tmp = tmp_path
        self.board = MockSTM()
        self.gain = tmp_path / 'inject_pid_gain.yaml'
        self.gain.write_text(yaml.safe_dump({
            'metadata': 'keep',
            'wheel0': {'kp': 0.7, 'ki': 0.8, 'position': {'kp': 0.01}},
            'wheel2': {'kp': 0.12, 'ki': 0.3, 'position': {'kp': 0.01}},
        }))
        self.config = tmp_path / 'config.yaml'
        settings = yaml.safe_load((Path(__file__).parents[1] /
                                   'config' / 'inject_rack.yaml').read_text())
        self.settings = settings['param_check']['ros__parameters']
        # Match the mock plant independently of local hardware calibration edits.
        self.settings.update({'serial_path': self.board.path, 'gain_output_path': str(self.gain),
                              'measurement_max_duty': 0.1,
                              'encoder_ppr': [2048, 2048, 2048, 2048],
                              'encoder_polarities': [1, 1, 1, 1],
                              'gear_rate': [71.0, 245 / 6, 245 / 6, 245 / 6],
                              'position_measurement_max_rps': 0.03,
                              'position_range_counts': 1000,
                              'position_measurement_range_counts': 10000,
                              'position_filter_window_seconds': 0.05,
                              'minimum_samples': 20,
                              'limit_switch_index': 2, 'limit_switch_active_state': True,
                              'feedback_timeout_ms': 150, 'timeout_ms': 150})
        self.process = None
        self.log = None
        self.logs = []
        self.node = rclpy.create_node('single_motor_test_driver')
        self.publisher = self.node.create_publisher(Joy, 'controller/joy', 10)
        self.message = Joy()
        self.message.axes = [0.0] * 8
        self.message.buttons = [0] * 13

    def start(self, operation, seconds=2.0):
        self.settings.update({'measurement_seconds': seconds, 'position_operation': operation,
                              'inspect_switches': operation == 'inspect'})
        self.config.write_text(yaml.safe_dump({'param_check': {'ros__parameters': self.settings}}))
        executable = os.environ['VELOCITY_TEST_EXECUTABLE' if operation in ('velocity', 'inspect')
                                else 'POSITION_TEST_EXECUTABLE']
        path = self.tmp / f'{operation}_{len(self.logs)}.log'
        self.logs.append(path)
        self.log = path.open('w')
        self.board.frames.clear()
        self.process = subprocess.Popen([executable, '--ros-args', '--params-file', str(self.config)],
                                        stdout=self.log, stderr=self.log)
        self.wait(lambda: any(fid == 3 for fid, _ in self.board.frames) and
                  self.publisher.get_subscription_count() >= 1)
        self.pump(0.12)

    def text(self):
        return '\n'.join(path.read_text() for path in self.logs)

    def pump(self, seconds, publish=True, sine=False):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if sine:
                self.message.axes[1] = 0.8 * math.sin(time.monotonic() * 10)
            if publish:
                self.publisher.publish(self.message)
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait(self, condition, timeout=8):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if condition():
                return
            self.pump(0.03)
        assert condition(), self.text()

    def finish(self, sine=True):
        self.wait_with_sine(sine)
        assert self.process.returncode == 0, self.text()
        self.log.close()
        self.message.axes[1] = 0.0
        # Only the selected motor ever receives a nonzero command.
        for fid, payload in self.board.frames:
            if fid in (12, 13, 15):
                assert struct.unpack('<f', payload)[0] == 0
            assert fid not in (4, 5, 7, 8, 9, 11)
        assert self.board.command in ((14, struct.pack('<f', 0.0)), (6, struct.pack('<f', 0.0)))

    def wait_with_sine(self, sine):
        until = time.monotonic() + 8
        while time.monotonic() < until and self.process.poll() is None:
            self.pump(0.03, sine=sine)
        assert self.process.poll() is not None, self.text()

    def close(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
        if self.log and not self.log.closed:
            self.log.close()
        self.node.destroy_node()
        self.board.close()


@pytest.fixture
def driver(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = '181'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(tmp_path / 'ros_log')
    rclpy.init()
    item = Driver(tmp_path)
    try:
        yield item
    finally:
        item.close()
        rclpy.shutdown()


def test_velocity_position_verify_pipeline(driver):
    d = driver
    d.start('velocity')
    d.pump(0.4, sine=True)
    assert any(fid == 14 and struct.unpack('<f', payload)[0] != 0 for fid, payload in d.board.frames)
    assert all(abs(struct.unpack('<f', payload)[0]) <= 0.101
               for fid, payload in d.board.frames if fid == 14)
    # Joy loss and a held input cannot resume without neutral.
    d.pump(0.2, publish=False)
    assert d.board.command == (14, struct.pack('<f', 0.0))
    d.message.axes[1] = 0.5
    d.pump(0.06)
    assert d.board.command == (14, struct.pack('<f', 0.0))
    d.message.axes[1] = 0.0
    d.pump(0.05)
    d.finish()
    root = yaml.safe_load(d.gain.read_text())
    assert root['metadata'] == 'keep' and root['wheel0']['kp'] == 0.7
    assert root['wheel0']['position']['kp'] == 0.01
    assert 'position' not in root['wheel2']
    velocity = root['wheel2']['velocity']
    assert velocity['kp'] > 0 and velocity['ki'] >= 0 and velocity['kd'] == 0
    assert velocity['fit_r_squared'] >= 0.7

    d.start('tune')
    d.finish()
    root = yaml.safe_load(d.gain.read_text())
    assert root['wheel2']['velocity'] == velocity
    assert root['wheel2']['position']['kp'] > 0
    assert root['wheel2']['position']['ki'] == root['wheel2']['position']['kd'] == 0
    assert root['wheel2']['position']['encoder_polarity'] == 1
    assert root['wheel2']['position']['identification_method'] == 'velocity_response_model_v1'
    assert root['wheel2']['position']['training_fit_r_squared'] >= 0.7
    assert root['wheel2']['position']['validation_fit_r_squared'] >= 0.7
    assert root['wheel2']['position']['velocity_response_time_seconds'] < 0.1
    before_verify = d.gain.read_text()
    d.start('verify', seconds=0.6)
    d.finish()
    assert any(fid == 10 for fid, _ in d.board.frames)
    assert d.gain.read_text() == before_verify


def test_position_speed_and_separate_travel_limit_preserve_velocity_gains(driver):
    d = driver
    before = d.gain.read_text()
    d.settings.update({'position_measurement_max_rps': 0.006,
                       'position_measurement_range_counts': 300,
                       'measurement_range_counts': 10000})
    d.start('tune', seconds=3.0)
    init = next(payload for fid, payload in reversed(d.board.frames) if fid == 2)
    assert struct.unpack_from('<f', init, 32)[0] == pytest.approx(0.006)
    d.message.axes[1] = 1.0
    d.pump(0.05)
    speeds = [struct.unpack('<f', payload)[0] for fid, payload in d.board.frames if fid == 6]
    assert max(speeds) == pytest.approx(0.006)
    assert all(abs(speed) <= 0.006001 for speed in speeds)
    d.pump(0.2, publish=False)
    assert d.board.command == (6, struct.pack('<f', 0.0))
    d.pump(0.05)  # Held Joy cannot resume after timeout.
    assert d.board.command == (6, struct.pack('<f', 0.0))
    d.message.axes[1] = 0.0
    d.pump(0.04)
    d.board.base_count += 301  # Below shared limit, above the position-only limit.
    d.wait(lambda: d.process.poll() is not None)
    assert d.process.returncode != 0, d.text()
    assert 'measurement travel limit reached' in d.text()
    assert d.gain.read_text() == before
    assert d.board.command == (6, struct.pack('<f', 0.0))


@pytest.mark.parametrize('operation', ['tune', 'verify', 'velocity', 'inspect'])
def test_rack_launch_position_arguments_only_affect_position_operations(monkeypatch, operation):
    from launch import LaunchContext
    path = Path(__file__).parents[1] / 'launch' / 'param_checker_rack.launch.py'
    spec = importlib.util.spec_from_file_location('rack_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'operation': operation, 'serial_path': '/tmp/mock-stm',
        'gain_output_path': '/tmp/mock-gains.yaml', 'target_motor_index': '2',
        'measurement_seconds': '15.0', 'serial_config': '/tmp/mock-config.yaml',
        'joy_topic': '/inject/joy', 'position_max_rps': '0.012',
        'position_range_counts': '64', 'position_travel_limit_counts': '128',
        'position_response_time_seconds': '3.0',
        'position_filter_window_seconds': '0.15',
    })
    result = module.make_node(context)[0]
    parameters = result['parameters'][1]
    if operation in ('tune', 'verify'):
        assert parameters['position_measurement_max_rps'] == 0.012
        assert parameters['position_range_counts'] == 64
        assert parameters['position_measurement_range_counts'] == 128
        assert parameters['position_response_time_seconds'] == 3.0
        assert parameters['position_filter_window_seconds'] == 0.15
    else:
        assert not any(key.startswith('position_') for key in parameters)


@pytest.mark.parametrize('fault', ['motor', 'switch_feedback', 'contact', 'travel'])
@pytest.mark.parametrize('operation', ['velocity', 'tune'])
def test_limits_stop_without_saving(driver, operation, fault):
    d = driver
    # Supply a valid inner PI so position tuning can start independently.
    root = yaml.safe_load(d.gain.read_text())
    root['wheel2'] = {'kp': 0.12, 'ki': 0.3}
    d.gain.write_text(yaml.safe_dump(root))
    before = d.gain.read_text()
    d.start(operation)
    d.message.axes[1] = 0.5
    d.pump(0.05)
    if fault == 'motor':
        d.board.motor_feedback = False  # Other motors/switch continue.
    elif fault == 'switch_feedback':
        d.board.switch_feedback = False
    elif fault == 'contact':
        d.board.contact = True
    else:
        d.board.base_count += 10001
    d.wait(lambda: d.process.poll() is not None)
    assert d.process.returncode != 0, d.text()
    assert d.gain.read_text() == before
    assert d.board.command in ((14, struct.pack('<f', 0.0)), (6, struct.pack('<f', 0.0)))


def test_still_motor_measurement_does_not_overwrite_gains(driver):
    before = driver.gain.read_text()
    driver.start('velocity', seconds=0.4)
    driver.finish(sine=False)
    assert driver.gain.read_text() == before
    assert 'not saved' in driver.text()


def test_changed_velocity_response_does_not_overwrite_position_gains(driver):
    before = driver.gain.read_text()
    driver.start('tune', seconds=3.0)
    driver.pump(1.8, sine=True)
    # Training observes the original plant; held-out data observes a loss of gain.
    driver.board.velocity_gain = 0.2
    driver.finish()
    assert driver.gain.read_text() == before
    assert '保存しませんでした' in driver.text()
    assert driver.board.command == (6, struct.pack('<f', 0.0))


@pytest.mark.parametrize('operation', ['velocity', 'tune'])
def test_start_waits_for_neutral_before_countdown(driver, operation):
    d = driver
    before = d.gain.read_text()
    d.message.axes[1] = 0.5
    d.start(operation, seconds=0.4)
    d.pump(0.45)
    assert d.process.poll() is None, d.text()
    assert d.board.command in ((14, struct.pack('<f', 0.0)), (6, struct.pack('<f', 0.0)))
    d.message.axes[1] = 0.0
    d.pump(0.04)
    d.finish(sine=False)
    assert d.gain.read_text() == before


def test_legacy_mecanum_mode_keeps_four_motor_commands(driver):
    d = driver
    d.settings['target_motor_index'] = -1
    d.start('velocity', seconds=1.0)
    d.message.axes[1] = 0.5
    d.pump(0.08)
    for fid in (12, 13, 14, 15):
        assert any(frame == fid and abs(struct.unpack('<f', payload)[0]) > 0.4
                   for frame, payload in d.board.frames)
    d.wait_with_sine(sine=True)
    assert d.process.returncode == 0, d.text()
    assert all('kp' in yaml.safe_load(d.gain.read_text())[f'wheel{i}'] for i in range(4))


def test_switch_inspection_reports_both_levels_without_motion_or_gain_changes(driver):
    d = driver
    before = d.gain.read_text()
    d.board.contact = True  # Would abort normal velocity measurement immediately.
    d.message.axes[1] = 0.5
    d.start('inspect', seconds=0.1)
    d.wait(lambda: '[2]=true' in d.text())
    d.board.contact = False
    d.wait(lambda: '[2]=false' in d.text())
    d.pump(0.2)
    assert d.process.poll() is None  # Inspection has no measurement countdown.
    assert d.gain.read_text() == before
    assert all(fid in (0, 1, 2, 3, 12, 13, 14, 15) for fid, _ in d.board.frames)
    assert all(struct.unpack('<f', payload)[0] == 0
               for fid, payload in d.board.frames if fid in (12, 13, 14, 15))


@pytest.mark.parametrize('sign', [1, -1])
def test_inspection_preserves_raw_counts_beyond_double_precision(driver, sign):
    d = driver
    before = d.gain.read_text()
    d.board.base_count = sign * (2**53 + 17)
    observed = []
    subscription = d.node.create_subscription(
        Int64, 'encoder_raw_count2', lambda msg: observed.append(msg.data), 10)
    d.message.axes[1] = 0.8
    d.start('inspect', seconds=0.1)
    d.wait(lambda: d.board.base_count in observed)
    d.board.base_count += 37
    d.wait(lambda: d.board.base_count in observed)
    assert d.process.poll() is None
    assert d.gain.read_text() == before
    assert all(struct.unpack('<f', payload)[0] == 0
               for fid, payload in d.board.frames if fid in (12, 13, 14, 15))
    d.node.destroy_subscription(subscription)
