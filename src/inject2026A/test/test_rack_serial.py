"""Run both production nodes against PTYs; never open a physical serial device."""
import os
import importlib.util
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
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64, Int64, String
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
    return b"\0" + result + b"\0"


def decode(data):
    result = bytearray()
    cursor = 0
    while cursor < len(data):
        code = data[cursor]
        if not code or cursor + code > len(data):
            raise ValueError("bad COBS")
        result.extend(data[cursor + 1:cursor + code])
        cursor += code
        if code != 255 and cursor < len(data):
            result.append(0)
    return bytes(result)


class Board:
    def __init__(self, switch_index=2):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.frames = []
        self.switch_index = switch_index
        self.contact = False
        self.raw = 2**40 + 321
        self.follow_position = False
        self.follow_velocity = False
        self.velocity_position = self.velocity_rps = self.velocity_target = 0.0
        self.velocity_origin = self.raw
        self.velocity_events = []
        self.feedback_enabled = True
        self.motor_feedback = True
        self.switch_feedback = True
        self.initialized = False
        self.identities = [(0, 0)] * 4
        self.closed = False
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, frame, payload):
        packet = bytes([frame]) + payload
        os.write(self.master, encode(packet + bytes([sum(packet) % 256])))

    def run(self):
        buffer = b""
        last_feedback = 0
        last_simulation = time.monotonic()
        while not self.closed:
            readable, _, _ = select.select([self.master], [], [], 0.005)
            if readable:
                try:
                    buffer += os.read(self.master, 4096)
                except OSError:
                    break
                while b"\0" in buffer:
                    frame, buffer = buffer.split(b"\0", 1)
                    if not frame:
                        continue
                    try:
                        packet = decode(frame)
                    except ValueError:
                        continue
                    if len(packet) >= 2 and packet[-1] == sum(packet[:-1]) % 256:
                        self.frames.append((time.monotonic(), packet[0], packet[1:-1]))
                        if packet[0] < 4 and len(packet[1:-1]) == 52:
                            fid = packet[0]
                            identity = struct.unpack_from('<II', packet, 45)
                            result = int(self.identities[fid] == identity)
                            self.identities[fid] = identity
                            self.send(21 + fid, struct.pack('<IIIIB3x', *identity, *identity, result))
                        self.initialized |= packet[0] == 3
                        if self.follow_position and packet[0] == 10:
                            self.raw = struct.unpack("<q", packet[1:-1])[0]
                        if self.follow_velocity and packet[0] in (6, 14):
                            target = struct.unpack('<f', packet[1:-1])[0] if packet[0] == 6 else 0.0
                            self.velocity_events.append((time.monotonic() + 0.03936, target))
            if self.follow_velocity:
                instant = time.monotonic()
                while last_simulation < instant:
                    step = min(0.001, instant - last_simulation)
                    while self.velocity_events and self.velocity_events[0][0] <= last_simulation:
                        self.velocity_target = self.velocity_events.pop(0)[1]
                    self.velocity_rps += (0.9966671 * self.velocity_target - self.velocity_rps) * step / 0.3024
                    self.velocity_position += self.velocity_rps * 8192 * step
                    last_simulation += step
                self.raw = self.velocity_origin + round(self.velocity_position)
            if self.initialized and self.feedback_enabled and time.monotonic() - last_feedback > 0.01:
                last_feedback = time.monotonic()
                try:
                    for motor in range(4):
                        if motor == 2 and not self.motor_feedback:
                            continue
                        self.send(16 + motor, struct.pack("<ffi4xqf?3xII", self.velocity_rps if motor == 2 else 0.0, 0.0, 0,
                                                         self.raw if motor == 2 else 0,
                                                         0.0, False, *self.identities[motor]))
                    if self.switch_feedback:
                        switches = [0] * 4
                        switches[self.switch_index] = self.contact
                        self.send(20, bytes(switches))
                except OSError:
                    break

    def latest(self, frame):
        return next((payload for _, fid, payload in reversed(self.frames) if fid == frame), None)

    def rack_command(self):
        return next(((fid, payload) for _, fid, payload in reversed(self.frames)
                     if fid in (6, 10, 14)), None)

    def duty_zero(self):
        return self.rack_command() == (14, struct.pack("<f", 0.0))

    def close(self):
        self.closed = True
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


@pytest.mark.parametrize('angle_sign', [None, -1, 1])
def test_button_profiles_origin_hold_and_watchdog(tmp_path, angle_sign):
    angle_profile = angle_sign is not None
    inject, roll = Board(), Board()
    inject.follow_position = True
    origin = inject.raw
    primary_angle = 1.0 if angle_sign == 1 else 5.0
    secondary_angle = 2.0 if angle_sign == 1 else 10.0
    primary_count = angle_sign * round(primary_angle * 8192 / 360) if angle_profile else -30
    secondary_count = angle_sign * round(secondary_angle * 8192 / 360) if angle_profile else 0
    process = log = node = None
    gain_path = tmp_path / "gains.yaml"
    gain_path.write_text(yaml.safe_dump({"wheel2": {
        "kp": 0.12, "ki": 0.3,
        "position": {"controller": "P", "kp": 0.0001, "ki": 0.0, "kd": 0.0,
                     "input_unit": "encoder_count", "output_unit": "rps",
                     "max_rps": 0.02, "encoder_ppr": 2048, "gear_rate": 1.0,
                     "encoder_polarity": 1},
    }}))
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump({
        "inject_serial_node": {"ros__parameters": {
            "serial_path": inject.path, "rack_pid_gain_path": str(gain_path),
            # Omit rack_count_profile to exercise the production default.
            "rack_max_angle_deg": 0.0, "rack_transmission_ratio": 0.0,
            "rack_max_speed_counts_s": 60.0, "rack_acceleration_counts_s2": 120.0,
            "rack_motion_timeout_s": 3.0, "rack_feedback_timeout_ms": 100,
            "rack_angle_profile": angle_profile,
            "rack_velocity_feedforward": not angle_profile,  # Ignored outside angle profiles.
            **({"rack_transmission_ratio": 1.0, "rack_max_angle_deg": 20.0,
                "rack_angle_count_sign": angle_sign, "rack_target_angle_4_deg": primary_angle,
                "rack_target_angle_5_deg": secondary_angle, "rack_speed_deg_s": 5.0,
                "rack_acceleration_deg_s2": 10.0, "rack_position_tolerance_deg": 0.2}
               if angle_profile else {}),
            "timeout_ms": 150,
        }},
        "roll_serial_node": {"ros__parameters": {"serial_path": roll.path}},
    }))
    os.environ["ROS_DOMAIN_ID"] = "179"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros_log")
    try:
        log = (tmp_path / "node.log").open("w")
        command = (['ros2', 'launch', 'inject2026A', 'inject.launch.xml', f'serial_config:={config}']
                   if angle_profile else
                   [os.environ["INJECT_TEST_EXECUTABLE"], '--ros-args', '--params-file', str(config)])
        process = subprocess.Popen(command, stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node("rack_count_driver")
        publisher = node.create_publisher(Joy, "inject/joy", 10)
        relative, targets, states = [], [], []
        angles, angle_targets = [], []
        subscriptions = [
            node.create_subscription(Int64, "inject/rack/relative_count", lambda m: relative.append(m.data), 10),
            node.create_subscription(Int64, "inject/rack/target_relative_count", lambda m: targets.append(m.data), 10),
            node.create_subscription(String, "inject/rack/state", lambda m: states.append(m.data), 10),
            node.create_subscription(Float64, "inject/rack/angle_deg", lambda m: angles.append(m.data), 10),
            node.create_subscription(Float64, "inject/rack/target_angle_deg", lambda m: angle_targets.append(m.data), 10),
        ]
        msg = Joy(buttons=[0] * 13, axes=[0.0] * 8)

        def pump(seconds=0.1, publish=True):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                if publish:
                    publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.01)
                assert process.poll() is None, (tmp_path / "node.log").read_text()

        def wait(predicate, timeout=5):
            until = time.monotonic() + timeout
            while time.monotonic() < until and not predicate():
                pump(0.03)
            assert predicate(), (tmp_path / "node.log").read_text()

        wait(lambda: inject.initialized and publisher.get_subscription_count() == 2)
        pump(0.15)
        msg.buttons[4] = 1
        pump(0.1)
        assert inject.duty_zero()  # Cannot move without zero.
        inject.contact = True
        pump(0.12)
        assert inject.duty_zero()  # Homing with a held button cannot auto-start.
        msg.buttons[4] = 0
        wait(lambda: states and states[-1] == "ready")
        assert relative[-1] == 0
        start_index = len(inject.frames)
        msg.buttons[4] = 1
        if angle_profile:
            wait(lambda: states and states[-1] == "moving")
            msg.buttons[4] = 0
            msg.buttons[5] = 1
            pump(0.08)  # Cannot retarget an active trajectory with button 5.
            msg.buttons[5] = 0
        wait(lambda: states and states[-1] == "holding")
        assert relative[-1] == primary_count
        assert targets[-1] == primary_count
        commands = [struct.unpack("<q", payload)[0]
                    for _, fid, payload in inject.frames[start_index:] if fid == 10]
        assert commands[-1] == origin + primary_count
        assert len(set(commands)) > 10  # Intermediate targets, rather than a -30 step.
        assert commands == sorted(commands, reverse=primary_count < 0)
        assert all(min(origin + primary_count, origin) <= value <= max(origin + primary_count, origin)
                   for value in commands)
        pump(0.2)
        assert inject.rack_command() == (10, struct.pack("<q", origin + primary_count))
        assert relative[-1] == primary_count  # Held switch cannot redefine zero.
        if angle_profile:
            assert angles[-1] == pytest.approx(primary_angle, abs=360 / 8192 / 2)
            assert angle_targets[-1] == pytest.approx(angles[-1])
            msg.buttons[4] = msg.buttons[5] = 1
            pump(0.1)
            assert targets[-1] == primary_count  # Simultaneous press cannot start either target.
            msg.buttons[4] = msg.buttons[5] = 0
            pump(0.08)
            msg.buttons[5] = 1
            wait(lambda: states[-1] == "holding" and relative[-1] == secondary_count)
            assert targets[-1] == secondary_count
            assert angles[-1] == pytest.approx(secondary_angle, abs=360 / 8192 / 2)

        # Button2/3 retain their original held Duty operation in PID mode.
        # A manual command cancels holding and cannot resume an old button4/5.
        msg.buttons[2] = 1
        pump(0.08)
        assert inject.rack_command() == (14, struct.pack('<f', 0.2))
        msg.buttons[2] = 0
        msg.buttons[3] = 1
        inject.contact = False
        pump(0.08)
        assert inject.rack_command() == (14, struct.pack('<f', -0.2))
        msg.buttons[2] = 1
        pump(0.08)
        assert inject.duty_zero()
        msg.buttons[2] = msg.buttons[3] = 0
        pump(0.08)
        assert inject.duty_zero()

        # Resume a new automatic press before checking the existing Joy stop.
        msg.buttons[4] = msg.buttons[5] = 0
        pump(0.08)
        msg.buttons[4] = 1
        wait(lambda: states[-1] == 'holding')

        pump(0.25, publish=False)  # Joy timeout releases position holding.
        assert inject.duty_zero()
        pump(0.15)  # Button still held: no resume.
        assert inject.duty_zero()
        msg.buttons[4] = 0
        msg.buttons[5] = 0
        pump(0.15)
        assert inject.duty_zero()  # Neutral alone cannot resume either.
        inject.contact = False
        inject.raw = origin + (angle_sign * 10 if angle_profile else -10)
        pump(0.05)
        msg.buttons[4] = 1
        wait(lambda: states and states[-1] == "moving")
        inject.motor_feedback = False
        pump(0.2)
        assert inject.duty_zero()
        assert states[-1] == "waiting_feedback"
        inject.motor_feedback = True
        pump(0.15)
        assert inject.duty_zero()  # Requires fresh origin after feedback loss.
        assert subscriptions  # Keep ROS subscription handles alive through the test.
    finally:
        if process:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log:
            log.close()
        inject.close()
        roll.close()


@pytest.mark.parametrize('overrides', [
    {},
    {'rack_target_angle_4_deg': '1.0', 'rack_target_angle_5_deg': '2.0'},
    {'rack_angle_count_sign': '-1'},
])
def test_launch_uses_yaml_unless_target_or_direction_is_explicit(monkeypatch, overrides):
    from launch import LaunchContext
    path = Path(__file__).parents[1] / 'launch' / 'inject.launch.py'
    spec = importlib.util.spec_from_file_location('inject_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'serial_config': '/tmp/mock-config.yaml', 'joy_topic': '/inject/joy',
        'auto_restart': 'false', 'inject_pid': 'true', 'rack_angle_profile': 'true',
        'rack_target_angle_4_deg': '', 'rack_target_angle_5_deg': '', 'rack_angle_count_sign': '',
        **overrides,
    })
    result = module.make_node(context)[0]
    assert result['parameters'][0] == '/tmp/mock-config.yaml'
    parameters = result['parameters'][1]
    for name, convert in [('rack_target_angle_4_deg', float), ('rack_target_angle_5_deg', float),
                          ('rack_angle_count_sign', int)]:
        if name in overrides:
            assert parameters[name] == convert(overrides[name])
        else:
            assert name not in parameters


@pytest.mark.parametrize('goal,button', [(20.0, 4), (30.0, 5)])
def test_model_feedforward_reaches_goal_and_stops_on_feedback_loss(tmp_path, goal, button):
    inject, roll = Board(), Board()
    inject.follow_velocity = True
    process = log = node = None
    origin = inject.raw
    gain_path = tmp_path / 'gains.yaml'
    gain_path.write_text(yaml.safe_dump({'wheel2': {
        'kp': 0.051154, 'ki': 5.251389,
        'position': {'controller': 'P', 'kp': 5.0966919e-5, 'ki': 0.0, 'kd': 0.0,
                     'input_unit': 'encoder_count', 'output_unit': 'rps',
                     'max_rps': 0.06, 'encoder_ppr': 2048, 'gear_rate': 1.0, 'encoder_polarity': 1,
                     'identification_method': 'velocity_response_model_v1',
                     'velocity_response_time_seconds': 0.3024,
                     'velocity_delay_seconds': 0.03936, 'velocity_steady_gain': 0.9966671},
    }}))
    before = gain_path.read_text()
    config = tmp_path / 'config.yaml'
    config.write_text(yaml.safe_dump({
        'inject_serial_node': {'ros__parameters': {
            'serial_path': inject.path, 'rack_pid_gain_path': str(gain_path),
            'rack_angle_profile': True, 'rack_velocity_feedforward': True,
            'rack_transmission_ratio': 1.0, 'rack_angle_count_sign': 1,
            'rack_max_angle_deg': 30.0, 'rack_target_angle_4_deg': 20.0, 'rack_target_angle_5_deg': 30.0,
            'rack_speed_deg_s': 15.0, 'rack_acceleration_deg_s2': 30.0,
            'rack_position_tolerance_deg': 0.2, 'rack_max_rps': 0.2,
            'rack_motion_timeout_s': 5.0, 'rack_feedback_timeout_ms': 150, 'timeout_ms': 150,
        }},
        'roll_serial_node': {'ros__parameters': {'serial_path': roll.path}},
    }))
    os.environ['ROS_DOMAIN_ID'] = '179'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(tmp_path / 'ros_log')
    try:
        log = (tmp_path / 'node.log').open('w')
        process = subprocess.Popen([os.environ['INJECT_TEST_EXECUTABLE'], '--ros-args',
                                    '--params-file', str(config)], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node('rack_model_driver')
        publisher = node.create_publisher(Joy, 'inject/joy', 10)
        states = []
        subscription = node.create_subscription(String, 'inject/rack/state', lambda m: states.append(m.data), 10)
        msg = Joy(buttons=[0] * 13, axes=[0.0] * 8)

        def pump(seconds=0.05, publish=True):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                if publish:
                    publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.005)
                assert process.poll() is None, (tmp_path / 'node.log').read_text()

        def wait(predicate, timeout=5):
            until = time.monotonic() + timeout
            while time.monotonic() < until and not predicate():
                pump(0.02)
            assert predicate(), (tmp_path / 'node.log').read_text()

        wait(lambda: inject.initialized and publisher.get_subscription_count() == 2)
        pump(0.15)
        inject.contact = True
        wait(lambda: states and states[-1] == 'ready')
        inject.contact = False
        started = time.monotonic()
        msg.buttons[button] = 1
        wait(lambda: states[-1] == 'holding')
        assert time.monotonic() - started < 3.2
        assert (inject.raw - origin) * 360 / 8192 == pytest.approx(goal, abs=0.2)
        velocity_commands = [struct.unpack('<f', payload)[0] for _, fid, payload in inject.frames if fid == 6]
        assert velocity_commands and max(abs(u) for u in velocity_commands) <= 0.060001
        assert not any(fid == 10 for _, fid, _ in inject.frames)
        if button == 5:
            # Reproduce the real log: the shaft stalls six counts above the
            # 20-degree goal, outside its four-count arrival tolerance.
            msg.buttons[5] = 0
            pump(0.08)
            inject.follow_velocity = False
            msg.buttons[4] = 1
            wait(lambda: states[-1] == 'moving')
            inject.raw = origin + round(20 * 8192 / 360) + 6
            wait(lambda: states[-1] == 'settling')
            msg.buttons[4] = 0
            pump(0.08)
            msg.buttons[5] = 1
            wait(lambda: states[-1] == 'moving')
            assert inject.rack_command()[0] == 6
            assert struct.unpack('<f', inject.rack_command()[1])[0] > 0
            wait(lambda: states[-1] == 'motion_timeout', timeout=6)
            assert inject.duty_zero()
            pump(0.1)
            assert inject.duty_zero()  # Held through the timeout cannot resume.
            msg.buttons[5] = 0
            pump(0.08)
            assert inject.duty_zero()  # Neutral alone does not restart.
            msg.buttons[4] = 1
            wait(lambda: states[-1] == 'moving')
            assert inject.rack_command()[0] == 6
            assert struct.unpack('<f', inject.rack_command()[1])[0] < 0
        else:
            pump(0.25, publish=False)
            assert inject.duty_zero()
            pump(0.08)
            assert inject.duty_zero()  # A held button cannot resume.
            msg.buttons[button] = 0
            pump(0.08)
            msg.buttons[5 if button == 4 else 4] = 1
            wait(lambda: states[-1] == 'moving')
        inject.motor_feedback = False
        pump(0.2)
        assert inject.duty_zero()
        assert states[-1] == 'waiting_feedback'
        assert gain_path.read_text() == before
        assert subscription
    finally:
        if process:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log:
            log.close()
        inject.close()
        roll.close()


def test_roll_sent_duty_matches_serial_and_watchdog_stops(tmp_path):
    inject, roll = Board(), Board()
    process = log = node = None
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump({
        "inject_serial_node": {"ros__parameters": {
            "serial_path": inject.path, "inject_pid": False,
        }},
        "roll_serial_node": {"ros__parameters": {
            "serial_path": roll.path, "timeout_ms": 150,
        }},
    }))
    os.environ["ROS_DOMAIN_ID"] = "179"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros_log")
    try:
        log = (tmp_path / "node.log").open("w")
        process = subprocess.Popen([os.environ["INJECT_TEST_EXECUTABLE"], "--ros-args",
                                    "--params-file", str(config)], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node("roll_sent_duty_driver")
        publisher = node.create_publisher(Joy, "inject/joy", 10)
        observed = []
        subscription = node.create_subscription(
            Float32MultiArray, "roll/command_duty",
            lambda message: observed.append(list(message.data)), 10)
        msg = Joy()
        msg.buttons = [0] * 13
        msg.axes = [0.0] * 8
        msg.axes[6] = msg.axes[7] = 0.45

        def pump(seconds=0.1, publish=True):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                if publish:
                    publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.015)
                assert process.poll() is None, (tmp_path / "node.log").read_text()

        def wait(predicate):
            until = time.monotonic() + 6
            while time.monotonic() < until:
                if predicate():
                    return
                pump(0.03)
            assert predicate(), (tmp_path / "node.log").read_text()

        def sent_and_observed(values):
            packets = [roll.latest(12 + motor) for motor in range(4)]
            if not observed or any(packet is None for packet in packets):
                return False
            sent = [struct.unpack("<f", packet)[0] for packet in packets]
            return sent == pytest.approx(values) and observed[-1] == pytest.approx(sent)

        wait(lambda: publisher.get_subscription_count() == 2 and
             subscription.get_publisher_count() == 1 and observed)
        pump(0.08)
        assert sent_and_observed([0.0] * 4)  # Held input cannot arm at startup.
        msg.axes[6] = msg.axes[7] = 0.0
        pump(0.08)
        msg.axes[6] = msg.axes[7] = 0.45
        wait(lambda: sent_and_observed([-0.45, 0.45, 0.0, 0.0]))

        msg.axes[6], msg.axes[7] = 1.5, -2.0
        wait(lambda: sent_and_observed([-1.0, -1.0, 0.0, 0.0]))
        pump(0.25, publish=False)
        assert sent_and_observed([0.0] * 4)  # Publish the actual watchdog zero.
        pump(0.08)
        assert sent_and_observed([0.0] * 4)  # Non-neutral input must stay stopped.
        msg.axes[6] = msg.axes[7] = 0.0
        pump(0.08)
        msg.axes[6] = msg.axes[7] = 0.45
        wait(lambda: sent_and_observed([-0.45, 0.45, 0.0, 0.0]))
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log:
            log.close()
        inject.close()
        roll.close()


def test_rack_homing_hold_faults_reconnect_and_roll(tmp_path):
    inject, roll = Board(switch_index=1), Board()
    process = None
    log = None
    node = None
    origin = inject.raw
    link = tmp_path / "inject-port"
    link.symlink_to(inject.path)
    gain_path = tmp_path / "gains.yaml"
    gain_path.write_text(yaml.safe_dump({"wheel2": {
        "kp": 0.12, "ki": 0.3, "kd": -0.005,
        "position": {"controller": "P", "kp": 0.0001, "ki": 0.0, "kd": 0.0,
                     "input_unit": "encoder_count", "output_unit": "rps",
                     "max_rps": 0.02, "encoder_ppr": 360, "gear_rate": 1.0,
                     "encoder_polarity": 1},
    }}))
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump({
        "inject_serial_node": {"ros__parameters": {
            "serial_path": str(link), "rack_pid_gain_path": str(gain_path),
            "rack_count_profile": False,
            "rack_transmission_ratio": 1.0, "rack_max_angle_deg": 5.0,
            "rack_speed_deg_s": 20.0, "rack_feedback_timeout_ms": 100,
            "rack_switch_index": 1, "rack_switch_debounce_ms": 30, "rack_homing_timeout_s": 1.0,
            "timeout_ms": 150, "reconnect_interval_ms": 50,
        }},
        "roll_serial_node": {"ros__parameters": {"serial_path": roll.path}},
    }))
    # Each CTest process has its own domain and local-only discovery.
    os.environ["ROS_DOMAIN_ID"] = "179"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros_log")
    try:
        log = (tmp_path / "node.log").open("w")
        process = subprocess.Popen([os.environ["INJECT_TEST_EXECUTABLE"], "--ros-args",
                                    "--params-file", str(config)], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node("rack_integration_driver")
        publisher = node.create_publisher(Joy, "inject/joy", 10)
        msg = Joy()
        msg.buttons = [0] * 13
        msg.axes = [0.0] * 8

        def pump(seconds=0.1, publish=True):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                if publish:
                    publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.015)
                assert process.poll() is None, (tmp_path / "node.log").read_text()

        def wait(predicate, timeout=6):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                if predicate():
                    return
                pump(0.03)
            assert predicate(), (tmp_path / "node.log").read_text()

        wait(lambda: inject.initialized and roll.initialized and publisher.get_subscription_count() == 2)
        pump(0.15)  # Neutral arms both nodes.
        init = inject.latest(2)
        assert len(init) == 52
        assert init[0:2] == b"\0\0"  # No MCU origin reset.
        assert struct.unpack_from("<f", init, 4)[0] > 0  # Speed PI loaded.
        assert struct.unpack_from("<f", init, 16)[0] > 0
        assert struct.unpack_from("<H", init, 36)[0] == 360
        assert struct.unpack_from("<f", init, 32)[0] <= 0.021
        assert inject.duty_zero()

        msg.buttons[2] = 1  # Positive input cannot move before homing.
        pump(0.1)
        assert inject.duty_zero()
        msg.buttons[2] = 0
        pump(0.05)
        msg.buttons[3] = 1
        wait(lambda: inject.rack_command() == (14, struct.pack("<f", -0.1)))
        inject.contact = True
        wait(inject.duty_zero)
        pump(0.1)
        assert inject.duty_zero()  # Holding triangle after contact must not restart.
        msg.buttons[3] = 0
        wait(lambda: inject.rack_command() == (10, struct.pack("<q", origin)))
        inject.contact = False
        msg.buttons[2] = 1
        pump(0.4)
        assert inject.rack_command() == (10, struct.pack("<q", origin + 20))  # 5 deg * 4 count/deg.
        msg.buttons[2] = 0
        pump(0.08)
        assert inject.rack_command() == (10, struct.pack("<q", origin + 20))
        msg.buttons[2] = msg.buttons[3] = 1
        pump(0.08)
        assert inject.rack_command() == (10, struct.pack("<q", origin + 20))
        msg.buttons[2] = msg.buttons[3] = 0

        # Joy loss stops and cancels the old target; current measured position is resumed.
        pump(0.22, publish=False)
        assert inject.duty_zero()
        inject.raw = origin + 4
        msg.buttons[2] = 1
        pump(0.08)
        assert inject.duty_zero()
        msg.buttons[2] = 0
        wait(lambda: inject.rack_command() == (10, struct.pack("<q", origin + 4)))

        # Other motors and switches keep reporting, but motor 2 loss invalidates homing.
        inject.motor_feedback = False
        pump(0.15)
        assert inject.duty_zero()
        msg.buttons[3] = 1
        inject.motor_feedback = True
        pump(0.08)
        assert inject.duty_zero()  # Requires actual neutral after feedback returns.
        msg.buttons[3] = 0
        pump(0.05)
        msg.buttons[3] = 1
        wait(lambda: inject.rack_command() == (14, struct.pack("<f", -0.1)))
        pump(1.1)
        assert inject.duty_zero()  # Homing timeout is latched.
        msg.buttons[3] = 0
        pump(0.05)
        msg.buttons[3] = 1
        pump(0.05)
        assert inject.duty_zero()
        inject.contact = True
        pump(0.1)
        msg.buttons[3] = 0
        wait(lambda: inject.rack_command() == (10, struct.pack("<q", inject.raw)))

        inject.switch_feedback = False
        pump(0.15)
        assert inject.duty_zero()  # Motor-only traffic is insufficient too.
        inject.switch_feedback = True
        pump(0.15)
        wait(lambda: inject.rack_command()[0] == 10)

        # Disconnect/reconnect to a different PTY; no cached origin or command survives.
        inject.close()
        inject = Board(switch_index=1)
        link.unlink()
        link.symlink_to(inject.path)
        msg.buttons[2] = 1
        wait(lambda: inject.initialized)
        pump(0.15)
        assert inject.duty_zero()
        assert not any(fid == 10 for _, fid, _ in inject.frames)
        msg.buttons[2] = 0
        pump(0.08)

        # Roll motor 2 retains its independent stick-driven Duty protocol.
        msg.axes[1] = 0.5
        pump(0.08)
        assert abs(struct.unpack("<f", roll.latest(15))[0] - 0.15) < 1e-5
        assert not any(fid == 10 for _, fid, _ in roll.frames)
        msg.axes[1] = 0.0
        pump(0.06)

        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert inject.duty_zero()
        assert struct.unpack("<f", roll.latest(15))[0] == 0
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log:
            log.close()
        inject.close()
        roll.close()


@pytest.mark.parametrize("duty_limit", [0.2, 0.07])
def test_pid_off_without_encoder_gains_preserves_stops_and_reconnect(tmp_path, duty_limit):
    inject, roll = Board(), Board()
    inject.feedback_enabled = False  # No motor or switch feedback at all.
    process = log = node = None
    link = tmp_path / "inject-port"
    link.symlink_to(inject.path)
    settings = {
        "serial_path": str(link), "inject_pid": False,
        "rack_pid_gain_path": str(tmp_path / "missing-gains.yaml"),
        "rack_transmission_ratio": 0.0, "rack_max_angle_deg": 0.0,
        "timeout_ms": 150, "reconnect_interval_ms": 50,
    }
    if duty_limit != 0.2:  # Exercise the default too.
        settings["rack_open_loop_duty"] = duty_limit
    config = tmp_path / "config.yaml"
    config.write_text(yaml.safe_dump({
        "inject_serial_node": {"ros__parameters": settings},
        "roll_serial_node": {"ros__parameters": {"serial_path": roll.path}},
    }))
    os.environ["ROS_DOMAIN_ID"] = "179"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros_log")
    try:
        log = (tmp_path / "node.log").open("w")
        process = subprocess.Popen([os.environ["INJECT_TEST_EXECUTABLE"], "--ros-args",
                                    "--params-file", str(config)], stdout=log, stderr=log)
        rclpy.init()
        node = rclpy.create_node("rack_duty_driver")
        publisher = node.create_publisher(Joy, "inject/joy", 10)
        msg = Joy()
        msg.buttons = [0] * 13
        msg.axes = [0.0] * 8
        msg.buttons[2] = 1  # Held at startup must not arm motion.

        def pump(seconds=0.1, publish=True):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                if publish:
                    publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.015)
                assert process.poll() is None, (tmp_path / "node.log").read_text()

        def wait(predicate, timeout=6):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                if predicate():
                    return
                pump(0.03)
            assert predicate(), (tmp_path / "node.log").read_text()

        def duty_is(value):
            return inject.rack_command() == (14, struct.pack("<f", value))

        wait(lambda: inject.initialized and roll.initialized and publisher.get_subscription_count() == 2)
        pump(0.1)
        assert inject.duty_zero()
        for motor in range(4):
            assert inject.latest(motor)[30] == 0  # MCU PID disabled.
        msg.buttons[2] = 0
        pump(0.08)
        msg.buttons[2] = 1
        wait(lambda: duty_is(duty_limit))

        # Mode changes require restart; reject a live change rather than ignore it.
        client = AsyncParameterClient(node, "inject_serial_node")
        wait(client.services_are_ready)
        future = client.set_parameters([Parameter("inject_pid", value=True)])
        wait(future.done)
        assert not future.result().results[0].successful
        assert duty_is(duty_limit)

        msg.buttons[2] = 0
        msg.buttons[3] = 1
        wait(lambda: duty_is(-duty_limit))
        msg.buttons[2] = 1
        wait(inject.duty_zero)  # Both buttons cancel.
        msg.buttons[2] = msg.buttons[3] = 0
        pump(0.08)
        assert inject.duty_zero()

        # Other mechanisms retain their existing command mapping.
        msg.buttons[0] = msg.buttons[12] = 1
        msg.axes[1] = 0.5
        wait(lambda: inject.latest(12) == struct.pack("<f", 0.4) and
             inject.latest(13) == struct.pack("<f", 0.4) and
             abs(struct.unpack("<f", roll.latest(15))[0] - 0.15) < 1e-5)
        msg.buttons[0] = msg.buttons[12] = 0
        msg.axes[1] = 0.0
        msg.buttons[2] = 1
        wait(lambda: duty_is(duty_limit))
        pump(0.22, publish=False)
        assert inject.duty_zero()
        pump(0.08)  # A held button cannot resume after Joy loss.
        assert inject.duty_zero()
        msg.buttons[2] = 0
        pump(0.08)
        msg.buttons[2] = 1
        wait(lambda: duty_is(duty_limit))

        inject.close()
        inject = Board()
        inject.feedback_enabled = False
        link.unlink()
        link.symlink_to(inject.path)
        wait(lambda: inject.initialized)
        pump(0.1)
        assert inject.duty_zero()
        msg.buttons[2] = 0
        pump(0.08)
        msg.buttons[3] = 1
        wait(lambda: duty_is(-duty_limit))
        assert all(fid in (0, 1, 2, 3, 12, 13, 14, 15) for _, fid, _ in inject.frames)
        assert all(fid not in range(4, 12) for _, fid, _ in roll.frames)

        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert inject.duty_zero()
        assert struct.unpack("<f", roll.latest(15))[0] == 0
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log:
            log.close()
        inject.close()
        roll.close()
