"""Exercise real ROS nodes against three concurrent PTY firmware emulators."""

import csv
import importlib.machinery
import importlib.util
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import threading
import time

import pytest
import yaml


MANAGER = Path(__file__).resolve().parents[1] / "scripts/multi_stm32_control"


def load_manager():
    loader = importlib.machinery.SourceFileLoader("multi_stm32_control", str(MANAGER))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def encode(data):
    result = bytearray([0])
    code_index, code = 0, 1
    for value in data:
        if value == 0:
            result[code_index] = code
            code_index, code = len(result), 1
            result.append(0)
        else:
            result.append(value)
            code += 1
            if code == 255:
                result[code_index] = code
                code_index, code = len(result), 1
                result.append(0)
    result[code_index] = code
    return b"\0" + result + b"\0"


def decode(data):
    result = bytearray()
    cursor = 0
    while cursor < len(data):
        code = data[cursor]
        if not code or cursor + code > len(data):
            raise ValueError("invalid COBS packet")
        result.extend(data[cursor + 1:cursor + code])
        cursor += code
        if code != 255 and cursor < len(data):
            result.append(0)
    return bytes(result)


def wait_until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    assert predicate(), "timed out waiting for mock STM condition"


class MockSTM:
    def __init__(self, raw_count):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.raw_count = raw_count
        self.frames = []
        self.feedback = True
        self.closed = False
        self.initialized = False
        self.identities = [(0, 0)] * 4
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = b""
        last_feedback = 0.0
        while not self.closed:
            readable, _, _ = select.select([self.master], [], [], 0.01)
            if readable:
                buffer += os.read(self.master, 4096)
                while b"\0" in buffer:
                    frame, buffer = buffer.split(b"\0", 1)
                    if not frame:
                        continue
                    try:
                        packet = decode(frame)
                    except ValueError:
                        continue
                    if len(packet) >= 2 and packet[-1] == sum(packet[:-1]) % 256:
                        self.frames.append((packet[0], packet[1:-1]))
                        if packet[0] < 4 and len(packet[1:-1]) == 52:
                            self.identities[packet[0]] = struct.unpack_from('<II', packet, 45)
                        self.initialized |= packet[0] == 3
            if self.initialized and self.feedback and time.monotonic() - last_feedback > 0.02:
                last_feedback = time.monotonic()
                for motor in range(4):
                    # Natural ARM/x86 ABI: raw_count at 16, applied IDs at 32/36.
                    payload = struct.pack("<ffi4xqf?3xII", 0.005, 0.2, 123,
                                          self.raw_count + motor, 0.2, False, *self.identities[motor])
                    packet = bytes([16 + motor]) + payload
                    os.write(self.master, encode(packet + bytes([sum(packet) % 256])))

    def has(self, frame_id, payload):
        return (frame_id, payload) in self.frames

    def close(self):
        self.closed = True
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


def test_three_stm_isolation_stop_and_timeout(tmp_path):
    executable = os.environ["STM32_TEST_EXECUTABLE"]
    boards = [MockSTM(2**40 + i * 1000) for i in range(3)]
    process = None
    output = []
    reader = None
    try:
        a_gain = tmp_path / "a_gain.yaml"
        # The local param_checker's flat velocity schema, including nonzero Kd.
        a_gain.write_text(yaml.safe_dump({"wheel0": {
            "kp": 0.12, "ki": 0.3, "kd": -0.005,
            "position": {"controller": "P", "kp": 0.0001, "ki": 0.0, "kd": 0.0,
                         "input_unit": "encoder_count", "output_unit": "rps",
                         "max_rps": 0.02, "encoder_ppr": 2048, "gear_rate": 71.0,
                         "encoder_polarity": 1},
        }}))
        b_gain = tmp_path / "b_gain.yaml"
        # The original repository's unified v2 schema.
        b_gain.write_text(yaml.safe_dump({"schema_version": 2, "wheel0": {
            "velocity": {"controller": "PI", "kp": 0.4, "ki": 0.8, "kd": 0.0,
                         "input_unit": "rps", "output_unit": "duty", "max_rps": 0.015},
            "encoder": {"pulses_per_revolution": 2048, "gear_ratio": 40.0,
                        "counts_per_revolution": 327680.0},
        }}))
        config = tmp_path / "boards.yaml"
        config.write_text(yaml.safe_dump({
            "default_board": "a",
            "parameters": {"allow_missing_pid_gains": True, "max_motor_rps": 0.03,
                           "feedback_timeout_seconds": 0.3},
            "boards": {
                name: {"serial_path": board.path, "pid_gain_path": str(gain),
                       "csv_path": str(tmp_path / f"{name}.csv")}
                for name, board, gain in zip(
                    ("a", "b", "c"), boards, (a_gain, b_gain, tmp_path / "missing.yaml"))
            },
        }))
        env = {**os.environ, "ROS_DOMAIN_ID": "167", "ROS_LOG_DIR": str(tmp_path / "ros_log")}
        process = subprocess.Popen(
            [sys.executable, str(MANAGER), "--config", str(config), "--executable", executable],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, env=env,
        )
        reader = threading.Thread(target=lambda: output.extend(process.stdout), daemon=True)
        reader.start()

        def send(command):
            process.stdin.write(command + "\n")
            process.stdin.flush()

        wait_until(lambda: sum("Ready:" in line for line in output) == 3)
        wait_until(lambda: all((tmp_path / f"{name}.csv").stat().st_size > 300
                              for name in ("a", "b", "c")))
        for board in boards:
            init = next(payload for frame, payload in board.frames if frame == 0)
            assert len(init) == 52
            assert init[30] == 0  # Start with all motors off.
            assert init[0:3] == bytes([0, 0, 1])  # No reset, no switch homing.
        a_init = next(payload for frame, payload in boards[0].frames if frame == 0)
        b_init = next(payload for frame, payload in boards[1].frames if frame == 0)
        assert struct.unpack_from("<f", a_init, 4)[0] == pytest.approx(0.12)
        assert struct.unpack_from("<f", b_init, 4)[0] == pytest.approx(0.4)
        assert struct.unpack_from("<f", a_init, 12)[0] == 0.0  # Ignore saved velocity Kd.
        assert a_init[28] == 0 and b_init[28] == 1  # Board-specific encoder polarity.
        assert struct.unpack_from("<f", a_init, 40)[0] == 71.0
        assert struct.unpack_from("<f", b_init, 40)[0] == 40.0
        assert struct.unpack_from("<f", a_init, 32)[0] == pytest.approx(0.02)
        assert struct.unpack_from("<f", b_init, 32)[0] == pytest.approx(0.015)

        position = 2**40 + 321
        send(f"a pos 0 {position}")
        send("b vel 0 -0.01")
        wait_until(lambda: boards[0].has(8, struct.pack("<q", position)))
        wait_until(lambda: boards[1].has(4, struct.pack("<f", -0.01)))
        assert not any(frame == 8 for frame, _ in boards[1].frames)
        assert not any(4 <= frame < 12 for frame, _ in boards[2].frames)
        # Per-board namespace, not merely per-board log labels.
        assert any("[a.stm32_pos_control_test]" in line for line in output)
        assert any("[b.stm32_pos_control_test]" in line for line in output)
        send("b vel 0.02")  # Above the saved limit.
        send("c vel 0.01")  # Missing gains.
        send("c pos 1000")
        wait_until(lambda: sum("rejected" in line for line in output) >= 3)
        assert not boards[1].has(4, struct.pack("<f", 0.02))
        assert not any(4 <= frame < 12 for frame, _ in boards[2].frames)
        wait_until(lambda: any(row["pid_state"] == "2" and
                              int(row["target_pos"]) == position and
                              int(row["encoder_raw_count"]) == boards[0].raw_count
                              for row in csv.DictReader((tmp_path / "a.csv").open())))
        # One board's stale feedback stops it, while the other keeps transmitting.
        before = sum(frame == 12 for frame, _ in boards[0].frames)
        boards[0].feedback = False
        wait_until(lambda: sum(frame == 12 for frame, _ in boards[0].frames) > before)
        wait_until(lambda: any("[a]" in line and "feedback timed out" in line for line in output))
        send("b vel 0.005")
        wait_until(lambda: boards[1].has(4, struct.pack("<f", 0.005)))
        # Individual child exit must leave the remaining serial connections alive.
        send("c quit")
        wait_until(lambda: any("[c] exited (code=0)" in line for line in output))
        send("b vel 0.006")
        wait_until(lambda: boards[1].has(4, struct.pack("<f", 0.006)))
        send("stop")
        send("quit")
        assert process.wait(timeout=10) == 0
        reader.join(timeout=2)
        for board in boards:
            for motor in range(4):
                assert board.has(12 + motor, struct.pack("<f", 0.0))
            assert all(12 <= frame <= 15 for frame, _ in board.frames[-4:])
    finally:
        if process is not None:
            if process.poll() is None:
                process.stdin.write("quit\n")
                process.stdin.flush()
                process.wait(timeout=10)
            process.stdin.close()
            if reader is not None:
                reader.join(timeout=2)
            process.stdout.close()
            print("".join(output))
        for board in boards:
            board.close()


def test_duplicate_device_rejected_before_start(tmp_path):
    manager = load_manager()
    device = tmp_path / "tty"
    device.touch()
    alias = tmp_path / "alias"
    alias.symlink_to(device)
    config = tmp_path / "bad.yaml"
    config.write_text(yaml.safe_dump({"boards": {
        "a": {"serial_path": str(device), "pid_gain_path": "a.yaml"},
        "b": {"serial_path": str(alias), "pid_gain_path": "b.yaml"},
    }}))
    with pytest.raises(ValueError, match="duplicate serial device"):
        manager.load_config(config)


def test_csv_cannot_overwrite_other_board_gain(tmp_path):
    manager = load_manager()
    gain = str(tmp_path / "calibration.yaml")
    config = tmp_path / "bad.yaml"
    config.write_text(yaml.safe_dump({"boards": {
        "a": {"serial_path": "/tmp/device-a", "pid_gain_path": "a.yaml", "csv_path": gain},
        "b": {"serial_path": "/tmp/device-b", "pid_gain_path": gain},
    }}))
    with pytest.raises(ValueError, match="CSV outputs"):
        manager.load_config(config)
