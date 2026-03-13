#!/usr/bin/env python3
#
# 20-channel variant of g1_bridge_lib.py.
# Parses 20-float packets from the MCP3008 firmware variant.
# Legacy (no-CRC) packet parsing is removed — this variant expects CRC packets only.

import importlib.util
import math
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

ROOT_DIR = Path(__file__).resolve().parents[1]
SDK_PYTHON_DIR = ROOT_DIR / "unitree_sdk2_python"
if str(SDK_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_PYTHON_DIR))

from unitree_sdk2py.core.channel import (  # noqa: E402
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HG_LowCmd  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HG_LowState  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_ as HG_MotorCmd  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402


CMD_TOPIC = "rt/lowcmd"
STATE_TOPIC = "rt/lowstate"
DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
HEADER = b"\xAA\x55"
FLOAT_COUNT = 20
PAYLOAD_FORMAT = "<H20f"
PAYLOAD_BYTES = struct.calcsize(PAYLOAD_FORMAT)
PACKET_BYTES = len(HEADER) + PAYLOAD_BYTES + 2  # header + payload + CRC16
COMMAND_DT_SEC = 0.001
PRINT_PERIOD_SEC = 2.0
SERIAL_SETTLE_TIME_SEC = 1.0
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
NUM_MOTORS = 35


@dataclass(frozen=True)
class JointBinding:
    name: str
    packet_index: int
    joint_index: int
    kp: float
    kd: float
    scale: float = 1.0
    offset: float = 0.0
    min_q: Optional[float] = None
    max_q: Optional[float] = None
    average_window_size: int = 1


@dataclass(frozen=True)
class BridgeConfig:
    name: str
    joint_bindings: tuple[JointBinding, ...]
    packet_value_count: int = FLOAT_COUNT


@dataclass
class Packet:
    sequence: int
    values: tuple[float, ...]
    received_at: float


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class SerialPort:
    def __init__(self, path: str, baud_rate: int):
        self._serial = serial.Serial(path, baudrate=baud_rate, timeout=0, write_timeout=0)
        time.sleep(SERIAL_SETTLE_TIME_SEC)
        self._serial.reset_input_buffer()

    def read(self, size: int) -> bytes:
        return self._serial.read(size)

    @property
    def in_waiting(self) -> int:
        return self._serial.in_waiting


class SerialPacketReader:
    """Reads CRC-validated 20-float packets. No legacy fallback."""

    @dataclass
    class Stats:
        valid_packets: int = 0
        crc_failures: int = 0
        invalid_packets: int = 0
        stale_packets: int = 0
        desync_bytes: int = 0

    def __init__(self, serial_port: SerialPort):
        self._serial_port = serial_port
        self._buffer = bytearray()
        self._stats = self.Stats()
        self._last_sequence: Optional[int] = None
        self._last_print_time = 0.0

    def read_latest_packet(self) -> Optional[Packet]:
        self._read_into_buffer()
        latest = None

        while len(self._buffer) >= PACKET_BYTES:
            header_index = self._find_header()
            if header_index < 0:
                self._stats.desync_bytes += len(self._buffer)
                self._buffer.clear()
                break

            if header_index > 0:
                self._stats.desync_bytes += header_index
                del self._buffer[:header_index]

            if len(self._buffer) < PACKET_BYTES:
                break

            packet = self._try_parse_packet()
            if packet is not None:
                latest = packet
            else:
                # CRC mismatch — skip this header byte and resync
                del self._buffer[:1]

        self._print_status_if_needed()
        return latest

    def _read_into_buffer(self) -> None:
        while True:
            available = self._serial_port.in_waiting
            chunk = self._serial_port.read(available if available > 0 else 256)
            if not chunk:
                break
            self._buffer.extend(chunk)

    def _find_header(self) -> int:
        return self._buffer.find(HEADER)

    def _try_parse_packet(self) -> Optional[Packet]:
        expected_crc = int.from_bytes(self._buffer[PACKET_BYTES - 2:PACKET_BYTES], "little")
        actual_crc = crc16_ccitt(bytes(self._buffer[2:2 + PAYLOAD_BYTES]))
        if actual_crc != expected_crc:
            self._stats.crc_failures += 1
            return None

        sequence, *values = struct.unpack_from(PAYLOAD_FORMAT, self._buffer, 2)
        del self._buffer[:PACKET_BYTES]

        if not all(math.isfinite(v) for v in values):
            self._stats.invalid_packets += 1
            return None

        if not self._sequence_is_new(sequence):
            return None

        self._stats.valid_packets += 1
        return Packet(sequence=sequence, values=tuple(values), received_at=time.monotonic())

    def _sequence_is_new(self, sequence: int) -> bool:
        if self._last_sequence is None:
            self._last_sequence = sequence
            return True

        delta = (sequence - self._last_sequence) & 0xFFFF
        if delta == 0 or delta > 0x8000:
            self._stats.stale_packets += 1
            return False

        self._last_sequence = sequence
        return True

    def _print_status_if_needed(self) -> None:
        now = time.monotonic()
        if now - self._last_print_time < PRINT_PERIOD_SEC:
            return

        print(
            "serial stats:",
            f"valid={self._stats.valid_packets}",
            f"crc_fail={self._stats.crc_failures}",
            f"invalid={self._stats.invalid_packets}",
            f"stale={self._stats.stale_packets}",
            f"desync={self._stats.desync_bytes}",
        )
        self._last_print_time = now


class G1SerialBridge:
    def __init__(self, config: BridgeConfig):
        if not config.joint_bindings:
            raise ValueError("Bridge config must define at least one joint binding")

        for binding in config.joint_bindings:
            if binding.packet_index < 0 or binding.packet_index >= config.packet_value_count:
                raise ValueError(
                    f"{binding.name}: packet_index {binding.packet_index} is out of range "
                    f"for packet_value_count={config.packet_value_count}"
                )
            if binding.joint_index < 0 or binding.joint_index >= NUM_MOTORS:
                raise ValueError(
                    f"{binding.name}: joint_index {binding.joint_index} is out of range "
                    f"for NUM_MOTORS={NUM_MOTORS}"
                )
            if binding.average_window_size < 1:
                raise ValueError(f"{binding.name}: average_window_size must be >= 1")
            if (
                binding.min_q is not None
                and binding.max_q is not None
                and binding.min_q > binding.max_q
            ):
                raise ValueError(f"{binding.name}: min_q must be <= max_q")

        self._config = config
        self._crc = CRC()
        self._low_cmd = HG_LowCmd(
            mode_pr=0,
            mode_machine=0,
            motor_cmd=[
                HG_MotorCmd(
                    mode=1,
                    q=POS_STOP_F,
                    dq=VEL_STOP_F,
                    tau=0.0,
                    kp=0.0,
                    kd=0.0,
                    reserve=0,
                )
                for _ in range(NUM_MOTORS)
            ],
            reserve=[0, 0, 0, 0],
            crc=0,
        )
        self._lowcmd_publisher: Optional[ChannelPublisher] = None
        self._lowstate_subscriber: Optional[ChannelSubscriber] = None
        self._latest_state: Optional[HG_LowState] = None
        self._have_state = False
        self._reported_packet = False
        self._input_history = [
            deque(maxlen=binding.average_window_size) for binding in config.joint_bindings
        ]
        self._last_targets = [0.0 for _ in config.joint_bindings]
        self._init_low_cmd()

    def init(self) -> None:
        self._lowcmd_publisher = ChannelPublisher(CMD_TOPIC, HG_LowCmd)
        self._lowcmd_publisher.Init()

        self._lowstate_subscriber = ChannelSubscriber(STATE_TOPIC, HG_LowState)
        self._lowstate_subscriber.Init()

    def wait_for_first_state(self) -> None:
        print("Waiting for robot state...")
        while not self._have_state:
            self.update_state()
            time.sleep(0.1)
        print("Got robot state.")

    def update_state(self) -> None:
        if self._lowstate_subscriber is None:
            raise RuntimeError("Subscriber is not initialized")
        message = self._lowstate_subscriber.Read()
        if message is None:
            return

        self._latest_state = message
        self._low_cmd.mode_machine = message.mode_machine
        self._have_state = True

    def apply_packet(self, packet: Packet) -> None:
        for axis, binding in enumerate(self._config.joint_bindings):
            raw_target = packet.values[binding.packet_index]
            mapped_target = raw_target * binding.scale + binding.offset
            if binding.min_q is not None:
                mapped_target = max(binding.min_q, mapped_target)
            if binding.max_q is not None:
                mapped_target = min(binding.max_q, mapped_target)
            history = self._input_history[axis]
            if not history:
                history.extend([mapped_target] * binding.average_window_size)
            else:
                history.append(mapped_target)

            filtered_target = sum(history) / len(history)
            self._last_targets[axis] = mapped_target

            motor_cmd = self._low_cmd.motor_cmd[binding.joint_index]
            motor_cmd.mode = 1
            motor_cmd.q = filtered_target
            motor_cmd.dq = 0.0
            motor_cmd.kp = binding.kp
            motor_cmd.kd = binding.kd
            motor_cmd.tau = 0.0

        if not self._reported_packet:
            print(
                f"First packet mapped to {self._config.name}:",
                ", ".join(
                    f"{binding.name}={self._last_targets[index]:.3f}"
                    for index, binding in enumerate(self._config.joint_bindings)
                ),
            )
            self._reported_packet = True

    def publish(self) -> None:
        if self._lowcmd_publisher is None:
            raise RuntimeError("Publisher is not initialized")

        self._low_cmd.crc = self._crc.Crc(self._low_cmd)
        self._lowcmd_publisher.Write(self._low_cmd)

    def _init_low_cmd(self) -> None:
        self._low_cmd.mode_pr = 0
        self._low_cmd.mode_machine = 0


def run_bridge(config: BridgeConfig, argv: Optional[list[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    serial_port = args[0] if args else DEFAULT_SERIAL_PORT
    sim_mode = len(args) < 2
    network_interface = "lo" if sim_mode else args[1]
    domain_id = 1 if sim_mode else 0

    ChannelFactoryInitialize(domain_id, network_interface)

    serial_reader = SerialPacketReader(SerialPort(serial_port, DEFAULT_BAUD_RATE))
    bridge = G1SerialBridge(config)

    print(f"Opened serial port {serial_port}")
    print(
        "DDS mode:",
        "sim" if sim_mode else "robot",
        f"domain={domain_id}",
        f"interface={network_interface}",
    )

    bridge.init()
    bridge.wait_for_first_state()

    print(f"Controlling {config.name} from potentiometer packets.")
    print(
        "Packet mapping:",
        ", ".join(
            f"packet[{binding.packet_index}]->{binding.name} (joint {binding.joint_index})"
            for binding in config.joint_bindings
        ),
    )

    next_tick = time.monotonic()
    while True:
        bridge.update_state()
        packet = serial_reader.read_latest_packet()
        if packet is not None:
            bridge.apply_packet(packet)

        bridge.publish()
        next_tick += COMMAND_DT_SEC
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_tick = time.monotonic()


def load_config_from_python_file(path: str | Path) -> BridgeConfig:
    config_path = Path(path).resolve()
    spec = importlib.util.spec_from_file_location(config_path.stem, config_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to import config file: {config_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = getattr(module, "BRIDGE_CONFIG", None)
    if not isinstance(config, BridgeConfig):
        raise RuntimeError(f"{config_path} must define BRIDGE_CONFIG = BridgeConfig(...)")
    return config


def run_bridge_from_config_file(path: str | Path, argv: Optional[list[str]] = None) -> int:
    return run_bridge(load_config_from_python_file(path), argv=argv)
