#!/usr/bin/env python3

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
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HG_LowCmd  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_ as HG_MotorCmd  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HG_LowState  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402


CMD_TOPIC = "rt/lowcmd"
STATE_TOPIC = "rt/lowstate"
DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200

HEADER = b"\xAA\x55"
FLOAT_COUNT = 14
PAYLOAD_FORMAT = "<H14f"
PAYLOAD_BYTES = struct.calcsize(PAYLOAD_FORMAT)
PACKET_BYTES = len(HEADER) + PAYLOAD_BYTES + 2
LEGACY_PAYLOAD_FORMAT = "<14f"
LEGACY_PACKET_BYTES = len(HEADER) + struct.calcsize(LEGACY_PAYLOAD_FORMAT)

COMMAND_DT_SEC = 0.001
PRINT_PERIOD_SEC = 2.0
SERIAL_SETTLE_TIME_SEC = 1.0
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
WRIST_KP = 24.0
WRIST_KD = 1.6
AVERAGE_WINDOW_SIZE = 1

NUM_MOTORS = 35
JOINT_LEFT_WRIST_ROLL = 19
JOINT_LEFT_WRIST_PITCH = 20
JOINT_LEFT_WRIST_YAW = 21
PACKET_LEFT_WRIST_INDICES = (4, 5, 6)
LEFT_WRIST_JOINT_INDICES = (
    JOINT_LEFT_WRIST_ROLL,
    JOINT_LEFT_WRIST_PITCH,
    JOINT_LEFT_WRIST_YAW,
)


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
    @dataclass
    class Stats:
        valid_packets: int = 0
        legacy_packets: int = 0
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

        while len(self._buffer) >= LEGACY_PACKET_BYTES:
            header_index = self._find_header()
            if header_index < 0:
                self._stats.desync_bytes += len(self._buffer)
                self._buffer.clear()
                break

            if header_index > 0:
                self._stats.desync_bytes += header_index
                del self._buffer[:header_index]

            if len(self._buffer) < LEGACY_PACKET_BYTES:
                break

            packet = self._try_parse_crc_packet()
            if packet is not None:
                latest = packet
                continue

            packet = self._try_parse_legacy_packet()
            if packet is not None:
                latest = packet

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

    def _try_parse_crc_packet(self) -> Optional[Packet]:
        if len(self._buffer) < PACKET_BYTES:
            return None

        expected_crc = int.from_bytes(self._buffer[PACKET_BYTES - 2:PACKET_BYTES], "little")
        actual_crc = crc16_ccitt(bytes(self._buffer[2:2 + PAYLOAD_BYTES]))
        if actual_crc != expected_crc:
            self._stats.crc_failures += 1
            return None

        sequence, *values = struct.unpack_from(PAYLOAD_FORMAT, self._buffer, 2)
        del self._buffer[:PACKET_BYTES]
        packet = Packet(sequence=sequence, values=tuple(values), received_at=time.monotonic())
        return self._accept_packet(packet, legacy=False)

    def _try_parse_legacy_packet(self) -> Optional[Packet]:
        values = struct.unpack_from(LEGACY_PAYLOAD_FORMAT, self._buffer, 2)
        del self._buffer[:LEGACY_PACKET_BYTES]
        sequence = 0 if self._last_sequence is None else (self._last_sequence + 1) & 0xFFFF
        packet = Packet(sequence=sequence, values=tuple(values), received_at=time.monotonic())
        return self._accept_packet(packet, legacy=True)

    def _accept_packet(self, packet: Packet, legacy: bool) -> Optional[Packet]:
        if not all(math.isfinite(value) for value in packet.values):
            self._stats.invalid_packets += 1
            return None

        if not self._sequence_is_new(packet.sequence):
            return None

        if legacy:
            self._stats.legacy_packets += 1
        else:
            self._stats.valid_packets += 1
        return packet

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
            f"legacy={self._stats.legacy_packets}",
            f"crc_fail={self._stats.crc_failures}",
            f"invalid={self._stats.invalid_packets}",
            f"stale={self._stats.stale_packets}",
            f"desync={self._stats.desync_bytes}",
        )
        self._last_print_time = now


class G1LeftWristSerialBridge:
    def __init__(self):
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
        self._input_history = [deque(maxlen=AVERAGE_WINDOW_SIZE) for _ in range(3)]
        self._last_targets = [0.0, 0.0, 0.0]
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
        for axis, joint in enumerate(LEFT_WRIST_JOINT_INDICES):
            raw_target = packet.values[PACKET_LEFT_WRIST_INDICES[axis]]
            history = self._input_history[axis]
            if not history:
                history.extend([raw_target] * AVERAGE_WINDOW_SIZE)
            else:
                history.append(raw_target)

            filtered_target = sum(history) / len(history)
            self._last_targets[axis] = raw_target

            motor_cmd = self._low_cmd.motor_cmd[joint]
            motor_cmd.mode = 1
            motor_cmd.q = filtered_target
            motor_cmd.dq = 0.0
            motor_cmd.kp = WRIST_KP
            motor_cmd.kd = WRIST_KD
            motor_cmd.tau = 0.0

        if not self._reported_packet:
            print(
                "First packet mapped to left wrist:",
                f"roll={self._last_targets[0]:.3f}",
                f"pitch={self._last_targets[1]:.3f}",
                f"yaw={self._last_targets[2]:.3f}",
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

#basically saying main is expected to return int
#'-> int' is actually kinda just like a comment, its not needed
def main() -> int: 
    
    
    if len(sys.argv) >= 2: 
        serial_port = sys.argv[1]
    else:
        serial_port = DEFAULT_SERIAL_PORT

    sim_mode = len(sys.argv) < 3
    network_interface = "lo" if sim_mode else sys.argv[2]
    domain_id = 1 if sim_mode else 0

    ChannelFactoryInitialize(domain_id, network_interface)

    serial_reader = SerialPacketReader(SerialPort(serial_port, DEFAULT_BAUD_RATE))
    bridge = G1LeftWristSerialBridge()

    print(f"Opened serial port {serial_port}")
    print(
        "DDS mode:",
        "sim" if sim_mode else "robot",
        f"domain={domain_id}",
        f"interface={network_interface}",
    )

    bridge.init()
    bridge.wait_for_first_state()

    print("Controlling left wrist joints from potentiometer packets.")
    print("Packet mapping: left[4]->roll, left[5]->pitch, left[6]->yaw")

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

#only run if being called as main, when ctrl+c, interrupts
#goes to except KeyboardInterrupt
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nUSER CANCELLED. bye!")
