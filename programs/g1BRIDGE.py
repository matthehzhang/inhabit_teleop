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

#packet header used by the microcontroller / sender so we can
#scan the byte stream and figure out where a packet begins
#.......................................................
#0xAA 0x55 is a super common sync pattern for embedded comms
#because its easy to spot and unlikely to show up by accident
HEADER = b"\xAA\x55"

#14 floats are coming in from serial
#there is also a 16-bit sequence number in the newer packet format
FLOAT_COUNT = 14
PAYLOAD_FORMAT = "<H14f"
PAYLOAD_BYTES = struct.calcsize(PAYLOAD_FORMAT)
PACKET_BYTES = len(HEADER) + PAYLOAD_BYTES + 2

#older packets did not include sequence # or crc
#keep support for them so bridge can still read legacy senders
LEGACY_PAYLOAD_FORMAT = "<14f"
LEGACY_PACKET_BYTES = len(HEADER) + struct.calcsize(LEGACY_PAYLOAD_FORMAT)

#run command loop at 1 kHz
#this keeps the Unitree low-level command stream alive / fresh
COMMAND_DT_SEC = 0.001

#dont print serial diagnostics every loop or terminal becomes useless
PRINT_PERIOD_SEC = 2.0

#give usb serial device a second to enumerate / settle after open
SERIAL_SETTLE_TIME_SEC = 1.0

#these are the sdk's "do not command position / velocity" sentinel values
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0

#same gains used for the 3 left wrist joints when we drive them
WRIST_KP = 24.0
WRIST_KD = 1.6

#simple moving average window
#right now = 1, which means no smoothing, but code is ready for more
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
    #sequence is used to detect stale / repeated packets
    #values holds the 14 float payload from the sender
    #received_at gives us a local timestamp for when bridge got it
    sequence: int
    values: tuple[float, ...]
    received_at: float


def crc16_ccitt(data: bytes) -> int:
    #manual crc16-ccitt implementation for packets that append
    #their own integrity check at the end
    #.......................................................
    #if crc doesnt match, packet was corrupted or we are desynced
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
        #non-blocking serial open
        #timeout=0 means reads return immediately instead of hanging loop
        self._serial = serial.Serial(path, baudrate=baud_rate, timeout=0, write_timeout=0)

        #some boards spam partial boot text / junk right after connect
        #wait a moment, then clear input so parsing starts clean
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
        #keep counts so we can tell if link is healthy or noisy
        valid_packets: int = 0
        legacy_packets: int = 0
        crc_failures: int = 0
        invalid_packets: int = 0
        stale_packets: int = 0
        desync_bytes: int = 0

    def __init__(self, serial_port: SerialPort):
        self._serial_port = serial_port

        #serial is a raw byte stream, not message-oriented
        #so keep our own rolling buffer and carve packets out of it
        self._buffer = bytearray()
        self._stats = self.Stats()
        self._last_sequence: Optional[int] = None
        self._last_print_time = 0.0

    def read_latest_packet(self) -> Optional[Packet]:
        #pull in whatever bytes are available first
        self._read_into_buffer()
        latest = None

        #keep chewing through buffered bytes as long as there is enough
        #data to possibly contain at least a legacy packet
        while len(self._buffer) >= LEGACY_PACKET_BYTES:
            header_index = self._find_header()
            if header_index < 0:
                #no sync word anywhere in buffer
                #count all bytes as garbage and start fresh
                self._stats.desync_bytes += len(self._buffer)
                self._buffer.clear()
                break

            if header_index > 0:
                #found header, but only after junk bytes
                #drop junk and keep parsing from the first real header
                self._stats.desync_bytes += header_index
                del self._buffer[:header_index]

            if len(self._buffer) < LEGACY_PACKET_BYTES:
                #header is present but packet isnt complete yet
                #leave bytes in buffer and wait for next loop iteration
                break

            #try newer packet format first
            #if it fails, fall back to the older format
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
        #keep draining serial until read returns nothing
        #this lets us catch up if sender is producing packets faster than loop
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

        #crc covers payload only, not the 2-byte header
        expected_crc = int.from_bytes(self._buffer[PACKET_BYTES - 2:PACKET_BYTES], "little")
        actual_crc = crc16_ccitt(bytes(self._buffer[2:2 + PAYLOAD_BYTES]))
        if actual_crc != expected_crc:
            #leave bytes in buffer for legacy parser to inspect
            #if sender is actually legacy format, crc check will obviously fail
            self._stats.crc_failures += 1
            return None

        sequence, *values = struct.unpack_from(PAYLOAD_FORMAT, self._buffer, 2)
        del self._buffer[:PACKET_BYTES]
        packet = Packet(sequence=sequence, values=tuple(values), received_at=time.monotonic())
        return self._accept_packet(packet, legacy=False)

    def _try_parse_legacy_packet(self) -> Optional[Packet]:
        #legacy packets dont carry a real sequence #
        #so synthesize one locally so rest of pipeline can stay uniform
        values = struct.unpack_from(LEGACY_PAYLOAD_FORMAT, self._buffer, 2)
        del self._buffer[:LEGACY_PACKET_BYTES]
        sequence = 0 if self._last_sequence is None else (self._last_sequence + 1) & 0xFFFF
        packet = Packet(sequence=sequence, values=tuple(values), received_at=time.monotonic())
        return self._accept_packet(packet, legacy=True)

    def _accept_packet(self, packet: Packet, legacy: bool) -> Optional[Packet]:
        #protect rest of control code from nan / inf poisoning
        if not all(math.isfinite(value) for value in packet.values):
            self._stats.invalid_packets += 1
            return None

        #sequence gate throws away duplicates / old packets
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

        #16-bit wrap-safe delta
        #delta == 0 means exact duplicate
        #delta > 0x8000 means packet moved "backwards", so treat as stale
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

        #periodic health summary for the serial side
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

        #pre-build a full low-level command message for all 35 motors
        #every motor starts in "stopped / neutral" state
        #...................................................
        #then later we only overwrite the 3 left wrist joints we care about
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

        #one history deque per wrist axis
        #packet indices 4,5,6 -> roll,pitch,yaw
        self._input_history = [deque(maxlen=AVERAGE_WINDOW_SIZE) for _ in range(3)]
        self._last_targets = [0.0, 0.0, 0.0]
        self._init_low_cmd()

    def init(self) -> None:
        #publisher sends low-level motor commands to the robot / sim
        self._lowcmd_publisher = ChannelPublisher(CMD_TOPIC, HG_LowCmd)
        self._lowcmd_publisher.Init()

        #subscriber reads current robot state so we can stay synced with mode_machine
        self._lowstate_subscriber = ChannelSubscriber(STATE_TOPIC, HG_LowState)
        self._lowstate_subscriber.Init()

    def wait_for_first_state(self) -> None:
        #dont start streaming commands blindly before DDS state is alive
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

        #cache latest state and mirror robot's current mode_machine
        #this is important because outgoing command should match current control mode
        self._latest_state = message
        self._low_cmd.mode_machine = message.mode_machine
        self._have_state = True

    def apply_packet(self, packet: Packet) -> None:
        #map incoming serial packet values onto left wrist joints
        #packet values[4,5,6] -> roll,pitch,yaw
        for axis, joint in enumerate(LEFT_WRIST_JOINT_INDICES):
            raw_target = packet.values[PACKET_LEFT_WRIST_INDICES[axis]]
            history = self._input_history[axis]
            if not history:
                #seed history so first average is stable instead of under-filled
                history.extend([raw_target] * AVERAGE_WINDOW_SIZE)
            else:
                history.append(raw_target)

            filtered_target = sum(history) / len(history)
            self._last_targets[axis] = raw_target

            motor_cmd = self._low_cmd.motor_cmd[joint]

            #mode=1 puts that motor in servo / position control mode
            #q is target position
            #dq stays zero because we are not commanding a feedforward velocity
            motor_cmd.mode = 1
            motor_cmd.q = filtered_target
            motor_cmd.dq = 0.0
            motor_cmd.kp = WRIST_KP
            motor_cmd.kd = WRIST_KD
            motor_cmd.tau = 0.0

        if not self._reported_packet:
            #one-time print just to confirm packet mapping is sane
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

        #unitree low-level command requires crc before publish
        self._low_cmd.crc = self._crc.Crc(self._low_cmd)
        self._lowcmd_publisher.Write(self._low_cmd)

    def _init_low_cmd(self) -> None:
        #base message config
        #mode_machine gets updated later from the live state subscriber
        self._low_cmd.mode_pr = 0
        self._low_cmd.mode_machine = 0

#basically saying main is expected to return int
#'-> int' is actually kinda just like a comment, its not needed
def main() -> int: 
    #arg layout:
    #python g1BRIDGE.py
    #    -> use default serial port + sim DDS on loopback
    #
    #python g1BRIDGE.py /dev/ttyACM1
    #    -> custom serial port + sim DDS on loopback
    #
    #python g1BRIDGE.py /dev/ttyACM0 enp2s0
    #    -> custom serial port + real robot DDS on NIC enp2s0
    if len(sys.argv) >= 2: 
        serial_port = sys.argv[1]
    else:
        serial_port = DEFAULT_SERIAL_PORT

    #if user didnt provide a network interface, assume sim
    # - sim runs on DDS domain 1 and loopback interface
    #if interface is provided, assume real robot
    # - robot runs on DDS domain 0 and chosen NIC
    sim_mode = len(sys.argv) < 3
    network_interface = "lo" if sim_mode else sys.argv[2]
    domain_id = 1 if sim_mode else 0

    #set up DDS before creating publisher/subscriber objects
    ChannelFactoryInitialize(domain_id, network_interface)

    #serial reader handles framing + validation
    #bridge handles mapping packet values into Unitree lowcmd
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

    #fixed-rate control loop
    #read newest serial packet, update wrist targets, then publish command
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
            #if loop overruns, reset scheduler anchor so lag doesnt accumulate forever
            next_tick = time.monotonic()

#only run if being called as main, when ctrl+c, interrupts
#goes to except KeyboardInterrupt
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nUSER CANCELLED. bye!")
