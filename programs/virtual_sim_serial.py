"""
G1 Arm Teleop - Bridge Script
Reads joint angles from ESP32 over USB serial, publishes to G1 via DDS.

Serial protocol: 2-byte header (0xAA 0x55) + uint16 sequence +
14 little-endian floats (56 bytes) + uint16 CRC16-CCITT.
"""

import binascii
import time
import struct
import sys
import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_ as HG_LowCmd,
    LowState_ as HG_LowState,
    MotorCmd_ as HG_MotorCmd,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

try:
    import serial
except ImportError:
    print("pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
ACTIVE_LEFT_ARM_INDICES = [4, 5, 6]
ACTIVE_RIGHT_ARM_INDICES = []

NUM_MOTORS = 35
CMD_TOPIC = "rt/lowcmd"       # use "rt/arm_sdk" for real robot
STATE_TOPIC = "rt/lowstate"

# Serial config
SERIAL_PORT = "/dev/ttyACM0"  # change if different
SERIAL_BAUD = 115200
HEADER = bytes([0xAA, 0x55])
FLOAT_COUNT = 14
PAYLOAD_FORMAT = "<H14f"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)
CRC_SIZE = 2
PACKET_SIZE = len(HEADER) + PAYLOAD_SIZE + CRC_SIZE
LEGACY_FLOAT_FORMAT = "<14f"
LEGACY_PAYLOAD_SIZE = struct.calcsize(LEGACY_FLOAT_FORMAT)
LEGACY_PACKET_SIZE = len(HEADER) + LEGACY_PAYLOAD_SIZE

MAX_JOINT_OFFSET = 1.5
MAX_PACKET_ABS_VALUE = 1.75
OFFSET_DEADBAND = 0.01
LEFT_WRIST_INPUT_SCALE = {
    4: 1.8,  # roll
    5: 1.8,  # pitch
    6: 1.8,  # yaw
}
LEFT_WRIST_TARGET_LIMITS = {
    4: (-1.2, 1.2),   # roll
    5: (-1.0, 1.0),   # pitch
    6: (-1.0, 1.0),   # yaw
}
TARGET_BLEND = 0.35
TARGET_JITTER_EPS = 0.02
SERIAL_SETTLE_TIME_SEC = 1.0
SERIAL_READ_CHUNK = 256
STATUS_PRINT_PERIOD_SEC = 2.0


class G1ArmController:
    def __init__(self):
        self.crc = CRC()
        self.current_state = None
        self.arm_kp = 10.0
        self.arm_kd = 2.5

        self.cmd_msg = HG_LowCmd(
            mode_pr=0,
            mode_machine=0,
            motor_cmd=[
                HG_MotorCmd(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=0)
                for _ in range(NUM_MOTORS)
            ],
            reserve=[0, 0, 0, 0],
            crc=0,
        )

    def update_state(self, state_sub):
        msg = state_sub.Read()
        if msg is not None:
            self.current_state = msg

    def get_arm_positions(self):
        if self.current_state is None:
            return None, None
        left = [self.current_state.motor_state[j].q for j in LEFT_ARM_JOINTS]
        right = [self.current_state.motor_state[j].q for j in RIGHT_ARM_JOINTS]
        return left, right

    def set_arm_targets(self, left_targets, right_targets):
        for i, joint_idx in enumerate(LEFT_ARM_JOINTS):
            if i in ACTIVE_LEFT_ARM_INDICES:
                self.cmd_msg.motor_cmd[joint_idx].q = left_targets[i]
                self.cmd_msg.motor_cmd[joint_idx].kp = self.arm_kp
                self.cmd_msg.motor_cmd[joint_idx].kd = self.arm_kd
            else:
                self.cmd_msg.motor_cmd[joint_idx].kp = 0.0
                self.cmd_msg.motor_cmd[joint_idx].kd = 0.0
                self.cmd_msg.motor_cmd[joint_idx].tau = 0.0

        for i, joint_idx in enumerate(RIGHT_ARM_JOINTS):
            if i in ACTIVE_RIGHT_ARM_INDICES:
                self.cmd_msg.motor_cmd[joint_idx].q = right_targets[i]
                self.cmd_msg.motor_cmd[joint_idx].kp = self.arm_kp
                self.cmd_msg.motor_cmd[joint_idx].kd = self.arm_kd
            else:
                self.cmd_msg.motor_cmd[joint_idx].kp = 0.0
                self.cmd_msg.motor_cmd[joint_idx].kd = 0.0
                self.cmd_msg.motor_cmd[joint_idx].tau = 0.0

    def publish(self, publisher):
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        publisher.Write(self.cmd_msg)


def open_serial(port, baud):
    """Open serial port with timeout."""
    ser = serial.Serial(port, baud, timeout=0.01)
    ser.reset_input_buffer()
    return ser


def crc16_ccitt(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clamp_offsets(offsets):
    clamped = []
    for index, value in enumerate(offsets):
        scale = LEFT_WRIST_INPUT_SCALE.get(index, 1.0)
        limited = clamp(value * scale, -MAX_JOINT_OFFSET, MAX_JOINT_OFFSET)
        if abs(limited) < OFFSET_DEADBAND:
            limited = 0.0
        clamped.append(limited)
    return clamped


def blend_target(previous, desired, alpha):
    return previous + alpha * (desired - previous)


def build_safe_left_targets(left_init, desired_left_offsets, previous_left_targets):
    targets = list(previous_left_targets)

    for joint_index in ACTIVE_LEFT_ARM_INDICES:
        desired_q = left_init[joint_index] + desired_left_offsets[joint_index]
        lower, upper = LEFT_WRIST_TARGET_LIMITS[joint_index]
        desired_q = clamp(desired_q, lower, upper)

        if abs(desired_q - previous_left_targets[joint_index]) < TARGET_JITTER_EPS:
            desired_q = previous_left_targets[joint_index]

        targets[joint_index] = blend_target(previous_left_targets[joint_index], desired_q, TARGET_BLEND)

    return targets


class SerialPacketReader:
    def __init__(self, ser):
        self.ser = ser
        self.buffer = bytearray()
        self.last_sequence = None
        self.stats = {
            "valid": 0,
            "legacy_valid": 0,
            "crc_fail": 0,
            "invalid_values": 0,
            "stale_seq": 0,
            "desync_bytes": 0,
        }
        self.last_status_print = time.monotonic()
        self.reported_packet = False

    def _read_into_buffer(self):
        available = max(self.ser.in_waiting, 1)
        chunk = self.ser.read(min(available, SERIAL_READ_CHUNK))
        if chunk:
            self.buffer.extend(chunk)

    def _print_status_if_needed(self):
        now = time.monotonic()
        if now - self.last_status_print >= STATUS_PRINT_PERIOD_SEC:
            print(
                "serial stats:",
                f"valid={self.stats['valid']}",
                f"legacy={self.stats['legacy_valid']}",
                f"crc_fail={self.stats['crc_fail']}",
                f"invalid={self.stats['invalid_values']}",
                f"stale={self.stats['stale_seq']}",
                f"desync={self.stats['desync_bytes']}",
            )
            self.last_status_print = now

    def _validate_values(self, values):
        return np.isfinite(values).all() and not any(abs(value) > MAX_PACKET_ABS_VALUE for value in values)

    def _record_valid_packet(self, sequence, values, legacy=False):
        if not self._validate_values(values):
            self.stats["invalid_values"] += 1
            return None

        left = clamp_offsets(values[:7])
        right = clamp_offsets(values[7:])

        if legacy:
            self.stats["legacy_valid"] += 1
        else:
            self.stats["valid"] += 1

        if not self.reported_packet:
            packet_type = "legacy" if legacy else "crc"
            print(
                f"Received first {packet_type} packet:",
                f"left_wrist=[roll={left[4]:+.3f}, pitch={left[5]:+.3f}, yaw={left[6]:+.3f}]",
            )
            self.reported_packet = True

        return (sequence, left, right)

    def _sequence_is_new(self, sequence):
        if self.last_sequence is None:
            self.last_sequence = sequence
            return True

        delta = (sequence - self.last_sequence) & 0xFFFF
        if delta == 0 or delta > 0x8000:
            self.stats["stale_seq"] += 1
            return False

        self.last_sequence = sequence
        return True

    def _parse_buffer(self):
        latest = None

        while len(self.buffer) >= min(PACKET_SIZE, LEGACY_PACKET_SIZE):
            header_index = self.buffer.find(HEADER)
            if header_index < 0:
                self.stats["desync_bytes"] += len(self.buffer)
                self.buffer.clear()
                break

            if header_index > 0:
                self.stats["desync_bytes"] += header_index
                del self.buffer[:header_index]

            if len(self.buffer) < LEGACY_PACKET_SIZE:
                break

            if len(self.buffer) >= PACKET_SIZE:
                packet = bytes(self.buffer[:PACKET_SIZE])
                payload = packet[len(HEADER):-CRC_SIZE]
                packet_crc = struct.unpack("<H", packet[-CRC_SIZE:])[0]

                if crc16_ccitt(payload) == packet_crc:
                    sequence, *values = struct.unpack(PAYLOAD_FORMAT, payload)
                    del self.buffer[:PACKET_SIZE]

                    if not self._sequence_is_new(sequence):
                        continue

                    latest = self._record_valid_packet(sequence, values, legacy=False)
                    continue

                self.stats["crc_fail"] += 1

            if len(self.buffer) < LEGACY_PACKET_SIZE:
                break

            legacy_packet = bytes(self.buffer[:LEGACY_PACKET_SIZE])
            legacy_payload = legacy_packet[len(HEADER):]
            try:
                values = struct.unpack(LEGACY_FLOAT_FORMAT, legacy_payload)
            except struct.error:
                del self.buffer[0]
                continue

            del self.buffer[:LEGACY_PACKET_SIZE]
            sequence = (self.last_sequence + 1) & 0xFFFF if self.last_sequence is not None else 0
            latest = self._record_valid_packet(sequence, values, legacy=True)

        return latest

    def read_latest_packet(self):
        self._read_into_buffer()
        packet = self._parse_buffer()
        self._print_status_if_needed()
        return packet


def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = SERIAL_PORT

    # Open serial
    print(f"Opening serial port {port}...")
    try:
        ser = open_serial(port, SERIAL_BAUD)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        print("Usage: python virtual_sim.py [serial_port]")
        print("Example: python virtual_sim.py /dev/ttyACM0")
        sys.exit(1)
    print("Serial connected!")
    print(f"Discarding startup serial output for {SERIAL_SETTLE_TIME_SEC:.1f}s...")
    time.sleep(SERIAL_SETTLE_TIME_SEC)
    ser.reset_input_buffer()

    # DDS setup
    # For simulation: domain_id=1, interface="lo"
    # For real robot: domain_id=0, interface="your_ethernet_iface"
    ChannelFactoryInitialize(1, "lo")

    cmd_pub = ChannelPublisher(CMD_TOPIC, HG_LowCmd)
    cmd_pub.Init()

    state_sub = ChannelSubscriber(STATE_TOPIC, HG_LowState)
    state_sub.Init()

    controller = G1ArmController()

    # Wait for robot state from sim
    print("Waiting for robot state...")
    while True:
        controller.update_state(state_sub)
        if controller.current_state is not None:
            break
        time.sleep(0.1)
    print("Got robot state!")

    left_init, right_init = controller.get_arm_positions()
    print(f"Left arm initial:  {[f'{q:.3f}' for q in left_init]}")
    print(f"Right arm initial: {[f'{q:.3f}' for q in right_init]}")

    left_target = list(left_init)
    right_target = list(right_init)
    desired_left_offsets = [0.0] * 7
    desired_right_offsets = [0.0] * 7
    serial_reader = SerialPacketReader(ser)

    print("Arm control active!")
    print("Reading from ESP32... Press Ctrl+C to stop.")

    # Main loop
    try:
        dt = 0.01  # 100 Hz
        while True:
            controller.update_state(state_sub)

            packet = serial_reader.read_latest_packet()
            if packet is not None:
                _, desired_left_offsets, desired_right_offsets = packet

            left_target = build_safe_left_targets(left_init, desired_left_offsets, left_target)
            right_target = list(right_init)

            controller.set_arm_targets(left_target, right_target)
            controller.publish(cmd_pub)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
