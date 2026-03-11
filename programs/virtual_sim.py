"""
G1 Arm Teleop - Starter Script
Publishes joint positions to the G1's arms via rt/arm_sdk topic.
Receives joint angle input over UDP from ESP32 or mock_esp32.py.

G1 29-DOF Joint Layout:
  Legs:  0-5 (left leg), 6-11 (right leg)
  Waist: 12-14
  Arms:  15-21 (left arm), 22-28 (right arm)

The rt/arm_sdk topic blends with the locomotion controller:
  actual_cmd = loco_cmd * (1 - weight) + arm_sdk_cmd * weight
  weight is stored in motor_cmd[29].q (range 0.0 to 1.0)
"""

import time
import socket
import struct
import numpy as np
from enum import IntEnum

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


# --- G1 29-DOF Joint Indices ---
class G1Joint(IntEnum):
    LEFT_HIP_PITCH = 0
    LEFT_HIP_ROLL = 1
    LEFT_HIP_YAW = 2
    LEFT_KNEE = 3
    LEFT_ANKLE_PITCH = 4
    LEFT_ANKLE_ROLL = 5
    RIGHT_HIP_PITCH = 6
    RIGHT_HIP_ROLL = 7
    RIGHT_HIP_YAW = 8
    RIGHT_KNEE = 9
    RIGHT_ANKLE_PITCH = 10
    RIGHT_ANKLE_ROLL = 11
    WAIST_YAW = 12
    WAIST_ROLL = 13
    WAIST_PITCH = 14
    LEFT_SHOULDER_PITCH = 15
    LEFT_SHOULDER_ROLL = 16
    LEFT_SHOULDER_YAW = 17
    LEFT_ELBOW = 18
    LEFT_WRIST_ROLL = 19
    LEFT_WRIST_PITCH = 20
    LEFT_WRIST_YAW = 21
    RIGHT_SHOULDER_PITCH = 22
    RIGHT_SHOULDER_ROLL = 23
    RIGHT_SHOULDER_YAW = 24
    RIGHT_ELBOW = 25
    RIGHT_WRIST_ROLL = 26
    RIGHT_WRIST_PITCH = 27
    RIGHT_WRIST_YAW = 28


LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
LEG_JOINTS = list(range(0, 12))
WAIST_JOINTS = list(range(12, 15))

NUM_MOTORS = 35
ARM_SDK_TOPIC = "rt/lowcmd"
LOW_STATE_TOPIC = "rt/lowstate"

UDP_IP = "0.0.0.0"
UDP_PORT = 8888


class G1ArmController:
    def __init__(self):
        self.crc = CRC()
        self.current_state = None

        self.arm_kp = 60.0
        self.arm_kd = 3.0
        self.leg_kp = 200.0
        self.leg_kd = 5.0
        self.waist_kp = 100.0
        self.waist_kd = 3.0
        self.weight = 0.0
        self.leg_hold_positions = None

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

    def capture_leg_hold(self):
        """Snapshot current leg/waist positions to hold."""
        if self.current_state is None:
            return
        self.leg_hold_positions = [
            self.current_state.motor_state[j].q for j in LEG_JOINTS + WAIST_JOINTS
        ]

    def set_leg_targets(self):
        """Apply PD hold for leg/waist joints at captured positions."""
        if self.leg_hold_positions is None:
            return
        for i, joint_idx in enumerate(LEG_JOINTS):
            self.cmd_msg.motor_cmd[joint_idx].q = self.leg_hold_positions[i]
            self.cmd_msg.motor_cmd[joint_idx].kp = self.leg_kp
            self.cmd_msg.motor_cmd[joint_idx].kd = self.leg_kd
        for i, joint_idx in enumerate(WAIST_JOINTS):
            self.cmd_msg.motor_cmd[joint_idx].q = self.leg_hold_positions[len(LEG_JOINTS) + i]
            self.cmd_msg.motor_cmd[joint_idx].kp = self.waist_kp
            self.cmd_msg.motor_cmd[joint_idx].kd = self.waist_kd

    def set_arm_targets(self, left_targets, right_targets):
        for i, joint_idx in enumerate(LEFT_ARM_JOINTS):
            self.cmd_msg.motor_cmd[joint_idx].q = left_targets[i]
            self.cmd_msg.motor_cmd[joint_idx].kp = self.arm_kp
            self.cmd_msg.motor_cmd[joint_idx].kd = self.arm_kd

        for i, joint_idx in enumerate(RIGHT_ARM_JOINTS):
            self.cmd_msg.motor_cmd[joint_idx].q = right_targets[i]
            self.cmd_msg.motor_cmd[joint_idx].kp = self.arm_kp
            self.cmd_msg.motor_cmd[joint_idx].kd = self.arm_kd

        # self.cmd_msg.motor_cmd[29].q = self.weight

    def publish(self, publisher):
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        publisher.Write(self.cmd_msg)


def setup_udp():
    """Create non-blocking UDP socket to receive ESP32 data."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)
    return sock


def read_udp(sock):
    """
    Read latest UDP packet. Returns (left_arm, right_arm) or (None, None).
    Expects 14 floats: 7 left arm + 7 right arm, little-endian.
    """
    data = None
    # Drain the socket buffer, keep only the latest packet
    while True:
        try:
            data, _ = sock.recvfrom(256)
        except BlockingIOError:
            break

    if data is not None and len(data) == struct.calcsize('<14f'):
        values = struct.unpack('<14f', data)
        left = list(values[:7])
        right = list(values[7:])
        return left, right

    return None, None


def main():
    # For simulation: domain_id=1, interface="lo"
    # For real robot: domain_id=0, interface="your_ethernet_iface"
    ChannelFactoryInitialize(1, "lo")

    arm_pub = ChannelPublisher(ARM_SDK_TOPIC, HG_LowCmd)
    arm_pub.Init()

    state_sub = ChannelSubscriber(LOW_STATE_TOPIC, HG_LowState)
    state_sub.Init()

    udp_sock = setup_udp()
    print(f"Listening for ESP32 input on UDP port {UDP_PORT}")

    controller = G1ArmController()

    # Wait for first state message from sim
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

    # Snapshot leg positions to hold throughout
    controller.capture_leg_hold()

    # Start with current positions as targets
    left_target = list(left_init)
    right_target = list(right_init)

    # Ramp up weight smoothly
    print("Ramping up arm control weight...")
    for w in np.linspace(0.0, 1.0, 50):
        controller.weight = w
        controller.update_state(state_sub)
        controller.set_leg_targets()
        controller.set_arm_targets(left_target, right_target)
        controller.publish(arm_pub)
        time.sleep(0.02)

    print("Arm control active! weight=1.0")
    print("Waiting for UDP input from ESP32 / mock_esp32.py ...")
    print("Press Ctrl+C to stop.")

    # Main control loop
    try:
        dt = 0.02  # 50 Hz
        while True:
            controller.update_state(state_sub)

            # Read teleop input from UDP
            udp_left, udp_right = read_udp(udp_sock)
            if udp_left is not None:
                # Add UDP offsets to initial positions
                left_target = [init + offset for init, offset in zip(left_init, udp_left)]
                right_target = [init + offset for init, offset in zip(right_init, udp_right)]

            controller.set_leg_targets()
            controller.set_arm_targets(left_target, right_target)
            controller.publish(arm_pub)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nShutting down, ramping weight to 0...")
        for w in np.linspace(1.0, 0.0, 50):
            controller.weight = w
            controller.set_leg_targets()
            controller.set_arm_targets(left_target, right_target)
            controller.publish(arm_pub)
            time.sleep(0.02)
        print("Done.")
    finally:
        udp_sock.close()


if __name__ == "__main__":
    main()
