"""
Mock ESP32 Input Sender
Simulates potentiometer joint angle data being sent over UDP,
mimicking what the real ESP32-S3 wearable controller would do.

Controls:
  WASD   - left shoulder pitch/roll
  IJKL   - right shoulder pitch/roll
  Q/E    - left elbow flex/extend
  U/O    - right elbow flex/extend
  R      - reset all joints to zero
  ESC    - quit

Joint angles are sent as 14 floats (7 per arm) packed as little-endian.
The virtual_sim.py script needs to be modified to receive these via UDP.
"""

import socket
import struct
import time
import sys
import select
import termios
import tty


UDP_IP = "127.0.0.1"
UDP_PORT = 8888
SEND_RATE = 50  # Hz


def get_key(timeout=0.02):
    """Non-blocking key read from terminal."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 7 joints per arm: shoulder pitch, roll, yaw, elbow, wrist yaw, roll, pitch
    left_arm = [0.0] * 7
    right_arm = [0.0] * 7

    step = 0.05  # radians per keypress

    print("Mock ESP32 Input Sender")
    print("=======================")
    print(f"Sending UDP to {UDP_IP}:{UDP_PORT} at {SEND_RATE}Hz")
    print()
    print("Controls:")
    print("  W/S  - left shoulder pitch up/down")
    print("  A/D  - left shoulder roll left/right")
    print("  Q/E  - left elbow flex/extend")
    print("  I/K  - right shoulder pitch up/down")
    print("  J/L  - right shoulder roll left/right")
    print("  U/O  - right elbow flex/extend")
    print("  R    - reset all to zero")
    print("  ESC  - quit")
    print()

    try:
        while True:
            key = get_key(timeout=1.0 / SEND_RATE)

            if key:
                k = key.lower()

                # Left arm
                if k == 'w':
                    left_arm[0] += step
                elif k == 's':
                    left_arm[0] -= step
                elif k == 'a':
                    left_arm[1] += step
                elif k == 'd':
                    left_arm[1] -= step
                elif k == 'q':
                    left_arm[3] += step
                elif k == 'e':
                    left_arm[3] -= step

                # Right arm
                elif k == 'i':
                    right_arm[0] += step
                elif k == 'k':
                    right_arm[0] -= step
                elif k == 'j':
                    right_arm[1] += step
                elif k == 'l':
                    right_arm[1] -= step
                elif k == 'u':
                    right_arm[3] += step
                elif k == 'o':
                    right_arm[3] -= step

                # Reset
                elif k == 'r':
                    left_arm = [0.0] * 7
                    right_arm = [0.0] * 7

                # Quit
                elif key == '\x1b':
                    break

            # Pack and send: 14 floats (7 left + 7 right)
            data = struct.pack('<14f', *left_arm, *right_arm)
            sock.sendto(data, (UDP_IP, UDP_PORT))

            # Print current state
            l_str = ' '.join(f'{a:+.2f}' for a in left_arm)
            r_str = ' '.join(f'{a:+.2f}' for a in right_arm)
            sys.stdout.write(f'\rL:[{l_str}]  R:[{r_str}]')
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print("\nStopped.")


if __name__ == "__main__":
    main()
