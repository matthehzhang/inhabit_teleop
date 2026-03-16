#!/usr/bin/env python3

import math
import struct
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc


HEADER = b"\xAA\x55"
FLOAT_COUNT = 24
PAYLOAD_FORMAT = "<H24f"
PAYLOAD_BYTES = struct.calcsize(PAYLOAD_FORMAT)
PACKET_BYTES = len(HEADER) + PAYLOAD_BYTES + 2
EXPECTED_MIN = -1.6
EXPECTED_MAX = 1.6


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


def format_changed(values: tuple[float, ...], threshold: float) -> str:
    parts = []
    for idx, value in enumerate(values):
        if abs(value) >= threshold:
            parts.append(f"{value:6.3f}")
        else:
            parts.append("     -")
    return " ".join(parts)


def format_all(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:6.3f}" for value in values)


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    threshold = float(sys.argv[3]) if len(sys.argv) > 3 else 0.02
    show_all = (len(sys.argv) > 4 and sys.argv[4].lower() == "all")

    ser = serial.Serial(port, baudrate=baud, timeout=0)
    ser.dtr = True
    ser.rts = True
    time.sleep(1.0)
    ser.reset_input_buffer()

    buffer = bytearray()
    valid_packets = 0
    crc_failures = 0
    invalid_packets = 0
    desync_bytes = 0
    last_status = time.monotonic()

    print(f"Reading {port} at {baud} baud, threshold={threshold:.3f}")
    print("0-7=chip0, 8-15=chip1, 16-23=chip2")
    header = "  seq  " + " ".join(f"  ch{i:02d}" for i in range(FLOAT_COUNT))
    print(header)
    print("-" * len(header))

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 256)
            if chunk:
                buffer.extend(chunk)

            latest = None
            while len(buffer) >= PACKET_BYTES:
                header_index = buffer.find(HEADER)
                if header_index < 0:
                    desync_bytes += len(buffer)
                    buffer.clear()
                    break

                if header_index > 0:
                    desync_bytes += header_index
                    del buffer[:header_index]

                if len(buffer) < PACKET_BYTES:
                    break

                expected_crc = int.from_bytes(buffer[PACKET_BYTES - 2:PACKET_BYTES], "little")
                actual_crc = crc16_ccitt(bytes(buffer[2:2 + PAYLOAD_BYTES]))
                if actual_crc != expected_crc:
                    crc_failures += 1
                    del buffer[:1]
                    continue

                sequence, *values = struct.unpack_from(PAYLOAD_FORMAT, buffer, 2)
                del buffer[:PACKET_BYTES]

                if not all(math.isfinite(value) for value in values):
                    invalid_packets += 1
                    continue

                latest = (sequence, tuple(values))
                valid_packets += 1

            if latest is not None:
                sequence, values = latest
                out_of_range = any(value < EXPECTED_MIN or value > EXPECTED_MAX for value in values)
                marker = " !" if out_of_range else ""
                formatted = format_all(values) if show_all else format_changed(values, threshold)
                print(f"{sequence:5d}{marker} {formatted}")

            now = time.monotonic()
            if now - last_status >= 2.0:
                print(
                    "stats",
                    f"valid={valid_packets}",
                    f"crc_fail={crc_failures}",
                    f"invalid={invalid_packets}",
                    f"desync={desync_bytes}",
                )
                last_status = now
    except KeyboardInterrupt:
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
