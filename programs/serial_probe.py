import sys
import time

try:
    import serial
except ImportError:
    print("pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


HEADER = b"\xAA\x55"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Opening {port} at {baud} baud")
    ser = serial.Serial(port, baud, timeout=0.2)
    ser.reset_input_buffer()

    total_bytes = 0
    header_hits = 0
    start = time.monotonic()

    try:
        while True:
            chunk = ser.read(256)
            now = time.monotonic()

            if chunk:
                total_bytes += len(chunk)
                header_hits += chunk.count(HEADER)
                preview = " ".join(f"{byte:02x}" for byte in chunk[:16])
                print(
                    f"{now - start:6.2f}s bytes={len(chunk):3d} total={total_bytes:6d} "
                    f"headers={header_hits:4d} preview={preview}"
                )
            elif now - start >= 2.0:
                print(f"{now - start:6.2f}s no serial data received")
                start = now
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
