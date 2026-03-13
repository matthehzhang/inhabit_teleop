#!/usr/bin/env python3

from g1_bridge_lib import run_bridge
from g1_left_wrist_bridge_config import BRIDGE_CONFIG


def main() -> int:
    return run_bridge(BRIDGE_CONFIG)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nUSER CANCELLED. bye!")
