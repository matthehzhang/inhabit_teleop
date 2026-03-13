#!/usr/bin/env python3

import sys
from pathlib import Path

from g1_bridge_lib_20ch import run_bridge_from_config_file


def main() -> int:
    if len(sys.argv) < 2:
        raise SystemExit(
            "Usage: python run_g1_bridge_20ch.py <config.py> [serial_port] [network_interface]"
        )

    config_path = Path(sys.argv[1]).resolve()
    return run_bridge_from_config_file(config_path, argv=sys.argv[2:])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nUSER CANCELLED. bye!")
