# Local C++ Unitree SDK Setup

This document explains the intended local C++ SDK layout inside this repo.

Paths:

- source checkout: `third_party/unitree_sdk2`
- local install prefix: `.local/unitree_robotics`
- program build dir: `programs/build`

## Intended setup flow

From the repo root:

```bash
cd /path/to/inhabit_teleop
./tools/setup_unitree_sdk2_local.sh
./tools/build_g1_left_wrist_serial.sh
```

## Current status

As of March 13, 2026:

- `tools/setup_unitree_sdk2_local.sh` is conceptually correct, but existing build caches created under the old `luke_unitree` path must be removed before reusing the same checkout
- `tools/build_g1_left_wrist_serial.sh` is blocked by the current `programs/CMakeLists.txt`, which references `programs/g1_left_wrist_serial.cpp`
- that source file is not present in the current working tree, so the C++ program does not configure from a clean checkout

## What to fix before relying on this flow

- restore or commit the intended `programs/g1_left_wrist_serial.cpp`
- or update `programs/CMakeLists.txt` to use the actual C++ source file you want to ship
- if you previously renamed the repo directory, remove stale CMake cache directories under `third_party/unitree_sdk2/build` and `programs/build`
