# Local C++ Unitree SDK Setup

This keeps the native C++ Unitree SDK inside this repo so it is easy to remove later.

Paths:

- source checkout: `third_party/unitree_sdk2`
- local install prefix: `.local/unitree_robotics`
- program build dir: `programs/build`

## Install the SDK locally

```bash
cd /home/matthew/projects/luke_unitree
./tools/setup_unitree_sdk2_local.sh
```

This installs the prebuilt Unitree SDK library and headers only. It does not build the upstream SDK example programs.

## Build the local C++ program

```bash
cd /home/matthew/projects/luke_unitree
./tools/build_g1_left_wrist_serial.sh
```

## Run in sim

```bash
/home/matthew/projects/luke_unitree/programs/build/g1_left_wrist_serial /dev/ttyACM0
```

## Run on robot

```bash
/home/matthew/projects/luke_unitree/programs/build/g1_left_wrist_serial /dev/ttyACM0 enp2s0
```

## Remove everything

```bash
rm -rf /home/matthew/projects/luke_unitree/third_party/unitree_sdk2
rm -rf /home/matthew/projects/luke_unitree/.local/unitree_robotics
rm -rf /home/matthew/projects/luke_unitree/programs/build
```
