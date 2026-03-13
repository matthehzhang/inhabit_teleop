# inhabit_teleop on Windows

This project should be run on Windows through Ubuntu, not through native Windows Python.

Use one of these:

- WSL2 with Ubuntu 24.04 or 22.04
- an Ubuntu virtual machine

WSL2 is the better option unless you specifically need USB passthrough behavior from a VM.

## Recommended path: WSL2 + Ubuntu

### 1. Install WSL2 and Ubuntu

Open PowerShell as Administrator and run:

```powershell
wsl --install -d Ubuntu-24.04
```

Then reboot if Windows asks you to.

Open Ubuntu and create your Linux username/password.

### 2. Install Linux dependencies inside Ubuntu

Run these commands inside the Ubuntu terminal:

```bash
sudo apt update
sudo apt install git cmake build-essential python3.12 python3.12-venv python3.12-tk usbutils
```

### 3. Clone the repo

Inside Ubuntu:

```bash
git clone https://github.com/matthehzhang/inhabit_teleop.git
cd inhabit_teleop
```

### 4. Create the virtual environment

```bash
python3.12 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

### 5. Install Python packages

```bash
pip install numpy pyserial customtkinter mujoco pygame opencv-python PyOpenGL
```

### 6. Build CycloneDDS

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install -j"$(nproc)"
cd ../..
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"
```

To keep that environment variable after re-opening the terminal:

```bash
echo 'export CYCLONEDDS_HOME="'"$(pwd)"'/cyclonedds/install"' >> .venv/bin/activate
```

### 7. Install the Unitree Python SDK

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
pip install -e unitree_sdk2_python/
```

### 8. Optional: install Unitree MuJoCo

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

### 9. Verify the setup

```bash
source .venv/bin/activate
python -c "import unitree_sdk2py; print('unitree_sdk2py OK')"
python -c "import serial; print('pyserial OK')"
python -c "import customtkinter; print('customtkinter OK')"
python -c "import mujoco; print('mujoco OK')"
```

## Running the main tools

Activate the environment first:

```bash
cd ~/inhabit_teleop
source .venv/bin/activate
```

Run the GUI:

```bash
python programs/g1_unified_studio.py
```

Run the standalone config editor:

```bash
python programs/g1_joint_config_ui.py
```

Run the standalone virtual pot simulator:

```bash
python programs/g1_virtual_pot_sim.py programs/uitest1.g1config.json
```

Run the serial bridge:

```bash
python programs/run_g1_bridge.py programs/g1_left_wrist_bridge_config.py /dev/ttyACM0 <network_interface>
```

For the 20-channel variant:

```bash
python programs/run_g1_bridge_20ch.py programs/g1_17pot_bridge_config.py /dev/ttyACM0 <network_interface>
```

Replace `<network_interface>` with the Ubuntu network interface name, such as `eth0`, `enp2s0`, or similar.

Find interfaces with:

```bash
ip link
```

## USB serial notes for Windows users

If the ESP32 is plugged into your Windows machine, Ubuntu in WSL2 may not see it automatically.

For WSL2 USB passthrough, use Microsoft's `usbipd-win` tooling on Windows. If USB passthrough becomes annoying, an Ubuntu VM with direct USB attachment may be simpler for serial-device work.

After passthrough, check for the serial device in Ubuntu:

```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

If you do not see a serial device, the bridge command will not work yet.

## What is not supported

- native Windows Python
- PowerShell-based setup instead of Ubuntu
- COM-port names like `COM3` in the documented commands

Use Linux device names from Ubuntu, such as `/dev/ttyACM0`.
