# 1. Install Python 3.12 (CycloneDDS doesn't build on 3.14)
# Ubuntu/Debian:
sudo apt install python3.12 python3.12-venv
# Arch:
yay -S python312

# 2. Clone the project
git clone <your_repo_url> luke_unitree
cd luke_unitree

# 3. Create venv
python3.12 -m venv .venv
source .venv/bin/activate

# 4. Build CycloneDDS
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
cd ../..

# 5. Install Unitree Python SDK
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
export CYCLONEDDS_HOME=$(pwd)/../cyclonedds/install
pip install -e .
cd ..

# 6. Install MuJoCo and the sim
pip install mujoco pygame pyserial numpy
git clone https://github.com/unitreerobotics/unitree_mujoco.git

# 7. Add CYCLONEDDS_HOME to venv activate script
echo "export CYCLONEDDS_HOME=$(pwd)/cyclonedds/install" >> .venv/bin/activate

# 8. Verify
python -c "import unitree_sdk2py; print('SDK OK')"
python -c "import mujoco; print('MuJoCo OK')"
