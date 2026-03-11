#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${ROOT_DIR}/third_party/unitree_sdk2"
BUILD_DIR="${SRC_DIR}/build"
PREFIX_DIR="${ROOT_DIR}/.local/unitree_robotics"

echo "Root:   ${ROOT_DIR}"
echo "Source: ${SRC_DIR}"
echo "Build:  ${BUILD_DIR}"
echo "Prefix: ${PREFIX_DIR}"

mkdir -p "${ROOT_DIR}/third_party"
mkdir -p "${ROOT_DIR}/.local"

if [[ ! -d "${SRC_DIR}/.git" ]]; then
  git clone https://github.com/unitreerobotics/unitree_sdk2.git "${SRC_DIR}"
else
  echo "Using existing checkout at ${SRC_DIR}"
fi

cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_INSTALL_PREFIX="${PREFIX_DIR}" \
  -DBUILD_EXAMPLES=OFF
cmake --install "${BUILD_DIR}"

echo
echo "Local unitree_sdk2 install complete."
echo "Source tree: ${SRC_DIR}"
echo "Install tree: ${PREFIX_DIR}"
echo
echo "Next:"
echo "  source \"${ROOT_DIR}/tools/env_unitree_sdk2_local.sh\""
echo "  cmake -S \"${ROOT_DIR}/programs\" -B \"${ROOT_DIR}/programs/build\""
echo "  cmake --build \"${ROOT_DIR}/programs/build\" --target g1_left_wrist_serial"
