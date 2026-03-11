#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/programs/build"

if [[ ! -d "${ROOT_DIR}/.local/unitree_robotics" ]]; then
  echo "Missing local unitree_sdk2 install at ${ROOT_DIR}/.local/unitree_robotics" >&2
  echo "Run tools/setup_unitree_sdk2_local.sh first." >&2
  exit 1
fi

source "${ROOT_DIR}/tools/env_unitree_sdk2_local.sh"

cmake -S "${ROOT_DIR}/programs" -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" --target g1_left_wrist_serial -j"$(nproc)"

echo
echo "Built:"
echo "  ${BUILD_DIR}/g1_left_wrist_serial"
