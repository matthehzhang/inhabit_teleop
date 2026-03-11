#!/usr/bin/env bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX_DIR="${ROOT_DIR}/.local/unitree_robotics"

export CMAKE_PREFIX_PATH="${PREFIX_DIR}:${PREFIX_DIR}/lib/cmake${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${PREFIX_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

echo "Using project-local unitree_sdk2 from ${PREFIX_DIR}"
