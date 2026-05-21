#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-librealsense2}"
BUILD_JOBS="${BUILD_JOBS:-1}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-1}"
ROSDEP_SKIP_ARGS=()
if [ -n "${ROSDEP_SKIP_KEYS}" ]; then
  read -r -a ROSDEP_SKIP_ARGS <<< "${ROSDEP_SKIP_KEYS}"
fi
export MAKEFLAGS="-j${BUILD_JOBS} -l${BUILD_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"
export NINJAFLAGS="-j${BUILD_JOBS}"

cd "$WORKSPACE"

./carkit/setup_vendor_repos.sh

set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
set -u

if [ "$(id -u)" -eq 0 ]; then
  ldconfig
fi

if ! ldconfig -p 2>/dev/null | grep -q 'librealsense2\.so' \
  && [ ! -e /usr/local/lib/librealsense2.so ] \
  && [ ! -f /usr/local/lib/cmake/realsense2/realsense2Config.cmake ]; then
  printf '%s\n' \
    "CARKit error: librealsense2 SDK is not installed in this Docker image." \
    "Expected one of:" \
    "  /usr/local/lib/librealsense2.so" \
    "  /usr/local/lib/cmake/realsense2/realsense2Config.cmake" \
    "Rebuild the image with the current docker/Dockerfile.jetson, then rerun this script:" \
    "  docker build -f docker/Dockerfile.jetson -t ariiees/carkit:latest ." >&2
  exit 1
fi

if [ "$(id -u)" -eq 0 ]; then
  apt-get update
fi

rosdep update
ROSDEP_INSTALL_CMD=(
  rosdep install --from-paths carkit --ignore-src -r -y
  --dependency-types build_export --dependency-types buildtool_export
  --dependency-types buildtool --dependency-types build --dependency-types exec
)
if [ "${#ROSDEP_SKIP_ARGS[@]}" -gt 0 ]; then
  ROSDEP_INSTALL_CMD+=(--skip-keys "${ROSDEP_SKIP_ARGS[@]}")
fi
"${ROSDEP_INSTALL_CMD[@]}"

colcon build --symlink-install \
  --executor sequential \
  --parallel-workers "${PARALLEL_WORKERS}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

set +u
source install/setup.bash
set -u

ros2 pkg list | grep carkit
