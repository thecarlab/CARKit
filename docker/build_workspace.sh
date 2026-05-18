#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-librealsense2}"
ROSDEP_SKIP_ARGS=()
if [ -n "${ROSDEP_SKIP_KEYS}" ]; then
  read -r -a ROSDEP_SKIP_ARGS <<< "${ROSDEP_SKIP_KEYS}"
fi

cd "$WORKSPACE"

./carkit/setup_vendor_repos.sh

set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
set -u

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

colcon build --symlink-install --executor sequential

set +u
source install/setup.bash
set -u

ros2 pkg list | grep carkit
