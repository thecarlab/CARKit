#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"

cd "$WORKSPACE"

./carkit/setup_vendor_repos.sh

set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
set -u

rosdep update
rosdep install --from-paths carkit --ignore-src -r -y

colcon build --symlink-install --executor sequential

set +u
source install/setup.bash
set -u

ros2 pkg list | grep carkit
