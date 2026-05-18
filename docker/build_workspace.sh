#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"

cd "$WORKSPACE"

./carkit/setup_vendor_repos.sh

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

rosdep update
rosdep install --from-paths carkit --ignore-src -r -y

colcon build --symlink-install --executor sequential

source install/setup.bash

ros2 pkg list | grep carkit
