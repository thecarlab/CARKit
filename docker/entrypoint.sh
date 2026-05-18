#!/usr/bin/env bash

set -e

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

if [ -f /workspaces/CARKit/install/setup.bash ]; then
  source /workspaces/CARKit/install/setup.bash
fi

exec "$@"
