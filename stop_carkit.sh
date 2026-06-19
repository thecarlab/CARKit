#!/usr/bin/env bash
#
# Stop CARKit ROS processes launched manually or via start_autonomous.sh.
#
# Run inside the Docker container:
#   ./stop_carkit.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/carkit_stack_lib.sh"

echo "==> Stopping CARKit processes..."
if carkit_stop_stack; then
  echo "==> CARKit processes stopped."
else
  echo "==> No matching CARKit processes were running."
fi
