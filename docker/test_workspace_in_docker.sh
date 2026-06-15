#!/usr/bin/env bash

set -euo pipefail

IMAGE="${IMAGE:-ariiees/carkit:latest}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not on PATH" >&2
  exit 1
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  docker pull "$IMAGE"
fi

docker run --rm -t \
  --runtime nvidia \
  --privileged \
  --network host \
  --ipc host \
  --shm-size 6g \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v "$ROOT_DIR:/workspaces/CARKit" \
  -w /workspaces/CARKit \
  "$IMAGE" \
  bash -lc './docker/build_workspace.sh && source install/setup.bash && ros2 pkg list | grep carkit && ros2 launch carkit_navigation navigation.launch.py --show-args'
