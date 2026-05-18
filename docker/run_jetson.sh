#!/usr/bin/env bash

set -euo pipefail

IMAGE="${IMAGE:-ariiees/carkit:latest}"
WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

xhost +si:localuser:root >/dev/null 2>&1 || true

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  docker pull "${IMAGE}"
fi

docker run --rm -it \
  --name carkit \
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
  -v "${WORKSPACE}:/workspaces/CARKit" \
  -w /workspaces/CARKit \
  "${IMAGE}" \
  bash
