#!/usr/bin/env bash

set -euo pipefail

IMAGE="${IMAGE:-ariiees/carkit:latest}"
WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

xhost +si:localuser:root >/dev/null 2>&1 || true

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  docker pull "${IMAGE}"
fi

DOCKER_GPU_ARGS=()
DOCKER_RUNTIMES="$(docker info --format '{{json .Runtimes}}' 2>/dev/null || true)"
if [[ "${DOCKER_RUNTIMES}" == *'"nvidia"'* ]]; then
  DOCKER_GPU_ARGS+=(--runtime nvidia)
else
  printf '%s\n' \
    "CARKit warning: Docker runtime 'nvidia' is not registered on this host." \
    "Starting without an explicit NVIDIA runtime. If GPU/TensorRT access fails," \
    "install/configure nvidia-container-toolkit for Jetson and rerun this script." >&2
fi

docker run --rm -it \
  --name carkit \
  "${DOCKER_GPU_ARGS[@]}" \
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
