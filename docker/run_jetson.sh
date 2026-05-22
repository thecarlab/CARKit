#!/usr/bin/env bash

set -euo pipefail

IMAGE="${IMAGE:-ariiees/carkit:latest}"
WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PULL_IMAGE="${PULL_IMAGE:-always}"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
HOST_USER="${USER:-carkit}"

xhost +si:localuser:root >/dev/null 2>&1 || true
xhost +si:localuser:"${HOST_USER}" >/dev/null 2>&1 || true

case "${PULL_IMAGE}" in
  always)
    docker pull "${IMAGE}"
    ;;
  missing)
    if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
      docker pull "${IMAGE}"
    fi
    ;;
  never)
    ;;
  *)
    echo "PULL_IMAGE must be one of: always, missing, never" >&2
    exit 1
    ;;
esac

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
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -e TZ="America/New_York" \
  -e CARKIT_HOST_UID="${HOST_UID}" \
  -e CARKIT_HOST_GID="${HOST_GID}" \
  -e CARKIT_HOST_USER="${HOST_USER}" \
  -e CARKIT_FIX_PERMISSIONS_ON_START="${CARKIT_FIX_PERMISSIONS_ON_START:-1}" \
  -e CARKIT_RUN_AS_ROOT="${CARKIT_RUN_AS_ROOT:-0}" \
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
  -v "${WORKSPACE}/docker/entrypoint.sh:/entrypoint.sh:ro" \
  -w /workspaces/CARKit \
  "${IMAGE}" \
  bash
