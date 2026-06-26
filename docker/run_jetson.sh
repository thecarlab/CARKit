#!/usr/bin/env bash

set -euo pipefail

IMAGE="${IMAGE:-ariiees/carkit:latest}"
WORKSPACE="${WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PULL_IMAGE="${PULL_IMAGE:-missing}"
CARKIT_REQUIRE_RUNTIME="${CARKIT_REQUIRE_RUNTIME:-${CARKIT_REQUIRE_NAV2:-1}}"
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

if [ "${CARKIT_REQUIRE_RUNTIME}" = "1" ]; then
  echo "Checking ${IMAGE} for CARKit runtime packages..."
  if ! docker run --rm --entrypoint bash "${IMAGE}" -lc '
      source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
      missing=0
      for pkg in \
        foxglove_bridge \
        nav2_bringup \
        nav2_regulated_pure_pursuit_controller \
        nav2_smac_planner \
        slam_toolbox
      do
        if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
          echo "missing ROS package: ${pkg}" >&2
          missing=1
        fi
      done
      exit "${missing}"
    '; then
    cat >&2 <<EOF
CARKit error: Docker image '${IMAGE}' does not contain the ROS runtime needed
by CARKit navigation, perception visualization, and behavior launches.

Rebuild or pull an updated image, then rerun:

  ./docker/publish_image.sh
  ./docker/run_jetson.sh

For local testing without pulling Docker Hub:

  PULL_IMAGE=never ./docker/run_jetson.sh

To bypass this check temporarily:

  CARKIT_REQUIRE_RUNTIME=0 ./docker/run_jetson.sh
EOF
    exit 1
  fi
fi

CONTAINER_CMD=(bash)

if [ "$#" -gt 0 ]; then
  CONTAINER_CMD=("$@")
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
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -e TZ="America/New_York" \
  -e CARKIT_HOST_UID="${HOST_UID}" \
  -e CARKIT_HOST_GID="${HOST_GID}" \
  -e CARKIT_HOST_USER="${HOST_USER}" \
  -e CARKIT_FIX_PERMISSIONS_ON_START="${CARKIT_FIX_PERMISSIONS_ON_START:-1}" \
  -e CARKIT_RUN_AS_ROOT="${CARKIT_RUN_AS_ROOT:-1}" \
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
  "${CONTAINER_CMD[@]}"
