#!/usr/bin/env bash

set -euo pipefail

IMAGE="${IMAGE:-ariiees/carkit:latest}"
DOCKERFILE="${DOCKERFILE:-docker/Dockerfile.jetson}"
PUSH_IMAGE="${PUSH_IMAGE:-1}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT_DIR"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not on PATH" >&2
  exit 1
fi

docker build -f "$DOCKERFILE" -t "$IMAGE" .

echo "Checking ${IMAGE} for CARKit Nav2 runtime packages..."
docker run --rm --entrypoint bash "$IMAGE" -lc '
  source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
  for pkg in nav2_bringup nav2_regulated_pure_pursuit_controller nav2_smac_planner slam_toolbox; do
    ros2 pkg prefix "${pkg}" >/dev/null
  done
'

if [ "$PUSH_IMAGE" = "1" ]; then
  docker push "$IMAGE"
else
  echo "Built and checked ${IMAGE}; skipping docker push because PUSH_IMAGE=${PUSH_IMAGE}."
fi
