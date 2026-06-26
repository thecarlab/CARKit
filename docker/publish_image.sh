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

echo "Checking ${IMAGE} for CARKit Nav2 and Foxglove runtime packages..."
docker run --rm --entrypoint bash "$IMAGE" -lc '
  source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
  for pkg in foxglove_bridge nav2_bringup nav2_regulated_pure_pursuit_controller nav2_smac_planner slam_toolbox; do
    ros2 pkg prefix "${pkg}" >/dev/null
  done
'

echo "Checking ${IMAGE} for RealSense CUDA/GLSL SDK packages..."
docker run --rm --entrypoint bash "$IMAGE" -lc '
  set -e
  ldconfig
  ldconfig -p | grep -q "librealsense2\.so"
  ldconfig -p | grep -q "librealsense2-gl\.so"

  check_dir="$(mktemp -d)"
  trap "rm -rf ${check_dir}" EXIT
  cat > "${check_dir}/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.10)
project(carkit_realsense_publish_check NONE)
find_package(realsense2 2.58.0 REQUIRED)
find_package(realsense2-gl 2.58.0 REQUIRED)
EOF
  cmake -S "${check_dir}" -B "${check_dir}/build" >/tmp/carkit-realsense-publish-check.log
'

if [ "$PUSH_IMAGE" = "1" ]; then
  docker push "$IMAGE"
else
  echo "Built and checked ${IMAGE}; skipping docker push because PUSH_IMAGE=${PUSH_IMAGE}."
fi
