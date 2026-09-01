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
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -v "$ROOT_DIR:/workspaces/CARKit" \
  -w /workspaces/CARKit \
  "$IMAGE" \
  bash -lc 'CARKIT_CHASSIS=osracer ./docker/build_workspace.sh && source install/setup.bash && ros2 pkg list | grep carkit && ros2 pkg prefix carkit_bringup && ros2 pkg prefix carkit_ada_academy && ros2 pkg prefix carkit_intro2av && ros2 pkg prefix carkit_intro2av_cpp && ros2 pkg prefix carkit_webui && ros2 pkg prefix osracer_base && ros2 pkg prefix osracer_bringup && ros2 pkg prefix lakibeam1 && ros2 pkg prefix sllidar_ros2 && ros2 launch carkit_bringup carkit.launch.py --show-args && ros2 launch osracer_bringup sensors_launch.py --show-args && ros2 launch carkit_human_control joystick.launch.py --show-args && ros2 launch carkit_navigation navigation.launch.py --show-args && ros2 launch carkit_perception perception.launch.py --show-args && ros2 launch carkit_behavior behavior_center.launch.py --show-args'
