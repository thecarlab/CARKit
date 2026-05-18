#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOS_FILE="$ROOT_DIR/carkit/vendor.repos"

mark_safe_directory() {
  local path="$1"
  if [ -d "$ROOT_DIR/$path/.git" ]; then
    git config --global --add safe.directory "$ROOT_DIR/$path" >/dev/null 2>&1 || true
  fi
}

mark_safe_directory "carkit/sensors/realsense-ros"
mark_safe_directory "carkit/sensors/sllidar_ros2"

if command -v vcs >/dev/null 2>&1; then
  vcs import "$ROOT_DIR" --skip-existing < "$REPOS_FILE"
  exit 0
fi

clone_if_missing() {
  local path="$1"
  local url="$2"
  local branch="$3"

  if [ -d "$ROOT_DIR/$path/.git" ] || [ -e "$ROOT_DIR/$path/package.xml" ]; then
    printf 'skip existing %s\n' "$path"
    return
  fi

  mkdir -p "$(dirname "$ROOT_DIR/$path")"
  git clone --branch "$branch" --depth 1 "$url" "$ROOT_DIR/$path"
}

clone_if_missing "carkit/sensors/realsense-ros" "https://github.com/IntelRealSense/realsense-ros.git" "ros2-master"
clone_if_missing "carkit/sensors/sllidar_ros2" "https://github.com/Slamtec/sllidar_ros2.git" "main"
