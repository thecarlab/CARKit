#!/usr/bin/env bash

# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"
ROSDEP_SKIP_KEYS="${ROSDEP_SKIP_KEYS:-librealsense2}"
REALSENSE_MIN_VERSION="${REALSENSE_MIN_VERSION:-2.58.0}"
BUILD_JOBS="${BUILD_JOBS:-1}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-1}"
REALSENSE_ACCELERATE_GPU_WITH_GLSL="${REALSENSE_ACCELERATE_GPU_WITH_GLSL:-auto}"
ROSDEP_SKIP_ARGS=()
if [ -n "${ROSDEP_SKIP_KEYS}" ]; then
  read -r -a ROSDEP_SKIP_ARGS <<< "${ROSDEP_SKIP_KEYS}"
fi
export MAKEFLAGS="-j${BUILD_JOBS} -l${BUILD_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"
export NINJAFLAGS="-j${BUILD_JOBS}"
COLCON_CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)

cd "$WORKSPACE"

if [ ! -f "${WORKSPACE}/docker/build_workspace.sh" ] \
  || [ ! -d "${WORKSPACE}/carkit/vehicle/osracer" ]; then
  echo "CARKit error: WORKSPACE does not point to the CARKit checkout: ${WORKSPACE}" >&2
  exit 1
fi

# Install exactly one hardware adapter. Algorithms and stable CARKit topics do
# not depend on this choice, so it can be changed later without touching them.
CARKIT_CHASSIS="${CARKIT_CHASSIS:-osracer}"
LEGACY_VEHICLE_PACKAGES=(
  ackermann_mux
  f1tenth_stack
  joy_teleop
  mouse_teleop
  teleop_tools
  teleop_tools_msgs
  vesc
  vesc_ackermann
  vesc_driver
  vesc_msgs
)
OSRACER_PACKAGES=(lakibeam1 osracer_base osracer_bringup)
REALSENSE_PACKAGES=(
  realsense2_camera
  realsense2_camera_msgs
  realsense2_description
)
UNUSED_VENDOR_PACKAGES=(
  realsense2_ros_mqtt_bridge
  realsense2_rviz_plugin
)
COLCON_SKIP_PACKAGES=("${UNUSED_VENDOR_PACKAGES[@]}")
case "${CARKIT_CHASSIS}" in
  osracer)
    COLCON_SKIP_PACKAGES+=(
      "${LEGACY_VEHICLE_PACKAGES[@]}"
      "${REALSENSE_PACKAGES[@]}"
    )
    INACTIVE_PACKAGES=(
      "${LEGACY_VEHICLE_PACKAGES[@]}"
      "${REALSENSE_PACKAGES[@]}"
      "${UNUSED_VENDOR_PACKAGES[@]}"
    )
    ;;
  f1tenth)
    COLCON_SKIP_PACKAGES+=("${OSRACER_PACKAGES[@]}")
    INACTIVE_PACKAGES=(
      "${OSRACER_PACKAGES[@]}"
      "${UNUSED_VENDOR_PACKAGES[@]}"
    )
    ;;
  *)
    echo "CARKIT_CHASSIS must be osracer or f1tenth" >&2
    exit 2
    ;;
esac

./carkit/setup_vendor_repos.sh

set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
set -u

# A clean checkout only contains the selected platform source. Intersect the
# skip list with discovered packages so colcon does not print misleading
# "unknown package" warnings for the platform that is intentionally absent.
mapfile -t DISCOVERED_PACKAGES < <(
  colcon list --base-paths carkit --names-only
)
DISCOVERED_SKIP_PACKAGES=()
for skipped in "${COLCON_SKIP_PACKAGES[@]}"; do
  for discovered in "${DISCOVERED_PACKAGES[@]}"; do
    if [ "${skipped}" = "${discovered}" ]; then
      DISCOVERED_SKIP_PACKAGES+=("${skipped}")
      break
    fi
  done
done

if [ "$(id -u)" -eq 0 ]; then
  ldconfig
fi

if [ "${CARKIT_CHASSIS}" = "f1tenth" ]; then
if ! ldconfig -p 2>/dev/null | grep -q 'librealsense2\.so' \
  && [ ! -e /usr/local/lib/librealsense2.so ] \
  && [ ! -f /usr/local/lib/cmake/realsense2/realsense2Config.cmake ]; then
  printf '%s\n' \
    "CARKit error: librealsense2 SDK is not installed in this Docker image." \
    "Expected one of:" \
    "  /usr/local/lib/librealsense2.so" \
    "  /usr/local/lib/cmake/realsense2/realsense2Config.cmake" \
    "Rebuild the image with the current docker/Dockerfile.jetson, then rerun this script:" \
    "  docker build -f docker/Dockerfile.jetson -t ariiees/carkit:latest ." >&2
  exit 1
fi

REALSENSE_CHECK_DIR="$(mktemp -d)"
cat > "${REALSENSE_CHECK_DIR}/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.10)
project(carkit_realsense_preflight NONE)
find_package(realsense2 ${REALSENSE_MIN_VERSION} REQUIRED)
EOF
if ! cmake -S "${REALSENSE_CHECK_DIR}" -B "${REALSENSE_CHECK_DIR}/build" \
  >/tmp/carkit-realsense-preflight.log 2>&1; then
  cat >&2 <<EOF
CARKit error: librealsense2 SDK is too old or not discoverable.
The vendored realsense2_camera package requires realsense2 >= ${REALSENSE_MIN_VERSION}.

Preflight output:
$(cat /tmp/carkit-realsense-preflight.log)

Rebuild the image with the current docker/Dockerfile.jetson, then rerun:
  docker build -f docker/Dockerfile.jetson -t ariiees/carkit:latest .
  PULL_IMAGE=never ./docker/run_jetson.sh
  ./docker/build_workspace.sh
EOF
  rm -rf "${REALSENSE_CHECK_DIR}"
  exit 1
fi
rm -rf "${REALSENSE_CHECK_DIR}" /tmp/carkit-realsense-preflight.log

REALSENSE_GL_CHECK_DIR="$(mktemp -d)"
cat > "${REALSENSE_GL_CHECK_DIR}/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.10)
project(carkit_realsense_gl_preflight NONE)
find_package(realsense2-gl ${REALSENSE_MIN_VERSION} REQUIRED)
EOF
if cmake -S "${REALSENSE_GL_CHECK_DIR}" -B "${REALSENSE_GL_CHECK_DIR}/build" \
  >/tmp/carkit-realsense-gl-preflight.log 2>&1; then
  REALSENSE_GL_AVAILABLE=1
else
  REALSENSE_GL_AVAILABLE=0
fi
rm -rf "${REALSENSE_GL_CHECK_DIR}" /tmp/carkit-realsense-gl-preflight.log

case "${REALSENSE_ACCELERATE_GPU_WITH_GLSL}" in
  auto)
    if [ "${REALSENSE_GL_AVAILABLE}" = "1" ]; then
      COLCON_CMAKE_ARGS+=(-DBUILD_ACCELERATE_GPU_WITH_GLSL=ON)
      echo "Building realsense2_camera with GLSL GPU acceleration support."
    else
      echo "realsense2-gl not found; building realsense2_camera without GLSL GPU acceleration support."
    fi
    ;;
  1|true|TRUE|on|ON)
    if [ "${REALSENSE_GL_AVAILABLE}" != "1" ]; then
      cat >&2 <<EOF
CARKit error: REALSENSE_ACCELERATE_GPU_WITH_GLSL was requested, but
realsense2-gl is not installed in this Docker image.

Rebuild the image from docker/Dockerfile.jetson, then rerun this script.
EOF
      exit 1
    fi
    COLCON_CMAKE_ARGS+=(-DBUILD_ACCELERATE_GPU_WITH_GLSL=ON)
    ;;
  0|false|FALSE|off|OFF)
    ;;
  *)
    echo "REALSENSE_ACCELERATE_GPU_WITH_GLSL must be auto, 1, or 0" >&2
    exit 1
    ;;
esac
fi

APT_GET=()
if [ "$(id -u)" -eq 0 ]; then
  APT_GET=(apt-get)
elif command -v sudo >/dev/null 2>&1 && sudo -n true >/dev/null 2>&1; then
  APT_GET=(sudo apt-get)
fi

if [ "${#APT_GET[@]}" -gt 0 ]; then
  "${APT_GET[@]}" update
fi

rosdep update
mapfile -t ACTIVE_PACKAGE_PATHS < <(
  colcon list \
    --base-paths carkit \
    --packages-skip "${DISCOVERED_SKIP_PACKAGES[@]}" \
    --paths-only
)
if [ "${#ACTIVE_PACKAGE_PATHS[@]}" -eq 0 ]; then
  echo "CARKit error: no active ROS packages were discovered." >&2
  exit 1
fi
ROSDEP_INSTALL_CMD=(
  rosdep install --from-paths "${ACTIVE_PACKAGE_PATHS[@]}" --ignore-src -r -y
  --dependency-types build_export --dependency-types buildtool_export
  --dependency-types buildtool --dependency-types build --dependency-types exec
)
if [ "${#ROSDEP_SKIP_ARGS[@]}" -gt 0 ]; then
  ROSDEP_INSTALL_CMD+=(--skip-keys "${ROSDEP_SKIP_ARGS[@]}")
fi
"${ROSDEP_INSTALL_CMD[@]}"

colcon build --symlink-install \
  --executor sequential \
  --parallel-workers "${PARALLEL_WORKERS}" \
  --packages-skip "${DISCOVERED_SKIP_PACKAGES[@]}" \
  --cmake-args "${COLCON_CMAKE_ARGS[@]}"

# These are generated artifacts and can always be recreated by selecting the
# other adapter again. Removing them makes the install overlay unambiguous.
for package in "${INACTIVE_PACKAGES[@]}"; do
  rm -rf -- "${WORKSPACE}/build/${package}" "${WORKSPACE}/install/${package}"
done

set +u
source install/setup.bash
set -u

ros2 pkg list | grep carkit
if [ "${CARKIT_CHASSIS}" = "osracer" ]; then
  ros2 pkg prefix osracer_base
  ros2 pkg prefix osracer_bringup
  ros2 pkg prefix lakibeam1
else
  ros2 pkg prefix f1tenth_stack
  ros2 pkg prefix vesc_driver
fi
