#!/usr/bin/env bash
#
# Start CARKit autonomous driving: human control, control center, navigation,
# perception, and behavior as described in README.md.
#
# Run inside the Docker container after building the workspace:
#   ./docker/build_workspace.sh
#   source install/setup.bash
#   ./start_autonomous.sh
#
# Optional:
#   ./start_autonomous.sh --map /workspaces/CARKit/map/map_3f.yaml
#   MAP=/workspaces/CARKit/map/map_3f.yaml ./start_autonomous.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-${SCRIPT_DIR}}"
MAP="${MAP:-${WORKSPACE}/map/map.yaml}"
START_HUMAN_CONTROL="${START_HUMAN_CONTROL:-1}"
START_CONTROL_CENTER="${START_CONTROL_CENTER:-1}"
CUSPARSELT_LIB="/usr/local/lib/python3.10/dist-packages/nvidia/cusparselt/lib"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Start CARKit autonomous driving per README.md:
  - joystick teleop with legacy mux remapped away from /ackermann_cmd
  - control center command arbiter
  - Nav2 navigation
  - RealSense + YOLO perception
  - behavior overrides and cone obstacle publishing

Options:
  --map PATH   Occupancy map YAML for navigation mode
               (default: ${MAP})
  --help       Show this help and exit

Environment:
  WORKSPACE              Repository/workspace root (default: script directory)
  MAP                    Same as --map
  LOG_DIR                Directory for per-launch log files (default: auto timestamp)
  START_HUMAN_CONTROL    Launch joystick/VESC stack (default: 1)
  START_CONTROL_CENTER   Launch control center (default: 1)

Press Ctrl+C to stop all launched processes.

To stop manually from another shell:
  ./stop_carkit.sh
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --map)
      shift
      MAP="${1:?--map requires a path}"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

if [ ! -f "${MAP}" ]; then
  echo "CARKit error: map file not found: ${MAP}" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"
else
  echo "CARKit error: workspace is not built. Run ./docker/build_workspace.sh first." >&2
  exit 1
fi
set -u

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/carkit_stack_lib.sh"
export CARKIT_STACK_OWNER=$$

echo "==> Cleaning up any stale CARKit processes..."
carkit_stop_stack || true
carkit_wait_for_serial_devices 10 0.5 || true
echo

cleanup() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then
    return
  fi
  CLEANUP_DONE=1
  trap - EXIT INT TERM

  echo
  echo "==> Stopping autonomous stack..."
  local pid
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      # Each launch runs in its own session/process group (setsid below).
      kill -INT -"${pid}" 2>/dev/null \
        || kill -TERM -"${pid}" 2>/dev/null \
        || kill -TERM "${pid}" 2>/dev/null \
        || true
    fi
  done

  local deadline=$((SECONDS + 5))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    local running=0
    for pid in "${PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        running=1
        break
      fi
    done
    [ "${running}" -eq 0 ] && break
    sleep 0.2
  done

  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -"${pid}" 2>/dev/null \
        || kill -KILL "${pid}" 2>/dev/null \
        || true
    fi
  done

  wait 2>/dev/null || true
  carkit_stop_stack >/dev/null 2>&1 || true
  carkit_wait_for_serial_devices 5 0.2 >/dev/null 2>&1 || true
  echo "==> All processes stopped."
}

PIDS=()
LOG_DIR="${LOG_DIR:-${WORKSPACE}/log/start_autonomous/$(date +%Y%m%d-%H%M%S)}"
mkdir -p "${LOG_DIR}"
trap cleanup EXIT INT TERM

launch() {
  local label="$1"
  local log_name="$2"
  shift 2
  local log_file="${LOG_DIR}/${log_name}.log"
  echo "==> Starting ${label}"
  # New session so Ctrl+C can stop ros2 launch and all child nodes together.
  setsid "$@" >>"${log_file}" 2>&1 &
  PIDS+=("$!")
}

if [ "${START_HUMAN_CONTROL}" = "1" ]; then
  launch "human control (joystick + VESC)" human_control \
    ros2 launch carkit_human_control joystick.launch.py \
      vehicle_command_topic:=/ackermann_mux_unused
fi

if [ "${START_CONTROL_CENTER}" = "1" ]; then
  launch "control center" control_center \
    ros2 launch carkit_control_center control_center.launch.py
fi

launch "navigation (Nav2)" navigation \
  ros2 launch carkit_navigation navigation.launch.py \
    mode:=navigation \
    start_command_mux:=false \
    "map:=${MAP}" \
    "lidar_serial_port:=${LIDAR_SERIAL_PORT:-$(carkit_detect_lidar_port)}"

if [ -d "${CUSPARSELT_LIB}" ]; then
  export LD_LIBRARY_PATH="${CUSPARSELT_LIB}:${LD_LIBRARY_PATH:-}"
else
  echo "CARKit warning: ${CUSPARSELT_LIB} not found; perception may fail to load TensorRT deps." >&2
fi

launch "perception (RealSense + YOLO)" perception \
  ros2 launch carkit_perception perception.launch.py

launch "behavior center" behavior \
  ros2 launch carkit_behavior behavior_center.launch.py

echo
echo "Autonomous stack is running."
echo "Map: ${MAP}"
echo "Logs: ${LOG_DIR}"
echo "In RViz: set 2D Pose Estimate, wait for AMCL, send a Nav2 goal,"
echo "then press joystick button 0 to enter AUTO_DRIVE."
echo "Press Ctrl+C here to stop all processes."
echo

wait
