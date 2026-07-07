#!/usr/bin/env bash
# Start CARKit autonomous stack (T1–T8) with per-subsystem CPU affinity.
#
# Equivalent to the five default launches, split for isolation:
#   T1 RealSense (cores 0–1)   T2 LiDAR (0–1)
#   T3 vehicle  T4 control  T5 behavior  T6 Foxglove (2–5)
#   T7 Nav2 (2–5)              T8 YOLO perception (2–5)
#
# Usage (inside Docker):
#   ./scripts/start_av_stack_affinity.sh start
#   ./scripts/start_av_stack_affinity.sh status
#   ./scripts/start_av_stack_affinity.sh stop
#
# Environment overrides:
#   WORKSPACE, MAP, IO_CPUS, CONTROL_CPUS, COMPUTE_CPUS, STATE_DIR, LIDAR_SERIAL_PORT

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"
MAP="${MAP:-${WORKSPACE}/map/map_3f.yaml}"
IO_CPUS="${IO_CPUS:-0,1}"
CONTROL_CPUS="${CONTROL_CPUS:-2-5}"
COMPUTE_CPUS="${COMPUTE_CPUS:-2-5}"
STATE_DIR="${STATE_DIR:-/tmp/carkit_av_stack}"
LOG_DIR="${STATE_DIR}/logs"

FOXGLOVE_TOPIC_WHITELIST="['^/map$', '^/map_metadata$', '^/tf$', '^/tf_static$', '^/scan$', '^/amcl_pose$', '^/particle_cloud$', '^/plan$', '^/plan_smoothed$', '^/received_global_plan$', '^/local_plan$', '^/goal_pose$', '^/move_base_simple/goal$', '^/initialpose$', '^/clicked_point$', '^/behavior/stop_sign_position$', '^/behavior/traffic_light_position$']"
FOXGLOVE_CLIENT_TOPIC_WHITELIST="['^/goal_pose$', '^/move_base_simple/goal$', '^/initialpose$', '^/clicked_point$']"

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${WORKSPACE}/install/setup.bash"
  else
    echo "ERROR: ${WORKSPACE}/install/setup.bash not found. Build the workspace first." >&2
    exit 1
  fi
  set -u
}

detect_lidar_port() {
  if [ -n "${LIDAR_SERIAL_PORT:-}" ]; then
    if [ -e "${LIDAR_SERIAL_PORT}" ]; then
      readlink -f "${LIDAR_SERIAL_PORT}"
      return
    fi
    echo "ERROR: LIDAR_SERIAL_PORT=${LIDAR_SERIAL_PORT} does not exist." >&2
    exit 1
  fi

  local pattern match
  for pattern in \
    '/dev/serial/by-id/usb-Silicon_Labs_*' \
    '/dev/serial/by-id/*SLLidar*' \
    '/dev/serial/by-id/*Slamtec*' \
    '/dev/ttyUSB*'
  do
    # shellcheck disable=SC2086
    match="$(ls ${pattern} 2>/dev/null | head -1 || true)"
    if [ -n "${match}" ]; then
      readlink -f "${match}"
      return
    fi
  done

  echo "ERROR: No LiDAR serial device found. Set LIDAR_SERIAL_PORT." >&2
  exit 1
}

pid_alive() {
  local pid="$1"
  [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null
}

write_pid() {
  local name="$1"
  local pid="$2"
  echo "${pid}" > "${STATE_DIR}/${name}.pid"
}

read_pid() {
  local name="$1"
  if [ -f "${STATE_DIR}/${name}.pid" ]; then
    cat "${STATE_DIR}/${name}.pid"
  fi
}

stop_pid_file() {
  local name="$1"
  local pid
  pid="$(read_pid "${name}" || true)"
  if pid_alive "${pid}"; then
    echo "Stopping ${name} (pid ${pid})..."
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      pid_alive "${pid}" || break
      sleep 0.5
    done
    if pid_alive "${pid}"; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
  rm -f "${STATE_DIR}/${name}.pid"
}

launch_bg() {
  local name="$1"
  local cpus="$2"
  shift 2

  mkdir -p "${LOG_DIR}"
  local log_file="${LOG_DIR}/${name}.log"
  echo "Starting ${name} on CPUs ${cpus} -> ${log_file}"

  # taskset applies to the ros2 child executable, not only the launch wrapper.
  taskset -c "${cpus}" "$@" >"${log_file}" 2>&1 &
  write_pid "${name}" "$!"
}

start_t1_realsense() {
  launch_bg t1 "${IO_CPUS}" \
    ros2 run realsense2_camera realsense2_camera_node \
      --ros-args \
      -r __ns:=/camera \
      -r __node:=camera \
      -p enable_color:=true \
      -p enable_depth:=false \
      -p enable_infra:=false \
      -p enable_infra1:=false \
      -p enable_infra2:=false \
      -p enable_gyro:=false \
      -p enable_accel:=false \
      -p enable_motion:=false \
      -p enable_rgbd:=false \
      -p enable_sync:=false \
      -p align_depth.enable:=false \
      -p pointcloud.enable:=false \
      -p rgb_camera.color_profile:=640x480x15 \
      -p rgb_camera.color_format:=RGB8 \
      -p rgb_camera.global_time_enabled:=false
}

start_t2_lidar() {
  local lidar_port
  lidar_port="$(detect_lidar_port)"
  echo "LiDAR serial port: ${lidar_port}"

  launch_bg t2 "${IO_CPUS}" \
    ros2 launch sllidar_ros2 sllidar_s2_launch.py \
      channel_type:=serial \
      "serial_port:=${lidar_port}" \
      serial_baudrate:=1000000 \
      frame_id:=laser \
      inverted:=false \
      angle_compensate:=true \
      scan_mode:=DenseBoost
}

start_lidar_motor() {
  echo "Waiting for /start_motor service..."
  local attempt
  for attempt in $(seq 1 30); do
    if ros2 service list 2>/dev/null | grep -qx '/start_motor'; then
      echo "Calling /start_motor..."
      timeout 10 ros2 service call /start_motor std_srvs/srv/Empty "{}" || true
      return
    fi
    sleep 1
  done
  echo "WARNING: /start_motor never appeared; LiDAR may not be spinning." >&2
}

start_t3_vehicle() {
  launch_bg t3 "${CONTROL_CPUS}" \
    ros2 launch carkit_human_control joystick.launch.py \
      vehicle_command_topic:=/ackermann_mux_unused
}

start_t4_control() {
  launch_bg t4 "${CONTROL_CPUS}" \
    ros2 launch carkit_control_center control_center.launch.py
}

start_t5_behavior() {
  launch_bg t5 "${CONTROL_CPUS}" \
    ros2 launch carkit_behavior behavior_center.launch.py
}

start_t6_foxglove() {
  launch_bg t6 "${CONTROL_CPUS}" \
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
      port:=8765 \
      "topic_whitelist:=${FOXGLOVE_TOPIC_WHITELIST}" \
      "client_topic_whitelist:=${FOXGLOVE_CLIENT_TOPIC_WHITELIST}" \
      capabilities:="[clientPublish,connectionGraph]"
}

start_t7_navigation() {
  launch_bg t7 "${COMPUTE_CPUS}" \
    ros2 launch carkit_navigation navigation.launch.py \
      mode:=navigation \
      start_command_mux:=false \
      "map:=${MAP}" \
      start_lidar:=false \
      auto_start_lidar_motor:=false \
      visualization:=none
}

start_t8_perception() {
  launch_bg t8 "${COMPUTE_CPUS}" \
    ros2 launch carkit_perception perception.launch.py \
      start_camera:=false \
      visualization:=none
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="${2:-45}"
  echo "Waiting for topic ${topic}..."
  local attempt
  for attempt in $(seq 1 "${timeout_sec}"); do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      echo "Topic ready: ${topic}"
      return 0
    fi
    sleep 1
  done
  echo "WARNING: Topic ${topic} did not appear within ${timeout_sec}s." >&2
  return 1
}

cmd_start() {
  if [ -f "${STATE_DIR}/running" ]; then
    echo "Stack may already be running (found ${STATE_DIR}/running)." >&2
    echo "Run: $0 stop" >&2
    exit 1
  fi

  mkdir -p "${STATE_DIR}" "${LOG_DIR}"
  source_ros

  echo "=== CARKit AV stack (CPU affinity) ==="
  echo "IO CPUs: ${IO_CPUS}  Control CPUs: ${CONTROL_CPUS}  Compute CPUs: ${COMPUTE_CPUS}"
  echo "Map: ${MAP}"
  echo "Logs: ${LOG_DIR}"
  echo ""

  start_t1_realsense
  sleep 3
  wait_for_topic "/camera/camera/color/image_raw" 45 || true

  start_t2_lidar
  sleep 5
  start_lidar_motor
  wait_for_topic "/scan" 30 || true

  start_t3_vehicle
  sleep 2

  start_t7_navigation
  sleep 5

  start_t8_perception
  sleep 2

  start_t4_control
  start_t5_behavior
  start_t6_foxglove

  date -Iseconds > "${STATE_DIR}/running"
  echo ""
  echo "Stack started. Foxglove: ws://<jetson-ip>:8765"
  cmd_status
}

cmd_stop() {
  echo "Stopping CARKit AV stack..."
  for name in t6 t5 t4 t8 t7 t3 t2 t1; do
    stop_pid_file "${name}"
  done

  pkill -TERM -f realsense2_camera_node 2>/dev/null || true
  pkill -TERM -f "ros2 launch sllidar_ros2" 2>/dev/null || true
  pkill -TERM -f "ros2 launch carkit_human_control" 2>/dev/null || true
  pkill -TERM -f "ros2 launch carkit_control_center" 2>/dev/null || true
  pkill -TERM -f "ros2 launch carkit_behavior" 2>/dev/null || true
  pkill -TERM -f "ros2 launch foxglove_bridge" 2>/dev/null || true
  pkill -TERM -f "ros2 launch carkit_navigation" 2>/dev/null || true
  pkill -TERM -f "ros2 launch carkit_perception" 2>/dev/null || true
  sleep 2

  rm -f "${STATE_DIR}/running"
  echo "Stopped."
}

cmd_status() {
  echo "=== Stack status (${STATE_DIR}) ==="
  for name in t1 t2 t3 t4 t5 t6 t7 t8; do
    local pid label
    pid="$(read_pid "${name}" || true)"
    case "${name}" in
      t1) label="T1 RealSense (${IO_CPUS})" ;;
      t2) label="T2 LiDAR (${IO_CPUS})" ;;
      t3) label="T3 vehicle (${CONTROL_CPUS})" ;;
      t4) label="T4 control (${CONTROL_CPUS})" ;;
      t5) label="T5 behavior (${CONTROL_CPUS})" ;;
      t6) label="T6 Foxglove (${CONTROL_CPUS})" ;;
      t7) label="T7 Nav2 (${COMPUTE_CPUS})" ;;
      t8) label="T8 YOLO (${COMPUTE_CPUS})" ;;
    esac
    if pid_alive "${pid}"; then
      local affinity
      affinity="$(taskset -cp "${pid}" 2>/dev/null | awk -F: '{print $2}' | xargs || echo '?')"
      echo "  ${label}: running pid=${pid} affinity=${affinity}"
    else
      echo "  ${label}: not running"
    fi
  done
  echo ""
  echo "Logs: ${LOG_DIR}/t*.log"
  echo "Tail example: tail -f ${LOG_DIR}/t1.log"
}

usage() {
  sed -n '2,12p' "$0"
  echo ""
  echo "Commands: start | stop | status"
}

main() {
  local cmd="${1:-start}"
  case "${cmd}" in
    start) cmd_start ;;
    stop) cmd_stop ;;
    status) cmd_status ;;
    -h|--help|help) usage ;;
    *)
      echo "Unknown command: ${cmd}" >&2
      usage
      exit 1
      ;;
  esac
}

main "$@"
