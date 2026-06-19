#!/usr/bin/env bash
#
# Shared helpers for start_autonomous.sh and stop_carkit.sh.

carkit_stop_stack() {
  local grace_seconds="${GRACE_SECONDS:-3}"
  local pattern
  local found=0
  local owner_pgid
  owner_pgid="$(ps -o pgid= -p "${CARKIT_STACK_OWNER:-$$}" 2>/dev/null | tr -d ' ')"

  local patterns=(
    'ros2 launch carkit_'
    'ros2 launch f1tenth_stack'
    'ros2 launch sllidar_ros2'
    'sllidar_node'
    'behavior_center_node'
    'control_center_node'
    'perception_2d_node'
    'odom_tf_broadcaster'
    'twist_to_ackermann'
    'joy_rate_filter'
    'throttle_interpolator'
    'joy_teleop'
    'ackermann_mux'
    'vesc_driver_node'
    'vesc_to_odom_node'
    'ackermann_to_vesc_node'
    'realsense2_camera_node'
    'rviz2'
    '/opt/ros/humble/lib/nav2_'
    '/opt/ros/humble/lib/joy/joy_node'
  )

  signal_matches() {
    local signal="$1"
    local pid
    local pgid
    for pattern in "${patterns[@]}"; do
      while IFS= read -r pid; do
        [ -n "${pid}" ] || continue
        pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
        if [ -n "${owner_pgid}" ] && [ "${pgid}" = "${owner_pgid}" ]; then
          continue
        fi
        if kill "-${signal}" "${pid}" 2>/dev/null; then
          found=1
        fi
      done < <(pgrep -f "${pattern}" 2>/dev/null || true)
    done
  }

  carkit_release_serial_devices TERM

  signal_matches TERM
  if [ "${found}" -eq 1 ]; then
    sleep "${grace_seconds}"
    signal_matches KILL
  fi

  carkit_release_serial_devices KILL

  return "${found}"
}

carkit_detect_lidar_port() {
  local candidate
  for candidate in /dev/serial/by-id/usb-Silicon_Labs_* /dev/serial/by-id/*SLLidar* /dev/serial/by-id/*Slamtec*; do
    [ -e "${candidate}" ] || continue
    readlink -f "${candidate}"
    return 0
  done
  for candidate in /dev/ttyUSB*; do
    [ -e "${candidate}" ] || continue
    echo "${candidate}"
    return 0
  done
  echo "/dev/ttyUSB0"
}

carkit_serial_devices() {
  local dev
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "${dev}" ] || continue
    echo "${dev}"
  done
}

carkit_release_serial_devices() {
  local signal="$1"
  local dev
  local pid
  local owner_pgid
  owner_pgid="$(ps -o pgid= -p "${CARKIT_STACK_OWNER:-$$}" 2>/dev/null | tr -d ' ')"

  for dev in $(carkit_serial_devices); do
    [ -e "${dev}" ] || continue

    if command -v fuser >/dev/null 2>&1; then
      fuser "-${signal}" -k "${dev}" >/dev/null 2>&1 || true
      continue
    fi

    for pid in /proc/[0-9]*; do
      pid="${pid#/proc/}"
      if [ -n "${owner_pgid}" ]; then
        local pgid
        pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
        [ "${pgid}" = "${owner_pgid}" ] && continue
      fi
      ls -l "/proc/${pid}/fd" 2>/dev/null | grep -q "${dev}" || continue
      kill "-${signal}" "${pid}" 2>/dev/null || true
    done
  done
}

carkit_wait_for_serial_devices() {
  local tries="${1:-10}"
  local delay="${2:-0.5}"

  for dev in $(carkit_serial_devices); do
    [ -e "${dev}" ] || continue
    local remaining="${tries}"
    while [ "${remaining}" -gt 0 ]; do
      if ! carkit_device_in_use "${dev}"; then
        break
      fi
      remaining=$((remaining - 1))
      sleep "${delay}"
    done
    if carkit_device_in_use "${dev}"; then
      echo "CARKit warning: ${dev} is still in use after cleanup." >&2
      return 1
    fi
  done
}

carkit_device_in_use() {
  local dev="$1"

  if command -v fuser >/dev/null 2>&1; then
    fuser "${dev}" >/dev/null 2>&1
    return $?
  fi

  local pid
  for pid in /proc/[0-9]*; do
    pid="${pid#/proc/}"
    ls -l "/proc/${pid}/fd" 2>/dev/null | grep -q "${dev}" && return 0
  done
  return 1
}
