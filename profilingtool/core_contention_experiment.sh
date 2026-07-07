#!/usr/bin/env bash
# Compare RealSense image rate under CPU burn with same-core vs isolated-core affinity.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DURATION="${HZ_DURATION:-12}"
BURN_CORES="${BURN_CORES:-2}"
BURN_LOAD="${BURN_LOAD:-60}"
TOPIC="/camera/camera/color/image_raw"
RESULTS="${ROOT}/core_contention_results.tsv"

set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /workspaces/CARKit/install/setup.bash 2>/dev/null || true
set -u

cleanup() {
  pkill -9 -f cpu_occupier.py 2>/dev/null || true
  pkill -9 -f realsense2_camera_node 2>/dev/null || true
  pkill -9 -f "ros2 launch realsense2_camera" 2>/dev/null || true
  sleep 3
}

measure_hz() {
  local label="$1"
  timeout "${DURATION}" ros2 topic hz "${TOPIC}" 2>&1 \
    | tee "/tmp/hz_${label}.log" \
    | grep -E "average rate|no new messages" \
    | tail -1 \
    || true
}

parse_hz() {
  local log_file="$1"
  if grep -q "average rate" "${log_file}" 2>/dev/null; then
    grep "average rate" "${log_file}" | tail -1 | awk '{print $3}'
  else
    echo "0"
  fi
}

wait_for_topic() {
  for _ in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "color/image_raw"; then
      return 0
    fi
    sleep 1
  done
  echo "ERROR: ${TOPIC} never appeared" >&2
  return 1
}

start_realsense() {
  local affinity="$1"
  if [ -n "${affinity}" ]; then
    taskset -c "${affinity}" ros2 launch realsense2_camera rs_launch.py \
      enable_color:=true enable_depth:=false enable_infra:=false \
      enable_infra1:=false enable_infra2:=false align_depth.enable:=false \
      enable_sync:=false pointcloud.enable:=false \
      rgb_camera.color_profile:=640x480x15 \
      >"/tmp/realsense_${affinity//,/_}.log" 2>&1 &
  else
    ros2 launch realsense2_camera rs_launch.py \
      enable_color:=true enable_depth:=false enable_infra:=false \
      enable_infra1:=false enable_infra2:=false align_depth.enable:=false \
      enable_sync:=false pointcloud.enable:=false \
      rgb_camera.color_profile:=640x480x15 \
      >"/tmp/realsense_none.log" 2>&1 &
  fi
  wait_for_topic
  sleep 3
}

start_burn() {
  local affinity="$1"
  if [ -n "${affinity}" ]; then
    taskset -c "${affinity}" python3 "${ROOT}/cpu_occupier.py" \
      --cores "${BURN_CORES}" --load-percent "${BURN_LOAD}" --interval 60 \
      >"/tmp/burn_${affinity//,/_}.log" 2>&1 &
  else
    python3 "${ROOT}/cpu_occupier.py" \
      --cores "${BURN_CORES}" --load-percent "${BURN_LOAD}" --interval 60 \
      >"/tmp/burn_none.log" 2>&1 &
  fi
  sleep 2
}

thread_snapshot() {
  local tag="$1"
  local rs_pid burn_pids
  rs_pid="$(pgrep -f realsense2_camera_node | head -1 || true)"
  if [ -z "${rs_pid}" ]; then
    echo "  (realsense node not running)"
    return
  fi
  echo "  RealSense cpuset: $(taskset -cp "${rs_pid}" 2>/dev/null | awk -F: '{print $2}')"
  echo "  RealSense hot threads (tid psr pcpu comm):"
  ps -L -o tid,psr,pcpu,comm -p "${rs_pid}" 2>/dev/null \
    | awk 'NR==1 || $3+0 > 0.5' \
    | head -12
  burn_pids="$(pgrep -f 'cpu_occupier.py' || true)"
  if [ -n "${burn_pids}" ]; then
    echo "  Burn processes:"
    while read -r bp; do
      [ -n "${bp}" ] || continue
      echo "    pid ${bp} cpuset:$(taskset -cp "${bp}" 2>/dev/null | awk -F: '{print $2}')"
      ps -L -o tid,psr,pcpu,comm -p "${bp}" 2>/dev/null | tail -n +2 | head -4
    done <<< "${burn_pids}"
  else
    echo "  (no burn processes)"
  fi
}

run_scenario() {
  local name="$1"
  local rs_aff="$2"
  local burn_aff="$3"
  local with_burn="$4"

  echo ""
  echo "========== ${name} =========="
  cleanup
  start_realsense "${rs_aff}"

  if [ "${with_burn}" = "yes" ]; then
    start_burn "${burn_aff}"
  fi

  measure_hz "${name}"
  local hz
  hz="$(parse_hz "/tmp/hz_${name}.log")"

  thread_snapshot "${name}"

  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${name}" "${rs_aff:-any}" "${burn_aff:-none}" "${with_burn}" \
    "${BURN_CORES}x${BURN_LOAD}%" "${hz}" >> "${RESULTS}"

  echo "  -> average rate: ${hz} Hz (target 15)"
}

main() {
  cleanup
  echo -e "scenario\trs_affinity\tburn_affinity\tburn\ttload\thz" > "${RESULTS}"

  run_scenario "A_baseline" "" "" "no"
  run_scenario "B_unpinned_burn" "" "" "yes"
  run_scenario "C_rs0_burn0" "0" "0" "yes"
  run_scenario "D_rs01_burn45" "0,1" "4,5" "yes"

  echo ""
  echo "========== SUMMARY =========="
  awk -F'\t' 'NR==1 {print; next} {printf "%-18s rs=%-8s burn=%-8s load=%-8s %s Hz\n", $1, $2, $3, $5, $6}' "${RESULTS}"
  cleanup
}

main "$@"
