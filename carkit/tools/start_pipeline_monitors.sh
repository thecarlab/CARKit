#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  carkit/tools/start_pipeline_monitors.sh [NAME] [--duration SECONDS]
  carkit/tools/start_pipeline_monitors.sh [NAME] --dry-run

NAME is optional. Output is written to:
  test/<NAME>_<YYYYmmdd_HHMMSS>/

Without NAME, the directory name is only the timestamp. Without --duration,
the monitors run until Ctrl+C. When NAME is supplied, older monitor directories
with the same NAME are deleted regardless of their timestamp. Five CSV files
are recorded: system, node CPU, pipeline rates, pipeline events, and stop
latency.
EOF
}

custom_name=""
duration_sec=0
dry_run=false

while (($#)); do
  case "$1" in
    --duration)
      [[ $# -ge 2 ]] || { echo "--duration requires a value" >&2; exit 2; }
      duration_sec="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      [[ -z "${custom_name}" ]] || {
        echo "Only one custom NAME may be supplied" >&2
        exit 2
      }
      custom_name="$1"
      shift
      ;;
  esac
done

[[ "${duration_sec}" =~ ^[0-9]+$ ]] || {
  echo "--duration must be a non-negative whole number of seconds" >&2
  exit 2
}
if [[ "${custom_name}" == "." || "${custom_name}" == ".." ||
      "${custom_name}" == *"/"* || "${custom_name}" == *"\\"* ||
      "${custom_name}" =~ [[:cntrl:]] ]]; then
  echo "NAME cannot be '.', '..', or contain slashes/control characters" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd "${script_dir}/../.." && pwd -P)"
timestamp="$(date +%Y%m%d_%H%M%S)"
directory_name="${timestamp}"
if [[ -n "${custom_name}" ]]; then
  directory_name="${custom_name}_${timestamp}"
fi

test_root="${repo_root}/test"
output_dir="${test_root}/${directory_name}"

system_csv="${output_dir}/system_metrics.csv"
node_csv="${output_dir}/node_cpu_metrics.csv"
rate_csv="${output_dir}/pipeline_rates.csv"
event_csv="${output_dir}/pipeline_events.csv"
latency_csv="${output_dir}/stop_latency.csv"

system_command=(
  python3 "${repo_root}/carkit/tools/jetson_system_monitor.py"
  mode2 --no-clear --output "${system_csv}"
)
node_command=(
  python3 "${repo_root}/carkit/tools/node_cpu_monitor.py"
  --terminal --no-clear --csv --output "${node_csv}"
)
pipeline_command=(
  ros2 run carkit_latency_monitor pipeline_rate_monitor
  --ros-args
  -p "output_path:=${rate_csv}"
  -p "event_output_path:=${event_csv}"
)
latency_command=(
  ros2 run carkit_latency_monitor stop_latency_monitor
  --ros-args
  -p "output_path:=${latency_csv}"
)

print_command() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

if ${dry_run}; then
  echo "Output directory: ${output_dir}"
  echo "Commands:"
  print_command "${system_command[@]}"
  print_command "${node_command[@]}"
  print_command "${pipeline_command[@]}"
  print_command "${latency_command[@]}"
  exit 0
fi

setup_file="${repo_root}/install/setup.bash"
[[ -r "${setup_file}" ]] || {
  echo "Missing ${setup_file}; build the workspace first" >&2
  exit 1
}
# ROS/colcon setup files may probe optional environment variables.
set +u
# shellcheck disable=SC1090
source "${setup_file}"
set -u
command -v setsid >/dev/null || {
  echo "setsid is required to manage monitor process groups" >&2
  exit 1
}
command -v ros2 >/dev/null || {
  echo "ros2 is unavailable after sourcing ${setup_file}" >&2
  exit 1
}
python3 -c 'import psutil, rclpy' >/dev/null || {
  echo "Python packages psutil and rclpy are required" >&2
  exit 1
}
latency_monitor_executables="$(
  ros2 pkg executables carkit_latency_monitor 2>/dev/null
)" || {
  echo "Cannot query executables from carkit_latency_monitor" >&2
  exit 1
}
grep -Eq '(^|[[:space:]])pipeline_rate_monitor$' \
  <<<"${latency_monitor_executables}" || {
  echo "pipeline_rate_monitor is not installed; build carkit_latency_monitor" >&2
  exit 1
}
grep -Eq '(^|[[:space:]])stop_latency_monitor$' \
  <<<"${latency_monitor_executables}" || {
  echo "stop_latency_monitor is not installed; build carkit_latency_monitor" >&2
  exit 1
}

delete_output_directory() {
  local target="$1"
  case "${target}" in
    "${test_root}/"*) ;;
    *)
      echo "Refusing to replace output outside ${test_root}" >&2
      exit 1
      ;;
  esac
  echo "Replacing existing output directory: ${target}"
  find "${target}" -xdev -depth -delete
}

if [[ -n "${custom_name}" ]]; then
  custom_prefix_length=$((${#custom_name} + 1))
  for previous_output in "${test_root}"/*; do
    [[ -d "${previous_output}" || -L "${previous_output}" ]] || continue
    previous_name="${previous_output##*/}"
    if [[ "${previous_name}" == "${custom_name}" ]]; then
      delete_output_directory "${previous_output}"
      continue
    fi
    case "${previous_name}" in
      "${custom_name}"_????????_??????)
        previous_timestamp="${previous_name:${custom_prefix_length}}"
        if [[ "${previous_timestamp}" =~ ^[0-9]{8}_[0-9]{6}$ ]]; then
          delete_output_directory "${previous_output}"
        fi
        ;;
    esac
  done
elif [[ -e "${output_dir}" || -L "${output_dir}" ]]; then
  delete_output_directory "${output_dir}"
fi
mkdir -p "${output_dir}"
declare -a monitor_names=()
declare -a monitor_pids=()
declare -a monitor_logs=()
stopping=false

start_monitor() {
  local name="$1"
  local log_path="$2"
  shift 2
  setsid bash -c 'trap - INT TERM; exec "$@"' _ "$@" \
    >"${log_path}" 2>&1 &
  monitor_names+=("${name}")
  monitor_pids+=("$!")
  monitor_logs+=("${log_path}")
}

process_is_active() {
  local state
  state="$(ps -o stat= -p "$1" 2>/dev/null)" || return 1
  state="${state//[[:space:]]/}"
  [[ -n "${state}" && "${state}" != Z* ]]
}

cleanup() {
  local pid
  ${stopping} && return
  stopping=true
  trap - EXIT INT TERM

  # All monitors flush every completed row; rclcpp also handles SIGTERM.
  for pid in "${monitor_pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  for _ in {1..50}; do
    local running=false
    for pid in "${monitor_pids[@]}"; do
      process_is_active "${pid}" && running=true
    done
    ${running} || break
    sleep 0.1
  done
  for pid in "${monitor_pids[@]}"; do
    if process_is_active "${pid}"; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
  done

  echo
  echo "Monitors stopped. CSV files:"
  printf '  %s\n' \
    "${system_csv}" \
    "${node_csv}" \
    "${rate_csv}" \
    "${event_csv}" \
    "${latency_csv}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_monitor "Jetson system" "${output_dir}/system_monitor.log" \
  "${system_command[@]}"
start_monitor "Node CPU" "${output_dir}/node_cpu_monitor.log" \
  "${node_command[@]}"
start_monitor "Pipeline rate/events" "${output_dir}/pipeline_monitor.log" \
  "${pipeline_command[@]}"
start_monitor "Stop latency" "${output_dir}/stop_latency_monitor.log" \
  "${latency_command[@]}"

startup_timeout_sec=15
startup_deadline=$((SECONDS + startup_timeout_sec))
csv_paths=(
  "${system_csv}"
  "${node_csv}"
  "${rate_csv}"
  "${event_csv}"
  "${latency_csv}"
)
while true; do
  for index in "${!monitor_pids[@]}"; do
    if ! process_is_active "${monitor_pids[index]}"; then
      echo "${monitor_names[index]} monitor failed to start; see ${monitor_logs[index]}" >&2
      exit 1
    fi
  done

  all_csv_ready=true
  for csv_path in "${csv_paths[@]}"; do
    if [[ ! -s "${csv_path}" ]]; then
      all_csv_ready=false
      break
    fi
  done
  ${all_csv_ready} && break

  if ((SECONDS >= startup_deadline)); then
    echo "Monitors did not initialize all CSV files within ${startup_timeout_sec}s:" >&2
    for csv_path in "${csv_paths[@]}"; do
      [[ -s "${csv_path}" ]] || echo "  missing/empty: ${csv_path}" >&2
    done
    echo "Monitor logs are in ${output_dir}" >&2
    exit 1
  fi
  sleep 0.1
done

echo "Four monitors are running; five CSV files are being recorded in:"
echo "  ${output_dir}"
echo "Press Ctrl+C to stop them cleanly."

start_time="${SECONDS}"
while true; do
  if [[ "${duration_sec}" != "0" ]] &&
     ((SECONDS - start_time >= duration_sec)); then
    break
  fi
  for index in "${!monitor_pids[@]}"; do
    if ! kill -0 "${monitor_pids[index]}" 2>/dev/null; then
      echo "${monitor_names[index]} monitor exited unexpectedly; see ${output_dir}" >&2
      exit 1
    fi
  done
  sleep 1
done
