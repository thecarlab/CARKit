#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MONITOR_URL="https://carlab-ada-monitor.udcarlab.chatgpt.site"
IMAGE="${IMAGE:-ariiees/carkit:latest}"

if [[ ${EUID} -eq 0 ]]; then
  INSTALL_USER="${SUDO_USER:-nvidia}"
  SUDO=()
else
  INSTALL_USER="$(id -un)"
  if ! sudo -n true; then
    [[ -t 0 ]] || fail_early="sudo authentication requires an interactive terminal"
    if [[ -n ${fail_early:-} ]]; then
      echo "CARKit setup error: ${fail_early}" >&2
      exit 1
    fi
    sudo -v
  fi
  SUDO=(sudo -n)
fi
INSTALL_GROUP="$(id -gn "${INSTALL_USER}")"

fail() {
  echo "CARKit setup error: $*" >&2
  exit 1
}

read_existing_setting() {
  local key=$1
  if "${SUDO[@]}" test -f /etc/carkit-webmonitor.env; then
    "${SUDO[@]}" awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' \
      /etc/carkit-webmonitor.env
  fi
}

vehicle_id="${1:-$(read_existing_setting CARKIT_VEHICLE_ID)}"
if [[ -z "${vehicle_id}" ]]; then
  detected_hostname="${HOSTNAME%%.*}"
  if [[ ${detected_hostname^^} =~ ^ADA[1-9][0-9]{0,2}$ ]]; then
    vehicle_id="${detected_hostname^^}"
  elif [[ -t 0 ]]; then
    read -r -p "Vehicle ID (for example ADA5): " vehicle_id
  else
    fail "pass the vehicle ID as the first argument (for example ADA5)"
  fi
fi
vehicle_id="${vehicle_id^^}"
[[ ${vehicle_id} =~ ^ADA[1-9][0-9]{0,2}$ ]] \
  || fail "vehicle ID must look like ADA5"

chassis="${2:-}"
if [[ -z "${chassis}" && -f "${ROOT_DIR}/.carkit/config.json" ]]; then
  chassis="$(sed -nE 's/.*"chassis"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' \
    "${ROOT_DIR}/.carkit/config.json" | head -n 1)"
fi
if [[ -z "${chassis}" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Chassis [f1tenth/osracer] (f1tenth): " chassis
    chassis="${chassis:-f1tenth}"
  else
    fail "pass f1tenth or osracer as the second argument"
  fi
fi
chassis="${chassis,,}"
[[ ${chassis} == "f1tenth" || ${chassis} == "osracer" ]] \
  || fail "chassis must be f1tenth or osracer"

reporter_token="${CARKIT_REPORTER_TOKEN:-$(read_existing_setting CARKIT_REPORTER_TOKEN)}"
if [[ -z "${reporter_token}" ]]; then
  if [[ -t 0 ]]; then
    read -r -s -p "CARLab monitor token (input hidden): " reporter_token
    echo
  else
    fail "set CARKIT_REPORTER_TOKEN for a first-time non-interactive setup"
  fi
fi
[[ ${#reporter_token} -ge 32 ]] || fail "the CARLab monitor token is invalid"

command -v systemctl >/dev/null || fail "systemd is not available"
packages=()
command -v docker >/dev/null || packages+=(docker.io)
command -v python3 >/dev/null || packages+=(python3)
command -v curl >/dev/null || packages+=(curl ca-certificates)
if ((${#packages[@]})); then
  echo "Installing host requirements: ${packages[*]}"
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y "${packages[@]}"
fi
"${SUDO[@]}" systemctl enable --now docker.service

if ! getent group docker >/dev/null; then
  "${SUDO[@]}" groupadd --system docker
fi
if ! id -nG "${INSTALL_USER}" | tr ' ' '\n' | grep -qx docker; then
  "${SUDO[@]}" usermod -aG docker "${INSTALL_USER}"
  echo "Added ${INSTALL_USER} to the docker group."
fi

if [[ $("${SUDO[@]}" docker info --format '{{json .Runtimes}}') != *'"nvidia"'* ]]; then
  if ! command -v nvidia-ctk >/dev/null; then
    echo "Installing the NVIDIA Container Toolkit..."
    "${SUDO[@]}" apt-get update
    "${SUDO[@]}" apt-get install -y nvidia-container-toolkit
  fi
  "${SUDO[@]}" nvidia-ctk runtime configure --runtime=docker
  "${SUDO[@]}" systemctl restart docker.service
fi
[[ $("${SUDO[@]}" docker info --format '{{json .Runtimes}}') == *'"nvidia"'* ]] \
  || fail "the NVIDIA Docker runtime could not be configured"

echo "Pulling ${IMAGE}..."
"${SUDO[@]}" docker pull "${IMAGE}"

if [[ ${chassis} == "osracer" ]]; then
  echo "Installing OSRacer host device and LakiBeam network rules..."
  USER="${INSTALL_USER}" "${ROOT_DIR}/docker/setup_osracer_device.sh"
fi

escaped_root="${ROOT_DIR//\\/\\\\}"
escaped_root="${escaped_root//&/\\&}"
escaped_root="${escaped_root//|/\\|}"
service_tmp="$(mktemp)"
trap 'rm -f "${service_tmp}"' EXIT
sed \
  -e "s|@CARKIT_USER@|${INSTALL_USER}|g" \
  -e "s|@CARKIT_GROUP@|${INSTALL_GROUP}|g" \
  -e "s|@CARKIT_ROOT@|${escaped_root}|g" \
  "${ROOT_DIR}/docker/carkit.service.in" > "${service_tmp}"
"${SUDO[@]}" install -m 0644 "${service_tmp}" /etc/systemd/system/carkit.service

printf '%s\n' "${reporter_token}" \
  | "${SUDO[@]}" "${ROOT_DIR}/docker/webmonitor/install-reporter.sh" \
      "${vehicle_id}" "${MONITOR_URL}"
unset reporter_token

"${SUDO[@]}" systemctl daemon-reload
if "${SUDO[@]}" systemctl is-active --quiet carkit.service; then
  "${SUDO[@]}" systemctl stop carkit.service
fi
if "${SUDO[@]}" docker container inspect carkit >/dev/null 2>&1; then
  fail "a manually started 'carkit' container is running; stop it and rerun setup"
fi
"${SUDO[@]}" systemctl enable carkit.service carkit-webmonitor.service
"${SUDO[@]}" systemctl restart carkit-webmonitor.service
"${SUDO[@]}" systemctl restart carkit.service

"${SUDO[@]}" systemctl is-active --quiet carkit.service \
  || fail "carkit.service did not start; run: journalctl -u carkit.service -n 100"
"${SUDO[@]}" systemctl is-active --quiet carkit-webmonitor.service \
  || fail "carkit-webmonitor.service did not start"

webui_ready=0
for _ in {1..60}; do
  if curl --fail --silent --max-time 2 http://127.0.0.1:8080/ >/dev/null; then
    webui_ready=1
    break
  fi
  sleep 2
done
[[ ${webui_ready} -eq 1 ]] \
  || fail "WebUI did not become ready; run: journalctl -u carkit.service -n 100"

echo
echo "CARKit setup complete for ${vehicle_id} (${chassis})."
echo "WebUI/container autostart: enabled"
echo "Hourly IP reporting: enabled"
echo "Fleet monitor: ${MONITOR_URL}"
echo "Local WebUI: http://<this-vehicle-IP>:8080"
