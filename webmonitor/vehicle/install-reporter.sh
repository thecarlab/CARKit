#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: sudo $0 VEHICLE_ID ENDPOINT" >&2
  echo "The reporter token is read from standard input." >&2
}

if [[ $# -ne 2 ]] || [[ ${EUID} -ne 0 ]]; then
  usage
  exit 2
fi

vehicle_id=${1^^}
endpoint=${2%/}/api/check-in
if [[ ! ${vehicle_id} =~ ^ADA[1-9][0-9]{0,2}$ ]] || [[ ! ${endpoint} =~ ^https:// ]]; then
  usage
  exit 2
fi

IFS= read -r reporter_token
if [[ ${#reporter_token} -lt 32 ]]; then
  echo "Reporter token must be at least 32 characters." >&2
  exit 2
fi

install -d -m 0755 /usr/local/lib/carkit-webmonitor
install -m 0755 "$(dirname "$0")/report_ip.py" /usr/local/lib/carkit-webmonitor/report_ip.py
install -m 0644 "$(dirname "$0")/carkit-webmonitor.service" /etc/systemd/system/carkit-webmonitor.service

config_tmp=$(mktemp /etc/carkit-webmonitor.env.XXXXXX)
chmod 0600 "${config_tmp}"
printf 'CARKIT_VEHICLE_ID=%s\n' "${vehicle_id}" >> "${config_tmp}"
printf 'CARKIT_MONITOR_ENDPOINT=%s\n' "${endpoint}" >> "${config_tmp}"
printf 'CARKIT_REPORTER_TOKEN=%s\n' "${reporter_token}" >> "${config_tmp}"
printf 'CARKIT_WEBUI_PORT=8080\nCARKIT_REPORT_INTERVAL=3600\nCARKIT_REPORT_RETRY=60\n' >> "${config_tmp}"
mv "${config_tmp}" /etc/carkit-webmonitor.env

systemctl daemon-reload
systemctl enable --now carkit-webmonitor.service
