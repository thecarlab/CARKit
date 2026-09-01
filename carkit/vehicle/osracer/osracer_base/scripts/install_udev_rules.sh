#!/usr/bin/env bash
# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULE_SRC="${REPO_ROOT}/udev/99-osrbot-osracer.rules"
RULE_DST="/etc/udev/rules.d/99-osrbot-osracer.rules"

if [[ ! -f "${RULE_SRC}" ]]; then
  echo "ERROR: udev rule not found: ${RULE_SRC}"
  exit 1
fi

sudo install -m 0644 "${RULE_SRC}" "${RULE_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -a -G dialout "${USER}"

echo "Installed ${RULE_DST}"
echo "Reconnect the vehicle USB cable."
echo "Log out and log back in if this user was not already in the dialout group."
