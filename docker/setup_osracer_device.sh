#!/usr/bin/env bash

# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SOURCE="${ROOT_DIR}/carkit/vehicle/osracer/osracer_base/udev/99-osrbot-osracer.rules"
RULE_TARGET="/etc/udev/rules.d/99-osrbot-osracer.rules"
CAMERA_RULE_SOURCE="${ROOT_DIR}/carkit/vehicle/osracer/osracer_bringup/udev/99-osrbot-usb-cam.rules"
CAMERA_RULE_TARGET="/etc/udev/rules.d/99-osrbot-usb-cam.rules"
DEVICE="/dev/osrbot_base"
CAMERA_DEVICE="/dev/osrbot_usb_cam"

if [ ! -f "${RULE_SOURCE}" ]; then
  echo "OSRacer udev rule not found: ${RULE_SOURCE}" >&2
  exit 1
fi

if [ -e "${DEVICE}" ] && [ ! -c "${DEVICE}" ]; then
  if [ -f "${DEVICE}" ] && [ ! -s "${DEVICE}" ]; then
    echo "Removing stale empty file ${DEVICE} created by the old Docker bind mount."
    sudo unlink "${DEVICE}"
  else
    echo "Refusing to replace unexpected non-device path: ${DEVICE}" >&2
    ls -ld "${DEVICE}" >&2
    exit 1
  fi
fi

sudo install -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
sudo install -m 0644 "${CAMERA_RULE_SOURCE}" "${CAMERA_RULE_TARGET}"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo udevadm trigger --subsystem-match=video4linux
sudo udevadm settle

if [ ! -c "${DEVICE}" ]; then
  cat >&2 <<EOF
The rule was installed, but ${DEVICE} is not present yet.
Reconnect the OSRacer USB cable, then run this script again.
Available controller candidates:
$(ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
EOF
  exit 1
fi

echo "OSRacer device ready: $(ls -l "${DEVICE}")"
udevadm info --query=property --name="${DEVICE}" \
  | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT)=' || true

if [ -c "${CAMERA_DEVICE}" ]; then
  echo "OSRacer camera ready: $(ls -l "${CAMERA_DEVICE}")"
else
  echo "OSRacer warning: ${CAMERA_DEVICE} is missing; camera launch will not work." >&2
fi

if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
  sudo usermod -aG dialout "${USER}"
  echo "Added ${USER} to dialout; log out and back in before running CARKit."
fi
