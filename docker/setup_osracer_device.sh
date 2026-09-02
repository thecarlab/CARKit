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
LIDAR_CONNECTION="carkit-lakibeam"
LIDAR_HOST_ADDRESS="192.168.8.1/24"
LIDAR_SENSOR_ADDRESS="192.168.8.2"

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

# The LakiBeam USB-C connection exposes a Richbeam RNDIS network adapter.
# Give that adapter its own link-only subnet so NetworkManager cannot attach it
# to NVIDIA's l4tbr0 bridge or use the CUDY gateway for sensor traffic.
LIDAR_INTERFACE=""
for interface_path in /sys/class/net/*; do
  interface_properties="$(
    udevadm info --query=property --path="${interface_path}" 2>/dev/null || true
  )"
  if grep -qx 'ID_VENDOR=Richbeam' <<<"${interface_properties}" \
    && grep -qx 'ID_MODEL_ID=0137' <<<"${interface_properties}"; then
    LIDAR_INTERFACE="${interface_path##*/}"
    break
  fi
done

if [ -n "${LIDAR_INTERFACE}" ]; then
  LIDAR_CONNECTION_UUID="$(
    sudo nmcli --terse --fields UUID,NAME connection show \
      | awk -F: -v name="${LIDAR_CONNECTION}" '$2 == name {print $1; exit}'
  )"
  if [ -z "${LIDAR_CONNECTION_UUID}" ]; then
    sudo nmcli connection add \
      type ethernet \
      ifname "${LIDAR_INTERFACE}" \
      con-name "${LIDAR_CONNECTION}"
    LIDAR_CONNECTION_UUID="$(
      sudo nmcli --terse --fields UUID,NAME connection show \
        | awk -F: -v name="${LIDAR_CONNECTION}" '$2 == name {print $1; exit}'
    )"
  fi

  sudo nmcli connection modify "${LIDAR_CONNECTION_UUID}" \
    connection.interface-name "${LIDAR_INTERFACE}" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100 \
    ipv4.method manual \
    ipv4.addresses "${LIDAR_HOST_ADDRESS}" \
    ipv4.gateway "" \
    ipv4.never-default yes \
    ipv4.ignore-auto-routes yes \
    ipv4.ignore-auto-dns yes \
    ipv6.method disabled

  LIDAR_ACTIVE_UUID="$(
    nmcli --get-values GENERAL.CON-UUID device show "${LIDAR_INTERFACE}" \
      2>/dev/null || true
  )"
  if [ "${LIDAR_ACTIVE_UUID}" != "${LIDAR_CONNECTION_UUID}" ] \
    || ! ip -4 address show dev "${LIDAR_INTERFACE}" \
      | grep -q "inet ${LIDAR_HOST_ADDRESS}" \
    || ip -o link show dev "${LIDAR_INTERFACE}" | grep -q ' master '; then
    sudo nmcli connection up uuid "${LIDAR_CONNECTION_UUID}"
  else
    echo "LakiBeam network connection is already active."
  fi

  if ! ip -4 address show dev "${LIDAR_INTERFACE}" \
    | grep -q "inet ${LIDAR_HOST_ADDRESS}"; then
    echo "OSRacer error: failed to configure ${LIDAR_INTERFACE} as ${LIDAR_HOST_ADDRESS}." >&2
    exit 1
  fi

  echo "LakiBeam network ready: ${LIDAR_INTERFACE}=${LIDAR_HOST_ADDRESS} -> ${LIDAR_SENSOR_ADDRESS}"
  if ! ping -c 1 -W 1 "${LIDAR_SENSOR_ADDRESS}" >/dev/null 2>&1; then
    echo "OSRacer warning: LakiBeam ${LIDAR_SENSOR_ADDRESS} did not answer ping." >&2
  fi
else
  echo "OSRacer warning: Richbeam LakiBeam USB network adapter is not connected." >&2
fi

if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
  sudo usermod -aG dialout "${USER}"
  echo "Added ${USER} to dialout; log out and back in before running CARKit."
fi
