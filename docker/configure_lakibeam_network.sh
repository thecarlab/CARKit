#!/usr/bin/env bash

# Configure the Richbeam RNDIS link independently of its randomized interface
# name. This script is safe to run repeatedly and is used at every CARKit boot.
set -euo pipefail

LIDAR_CONNECTION="carkit-lakibeam"
LIDAR_HOST_ADDRESS="192.168.8.1/24"
LIDAR_SENSOR_ADDRESS="192.168.8.2"

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

if [[ -z ${LIDAR_INTERFACE} ]]; then
  echo "CARKit LakiBeam: Richbeam USB network adapter is not connected; skipping."
  exit 0
fi

command -v nmcli >/dev/null \
  || { echo "CARKit LakiBeam error: nmcli is not installed." >&2; exit 1; }

LIDAR_CONNECTION_UUID="$(
  nmcli --terse --fields UUID,NAME connection show \
    | awk -F: -v name="${LIDAR_CONNECTION}" '$2 == name {print $1; exit}'
)"
LIDAR_ACTIVE_UUID="$(
  nmcli --get-values GENERAL.CON-UUID device show "${LIDAR_INTERFACE}" \
    2>/dev/null || true
)"
if [[ -n ${LIDAR_CONNECTION_UUID} \
  && ${LIDAR_ACTIVE_UUID} == "${LIDAR_CONNECTION_UUID}" ]] \
  && ip -4 address show dev "${LIDAR_INTERFACE}" \
    | grep -q "inet ${LIDAR_HOST_ADDRESS}" \
  && ! ip -o link show dev "${LIDAR_INTERFACE}" | grep -q ' master '; then
  echo "CARKit LakiBeam: network connection is already active."
  echo "CARKit LakiBeam ready: ${LIDAR_INTERFACE}=${LIDAR_HOST_ADDRESS} -> ${LIDAR_SENSOR_ADDRESS}"
  exit 0
fi

if [[ ${EUID} -eq 0 ]]; then
  SUDO=()
elif sudo -n true 2>/dev/null; then
  SUDO=(sudo -n)
else
  echo "CARKit LakiBeam error: root access is required to configure NetworkManager." >&2
  exit 1
fi

if [[ -z ${LIDAR_CONNECTION_UUID} ]]; then
  "${SUDO[@]}" nmcli connection add \
    type ethernet \
    ifname "${LIDAR_INTERFACE}" \
    con-name "${LIDAR_CONNECTION}"
  LIDAR_CONNECTION_UUID="$(
    "${SUDO[@]}" nmcli --terse --fields UUID,NAME connection show \
      | awk -F: -v name="${LIDAR_CONNECTION}" '$2 == name {print $1; exit}'
  )"
fi

# Rebind the persistent profile because Richbeam supplies a different RNDIS
# MAC address, and therefore a different enx* name, across some cold boots.
"${SUDO[@]}" nmcli connection modify "${LIDAR_CONNECTION_UUID}" \
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

if [[ ${LIDAR_ACTIVE_UUID} != "${LIDAR_CONNECTION_UUID}" ]] \
  || ! ip -4 address show dev "${LIDAR_INTERFACE}" \
    | grep -q "inet ${LIDAR_HOST_ADDRESS}" \
  || ip -o link show dev "${LIDAR_INTERFACE}" | grep -q ' master '; then
  "${SUDO[@]}" nmcli connection up uuid "${LIDAR_CONNECTION_UUID}"
else
  echo "CARKit LakiBeam: network connection is already active."
fi

if ! ip -4 address show dev "${LIDAR_INTERFACE}" \
  | grep -q "inet ${LIDAR_HOST_ADDRESS}"; then
  echo "CARKit LakiBeam error: ${LIDAR_INTERFACE} does not have ${LIDAR_HOST_ADDRESS}." >&2
  exit 1
fi
if ip -o link show dev "${LIDAR_INTERFACE}" | grep -q ' master '; then
  echo "CARKit LakiBeam error: ${LIDAR_INTERFACE} is still attached to a bridge." >&2
  exit 1
fi

echo "CARKit LakiBeam ready: ${LIDAR_INTERFACE}=${LIDAR_HOST_ADDRESS} -> ${LIDAR_SENSOR_ADDRESS}"
if ! ping -c 1 -W 1 "${LIDAR_SENSOR_ADDRESS}" >/dev/null 2>&1; then
  echo "CARKit LakiBeam warning: ${LIDAR_SENSOR_ADDRESS} did not answer ping." >&2
fi
