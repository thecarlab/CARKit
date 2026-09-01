#!/usr/bin/env bash
# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

DEVICE="${1:-/dev/osrbot_base}"

echo "Checking OSRacer device: ${DEVICE}"
if [[ -c "${DEVICE}" ]]; then
  ls -l "${DEVICE}"
  if [[ ! -r "${DEVICE}" || ! -w "${DEVICE}" ]]; then
    echo "ERROR: current user cannot read and write ${DEVICE}." >&2
    exit 1
  fi
  if command -v udevadm >/dev/null 2>&1; then
    UDEV_INFO="$(udevadm info --query=property --name="${DEVICE}" 2>/dev/null || true)"
    VENDOR_ID="$(printf '%s\n' "${UDEV_INFO}" | awk -F= '$1 == "ID_VENDOR_ID" {print $2; exit}')"
    MODEL_ID="$(printf '%s\n' "${UDEV_INFO}" | awk -F= '$1 == "ID_MODEL_ID" {print $2; exit}')"
    SERIAL="$(printf '%s\n' "${UDEV_INFO}" | awk -F= '$1 == "ID_SERIAL_SHORT" {print $2; exit}')"
    RAW_VENDOR="$(printf '%s\n' "${UDEV_INFO}" | awk -F= '$1 == "ID_VENDOR" {print $2; exit}')"
    RAW_MODEL="$(printf '%s\n' "${UDEV_INFO}" | awk -F= '$1 == "ID_MODEL" {print $2; exit}')"

    MANUFACTURER="${RAW_VENDOR//_/ }"
    PRODUCT="${RAW_MODEL//_/ }"

    [[ -n "${VENDOR_ID}" ]] && echo "USB vendor ID: ${VENDOR_ID}"
    [[ -n "${MODEL_ID}" ]] && echo "USB product ID: ${MODEL_ID}"
    [[ -n "${MANUFACTURER}" ]] && echo "Manufacturer: ${MANUFACTURER}"
    [[ -n "${PRODUCT}" ]] && echo "Product: ${PRODUCT}"
    [[ -n "${SERIAL}" ]] && echo "Serial: ${SERIAL}"
  fi
  exit 0
fi

if [[ -e "${DEVICE}" ]]; then
  echo "ERROR: ${DEVICE} exists but is not a serial character device." >&2
  echo "Run ./docker/setup_osracer_device.sh on the host, then restart Docker." >&2
  exit 1
fi

echo "MISSING ${DEVICE}"
echo
echo "Available serial devices:"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
echo
echo "If the vehicle is connected, install the udev rule and reconnect USB:"
echo "  ros2 run osracer_base install_udev_rules"
exit 1
