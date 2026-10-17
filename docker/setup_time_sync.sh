#!/usr/bin/env bash
# Configure the host clock; containers share this kernel clock.
set -euo pipefail
if [[ ${EUID} -ne 0 ]]; then
  echo "Run time synchronization setup as root." >&2
  exit 1
fi
if [[ $(timedatectl show --property=CanNTP --value) != yes ]]; then
  if [[ ${1:-} != --install ]]; then
    echo "No time synchronization provider found; rerun docker/setup_jetson.sh." >&2
    exit 1
  fi
  apt-get update
  apt-get install -y systemd-timesyncd
  systemctl daemon-reload
fi
# systemd selects the installed provider (including chrony when configured).
timedatectl set-ntp true
echo "CARKit host time synchronization enabled; reporter waits for synchronization."
