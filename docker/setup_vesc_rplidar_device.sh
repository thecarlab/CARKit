#!/usr/bin/env bash
set -euo pipefail
SUDO=()
if [[ $EUID -ne 0 ]]; then SUDO=(sudo -n); fi
"${SUDO[@]}" tee /etc/udev/rules.d/99-carkit-vesc-rplidar.rules >/dev/null <<'RULES'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ATTRS{product}=="ChibiOS/RT Virtual COM Port", SYMLINK+="carkit_vesc", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="carkit_rplidar", GROUP="dialout", MODE="0660"
RULES
"${SUDO[@]}" udevadm control --reload-rules
"${SUDO[@]}" udevadm trigger --subsystem-match=tty
"${SUDO[@]}" udevadm settle
ls -l /dev/carkit_vesc /dev/carkit_rplidar
