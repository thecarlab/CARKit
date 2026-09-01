#!/usr/bin/env bash

# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"
CHASSIS="${1:-}"
F1TENTH_COMMIT="8f724985dd8517f44870b5348cca10a878935bea"
F1TENTH_VENDOR="${WORKSPACE}/carkit/vendor/ada_system"

case "${CHASSIS}" in
  osracer)
    if [ ! -d "${WORKSPACE}/carkit/vehicle/osracer/osracer_bringup" ]; then
      echo "CARKit error: the OSRacer platform source is missing." >&2
      exit 1
    fi
    ;;
  f1tenth)
    if [ ! -d "${WORKSPACE}/carkit/vehicle/f1tenth_system/f1tenth_stack" ] \
      && [ ! -d "${F1TENTH_VENDOR}/src/f1tenth_system/f1tenth_stack" ]; then
      echo "Fetching the pinned F1TENTH/VESC platform source..."
      mkdir -p "$(dirname "${F1TENTH_VENDOR}")"
      git clone --filter=blob:none --no-checkout \
        https://github.com/thecarlab/ada_system "${F1TENTH_VENDOR}"
      git -C "${F1TENTH_VENDOR}" sparse-checkout set src/f1tenth_system
      git -C "${F1TENTH_VENDOR}" checkout "${F1TENTH_COMMIT}"
    fi
    ;;
  *)
    echo "Usage: $0 osracer|f1tenth" >&2
    exit 2
    ;;
esac

echo "Installing CARKit with ${CHASSIS} chassis support..."
CARKIT_CHASSIS="${CHASSIS}" "${WORKSPACE}/docker/build_workspace.sh"
mkdir -p "${WORKSPACE}/.carkit"
config_tmp="${WORKSPACE}/.carkit/config.json.tmp"
printf '{\n  "chassis": "%s",\n  "installed_at": "%s"\n}\n' \
  "${CHASSIS}" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
  > "${config_tmp}"
mv "${config_tmp}" "${WORKSPACE}/.carkit/config.json"
echo "CARKit ${CHASSIS} installation complete."
