#!/usr/bin/env bash

# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"
PORT="${CARKIT_WEB_PORT:-8080}"

echo "Starting CARKit WebUI on http://${CARKIT_WEBUI_HOST:-127.0.0.1}:${PORT}"
exec python3 \
  "${WORKSPACE}/carkit/interface/carkit_webui/carkit_webui/server.py" \
  --workspace "${WORKSPACE}" \
  --host "${CARKIT_WEBUI_HOST:-127.0.0.1}" \
  --port "${PORT}"
