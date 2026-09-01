#!/usr/bin/env bash

# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

WORKSPACE="${WORKSPACE:-/workspaces/CARKit}"
PORT="${CARKIT_WEB_PORT:-8080}"

echo "Starting CARKit WebUI on http://0.0.0.0:${PORT}"
exec python3 \
  "${WORKSPACE}/carkit/interface/carkit_webui/carkit_webui/server.py" \
  --workspace "${WORKSPACE}" \
  --host 0.0.0.0 \
  --port "${PORT}"
