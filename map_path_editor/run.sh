#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"

HOST="${MAP_PATH_EDITOR_HOST:-0.0.0.0}"
PORT="${MAP_PATH_EDITOR_PORT:-8010}"
MAP_DIR="${MAP_PATH_EDITOR_MAP_DIR:-${ROOT}/../map}"

exec python3 "${ROOT}/server.py" --host "${HOST}" --port "${PORT}" --map-dir "${MAP_DIR}"
