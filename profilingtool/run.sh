#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"

HOST="${PROFILER_HOST:-0.0.0.0}"
PORT="${PROFILER_PORT:-8000}"
CONTAINER="${PROFILER_CONTAINER:-carkit}"

exec python3 "${ROOT}/server.py" --host "${HOST}" --port "${PORT}" --container "${CONTAINER}"
