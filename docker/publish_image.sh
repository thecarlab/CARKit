#!/usr/bin/env bash

set -euo pipefail

IMAGE="${IMAGE:-ariiees/carkit:latest}"
DOCKERFILE="${DOCKERFILE:-docker/Dockerfile.jetson}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT_DIR"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not on PATH" >&2
  exit 1
fi

docker build -f "$DOCKERFILE" -t "$IMAGE" .
docker push "$IMAGE"
