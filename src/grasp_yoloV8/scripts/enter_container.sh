#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

xhost +local:
docker compose -f "${ROOT_DIR}/docker-compose.yml" up -d
docker exec -it rm_humble bash
