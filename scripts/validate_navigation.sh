#!/bin/bash
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
docker cp "$ROOT_DIR/scripts/validate_navigation.py" "$CONTAINER:/root/validate_navigation.py"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 -u /root/validate_navigation.py'
