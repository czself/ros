#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
export JUDGE_DEMO=true
exec "$ROOT_DIR/scripts/start_standee_photo_route.sh" "${1:-/root/ros1_ws/maps/current_slam_preview_white_lines.yaml}"
