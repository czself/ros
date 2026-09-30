#!/usr/bin/env bash
set -eo pipefail
WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
set -u
exec python3 "$WORKSPACE/src/community_inspection/scripts/native_demo.py" "$@"
