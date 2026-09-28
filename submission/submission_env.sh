#!/usr/bin/env bash
# Shared settings for the generated submission copy.
INSPECTION_PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSPECTION_WORKSPACE_DIR="$(cd "$INSPECTION_PROJECT_DIR/../.." && pwd)"
export INSPECTION_CONTAINER="${INSPECTION_CONTAINER:-smart_community_submission}"
export INSPECTION_DATA_DIR="${INSPECTION_DATA_DIR:-$INSPECTION_WORKSPACE_DIR/runtime_data}"
export INSPECTION_GAZEBO_DIR="${INSPECTION_GAZEBO_DIR:-$INSPECTION_WORKSPACE_DIR/gazebo_cache}"
export DISPLAY="${DISPLAY:-:0}"
