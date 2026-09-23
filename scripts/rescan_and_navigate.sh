#!/usr/bin/env bash
set -euo pipefail
CONTAINER="ros1_modeling"
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-manual}"
[[ "$MODE" == manual ]] || { echo "用法: $0 manual" >&2; exit 2; }
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行，请先执行 ./scripts/start_sim.sh" >&2; exit 1; }
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
MAP_NAME="competition_rescan_${STAMP}"
echo "启动全新 GMapping 会话，地图输出: maps/${MAP_NAME}.yaml"
"$ROOT_DIR/scripts/start_slam_mapping.sh"
echo "现在请执行 ./scripts/teleop.sh，低速覆盖本次场景所有可达通道。"
read -r -p "完成覆盖后按 Enter 保存并切换到导航；Ctrl-C 可取消: " _
"$ROOT_DIR/scripts/save_slam_map.sh" "$MAP_NAME" manual_rescan operator_requested "$STAMP" manual_teleop
"$ROOT_DIR/scripts/start_navigation.sh" "/root/ros1_ws/maps/${MAP_NAME}.yaml"
echo "已使用本次新扫地图导航: ${MAP_NAME}"
