#!/bin/bash
# Start AMCL, move_base, mission patrol, and camera inspection on a saved map.
set -e

CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_NAME="${1:-competition_navigation_safe}"

"$ROOT_DIR/scripts/start_navigation.sh" "/root/ros1_ws/maps/$MAP_NAME.yaml"
docker cp "$ROOT_DIR/scripts/patrol_controller.py" "$CONTAINER:/root/patrol_controller.py"
docker cp "$ROOT_DIR/scripts/visual_inspector.py" "$CONTAINER:/root/visual_inspector.py"
docker exec "$CONTAINER" bash -lc '
  source /opt/ros/noetic/setup.bash
  rosnode kill /patrol_controller /visual_inspector 2>/dev/null || true
  nohup python3 /root/visual_inspector.py > /root/visual_inspector.log 2>&1 &
  nohup python3 /root/patrol_controller.py > /root/patrol_controller.log 2>&1 &
'
echo "巡检已启动。先在 RViz 使用 2D Pose Estimate 初始化定位，再由 /patrol_controller 依次发送巡检目标。"
