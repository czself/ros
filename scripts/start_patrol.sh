#!/bin/bash
# Start AMCL, move_base, mission patrol, and camera inspection on a saved map.
set -e

CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_NAME="${1:-}"
[[ -n "$MAP_NAME" ]] || { echo "必须显式提供本次重新扫图的地图名" >&2; exit 2; }

"$ROOT_DIR/scripts/start_navigation.sh" "/root/ros1_ws/maps/$MAP_NAME.yaml"
docker cp "$ROOT_DIR/scripts/patrol_controller.py" "$CONTAINER:/root/patrol_controller.py"
docker cp "$ROOT_DIR/scripts/route_executor.py" "$CONTAINER:/root/route_executor.py"
docker cp "$ROOT_DIR/scripts/visual_inspector.py" "$CONTAINER:/root/visual_inspector.py"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode kill /patrol_controller /route_executor /visual_inspector 2>/dev/null || true'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/visual_inspector.py >/root/visual_inspector.log 2>&1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/route_executor.py >/root/route_executor.log 2>&1'
echo "固定路线巡检已启动。路线执行器发送 /move_base 固定目标，并由 /cmd_vel_watchdog 实施红绿灯门控。"
