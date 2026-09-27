#!/bin/bash
# Start AMCL, move_base, mission patrol, and camera inspection on a saved map.
set -e

CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_NAME="${1:-}"
CAPTURE_PHOTOS="${CAPTURE_PHOTOS:-true}"
PHOTO_WAYPOINTS="${PHOTO_WAYPOINTS:-}"
PHOTO_POINTS_FILE="${PHOTO_POINTS_FILE:-/root/ros1_ws/photo_stops/teleop_points/points.json}"
[[ -n "$MAP_NAME" ]] || { echo "必须显式提供本次重新扫图的地图名" >&2; exit 2; }

"$ROOT_DIR/scripts/start_navigation.sh" "/root/ros1_ws/maps/$MAP_NAME.yaml"
docker cp "$ROOT_DIR/scripts/patrol_controller.py" "$CONTAINER:/root/patrol_controller.py"
docker cp "$ROOT_DIR/scripts/route_executor.py" "$CONTAINER:/root/route_executor.py"
docker exec "$CONTAINER" mkdir -p /root/ros1_ws/photo_stops/teleop_points
docker cp "$ROOT_DIR/../ros1_ws/photo_stops/teleop_points/points.json" "$CONTAINER:$PHOTO_POINTS_FILE"
docker cp "$ROOT_DIR/scripts/visual_inspector.py" "$CONTAINER:/root/visual_inspector.py"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode kill /patrol_controller /route_executor /visual_inspector 2>/dev/null || true'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/visual_inspector.py >/root/visual_inspector.log 2>&1'
docker exec "$CONTAINER" mkdir -p /root/ros1_ws/photo_stops
docker exec -d "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; exec python3 /root/route_executor.py _capture_photos:=$CAPTURE_PHOTOS _photo_points_file:='$PHOTO_POINTS_FILE' _photo_waypoints:='$PHOTO_WAYPOINTS' >/root/route_executor.log 2>&1"
echo "10 点拍照巡检已启动；拍照: $CAPTURE_PHOTOS，航点筛选: ${PHOTO_WAYPOINTS:-全部}。照片保存到 /home/sz/ros1_ws/photo_stops。"
