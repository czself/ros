#!/usr/bin/env bash
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
# Keep photo-route navigation on the same verified map used by the original
# navigation runs so the route, RViz, and costmaps share one consistent frame.
MAP_FILE="${1:-/root/ros1_ws/maps/current_slam_preview_white_lines.yaml}"
PHOTO_DIR="/root/ros1_ws/photo_stops/standee_route_runs/$(date +%Y%m%d_%H%M%S)"
OBSERVATION_SOURCES="${OBSERVATION_SOURCES:-laser}"
# The photo poses may cross painted lane/crosswalk markings. Keep the solid
# obstacle guard, depth, and footprint checks enabled; treat paint as traversable
# only in this photo route. Callers can override this to restore the gate.
ENFORCE_WHITE_LINES="${ENFORCE_WHITE_LINES:-false}"

# AMCL localizes wheel odometry in the same saved map used for the route.
# Traffic stop lines remain passable until YOLO traffic recognition is ready.
OBSERVATION_SOURCES="$OBSERVATION_SOURCES" \
ENFORCE_WHITE_LINES="$ENFORCE_WHITE_LINES" \
  "$ROOT_DIR/scripts/start_navigation.sh" "$MAP_FILE"
docker cp "$ROOT_DIR/scripts/route_executor.py" "$CONTAINER:/root/route_executor.py"
docker exec "$CONTAINER" mkdir -p "$PHOTO_DIR"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode kill /route_executor 2>/dev/null || true'
docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; exec python3 -u /root/route_executor.py _start_waypoint:=0 _park_only:=false _return_path:=auto _capture_photos:=true _photo_points_file:=/root/navigation/standee_photo_route.json _photo_waypoints:=POINT_1,POINT_2,POINT_3,POINT_4,POINT_5,POINT_6,POINT_7,POINT_8,POINT_9,POINT_10 _photo_dir:='$PHOTO_DIR' _photo_position_tolerance:=0.03 _photo_heading_tolerance:=0.04 _photo_nav_xy_tolerance:=0.03 _photo_nav_heading_tolerance:=0.04 _point_3_photo_position_tolerance:=0.05 _point_3_nav_xy_tolerance:=0.05 _point_5_photo_position_tolerance:=0.025 _point_5_photo_heading_tolerance:=0.025 _point_5_nav_xy_tolerance:=0.025 _point_5_nav_heading_tolerance:=0.025 _no_progress_timeout:=20.0 _photo_settle_seconds:=0.6 _photo_burst_count:=8 _photo_burst_interval:=0.2 >/root/standee_photo_route.log 2>&1"
echo "POINT_1 到 POINT_10 拍照巡检已启动；完成后返航并精确停车。照片保存到 /home/sz/ros1_ws/photo_stops/standee_route_runs。"
