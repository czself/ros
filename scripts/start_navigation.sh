#!/bin/bash
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_FILE="${1:-}"
if [[ -z "$MAP_FILE" ]]; then
  echo "必须显式提供本次重新扫图的地图路径；请先运行 ./scripts/rescan_and_navigate.sh" >&2
  exit 2
fi
OBSERVATION_SOURCES="${OBSERVATION_SOURCES:-laser depth}"
MAP_BASENAME="$(basename "$MAP_FILE" .yaml)"
[[ "$MAP_BASENAME" != */* ]] || { echo "非法地图名" >&2; exit 2; }
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }
[[ -f "$ROOT_DIR/maps/$MAP_BASENAME.yaml" && -f "$ROOT_DIR/maps/$MAP_BASENAME.pgm" ]] || { echo "没有已验证地图 $ROOT_DIR/maps/$MAP_BASENAME.yaml；请先完成真实建图。" >&2; exit 1; }
if [[ "${ALLOW_LEGACY_MAP:-0}" != 1 ]]; then
  case "$MAP_BASENAME" in
    competition_current*|competition_navigation*|competition_ground_truth|competition_slam_verified4) echo "拒绝加载旧/诊断地图；请先重新扫图。" >&2; exit 1 ;;
  esac
  [[ -f "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" ]] || { echo "地图缺少本次 SLAM manifest，拒绝导航" >&2; exit 1; }
  python3 - "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" <<'PY2'
import json,sys
import hashlib
data=json.load(open(sys.argv[1],encoding="utf-8"))
if data.get("kind") == "navigation_real_white_line_overlay":
    source=data.get("source_map")
    expected=data.get("source_map_sha256")
    if not source or not expected:
        raise SystemExit("真实白线叠加图缺少原始 SLAM 来源")
    digest=hashlib.sha256(open(source,"rb").read()).hexdigest()
    if digest != expected or data.get("permanent_white_line_cells", 0) <= 0:
        raise SystemExit("真实白线叠加图来源校验失败")
elif data.get("publisher") != "/slam_gmapping" or not data.get("session_id"):
    raise SystemExit("地图不是本次真实 GMapping 会话产物")
PY2
fi
docker cp "$ROOT_DIR/navigation/." "$CONTAINER:/root/navigation"
docker cp "$ROOT_DIR/navigation/route_contract.yaml" "$CONTAINER:/root/navigation/route_contract.yaml"
docker cp "$ROOT_DIR/scripts/model_state_odom.py" "$CONTAINER:/root/model_state_odom.py"
docker cp "$ROOT_DIR/scripts/cmd_vel_watchdog.py" "$CONTAINER:/root/cmd_vel_watchdog.py"
docker cp "$ROOT_DIR/models/competition_ground/materials/textures/map.png" "$CONTAINER:/root/competition_ground_map.png"
docker cp "$ROOT_DIR/scripts/check_foundation.py" "$CONTAINER:/root/check_foundation.py"
docker cp "$ROOT_DIR/scripts/runtime_control.py" "$CONTAINER:/root/runtime_control.py"
docker exec "$CONTAINER" mkdir -p /root/ros1_ws/maps
docker cp "$ROOT_DIR/maps/$MAP_BASENAME.yaml" "$CONTAINER:/root/ros1_ws/maps/$MAP_BASENAME.yaml"
docker cp "$ROOT_DIR/maps/$MAP_BASENAME.pgm" "$CONTAINER:/root/ros1_ws/maps/$MAP_BASENAME.pgm"
if [[ -f "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" ]]; then
  docker cp "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" "$CONTAINER:/root/ros1_ws/maps/$MAP_BASENAME.manifest.json"
fi
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; python3 /root/runtime_control.py reset'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/model_state_odom.py >/root/model_state_odom.log 2>&1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/cmd_vel_watchdog.py >/root/cmd_vel_watchdog.log 2>&1'
docker exec -d "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; exec roslaunch /root/navigation/navigation.launch map_file:=/root/ros1_ws/maps/$MAP_BASENAME.yaml observation_sources:='$OBSERVATION_SOURCES' initial_x:=1.714860 initial_y:=-1.599947 initial_yaw:=1.606236 >/root/navigation.log 2>&1"
sleep 6
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /map_server >/dev/null && rosnode ping -c 1 /amcl >/dev/null && rosnode ping -c 1 /move_base >/dev/null && rosnode ping -c 1 /cmd_vel_watchdog >/dev/null'
docker cp "$ROOT_DIR/scripts/competition.rviz" "$CONTAINER:/root/competition.rviz"
docker exec "$CONTAINER" bash -lc 'pkill -x rviz 2>/dev/null || true'
docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec rviz -geometry 1250x850+30+30 -d /root/competition.rviz >/root/navigation_rviz.log 2>&1'
echo "导航已启动，RViz 已打开。地图: $MAP_BASENAME；cmd_vel watchdog 已启用。"
