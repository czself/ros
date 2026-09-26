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
ENFORCE_TRAFFIC="${ENFORCE_TRAFFIC:-false}"
[[ "$ENFORCE_TRAFFIC" == true || "$ENFORCE_TRAFFIC" == false ]] || { echo "ENFORCE_TRAFFIC 必须是 true 或 false" >&2; exit 2; }
ENFORCE_WHITE_LINES="${ENFORCE_WHITE_LINES:-true}"
[[ "$ENFORCE_WHITE_LINES" == true || "$ENFORCE_WHITE_LINES" == false ]] || { echo "ENFORCE_WHITE_LINES 必须是 true 或 false" >&2; exit 2; }
MAP_BASENAME="$(basename "$MAP_FILE" .yaml)"
[[ "$MAP_BASENAME" != */* ]] || { echo "非法地图名" >&2; exit 2; }
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }
if ! docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosparam get /use_sim_time 2>/dev/null | grep -qx true'; then
  echo "导航启动失败：Gazebo 必须在 /use_sim_time=true 后启动；先运行 ./scripts/start_sim.sh" >&2
  exit 1
fi
[[ -f "$ROOT_DIR/maps/$MAP_BASENAME.yaml" && -f "$ROOT_DIR/maps/$MAP_BASENAME.pgm" ]] || { echo "没有已验证地图 $ROOT_DIR/maps/$MAP_BASENAME.yaml；请先完成真实建图。" >&2; exit 1; }
if [[ "${ALLOW_LEGACY_MAP:-0}" != 1 ]]; then
  case "$MAP_BASENAME" in
    competition_current*|competition_navigation*|competition_ground_truth|competition_slam_verified4) echo "拒绝加载旧/诊断地图；请先重新扫图。" >&2; exit 1 ;;
  esac
  [[ -f "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" ]] || { echo "地图缺少本次 SLAM manifest，拒绝导航" >&2; exit 1; }
  python3 - "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" <<'PY2'
import json,sys
import hashlib
import os
data=json.load(open(sys.argv[1],encoding="utf-8"))
if data.get("kind") == "navigation_real_white_line_overlay":
    source=data.get("source_map")
    expected=data.get("source_map_sha256")
    if not source or not expected:
        raise SystemExit("真实白线叠加图缺少原始 SLAM 来源")
    digest=hashlib.sha256(open(source,"rb").read()).hexdigest()
    if digest != expected or data.get("permanent_white_line_cells", 0) <= 0:
        raise SystemExit("真实白线叠加图来源校验失败")
elif data.get("kind") == "navigation_safety_overlay":
    source=data.get("source_map", {}).get("yaml")
    if not source or not data.get("collision_world", {}).get("sdf"):
        raise SystemExit("安全叠加地图缺少来源或场景碰撞来源")
    if not os.path.exists(source) or not os.path.exists(data["collision_world"]["sdf"]):
        raise SystemExit("安全叠加地图来源不存在")
elif data.get("publisher") != "/slam_gmapping" or not data.get("session_id"):
    raise SystemExit("地图不是本次真实 GMapping 会话产物")
PY2
fi
docker cp "$ROOT_DIR/navigation/." "$CONTAINER:/root/navigation"
docker cp "$ROOT_DIR/navigation/route_contract.yaml" "$CONTAINER:/root/navigation/route_contract.yaml"
docker cp "$ROOT_DIR/scripts/wheel_encoder_odom.py" "$CONTAINER:/root/wheel_encoder_odom.py"
docker cp "$ROOT_DIR/scripts/cmd_vel_watchdog.py" "$CONTAINER:/root/cmd_vel_watchdog.py"
docker cp "$ROOT_DIR/models/competition_ground/materials/textures/map.png" "$CONTAINER:/root/competition_ground_map.png"
docker cp "$ROOT_DIR/scripts/check_foundation.py" "$CONTAINER:/root/check_foundation.py"
docker cp "$ROOT_DIR/scripts/runtime_control.py" "$CONTAINER:/root/runtime_control.py"
docker cp "$ROOT_DIR/scripts/goal_sanitizer.py" "$CONTAINER:/root/goal_sanitizer.py"
docker cp "$ROOT_DIR/scripts/navigation_goal_safety.py" "$CONTAINER:/root/navigation_goal_safety.py"
docker cp "$ROOT_DIR/scripts/check_navigation_readiness.py" "$CONTAINER:/root/check_navigation_readiness.py"
docker exec "$CONTAINER" mkdir -p /root/ros1_ws/src/safe_escape_recovery
docker cp "$ROOT_DIR/ros_packages/safe_escape_recovery/." "$CONTAINER:/root/ros1_ws/src/safe_escape_recovery/"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; cd /root/ros1_ws; catkin_make --pkg safe_escape_recovery -j2 >/root/safe_escape_build.log 2>&1'
docker exec "$CONTAINER" mkdir -p /root/ros1_ws/maps
docker cp "$ROOT_DIR/maps/$MAP_BASENAME.yaml" "$CONTAINER:/root/ros1_ws/maps/$MAP_BASENAME.yaml"
docker cp "$ROOT_DIR/maps/$MAP_BASENAME.pgm" "$CONTAINER:/root/ros1_ws/maps/$MAP_BASENAME.pgm"
if [[ -f "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" ]]; then
  docker cp "$ROOT_DIR/maps/$MAP_BASENAME.manifest.json" "$CONTAINER:/root/ros1_ws/maps/$MAP_BASENAME.manifest.json"
fi
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; python3 /root/runtime_control.py reset'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/wheel_encoder_odom.py >/root/wheel_encoder_odom.log 2>&1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/goal_sanitizer.py >/root/goal_sanitizer.log 2>&1'
# rosparam survives node restarts.  While collecting data before the YOLO
# traffic-light gate is ready, signal-controlled stop lines and zebra crossings
# remain traversable by default.  Set ENFORCE_TRAFFIC=true when perception is
# ready; ordinary white-line checks can be independently disabled for the
# recorded photo route, whose camera poses require crossing painted markings.
docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; rosparam set /cmd_vel_watchdog/enforce_traffic $ENFORCE_TRAFFIC; rosparam set /cmd_vel_watchdog/enforce_white_lines $ENFORCE_WHITE_LINES"
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/cmd_vel_watchdog.py >/root/cmd_vel_watchdog.log 2>&1'
docker exec -d "$CONTAINER" bash -lc "source /root/ros1_ws/devel/setup.bash; exec roslaunch /root/navigation/navigation.launch map_file:=/root/ros1_ws/maps/$MAP_BASENAME.yaml observation_sources:='$OBSERVATION_SOURCES' initial_x:=1.714860 initial_y:=-1.599947 initial_yaw:=1.606236 >/root/navigation.log 2>&1"
sleep 6
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /map_server >/dev/null && rosnode ping -c 1 /amcl >/dev/null && rosnode ping -c 1 /move_base >/dev/null && rosnode ping -c 1 /cmd_vel_watchdog >/dev/null'
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; /usr/bin/python3 /root/check_navigation_readiness.py'
docker cp "$ROOT_DIR/scripts/competition.rviz" "$CONTAINER:/root/competition.rviz"
docker exec "$CONTAINER" bash -lc 'pkill -x rviz 2>/dev/null || true'
docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec rviz -geometry 1250x850+30+30 -d /root/competition.rviz >/root/navigation_rviz.log 2>&1'
echo "导航已启动，RViz 已打开。地图: $MAP_BASENAME；轮编码器里程计 + AMCL；交通灯门控: $ENFORCE_TRAFFIC；白线门控: $ENFORCE_WHITE_LINES。"
