#!/bin/bash
set -euo pipefail

CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_NAME="${1:-competition_slam_verified}"
ROUTE="${2:-wall_follower}"
STOP_REASON="${3:-}"
SESSION_ID="${4:-$(date -u +%Y%m%dT%H%M%SZ)}"
TRAJECTORY_MODE="${5:-automatic}"
SESSION_START="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

[[ "$MAP_NAME" != */* ]] || { echo "非法地图名" >&2; exit 2; }
case "$TRAJECTORY_MODE" in
  automatic|manual_teleop|focused_autonomous) ;;
  *) echo "轨迹模式必须是 automatic、manual_teleop 或 focused_autonomous: $TRAJECTORY_MODE" >&2; exit 2 ;;
esac

map_info() {
  docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rostopic info /map'
}

docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /slam_gmapping >/dev/null'
initial_info="$(map_info)"
printf '%s\n' "$initial_info" | grep -q ' /slam_gmapping' || {
  echo "保存前未发现 /slam_gmapping" >&2; exit 1;
}
if printf '%s\n' "$initial_info" | grep -Eq ' /(map_server|final_map_server|amcl|move_base|explore)([^a-zA-Z0-9_]|$)'; then
  echo "保存前存在互斥导航节点" >&2
  exit 1
fi
publishers_before="$(printf '%s\n' "$initial_info" | awk '/Publishers:/{p=1;next}/Subscribers:/{p=0}p && /^[[:space:]]*\*/{print $2}' | sed 's/(.*//' | sort -u | paste -sd, -)"
[[ "$publishers_before" == "/slam_gmapping" ]] || {
  echo "保存前 /map 发布者不唯一: $publishers_before" >&2; exit 1;
}

docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; mkdir -p /root/ros1_ws/maps; rosrun map_server map_saver -f /root/ros1_ws/maps/$MAP_NAME"
docker cp "$CONTAINER:/root/ros1_ws/maps/$MAP_NAME.pgm" "$ROOT_DIR/maps/$MAP_NAME.pgm"
docker cp "$CONTAINER:/root/ros1_ws/maps/$MAP_NAME.yaml" "$ROOT_DIR/maps/$MAP_NAME.yaml"
coverage_file="$ROOT_DIR/maps/$MAP_NAME.coverage.json"
python3 "$ROOT_DIR/scripts/coverage_report.py" --yaml "$ROOT_DIR/maps/$MAP_NAME.yaml" \
  --pgm "$ROOT_DIR/maps/$MAP_NAME.pgm" --birth 1.71499,-1.71499 \
  --inflate-radius-m 0.10 --output "$coverage_file" >/dev/null

mapper_file=""
log_file=""
if [[ "$TRAJECTORY_MODE" == "automatic" ]]; then
  docker exec "$CONTAINER" test -s /root/survey_trajectory.json
  docker exec "$CONTAINER" test -s /root/survey_mapper.py
  docker exec "$CONTAINER" test -s /root/survey_mapper.log
  docker cp "$CONTAINER:/root/survey_trajectory.json" "$ROOT_DIR/maps/$MAP_NAME.trajectory.json"
  docker cp "$CONTAINER:/root/survey_mapper.py" "$ROOT_DIR/maps/$MAP_NAME.mapper.py"
  docker cp "$CONTAINER:/root/survey_mapper.log" "$ROOT_DIR/maps/$MAP_NAME.survey.log"
  mapper_file="$ROOT_DIR/maps/$MAP_NAME.mapper.py"
  log_file="$ROOT_DIR/maps/$MAP_NAME.survey.log"
  measured_stop_reason="$(python3 - "$ROOT_DIR/maps/$MAP_NAME.trajectory.json" <<'PY'
import json
import sys
data = json.load(open(sys.argv[1], encoding='utf-8'))
print(data.get('stop_reason', ''))
PY
)"
  [[ -n "$measured_stop_reason" ]] || { echo "自动轨迹缺少 stop_reason" >&2; exit 1; }
  if [[ -n "$STOP_REASON" && "$STOP_REASON" != "$measured_stop_reason" ]]; then
    echo "命令行 stop_reason 与轨迹实测不一致: $STOP_REASON != $measured_stop_reason" >&2
    exit 1
  fi
  STOP_REASON="$measured_stop_reason"
else
  if [[ "$TRAJECTORY_MODE" == "focused_autonomous" ]]; then
    printf '{"mode":"focused_autonomous","trajectory_available":false,"note":"low-speed laser-protected waypoint survey; the mapper did not emit a trajectory artifact"}\n' > "$ROOT_DIR/maps/$MAP_NAME.trajectory.json"
  else
    printf '{"mode":"manual_teleop","trajectory_available":false,"note":"operator drove the robot during this mapping session; no waypoint trajectory was generated"}\n' > "$ROOT_DIR/maps/$MAP_NAME.trajectory.json"
  fi
  STOP_REASON="${STOP_REASON:-operator_requested}"
fi

docker cp "$ROOT_DIR/scripts/verify_slam_map.py" "$CONTAINER:/root/verify_slam_map.py"
docker cp "$ROOT_DIR/scripts/capture_tf_evidence.py" "$CONTAINER:/root/capture_tf_evidence.py"
after_info="$(map_info)"
publishers_after="$(printf '%s\n' "$after_info" | awk '/Publishers:/{p=1;next}/Subscribers:/{p=0}p && /^[[:space:]]*\*/{print $2}' | sed 's/(.*//' | sort -u | paste -sd, -)"
[[ "$publishers_after" == "/slam_gmapping" ]] || {
  echo "保存后 /map 发布者不唯一: $publishers_after" >&2; exit 1;
}

docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; python3 /root/capture_tf_evidence.py --output /root/task003_tf_evidence.json --duration 3'
docker cp "$CONTAINER:/root/task003_tf_evidence.json" "$ROOT_DIR/maps/$MAP_NAME.tf_evidence.json"
tf_auth="$(python3 - "$ROOT_DIR/maps/$MAP_NAME.tf_evidence.json" <<'PY'
import json
import sys
data = json.load(open(sys.argv[1], encoding='utf-8'))
print(','.join(sorted({caller for item in data.get('transforms', []) for caller in item.get('caller_ids', [])})))
PY
)"

if [[ "$TRAJECTORY_MODE" == "automatic" ]]; then
  mapper_sha="$(sha256sum "$mapper_file" | awk '{print $1}')"
  log_sha="$(sha256sum "$log_file" | awk '{print $1}')"
else
  mapper_sha="not-applicable-manual-teleop"
  log_sha="not-applicable-manual-teleop"
fi
trajectory_sha="$(sha256sum "$ROOT_DIR/maps/$MAP_NAME.trajectory.json" | awk '{print $1}')"
scan_rate="$(docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; timeout 8 rostopic hz -w 5 /scan 2>&1 | awk "/average rate:/{print \$3; exit}"' || true)"
points_rate="$(docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; timeout 8 rostopic hz -w 5 /camera/depth/points 2>&1 | awk "/average rate:/{print \$3; exit}"' || true)"
[[ -n "$scan_rate" && -n "$points_rate" ]] || { echo "无法采集传感器频率" >&2; exit 1; }
SESSION_END="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

coverage_note="birth-connected survey; coverage report required; unknown cells remain until Gate A approval"
if [[ "$TRAJECTORY_MODE" == "manual_teleop" ]]; then
  coverage_note="operator manual teleop supplement; coverage report required; full-world coverage is not claimed"
elif [[ "$TRAJECTORY_MODE" == "focused_autonomous" ]]; then
  coverage_note="focused low-speed autonomous supplement; target-area coverage only, not a full-world claim"
fi
python3 "$ROOT_DIR/scripts/verify_slam_map.py" \
  "$ROOT_DIR/maps/$MAP_NAME.pgm" "$ROOT_DIR/maps/$MAP_NAME.manifest.json" \
  --yaml "$ROOT_DIR/maps/$MAP_NAME.yaml" \
  --publisher "/slam_gmapping" --publisher-before "$publishers_before" \
  --publisher-after "$publishers_after" \
  --topic-rates "scan=$scan_rate,depth_points=$points_rate" \
  --route "$ROUTE" --stop-reason "$STOP_REASON" --session-id "$SESSION_ID" \
  --session-start "$SESSION_START" --session-end "$SESSION_END" \
  --tf-authorities "$tf_auth" \
  --tf-evidence-file "$ROOT_DIR/maps/$MAP_NAME.tf_evidence.json" \
  --coverage-report "$coverage_file" \
  --trajectory "$ROOT_DIR/maps/$MAP_NAME.trajectory.json" \
  --trajectory-sha256 "$trajectory_sha" --mapper-file "$mapper_file" \
  --mapper-sha256 "$mapper_sha" --log-file "$log_file" --log-sha256 "$log_sha" \
  --trajectory-frame chassis --birth-chassis "1.71499,-1.71499,1.5708" \
  --birth-axle "1.71499,-1.59995,1.5708" \
  --tf-evidence "message-level /tf and /tf_static caller IDs sampled at save" \
  --min-known-fraction 0.0 \
  --coverage-note "$coverage_note"
echo "真实 SLAM 地图已保存: maps/$MAP_NAME.yaml"
