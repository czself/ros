#!/usr/bin/env bash
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_FILE="${1:-/root/ros1_ws/maps/current_slam_preview_white_lines.yaml}"
RUN_ID="${PHOTO_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
PHOTO_DIR="/root/ros1_ws/photo_stops/standee_route_runs/$RUN_ID"
PHOTO_HOST_DIR="/home/sz/ros1_ws/photo_stops/standee_route_runs/$RUN_ID"
OBSERVATION_SOURCES="${OBSERVATION_SOURCES:-laser depth}"
GLOBAL_OBSERVATION_SOURCES="${GLOBAL_OBSERVATION_SOURCES:-}"
STRICT_ACCEPTANCE="${STRICT_ACCEPTANCE:-true}"

[[ "${ENFORCE_WHITE_LINES:-true}" == true ]] || { echo "十点拍照路线禁止关闭白线门禁。" >&2; exit 2; }
[[ "${ENFORCE_TRAFFIC:-true}" == true ]] || { echo "完整任务路线禁止关闭YOLO红绿灯门禁。" >&2; exit 2; }
[[ "$STRICT_ACCEPTANCE" == true || "$STRICT_ACCEPTANCE" == false ]] || { echo "STRICT_ACCEPTANCE必须是true或false" >&2; exit 2; }

OBSERVATION_SOURCES="$OBSERVATION_SOURCES" \
GLOBAL_OBSERVATION_SOURCES="$GLOBAL_OBSERVATION_SOURCES" \
ENFORCE_WHITE_LINES=true \
ENFORCE_TRAFFIC=true \
  "$ROOT_DIR/scripts/start_navigation.sh" "$MAP_FILE"

docker cp "$ROOT_DIR/scripts/route_executor.py" "$CONTAINER:/root/route_executor.py"
docker cp "$ROOT_DIR/scripts/person_reporting.py" "$CONTAINER:/root/person_reporting.py"
docker cp "$ROOT_DIR/scripts/hd_plate_capture.py" "$CONTAINER:/root/hd_plate_capture.py"
docker exec "$CONTAINER" mkdir -p "$PHOTO_DIR"

TOPICS=(
  /clock /tf /tf_static /odom /my_car/wheel_odom /my_car/cmd_vel_nav /my_car/cmd_vel
  /scan /camera/depth/points /camera/depth/image_raw /camera/depth/camera_info /camera/camera_info
  /camera/image_raw /inspection/image /amcl_pose /move_base/goal /move_base/status /move_base/feedback
  /move_base/NavfnROS/plan /move_base/ForwardPathFollower/local_plan
  /move_base/ForwardPathFollower/progress /move_base/ForwardPathFollower/status
  /move_base/result /move_base/global_costmap/costmap /move_base/local_costmap/costmap
  /move_base/global_costmap/footprint /move_base/local_costmap/footprint
  /route/status /inspection/detections /inspection/traffic_light
  /route/progress
  /inspection/person_report /inspection/person_image
  /inspection/plate_capture
  /inspection/traffic_light_confidence /inspection/yolo/metrics
  /traffic_light/state /traffic_light/time_remaining /traffic_light/gate_status
  /traffic_light/braking /gazebo/link_states
)
stop_recorder() {
  docker exec "$CONTAINER" pkill -INT -x rosbag 2>/dev/null || true
}
trap stop_recorder EXIT INT TERM
docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; exec rosbag record --lz4 -O '$PHOTO_DIR/motion.bag' ${TOPICS[*]} >'$PHOTO_DIR/rosbag_record.log' 2>&1"
sleep 2

docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; exec python3 -u /root/route_executor.py _start_waypoint:=0 _park_only:=false _return_path:=auto _capture_photos:=true _strict_acceptance:=$STRICT_ACCEPTANCE _photo_points_file:=/root/navigation/standee_photo_route.json _photo_waypoints:=POINT_1,POINT_2,POINT_3,POINT_4,POINT_5,POINT_6,POINT_7,POINT_8,POINT_9,POINT_10 _photo_dir:='$PHOTO_DIR' _photo_position_tolerance:=0.03 _photo_heading_tolerance:=0.04 _photo_nav_xy_tolerance:=0.03 _photo_nav_heading_tolerance:=0.04 _point_3_photo_position_tolerance:=0.05 _point_3_nav_xy_tolerance:=0.05 _point_5_photo_position_tolerance:=0.025 _point_5_photo_heading_tolerance:=0.025 _point_5_nav_xy_tolerance:=0.025 _point_5_nav_heading_tolerance:=0.025 _point_7_photo_position_tolerance:=0.05 _point_7_nav_xy_tolerance:=0.05 _point_10_photo_position_tolerance:=0.05 _point_10_nav_xy_tolerance:=0.05 _no_progress_timeout:=8.0 _photo_settle_seconds:=0.3 _photo_burst_count:=4 _photo_burst_interval:=0.15 >'$PHOTO_DIR/route_executor.log' 2>&1"

echo "路线已开始：HOME → POINT_1…POINT_10 → HOME；strict=$STRICT_ACCEPTANCE；本次照片和运动bag: /home/sz/ros1_ws/photo_stops/standee_route_runs/$RUN_ID"
FINAL_STATUS=""
PEOPLE_LINES=0
for _ in $(seq 1 900); do
  if [[ -f "$PHOTO_HOST_DIR/people_terminal.txt" ]]; then
    NEW_LINES="$(wc -l < "$PHOTO_HOST_DIR/people_terminal.txt")"
    if (( NEW_LINES > PEOPLE_LINES )); then
      sed -n "$((PEOPLE_LINES+1)),${NEW_LINES}p" "$PHOTO_HOST_DIR/people_terminal.txt"
      PEOPLE_LINES="$NEW_LINES"
    fi
  fi
  if [[ -f "$PHOTO_HOST_DIR/run_summary.json" ]]; then
    SUMMARY_STATUS="$(python3 -c "import json; print(json.load(open('$PHOTO_HOST_DIR/run_summary.json')).get('route_status',''))" 2>/dev/null || true)"
    if [[ "$SUMMARY_STATUS" == COMPLETE_PARKED ]]; then
      FINAL_STATUS=COMPLETE_PARKED
      break
    fi
    if [[ "$SUMMARY_STATUS" == FAILED ]]; then
      FINAL_STATUS=FAILED
      break
    fi
  fi
  STATUS_TEXT="$(docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; timeout 2 rostopic echo -n1 /route/status 2>/dev/null' 2>/dev/null || true)"
  if [[ "$STATUS_TEXT" == *COMPLETE_PARKED* ]]; then
    FINAL_STATUS=COMPLETE_PARKED
    break
  fi
  if [[ "$STATUS_TEXT" == *FAILED:* ]]; then
    FINAL_STATUS="$(printf '%s' "$STATUS_TEXT" | sed -n 's/.*data: //p' | head -n1)"
    break
  fi
  if (( _ % 15 == 0 )); then
    echo "路线仍在运行；照片目录: /home/sz/ros1_ws/photo_stops/standee_route_runs/$RUN_ID"
  fi
  sleep 1
done

stop_recorder
trap - EXIT INT TERM
sleep 2
if [[ "$FINAL_STATUS" != COMPLETE_PARKED ]]; then
  echo "路线未完成，最终状态：${FINAL_STATUS:-TIMEOUT}；保留失败数据: /home/sz/ros1_ws/photo_stops/standee_route_runs/$RUN_ID" >&2
  exit 1
fi

docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; rosbag info '$PHOTO_DIR/motion.bag' >/dev/null"
if [[ -f "$PHOTO_HOST_DIR/person_report.json" ]]; then
  cat "$PHOTO_HOST_DIR/person_report.txt"
  AUDIO_DIR="$(mktemp -d /tmp/person-report-audio.XXXXXX)"
  AUDIO_RESULT=0
  python3 "$ROOT_DIR/scripts/announce_people.py" \
    --report "$PHOTO_HOST_DIR/person_report.json" \
    --output "$AUDIO_DIR/person_report.wav" --play || AUDIO_RESULT=$?
  if [[ -f "$AUDIO_DIR/person_report.wav" ]]; then
    docker cp "$AUDIO_DIR/person_report.wav" "$CONTAINER:$PHOTO_DIR/person_report.wav"
  fi
  docker cp "$AUDIO_DIR/person_report.status.json" "$CONTAINER:$PHOTO_DIR/person_report.audio.json"
  rm -rf "$AUDIO_DIR"
  if (( AUDIO_RESULT != 0 )); then
    echo "路线已完成，但人物播报失败，详情见 person_report.audio.json" >&2
    exit 1
  fi
fi
echo "路线结束：COMPLETE_PARKED；bag已关闭并通过rosbag info；照片和数据: /home/sz/ros1_ws/photo_stops/standee_route_runs/$RUN_ID"
