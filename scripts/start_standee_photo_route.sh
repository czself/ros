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
PADDLE_OCR="${PADDLE_OCR:-true}"
OCR_PYTHON="${OCR_PYTHON:-/home/sz/.venvs/community-ocr/bin/python}"
OCR_PID=""
if [[ -e "$PHOTO_HOST_DIR/run_summary.json" ]]; then
  echo "RUN_ID已有运行记录，请使用新PHOTO_RUN_ID，保留原验收数据。" >&2
  exit 2
fi

[[ "${ENFORCE_WHITE_LINES:-true}" == true ]] || { echo "十点拍照路线禁止关闭白线门禁。" >&2; exit 2; }
[[ "${ENFORCE_TRAFFIC:-true}" == true ]] || { echo "完整任务路线禁止关闭YOLO红绿灯门禁。" >&2; exit 2; }
[[ "$STRICT_ACCEPTANCE" == true || "$STRICT_ACCEPTANCE" == false ]] || { echo "STRICT_ACCEPTANCE必须是true或false" >&2; exit 2; }
[[ "$PADDLE_OCR" == true || "$PADDLE_OCR" == false ]] || { echo "PADDLE_OCR必须是true或false" >&2; exit 2; }
if [[ "$PADDLE_OCR" == true ]]; then
  [[ -x "$OCR_PYTHON" ]] || { echo "缺少独立OCR环境，请先运行 scripts/setup_paddle_ocr.sh" >&2; exit 2; }
  docker exec "$CONTAINER" mkdir -p /root/ros1_ws/src/tl_vision
  docker cp "$ROOT_DIR/ros_packages/tl_vision/." "$CONTAINER:/root/ros1_ws/src/tl_vision/"
  docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; cd /root/ros1_ws; catkin_make --pkg tl_vision -j2 >/root/tl_vision_build.log 2>&1'
fi

OBSERVATION_SOURCES="$OBSERVATION_SOURCES" \
GLOBAL_OBSERVATION_SOURCES="$GLOBAL_OBSERVATION_SOURCES" \
ENFORCE_WHITE_LINES=true \
ENFORCE_TRAFFIC=true \
  "$ROOT_DIR/scripts/start_navigation.sh" "$MAP_FILE"

docker cp "$ROOT_DIR/scripts/route_executor.py" "$CONTAINER:/root/route_executor.py"
docker cp "$ROOT_DIR/scripts/person_reporting.py" "$CONTAINER:/root/person_reporting.py"
docker cp "$ROOT_DIR/scripts/hd_plate_capture.py" "$CONTAINER:/root/hd_plate_capture.py"
docker exec "$CONTAINER" mkdir -p "$PHOTO_DIR"
if [[ "$PADDLE_OCR" == true ]]; then
  docker exec "$CONTAINER" mkdir -p "$PHOTO_DIR/ocr"
  docker exec "$CONTAINER" chown "$(id -u):$(id -g)" "$PHOTO_DIR/ocr"
  docker cp "$ROOT_DIR/scripts/plate_result_bridge.py" "$CONTAINER:/root/plate_result_bridge.py"
  docker cp "$ROOT_DIR/assets/ocr_font.ttf" "$CONTAINER:/root/ocr_font.ttf"
  docker cp "$ROOT_DIR/assets/ocr_latin_font.ttf" "$CONTAINER:/root/ocr_latin_font.ttf"
fi
docker cp "$CONTAINER:/root/car_standees/materials/scripts/car_standees.material" /tmp/ocr_scene_materials_"$RUN_ID".txt
docker cp /tmp/ocr_scene_materials_"$RUN_ID".txt "$CONTAINER:$PHOTO_DIR/ocr_scene_materials.txt"
rm -f /tmp/ocr_scene_materials_"$RUN_ID".txt
docker cp "$ROOT_DIR/scripts/record_plate_scene_truth.py" "$CONTAINER:/root/record_plate_scene_truth.py"
docker exec "$CONTAINER" python3 /root/record_plate_scene_truth.py --output "$PHOTO_DIR/audit_inputs/plate_scene_truth.json"

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
  /inspection/plate_text /inspection/plate_image /inspection/plate_report
  /inspection/traffic_light_confidence /inspection/yolo/metrics
  /traffic_light/state /traffic_light/time_remaining /traffic_light/gate_status
  /traffic_light/braking /gazebo/link_states
)
stop_recorder() {
  docker exec "$CONTAINER" pkill -INT -x rosbag 2>/dev/null || true
}
stop_ocr() {
  if [[ -n "$OCR_PID" ]] && kill -0 "$OCR_PID" 2>/dev/null; then
    kill "$OCR_PID" 2>/dev/null || true
    wait "$OCR_PID" 2>/dev/null || true
  fi
  if [[ "$PADDLE_OCR" == true ]]; then
    docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode kill /plate_result_bridge >/dev/null 2>&1' || true
  fi
}
cleanup() { stop_recorder; stop_ocr; }
trap cleanup EXIT INT TERM
if [[ "$PADDLE_OCR" == true ]]; then
  "$OCR_PYTHON" "$ROOT_DIR/scripts/paddle_plate_worker.py" \
    --run-dir "$PHOTO_HOST_DIR" --watch --compare-low \
    --ready-file "$PHOTO_HOST_DIR/ocr/ready.json" > "$PHOTO_HOST_DIR/ocr/worker.log" 2>&1 &
  OCR_PID=$!
  OCR_READY=false
  for _ in $(seq 1 40); do
    if [[ -f "$PHOTO_HOST_DIR/ocr/ready.json" ]]; then OCR_READY=true; break; fi
    if ! kill -0 "$OCR_PID" 2>/dev/null; then break; fi
    sleep .5
  done
  if [[ "$OCR_READY" != true ]]; then
    tail -12 "$PHOTO_HOST_DIR/ocr/worker.log" >&2
    echo "OCR未准备好，任务未发车" >&2
    exit 1
  fi
  docker exec -d "$CONTAINER" bash -lc \
    "source /root/ros1_ws/devel/setup.bash; exec python3 /root/plate_result_bridge.py _photo_dir:='$PHOTO_DIR' >'$PHOTO_DIR/ocr/bridge.log' 2>&1"
fi
docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; exec rosbag record --lz4 -O '$PHOTO_DIR/motion.bag' ${TOPICS[*]} >'$PHOTO_DIR/rosbag_record.log' 2>&1"
sleep 2

docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; exec python3 -u /root/route_executor.py _start_waypoint:=0 _park_only:=false _return_path:=auto _capture_photos:=true _strict_acceptance:=$STRICT_ACCEPTANCE _ocr_enabled:=$PADDLE_OCR _ocr_worker_uid:=$(id -u) _ocr_worker_gid:=$(id -g) _photo_points_file:=/root/navigation/standee_photo_route.json _photo_waypoints:=POINT_1,POINT_2,POINT_3,POINT_4,POINT_5,POINT_6,POINT_7,POINT_8,POINT_9,POINT_10 _photo_dir:='$PHOTO_DIR' _photo_position_tolerance:=0.03 _photo_heading_tolerance:=0.04 _photo_nav_xy_tolerance:=0.03 _photo_nav_heading_tolerance:=0.04 _point_3_photo_position_tolerance:=0.05 _point_3_nav_xy_tolerance:=0.05 _point_5_photo_position_tolerance:=0.025 _point_5_photo_heading_tolerance:=0.025 _point_5_nav_xy_tolerance:=0.025 _point_5_nav_heading_tolerance:=0.025 _point_7_photo_position_tolerance:=0.05 _point_7_nav_xy_tolerance:=0.05 _point_10_photo_position_tolerance:=0.05 _point_10_nav_xy_tolerance:=0.05 _no_progress_timeout:=8.0 _photo_settle_seconds:=0.3 _photo_burst_count:=4 _photo_burst_interval:=0.15 >'$PHOTO_DIR/route_executor.log' 2>&1"

echo "路线已开始：HOME → POINT_1…POINT_10 → HOME；strict=$STRICT_ACCEPTANCE；本次照片和运动bag: /home/sz/ros1_ws/photo_stops/standee_route_runs/$RUN_ID"
FINAL_STATUS=""
PEOPLE_LINES=0
OCR_LINES=0
for _ in $(seq 1 900); do
  if [[ -f "$PHOTO_HOST_DIR/people_terminal.txt" ]]; then
    NEW_LINES="$(wc -l < "$PHOTO_HOST_DIR/people_terminal.txt")"
    if (( NEW_LINES > PEOPLE_LINES )); then
      sed -n "$((PEOPLE_LINES+1)),${NEW_LINES}p" "$PHOTO_HOST_DIR/people_terminal.txt"
      PEOPLE_LINES="$NEW_LINES"
    fi
  fi
  if [[ -f "$PHOTO_HOST_DIR/ocr/ocr_terminal.txt" ]]; then
    NEW_LINES="$(wc -l < "$PHOTO_HOST_DIR/ocr/ocr_terminal.txt")"
    if (( NEW_LINES > OCR_LINES )); then
      sed -n "$((OCR_LINES+1)),${NEW_LINES}p" "$PHOTO_HOST_DIR/ocr/ocr_terminal.txt"
      OCR_LINES="$NEW_LINES"
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
stop_ocr
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
