#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER=ros1_modeling
MAP_NAME="${1:-current_slam_preview_white_lines}"
RUN_TASKS="${RUN_TASKS:-true}"
STOP_AFTER="${STOP_AFTER:-}"
START_MANAGER="${START_MANAGER:-true}"
RUN_DIR="/root/ros1_ws/navigation_rebuild_runs/$(date +%Y%m%d_%H%M%S)"

[[ "$MAP_NAME" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo "非法地图名" >&2; exit 2; }
[[ "$RUN_TASKS" == true || "$RUN_TASKS" == false ]] || { echo "RUN_TASKS 必须是 true 或 false" >&2; exit 2; }
[[ "$START_MANAGER" == true || "$START_MANAGER" == false ]] || { echo "START_MANAGER 必须是 true 或 false" >&2; exit 2; }
[[ -f "$ROOT_DIR/maps/$MAP_NAME.yaml" && -f "$ROOT_DIR/maps/$MAP_NAME.pgm" ]] || {
  echo "缺少地图 maps/$MAP_NAME.yaml/.pgm" >&2; exit 2;
}
[[ -f "$ROOT_DIR/current_slam_preview_local.yaml" && -f "$ROOT_DIR/current_slam_preview.pgm" ]] || {
  echo "缺少原始 SLAM 定位图" >&2; exit 2;
}
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; test "$(rosparam get /use_sim_time)" = true'

docker exec "$CONTAINER" mkdir -p /root/navigation_rebuild /root/navigation /root/ros1_ws/maps "$RUN_DIR"
docker cp "$ROOT_DIR/navigation_rebuild/." "$CONTAINER:/root/navigation_rebuild/"
docker cp "$ROOT_DIR/navigation/robot.launch" "$CONTAINER:/root/navigation/robot.launch"
docker cp "$ROOT_DIR/scripts/wheel_encoder_odom.py" "$CONTAINER:/root/wheel_encoder_odom.py"
docker cp "$ROOT_DIR/scripts/runtime_control.py" "$CONTAINER:/root/runtime_control.py"
docker cp "$ROOT_DIR/scripts/visual_inspector.py" "$CONTAINER:/root/visual_inspector.py"
docker cp "$ROOT_DIR/maps/$MAP_NAME.yaml" "$CONTAINER:/root/ros1_ws/maps/$MAP_NAME.yaml"
docker cp "$ROOT_DIR/maps/$MAP_NAME.pgm" "$CONTAINER:/root/ros1_ws/maps/$MAP_NAME.pgm"
docker cp "$ROOT_DIR/current_slam_preview_local.yaml" "$CONTAINER:/root/ros1_ws/maps/current_slam_preview_local.yaml"
docker cp "$ROOT_DIR/current_slam_preview.pgm" "$CONTAINER:/root/ros1_ws/maps/current_slam_preview.pgm"

docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode kill /navigation_rebuild_manager /navigation_rebuild_velocity_gate /localization_map_server 2>/dev/null || true; python3 /root/runtime_control.py reset'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/wheel_encoder_odom.py >/root/wheel_encoder_odom.log 2>&1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; export PYTHONPATH=/root:$PYTHONPATH; exec python3 -u -m navigation_rebuild.velocity_gate >/root/navigation_rebuild_gate.log 2>&1'
docker exec -d "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; exec roslaunch /root/navigation_rebuild/stack.launch map_file:=/root/ros1_ws/maps/$MAP_NAME.yaml localization_map_file:=/root/ros1_ws/maps/current_slam_preview_local.yaml >/root/navigation_rebuild_stack.log 2>&1"

docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; for i in $(seq 1 40); do if rosnode ping -c 1 /map_server >/dev/null 2>&1 && rosnode ping -c 1 /localization_map_server >/dev/null 2>&1 && rosnode ping -c 1 /amcl >/dev/null 2>&1 && rosnode ping -c 1 /move_base >/dev/null 2>&1 && rosnode ping -c 1 /navigation_rebuild_velocity_gate >/dev/null 2>&1; then exit 0; fi; sleep 1; done; exit 1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/visual_inspector.py >/root/visual_inspector.log 2>&1'
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /visual_inspector >/dev/null'
if [[ "$START_MANAGER" == true ]]; then
  docker exec -d "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; export PYTHONPATH=/root:\$PYTHONPATH; exec python3 -u -m navigation_rebuild.manager _run_tasks:=$RUN_TASKS _stop_after:='$STOP_AFTER' _output_dir:='$RUN_DIR' >/root/navigation_rebuild_manager.log 2>&1"
fi
echo "新导航栈已启动：地图 $MAP_NAME；任务 $RUN_TASKS；停止点 ${STOP_AFTER:-HOME}；任务执行器 $START_MANAGER；结果 $RUN_DIR"
