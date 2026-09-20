#!/bin/bash
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }
docker cp "$ROOT_DIR/navigation/." "$CONTAINER:/root/navigation"
docker cp "$ROOT_DIR/scripts/model_state_odom.py" "$CONTAINER:/root/model_state_odom.py"
docker cp "$ROOT_DIR/scripts/survey_mapper.py" "$CONTAINER:/root/survey_mapper.py"
docker cp "$ROOT_DIR/scripts/runtime_control.py" "$CONTAINER:/root/runtime_control.py"
docker exec "$CONTAINER" bash -lc 'rm -f /root/survey_trajectory.json /root/survey_mapper.log /root/task003_tf_evidence.json'
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; python3 /root/runtime_control.py reset'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/model_state_odom.py >/root/model_state_odom.log 2>&1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec roslaunch /root/navigation/mapping_exploration.launch explore:=false >/root/slam_mapping.log 2>&1'
sleep 6
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /slam_gmapping >/dev/null; nohup python3 /root/survey_mapper.py >/root/survey_mapper.log 2>&1 &'
echo "自动建图已启动，使用唯一运动控制器 survey_mapper。"
