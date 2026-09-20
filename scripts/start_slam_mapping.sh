#!/bin/bash
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }
docker cp "$ROOT_DIR/navigation/." "$CONTAINER:/root/navigation"
docker cp "$ROOT_DIR/scripts/model_state_odom.py" "$CONTAINER:/root/model_state_odom.py"
docker cp "$ROOT_DIR/scripts/runtime_control.py" "$CONTAINER:/root/runtime_control.py"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; python3 /root/runtime_control.py reset'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/model_state_odom.py >/root/model_state_odom.log 2>&1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec roslaunch /root/navigation/mapping_exploration.launch explore:=false >/root/slam_mapping.log 2>&1'
sleep 6
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /slam_gmapping >/dev/null'
echo "GMapping 已启动。当前没有自动运动控制器；请使用 task003 的探索器或遥控。"
