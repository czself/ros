#!/bin/bash
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }
docker cp "$ROOT_DIR/navigation/." "$CONTAINER:/root/navigation"
docker cp "$ROOT_DIR/scripts/wheel_encoder_odom.py" "$CONTAINER:/root/wheel_encoder_odom.py"
docker cp "$ROOT_DIR/scripts/wheel_odometry.py" "$CONTAINER:/root/wheel_odometry.py"
docker cp "$ROOT_DIR/scripts/runtime_control.py" "$CONTAINER:/root/runtime_control.py"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; python3 /root/runtime_control.py reset'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/wheel_encoder_odom.py >/root/wheel_encoder_odom.log 2>&1'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec roslaunch /root/navigation/mapping_exploration.launch explore:=false >/root/slam_mapping.log 2>&1'
sleep 6
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /slam_gmapping >/dev/null'
echo "GMapping 已启动：激光 /scan + 轮编码器 /my_car/wheel_odom → /odom；不读取 Gazebo 位姿真值。"
echo "使用 scripts/teleop.sh 遥控建图；scripts/view_autonomous_mapping.sh 查看实时地图。"
