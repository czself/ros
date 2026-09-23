#!/bin/bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER=ros1_modeling
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="/root/calibrated_runs/$RUN_ID"
python3 "$ROOT_DIR/scripts/route_geometry.py" "$ROOT_DIR/navigation/inner_route.yaml" \
  "$ROOT_DIR/models/competition_ground/materials/textures/map.png" \
  "$ROOT_DIR/docs/agent_context/tasks/task005/route_preview.png"
for script in runtime_control model_state_odom cmd_vel_watchdog route_geometry calibrated_navigation; do
  docker cp "$ROOT_DIR/scripts/$script.py" "$CONTAINER:/root/$script.py"
done
docker cp "$ROOT_DIR/navigation/." "$CONTAINER:/root/navigation"
docker cp "$ROOT_DIR/models/competition_ground/materials/textures/map.png" "$CONTAINER:/root/competition_ground_map.png"
docker exec "$CONTAINER" mkdir -p "$RUN_DIR"
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; python3 /root/runtime_control.py reset'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec python3 /root/model_state_odom.py'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec roslaunch /root/navigation/robot.launch'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map __name:=world_map_frame'
docker exec -d "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec rosrun map_server map_server /root/ros1_ws/maps/competition_rescan_full_20260922_white_lines.yaml __name:=map_server'
docker exec -d "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; exec python3 -u /root/cmd_vel_watchdog.py _enforce_traffic:=false >$RUN_DIR/gate.log 2>&1"
sleep 3
docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /cmd_vel_watchdog; rostopic info /my_car/cmd_vel'
docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; exec rviz -d /root/navigation/calibrated.rviz'
docker exec -d "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; exec python3 -u /root/calibrated_navigation.py _output:=$RUN_DIR/evidence.json >$RUN_DIR/controller.log 2>&1"
echo "Run evidence: $RUN_DIR"
