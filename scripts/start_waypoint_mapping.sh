#!/bin/bash
# Start a conservative, laser-protected waypoint survey for SLAM mapping.
set -e

CONTAINER=ros1_modeling
DIR="$(cd "$(dirname "$0")" && pwd)"

docker cp "$DIR/waypoint_mapper.py" "${CONTAINER}:/root/waypoint_mapper.py"
docker exec "${CONTAINER}" bash -lc '
  source /opt/ros/noetic/setup.bash
  rosnode kill /car_teleop /autonomous_mapper /explore /move_base /waypoint_mapper /slam_gmapping 2>/dev/null || true
  rosservice call /gazebo/reset_world
  rosservice call /gazebo/set_model_state "{model_state: {model_name: my_car, pose: {position: {x: 2.30, y: -2.30, z: 0.105}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}"
  nohup rosrun gmapping slam_gmapping scan:=/scan > /root/slam_gmapping.log 2>&1 &
  sleep 3
  nohup python3 /root/waypoint_mapper.py > /root/waypoint_mapper.log 2>&1 &
'

sleep 2
docker exec "${CONTAINER}" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode list | grep -x /waypoint_mapper'
echo "安全巡线建图已启动。它遇到近距离障碍会自动停车。"
