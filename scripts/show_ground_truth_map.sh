#!/bin/bash
# Publish the generated final map as /map for RViz and later navigation.
set -e

CONTAINER=ros1_modeling
MAP_FILE=/root/ros1_ws/maps/competition_ground_truth.yaml
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

docker cp "$ROOT_DIR/maps/competition_ground_truth.pgm" "${CONTAINER}:/root/ros1_ws/maps/competition_ground_truth.pgm"
docker cp "$ROOT_DIR/maps/competition_ground_truth.yaml" "${CONTAINER}:/root/ros1_ws/maps/competition_ground_truth.yaml"
docker exec "${CONTAINER}" bash -lc '
  source /opt/ros/noetic/setup.bash
  rosnode kill /slam_gmapping /waypoint_mapper 2>/dev/null || true
  rosnode list | grep "^/map_server" | xargs -r rosnode kill 2>/dev/null || true
  rosnode kill /final_map_server 2>/dev/null || true
  nohup rosrun map_server map_server __name:=final_map_server '"${MAP_FILE}"' > /root/map_server.log 2>&1 &
'

sleep 3
docker exec "${CONTAINER}" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode list | grep -x /final_map_server'
echo "最终地图已发布到 /map"
