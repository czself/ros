#!/bin/bash
# Open RViz with SLAM, frontier-exploration, and navigation overlays.
set -e

CONTAINER=ros1_modeling
DIR="$(cd "$(dirname "$0")" && pwd)"

docker cp "$DIR/autonomous_mapping.rviz" "${CONTAINER}:/root/autonomous_mapping.rviz"
docker exec "${CONTAINER}" bash -lc 'pkill -x rviz 2>/dev/null || true'
docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 "${CONTAINER}" bash -lc \
  'source /opt/ros/noetic/setup.bash && exec rviz -geometry 1100x750+50+50 -d /root/autonomous_mapping.rviz > /root/autonomous_mapping_rviz.log 2>&1'

sleep 3
docker exec "${CONTAINER}" bash -lc 'DISPLAY=:1 xdotool search --onlyvisible --name "RViz" 2>/dev/null | while read w; do
  xdotool windowstate --remove MAXIMIZED_VERT --remove MAXIMIZED_HORZ "$w" 2>/dev/null || true
  xdotool windowsize "$w" 1100 750 2>/dev/null || true
done'

echo "自动建图 RViz 已启动"
