#!/bin/bash
# 打开 RViz 查看相机画面/雷达/TF
CONTAINER=ros1_modeling
DIR="$(cd "$(dirname "$0")" && pwd)"

sudo docker cp "$DIR/competition.rviz" ${CONTAINER}:/root/competition.rviz

# 启动 RViz
sudo docker exec ${CONTAINER} bash -lc 'pkill -x rviz 2>/dev/null || true'
sudo docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 ${CONTAINER} bash -c \
    "source /opt/ros/noetic/setup.bash && rosrun rviz rviz -geometry 1100x750+50+50 -d /root/competition.rviz > /root/rviz.log 2>&1"

# RViz may inherit a stale maximized state from a previous session. Clear it
# so the normal window borders and mouse resizing remain available.
sleep 2
sudo docker exec ${CONTAINER} bash -lc 'DISPLAY=:1 xdotool search --onlyvisible --name "RViz" 2>/dev/null | while read w; do
  xdotool windowstate --remove MAXIMIZED_VERT --remove MAXIMIZED_HORZ "$w" 2>/dev/null || true
  xdotool windowsize "$w" 1100 750 2>/dev/null || true
done'

echo "RViz 已启动"
