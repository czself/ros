#!/bin/bash
# 键盘遥控小车（保留 AMCL 与速度安全门控）
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
docker cp "$ROOT_DIR/scripts/car_teleop.py" ros1_modeling:/root/car_teleop.py
# Keep map_server and AMCL alive.  move_base is idle without a goal, while
# keeping it running prevents its required roslaunch parent from tearing down
# the map -> odom transform needed to record fixed points.
docker exec -it -e DISPLAY=:1 ros1_modeling bash -c \
    "source /opt/ros/noetic/setup.bash && python3 /root/car_teleop.py"
