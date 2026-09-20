#!/bin/bash
# 键盘遥控小车（建图脉冲控制）
docker cp "$(dirname "$0")/car_teleop.py" ros1_modeling:/root/car_teleop.py
docker exec -it -e DISPLAY=:1 ros1_modeling bash -c \
    "source /opt/ros/noetic/setup.bash && python3 /root/car_teleop.py"
