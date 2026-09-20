#!/bin/bash
# 启动比赛场地仿真 (Gazebo Classic in Docker)
set -e

CONTAINER=ros1_modeling

# 1. 确保容器在运行
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "启动容器 ${CONTAINER} ..."
    docker start ${CONTAINER} || {
        echo "容器不存在，创建中..."
        docker run -d --name ${CONTAINER} \
            -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 \
            -v /home/sz/ros1_ws:/root/ros1_ws \
            -v /home/sz/.gazebo:/root/.gazebo \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            osrf/ros:noetic-desktop-full tail -f /dev/null
    }
    sleep 3
fi

# 2. X11 权限
DISPLAY=:1 xhost +SI:localuser:root > /dev/null

# 3. 拷贝世界文件
docker cp "$(dirname "$0")/../worlds/competition_classic.world" ${CONTAINER}:/root/

# 4. 启动 roscore (若未运行)
if ! docker exec ${CONTAINER} pgrep -f roscore > /dev/null; then
    echo "启动 roscore ..."
    docker exec -d ${CONTAINER} bash -c "source /opt/ros/noetic/setup.bash && roscore"
    sleep 5
fi

# 5. 启动 Gazebo (若未运行)
# Ignore defunct children left by a previous Gazebo shutdown; they cannot publish
# /clock or serve Gazebo APIs and must not block a new simulation instance.
if ! docker exec ${CONTAINER} bash -lc "ps -eo stat,args | awk '\$1 !~ /^Z/ && /[g]zserver/ { found=1 } END { exit !found }'"; then
    echo "启动 Gazebo ..."
    docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 ${CONTAINER} \
        bash -c "source /opt/ros/noetic/setup.bash && rosrun gazebo_ros gazebo --verbose /root/competition_classic.world > /root/gz_out.log 2>&1"
    sleep 15
fi

echo "完成。查看话题: docker exec ${CONTAINER} bash -c 'source /opt/ros/noetic/setup.bash && rostopic list'"
