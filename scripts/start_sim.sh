#!/bin/bash
# 启动比赛场地仿真 (Gazebo Classic in Docker)
set -e

CONTAINER=ros1_modeling
FORCE_RESTART=0
if [ "${1:-}" = "--restart" ]; then
    FORCE_RESTART=1
elif [ "$#" -gt 0 ]; then
    echo "用法：$0 [--restart]" >&2
    exit 2
fi

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
docker cp "$(dirname "$0")/../insert/person_standees/." ${CONTAINER}:/root/person_standees
docker cp "$(dirname "$0")/../insert/car_standees/." ${CONTAINER}:/root/car_standees
docker cp "$(dirname "$0")/../models/traffic_light/." ${CONTAINER}:/root/traffic_light
docker cp "$(dirname "$0")/traffic_light_controller.py" ${CONTAINER}:/root/traffic_light_controller.py
docker exec ${CONTAINER} mkdir -p /root/.gazebo/models/traffic_light
docker cp "$(dirname "$0")/../models/traffic_light/." ${CONTAINER}:/root/.gazebo/models/traffic_light

# 4. 启动 roscore (若未运行)
if ! docker exec ${CONTAINER} pgrep -f roscore > /dev/null; then
    echo "启动 roscore ..."
    docker exec -d ${CONTAINER} bash -c "source /opt/ros/noetic/setup.bash && roscore"
    sleep 5
fi

# 5. 启动 Gazebo（以 ROS 服务可调用为健康标准，不只检查进程名）
if [ "$FORCE_RESTART" = 1 ] || ! docker exec ${CONTAINER} bash -lc "source /opt/ros/noetic/setup.bash; timeout 3 rosservice call /gazebo/get_world_properties >/dev/null 2>&1"; then
    echo "启动 Gazebo ..."
    # 先让 ROS master 注销旧节点，避免误把上一代 Gazebo 的幽灵服务
    # 当成新实例已经就绪。
    docker exec ${CONTAINER} bash -lc \
        "source /opt/ros/noetic/setup.bash; rosnode kill /gazebo /gazebo_gui >/dev/null 2>&1 || true"
    docker exec ${CONTAINER} pkill -x gzclient 2>/dev/null || true
    docker exec ${CONTAINER} pkill -x gzserver 2>/dev/null || true
    sleep 2
    docker exec -d -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 ${CONTAINER} \
        bash -c "source /opt/ros/noetic/setup.bash && rosrun gazebo_ros gazebo --verbose /root/competition_classic.world > /root/gz_out.log 2>&1"
    echo "等待 Gazebo ROS 服务 ..."
    READY_STREAK=0
    for _ in $(seq 1 45); do
        if docker exec ${CONTAINER} bash -lc \
            "ps -C gzserver -o stat= | grep -qv '^[[:space:]]*Z' && source /opt/ros/noetic/setup.bash && timeout 2 rosservice call /gazebo/get_world_properties 2>/dev/null | grep -q 'success: True'"; then
            READY_STREAK=$((READY_STREAK + 1))
            if [ "$READY_STREAK" -ge 2 ]; then
                GAZEBO_READY=1
                break
            fi
        else
            READY_STREAK=0
        fi
        sleep 1
    done
    if [ "${GAZEBO_READY:-0}" != 1 ]; then
        echo "错误：Gazebo 在 45 秒内未就绪，请查看容器内 /root/gz_out.log" >&2
        exit 1
    fi
fi

# 6. 启动两套同步交通灯：红 10 秒 -> 绿 15 秒 -> 黄 3 秒。
# 脚本可能已更新，或旧节点仍连着失效的 Gazebo 服务，因此每次都重启。
docker exec ${CONTAINER} pkill -f '^python3 /root/traffic_light_controller.py$' 2>/dev/null || true
for _ in $(seq 1 20); do
    if ! docker exec ${CONTAINER} pgrep -f '^python3 /root/traffic_light_controller.py$' >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done
echo "启动交通灯控制器 ..."
docker exec -d -e PYTHONUNBUFFERED=1 ${CONTAINER} bash -c "source /opt/ros/noetic/setup.bash && exec python3 /root/traffic_light_controller.py > /root/traffic_light.log 2>&1"

echo "完成。状态话题：/traffic_light/state /traffic_light/time_remaining /traffic_light/ready"
