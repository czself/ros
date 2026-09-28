#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$ROOT_DIR/src/community_inspection/scripts/submission_env.sh"
IMAGE="smart-community-submission:noetic"

docker build -t "$IMAGE" -f "$ROOT_DIR/src/community_inspection/Dockerfile" "$ROOT_DIR"
mkdir -p "$INSPECTION_DATA_DIR/src" "$INSPECTION_GAZEBO_DIR/models"
cp -a "$ROOT_DIR/src/." "$INSPECTION_DATA_DIR/src/"
cp -a "$ROOT_DIR/src/community_inspection/models/." "$INSPECTION_GAZEBO_DIR/models/"
if docker container inspect "$INSPECTION_CONTAINER" >/dev/null 2>&1; then
  docker start "$INSPECTION_CONTAINER" >/dev/null
else
  docker run -d --name "$INSPECTION_CONTAINER" \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
    -v "$INSPECTION_DATA_DIR:/root/ros1_ws" \
    -v "$INSPECTION_GAZEBO_DIR:/root/.gazebo" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "$IMAGE" tail -f /dev/null
fi
echo "环境已准备：容器 $INSPECTION_CONTAINER；数据 $INSPECTION_DATA_DIR"
echo "请按根目录 README 编译后启动仿真。"
