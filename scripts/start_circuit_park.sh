#!/bin/bash
# 一键复演: 绕场一圈回到出生点 + 尾段倒车入库(车头朝北, 与出发方向一致)。
# 依赖: ros1_modeling 容器已启动仿真; 若 move_base 未运行会自动先起导航。
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_NAME="${1:-competition_slam_verified4}"

docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }

if ! docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /move_base >/dev/null 2>&1'; then
  "$ROOT_DIR/scripts/start_navigation.sh" "$MAP_NAME"
fi

docker cp "$ROOT_DIR/scripts/reverse_park_loop.py" "$CONTAINER:/root/reverse_park_loop.py"
docker exec "$CONTAINER" bash -lc '
  source /opt/ros/noetic/setup.bash
  rosnode kill /reverse_park_loop 2>/dev/null || true
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 1.0
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_trans 1.0
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_theta 3.0
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS acc_lim_x 2.0
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS xy_goal_tolerance 0.08
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS yaw_goal_tolerance 0.03
  python3 /root/reverse_park_loop.py
' 2>&1
rc=$?
if [[ $rc -eq 0 ]]; then
  echo "完成: 绕圈回到出生点, 倒车入库, 车头朝北(出生方向)。证据: /root/reverse_park_loop_evidence.json"
else
  echo "未通过 exit=$rc, 证据: /root/reverse_park_loop_evidence.json" >&2
fi
exit "$rc"