#!/bin/bash
# 一键复演: 内圈环线(从出生点出发绕场地内侧一圈回到出生点)。
# 依赖: ros1_modeling 容器已启动仿真; 小车应已复位到出生点(start_navigation.sh 会做)。
#       若 move_base 未运行会自动先起导航。
set -euo pipefail
CONTAINER=ros1_modeling
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_NAME="${1:-competition_slam_verified4}"

docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "容器未运行" >&2; exit 1; }

if ! docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /move_base >/dev/null 2>&1'; then
  "$ROOT_DIR/scripts/start_navigation.sh" "$MAP_NAME"
fi

docker cp "$ROOT_DIR/scripts/inner_ring_loop.py" "$CONTAINER:/root/inner_ring_loop.py"
docker exec "$CONTAINER" bash -lc '
  source /opt/ros/noetic/setup.bash
  rosnode kill /inner_ring_loop 2>/dev/null || true
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_x 0.5
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_trans 0.5
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS max_vel_theta 1.0
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS acc_lim_x 0.8
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS xy_goal_tolerance 0.08
  rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS yaw_goal_tolerance 0.03
  python3 /root/inner_ring_loop.py
' 2>&1
rc=$?
if [[ $rc -eq 0 ]]; then
  echo "完成: 内圈环线回到出生点, 车头朝北。证据: /root/inner_ring_loop_evidence.json"
else
  echo "未通过 exit=$rc, 证据: /root/inner_ring_loop_evidence.json" >&2
fi
exit "$rc"