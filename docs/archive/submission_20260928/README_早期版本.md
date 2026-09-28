# 智慧社区 ROS/Gazebo 复赛工程

本工程用于展示 Gazebo 场景搭建、GMapping 自主建图、AMCL 定位、`move_base`
多点导航与基于相机的红绿灯颜色识别。仿真运行在 Docker 容器 `ros1_modeling` 内。

## 一次完整演示

1. 启动场景：`./scripts/start_sim.sh`；修改 world 或模型后使用 `./scripts/start_sim.sh --restart` 强制重载。
2. 打开 SLAM 视图：`./scripts/view_autonomous_mapping.sh`
3. 启动真实建图：`./scripts/start_slam_mapping.sh`
4. 用 `./scripts/teleop.sh` 低速覆盖所有可达走廊，RViz 同时录制 `/map`、`/scan` 和 TF；若使用自动路线则改用 `./scripts/start_autonomous_mapping.sh`。
5. 保存本次建图：自动路线使用 `./scripts/save_slam_map.sh competition_slam route_complete complete_session automatic`，人工遥控使用 `./scripts/save_slam_map.sh competition_slam manual_frontier operator_requested manual_session manual_teleop`。脚本会同时生成 coverage report、轨迹/mapper/日志 hash 和消息级 TF 证据。
6. 重启仿真后，以保存地图开始定位导航：`./scripts/start_navigation.sh /root/ros1_ws/maps/competition_navigation_safe.yaml`
   `competition_navigation_safe` 是把当前 world 的碰撞几何叠加到测量地图上的导航安全图，
   用于防止 SLAM 的自由单元覆盖实体墙；它不是纯 SLAM 产物。若只做 SLAM 证据验收，
   使用 `competition_slam_verified4`，不要把两者混称。
7. 在 RViz 点击 `2D Pose Estimate`，将定位箭头放在小车实际初始位置；随后点击 `2D Nav Goal` 验证避障。
8. 自动多点巡检与相机检测：`./scripts/start_patrol.sh competition_navigation_safe`。巡检每次
   都从实时全局膨胀代价地图的出生点连通安全区域随机抽取 5 个目标；如需复现一条
   路线，可在容器内给 `patrol_controller` 传入 `_seed:=整数`。

`maps/competition_ground_truth.yaml` 是用于调参和回归测试的真值地图，不能代替第 3 至第 5 步的 SLAM 录像或提交证据。

重新扫图并导航建议执行 `./scripts/rescan_and_navigate.sh`：它会启动全新 GMapping、生成时间戳地图名，等待你用 `./scripts/teleop.sh` 覆盖通道，保存带 manifest 的地图后自动启动导航。`start_navigation.sh` 默认拒绝旧地图；仅调试旧图时显式设置 `ALLOW_LEGACY_MAP=1`。

## 复赛交付清单

- `worlds/competition_classic.world`：可运行的比赛场景
- `maps/competition_slam.*`：录制演示时保存的 SLAM 地图
- `scripts/`：仿真、建图、导航、巡检与视觉节点
- `docs/technical_solution.md`：技术方案正文，可据此制作 PDF/PPT
- 录制视频时应同时显示 Gazebo 与 RViz，并展示关键代码和结果话题

## 验收话题

`/map`、`/scan`、`/my_car/odom`、`/move_base/NavfnROS/plan`、
`/move_base/local_costmap/costmap`、`/inspection/traffic_light`、`/inspection/image`。

## 注意

所有导航目标必须在已建图的可通行区域内。场地中存在实体墙隔开的区域；不能在单次
GMapping 会话中通过重置或瞬移机器人去拼接地图，否则 `map -> odom` 位姿关系会失效。
