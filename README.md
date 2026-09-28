# 智算三行队：智慧社区 ROS/Gazebo 工程

当前主线使用 ROS 1 Noetic / Gazebo Classic、GMapping、AMCL、Navfn、自定义 ForwardPathFollower、YOLO 九类检测和十点任务执行器。

## 当前运行入口

```bash
./scripts/start_sim.sh
./scripts/start_standee_photo_route.sh /root/ros1_ws/maps/current_slam_preview_white_lines.yaml
```

原运行脚本使用开发容器 `ros1_modeling`、本机检测权重 `/home/sz/下载/best.pt` 和 `/home/sz/ros1_ws` 数据目录。场景为 `worlds/competition_classic_adjusted_20260924.world`。完整任务路线 HOME → POINT_1…POINT_10 → HOME，激光与深度避障，普通白线禁行，合法绿灯授权后通行。

建图入口为 `start_slam_mapping.sh`、`view_autonomous_mapping.sh`、`teleop.sh`、`save_slam_map.sh`；自动建图入口为 `start_autonomous_mapping.sh`。建图里程计仍有 Gazebo 位姿依赖，实车必须替换；测量地图、白线规则叠加图和真值诊断图分别保留来源。

## 成果状态

旧导航冻结版本已保存连续 3/3；人物改进版目前有 1 次完整独立审计通过（20260928_person_report_2）：151.213 s、18 人（社区16、外来2）、A/B各9人、普通白线及未授权接触为0、HOME位置误差1.266 cm。不同版本次数分开记录。

车牌检测和裁剪已运行。OCR 引擎按用户要求暂停；新版压缩包的 PaddleOCR 源码会归档，不能写成字符识别已完成。人物分类验证针对本场景静态展板。

## 复赛材料整理

```bash
python3 scripts/organize_submission.py --team 智算三行队
```

默认整理目录：`/home/sz/game/复赛提交材料/智算三行队`。内容为 ROS src 工程、正式命名 ZIP、技术方案草稿、幻灯片提纲、视频脚本/素材、运行证据和历史参考。整理只生成副本，保留原工作区。工程副本补齐检测权重并提供独立容器环境说明。

- [材料整理说明](submission/README.md)
- [当前技术方案草稿](docs/technical_solution.md)
- [答辩提纲](submission/答辩展示提纲.md)
- [视频录制脚本](submission/视频录制脚本.md)
- [人物改进与证据记录](docs/agent_context/tasks/task008/person_refinement_20260928.md)

正式技术方案 PDF、答辩 PDF 和完整 MP4 尚需制作；工程包小于150MB，视频小于300MB并含实操、代码讲解和全队答辩。早期 HSV/DWA 文档存入 `docs/archive/submission_20260928/`，不作为当前方案说明。
