# 智算三行队：智慧社区 ROS/Gazebo 工程

当前主线使用 ROS 1 Noetic / Gazebo Classic、GMapping、AMCL、Navfn、自定义 ForwardPathFollower、YOLO 九类检测、高清 PaddleOCR 和十点任务执行器。

## 当前运行入口

```bash
./scripts/start_sim.sh
./scripts/start_standee_photo_route.sh /root/ros1_ws/maps/current_slam_preview_white_lines.yaml
```

OCR 采用独立主机环境：首次执行 `./scripts/setup_paddle_ocr.sh`，默认路径 `/home/sz/.venvs/community-ocr`，Python 3.12、PaddleOCR 3.7.0、PaddlePaddle 3.3.1。ROS 容器仍使用 Python 3.8。

原运行脚本使用开发容器 `ros1_modeling`、本机检测权重 `/home/sz/下载/best.pt` 和 `/home/sz/ros1_ws` 数据目录。场景为 `worlds/competition_classic_adjusted_20260924.world`。完整任务路线 HOME → POINT_1…POINT_10 → HOME，激光与深度避障，普通白线禁行，合法绿灯授权后通行。

建图入口为 `start_slam_mapping.sh`、`view_autonomous_mapping.sh`、`teleop.sh`、`save_slam_map.sh`；自动建图入口为 `start_autonomous_mapping.sh`。建图里程计仍有 Gazebo 位姿依赖，实车必须替换；测量地图、白线规则叠加图和真值诊断图分别保留来源。

## 成果状态

2026-09-29第一轮导航优化已验证：合并拍照稳定等待，最终航向增益调整为2.4。同一冻结运行源码连续3次完整审计通过，耗时142.121、149.334、148.715秒；十点拍照处理平均减少约2.92秒。仍有到点修正波动，人物低置信度尚未解决；离线试验没有替换检测权重。详见 [本轮实测与剩余工作](docs/agent_context/tasks/task009/optimization_results_20260929.md)。

最新导航版本 `d9a61b4` 已独立连续3/3通过：127.973、131.450、128.028秒，均值129.150秒。第7点三轮最终对齐1段、回转补位0次；剔信号等待后比原三轮基线估算省8.4秒。原速度/安全/识别/回位条件继续验收，同周期命令记录7710条全部配对。人物置信度仍波动，未训练新模型。详见 [最新导航实测与回退入口](docs/agent_context/tasks/task009/fast_navigation_results_20260929.md)。之前未通过及中间试验记录保留，不计入当前3/3。

旧导航冻结版本已保存连续 3/3；人物改进版目前有 1 次完整独立审计通过（20260928_person_report_2）：151.213 s、18 人（社区16、外来2）、A/B各9人、普通白线及未授权接触为0、HOME位置误差1.266 cm。不同版本次数分开记录。

高清车牌 OCR 已接入：独立 1920×1440 相机仅在车牌点取图，POINT_8 前移20厘米；每个新导航任务重启场景，抽取三张与上一批不同的车牌。识别使用本地 PP-OCRv5_mobile_rec，文字区域裁剪、多帧一致性和0.85置信度门槛；原始/规范化文字、图像及源帧分别保存。运行入口默认开启 OCR。

`20260928_ocr_closer_1/2/3` 在同一冻结运行源码下连续三次完整独立审计通过，九个不同号码整牌9/9正确，白线和未授权接触均为0；人物18/16/2、HOME与中文播报均通过。旧版本记录分别保留。结果适用于本次仿真场景。

## 复赛材料整理

```bash
python3 scripts/organize_submission.py --team 智算三行队
```

默认整理目录：`/home/sz/game/复赛提交材料/智算三行队`。内容为 ROS src 工程、正式命名 ZIP、技术方案草稿、幻灯片提纲、视频脚本/素材、运行证据和历史参考。整理只生成副本，保留原工作区。工程副本补齐检测权重并提供独立容器环境说明。

- [材料整理说明](submission/README.md)
- [当前技术方案草稿](docs/technical_solution.md)
- [答辩提纲](submission/答辩展示提纲.md)
- [视频录制脚本](submission/视频录制脚本.md)
- [高清OCR改进与证据记录](docs/agent_context/tasks/task008/ocr_hd_20260928.md)
- [OCR三轮验收记录](docs/agent_context/tasks/task008/ocr_hd_acceptance_20260928.json)
- [人物改进与证据记录](docs/agent_context/tasks/task008/person_refinement_20260928.md)

正式技术方案 PDF、答辩 PDF 和完整 MP4 尚需制作；工程包小于150MB，视频小于300MB并含实操、代码讲解和全队答辩。早期 HSV/DWA 文档存入 `docs/archive/submission_20260928/`，不作为当前方案说明。
