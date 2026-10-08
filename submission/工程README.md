# {{TEAM}}：智慧社区复赛工程代码

本目录是 ROS 1 Noetic / Gazebo Classic 的源码工作空间。主线为 GMapping 建图、AMCL 定位、Navfn 全局规划、自定义 ForwardPathFollower 局部控制、YOLO 检测、白线与交通灯门控、十点巡检及人物统计。

## 文件目录

```text
README.md                       本说明
setup_environment.sh            创建独立提交演示环境
src/community_inspection/       场景、模型、地图、导航配置、运行节点和脚本
src/forward_path_follower/      当前局部控制插件
src/safe_escape_recovery/       恢复插件源码；当前默认关闭恢复行为
src/tl_vision/                  用户提供的视觉/OCR ROS 包；OCR 当前暂停
PACKAGE_MANIFEST.json           来源、整理变更与文件 SHA-256
```

`community_inspection/models/`、`insert/` 包含场地、红绿灯、人物立牌、车牌和车辆资产。实际世界文件是 `worlds/competition_classic_adjusted_20260924.world`；实际默认导航地图是 `maps/current_slam_preview_white_lines.yaml`。`models/ocr` 中的 Tesseract 模型仅为历史离线试验资产，不表示已启用车牌字符识别。

## 环境和编译

主机需有 Docker、Linux X11 桌面、Python 3、OpenCV、NumPy、PyYAML、Pillow。中文播报需 `libespeak-ng` 与 `pw-play` 或 `aplay`。Ubuntu 主机可安装 `python3-opencv python3-numpy python3-yaml python3-pil libespeak-ng1 pipewire-bin alsa-utils x11-xserver-utils`。

```bash
# 在本 README 所在目录执行；会下载基础镜像及依赖，需要网络
chmod +x setup_environment.sh
./setup_environment.sh
```

默认使用独立容器 `smart_community_submission`，避免与开发容器混用。数据保存在本目录 `runtime_data/`。可以通过 `INSPECTION_CONTAINER`、`INSPECTION_DATA_DIR` 和 `INSPECTION_GAZEBO_DIR` 覆盖。所有后续命令需使用同一组环境变量。

准备环境后编译源码（建图、导航或视觉任务均不会自动启动）：

```bash
docker exec smart_community_submission bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /root/ros1_ws; catkin_make -j2'
```

也可把 `src/` 复制到已安装依赖的 Noetic catkin 工作空间，再执行 `catkin_make`。实际运行脚本采用 Docker 布局；单独原生编译成功不代表无需修改容器内 `/root/` 资产路径即可原生启动。

## 运行

```bash
cd src/community_inspection
# 1. 仿真、相机、激光与交通灯控制
./scripts/start_sim.sh
# 2. 建图演示：先开 GMapping，再开 RViz，再遥控覆盖全部可达走廊
./scripts/start_slam_mapping.sh
./scripts/view_autonomous_mapping.sh
./scripts/teleop.sh
# 3. 保存测量地图与来源证据
./scripts/save_slam_map.sh demo_slam manual_frontier operator_requested demo_session manual_teleop
```

手动建图入口 `scripts/start_slam_mapping.sh` 使用轮编码器里程计适配器和实时激光；自动建图入口 `scripts/start_autonomous_mapping.sh` 使用预先规划的巡查路线，不能表述为已实现对任意未知场景的通用探索。自动建图的 `model_state_odom.py` 仍使用 Gazebo 位姿生成仿真里程计，属于仿真依赖；当前十点导航使用轮编码器里程计与 AMCL。实车建图必须改为实际编码器/IMU 里程计。

固定场景的十点完整巡检使用已整理的默认地图：

```bash
./scripts/start_standee_photo_route.sh /root/ros1_ws/maps/current_slam_preview_white_lines.yaml
```

路线为 HOME → POINT_1…POINT_10 → HOME。导航会重置任务起始位姿、检查白线/交通灯门控、模型 SHA、AMCL TF 与传感器就绪；不能在一次建图会话内瞬移车辆拼接地图。激光与深度均参与局部避障；外来人员证据、街区统计与中文播报保存在本次运行目录。

如需从新测量地图重建真实白线导航图，请先查看 `scripts/build_white_line_navigation_map.py --help` 并保留原测量地图、纹理及其 SHA。导航白线图是测量地图的规则叠加图，不能当作纯 SLAM 原始地图。

## 结果与当前边界

主要话题：`/map`、`/scan`、`/camera/image_raw`、`/camera/depth/image_raw`、`/camera/depth/points`、`/odom`、`/inspection/detections`、`/inspection/image`、`/inspection/person_report`、`/inspection/person_image`、`/traffic_light/gate_status`、`/route/status`。

本项目已保存旧导航冻结版本的连续 3 次通过记录，以及人物改进版 `20260928_person_report_2` 的 1 次完整通过：151.213 秒、18 人（社区 16、外来 2）、A/B 各 9 人、普通白线接触 0、未授权通行 0。不同版本的次数不可混合。人物分类验证针对当前展板场景，不等同于真实社区身份核验。

车牌检测与裁剪已运行；OCR 引擎当前按用户要求暂停，字符识别还未达到提交可用状态。`tl_vision` 包包含 PaddleOCR 接口、独立 OCR 节点和测试桩，没有附带 PaddleOCR 模型文件。`best.pt` 是检测权重；它已打包到 `community_inspection/weights/`。上述模块默认不启动 OCR。

包内不包含多 GB 的运行 rosbag、Docker 镜像和训练数据集，编译与运行依赖需按环境脚本安装。当前整理检查不替代新机器上完整建图/导航/视觉流程的复现。
