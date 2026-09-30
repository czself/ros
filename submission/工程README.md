# {{TEAM}}：智慧社区复赛工程源码

本 ZIP 是 ROS 1 Noetic catkin 源码包，不含 Docker 镜像、Dockerfile、构建缓存或 bag。来源提交：`{{SOURCE_COMMIT}}`；路径适配和逐文件 SHA256 见 `PACKAGE_MANIFEST.json`。当前检测 `best.pt` 和 PP-OCRv5_mobile_rec 本地模型都在包中。

## 解压与编译

需要 Ubuntu 20.04 / ROS Noetic、Gazebo Classic 与 `gazebo_ros`、`gmapping`、`navigation`、`cv_bridge`、`tf`、`map_server`、`image_view`。Noetic 的 Python 3.8 环境需要 CPU 版 PyTorch 2.2.2、torchvision 0.17.2、Ultralytics 8.2.103、OpenCV、NumPy、SciPy、Pillow、PyYAML。高清车牌 OCR 另用 Python 3.12、PaddlePaddle 3.3.1、PaddleOCR 3.7.0；精确依赖列表和本地模型在 `src/community_inspection/models/ocr/paddle/`。首次安装系统和 Python 依赖需要网络，包内不附带系统软件。

```bash
unzip '{{TEAM}}-智慧社区复赛工程代码.zip'
cd '{{TEAM}}-智慧社区复赛工程代码'
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
OCR_ENV="$PWD/.venv-ocr" ./src/community_inspection/scripts/setup_paddle_ocr.sh
```

如已有满足相同版本要求的主机 OCR 环境，设置 `OCR_PYTHON=/绝对路径/bin/python` 即可使用。运行节点前建议设置 `MPLBACKEND=Agg`，避免无 Tk 环境时 Matplotlib 选择 GUI 后端。源码在普通 catkin 工作空间编译，不调用 Docker 命令。

## 十点任务运行

同一台机器应能访问 Noetic 与 OCR 环境。每次用新的任务编号；已有编号不会被覆盖。Linux 桌面展示需正确设置 `DISPLAY`。在本 README 所在目录运行：

```bash
export DISPLAY=:1
export OCR_PYTHON="$PWD/.venv-ocr/bin/python"
./native_demo.sh demo_001
./native_audit.sh demo_001
```

无图形显示服务器时，已有离屏相机渲染环境可用 `./native_demo.sh demo_002 --headless`。现场另有 ROS master 时使用未占用端口，例如 `--port 11312`，新任务会建立独立ROS会话。录像时保留默认展示入口。`runtime_data/<编号>/` 有10个点位照片、人物、OCR、终端、运行总结和 motion bag；`native_audit.sh` 在 bag 关闭后逐项审计绿灯、白线、速度、目标、人物、OCR、图文对应和回位。

若 ROS Noetic 与主机 OCR 位于不同 Python 环境或容器，Noetic 侧使用 `--external-ocr-worker --external-audio`；宿主环境在同一个可读写数据目录运行 `src/community_inspection/scripts/paddle_plate_worker.py --run-dir <运行目录> --watch --compare-low --ready-file <运行目录>/ocr/ready.json`，任务完成后生成 `person_report.wav` 与 `person_report.audio.json`。这两个参数是环境边界，不会关闭车牌识别或人物播报验收。审计仍需全部结果文件。

## 包内结构与来源

```text
README.md                         本说明
native_demo.sh                    原生 ROS/Gazebo 巡检入口
native_audit.sh                   已关闭 bag 的独立核查入口
PACKAGE_MANIFEST.json             来源提交、权重和逐文件 SHA256
src/community_inspection/         场景、地图、人物和车牌模型、配置、节点脚本
src/forward_path_follower/        自定义导航插件
src/safe_escape_recovery/         恢复插件源码，正式任务禁用恢复行为
src/tl_vision/                    ROS OCR 消息与识别代码
```

运行控制、交通规则阈值与已验证源码相同。为了脱离开发电脑，启动时只把世界材质和 launch 配置中的旧绝对资源路径改为本包的路径，并通过参数指定模型、地图和字体。原默认地图 `maps/current_slam_preview_white_lines.yaml` 的测量来源图、SHA和白线规则说明都在包内。十点完成不等于实车已精确回出生方向：历史20轮原独立审核16/20通过，但Gazebo车身起终点有偏差；新机器需要自己运行并公开独立审计。

完整建图过程仍需队伍录屏。包内有 GMapping 配置、`model_state_odom.py`、`survey_mapper.py`、地图保存与验证算法源码及历史地图；建图模式的 Gazebo 真值里程计属于仿真依赖，实车要换成真实编码器/IMU。
