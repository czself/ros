# tl_vision

红绿灯 / 车牌 / 行人一体化视觉，YOLOv8 + PaddleOCR，ROS1 (Noetic/Melodic)。

一套代码同时覆盖**车上实时推理**和**离线批量评测**，核心推理逻辑与 ROS 解耦，
所以没装 ROS 也能跑离线工具和单元测试。

| 能力 | 入口 | 需要 paddleocr？ |
|---|---|---|
| ROS 实时检测节点 | `rosrun tl_vision tl_vision_node` | 否 |
| ROS 车牌识别节点 | `rosrun tl_vision tl_vision_ocr_node` | **是** |
| 离线批量推理 / 评测 | `python3 -m tl_vision.offline` | 否（加 `--ocr` 才需要）|
| 离线车牌识别 | `python3 -m tl_vision.ocr` | **是** |
| 训练 | `python3 -m tl_vision.train` | 否 |
| 数据集体检 | `python3 -m tl_vision.dataset check` | 否 |

> **检测和识别是两个独立节点，不是绑在一起的。** 检测负责「车牌在哪」，
> 识别负责「车牌写了啥」，两者只通过 `/tl_vision/plate_crop` 一个话题通信。
> 为什么这么拆见 [§4](#4-车牌-ocr)。

---

## 1. 类别定义

9 类，`class_id` 的含义由下表固定，**改顺序会让历史模型全部张冠李戴**：

| id | 名称 | 说明 |
|---:|---|---|
| 0 | `resident` | 住户 |
| 1 | `stranger` | 陌生人 |
| 2 | `red_on` | 红灯亮 |
| 3 | `red_off` | 红灯灭 |
| 4 | `yellow_on` | 黄灯亮 |
| 5 | `yellow_off` | 黄灯灭 |
| 6 | `green_on` | 绿灯亮 |
| 7 | `green_off` | 绿灯灭 |
| 8 | `license_plate` | 车牌 |

此顺序在 `config/data.yaml`、`src/tl_vision/detector.py` 的
`DEFAULT_CLASS_NAMES`、`msg/Detection.msg` 三处各写了一份，
单元测试 `test_shipped_data_yaml_matches_detector_order` 会校验它们一致。

---

## 2. 安装

### 2.1 依赖

Noetic 固定 Python 3.8，而开发机可能是 3.10+，所以 `requirements.txt` 一律用
版本下限而非锁定版本——锁死会在队友机器上装不上。

```bash
# 先按 https://pytorch.org/get-started/locally/ 装匹配自己 CUDA 的 torch
pip3 install torch torchvision
pip3 install -r requirements.txt
```

ROS 侧依赖（一条命令装齐）：

```bash
sudo apt install ros-noetic-rospy ros-noetic-cv-bridge ros-noetic-sensor-msgs
```

### 2.2 编译

```bash
cd ~/catkin_ws/src
git clone <本仓库> tl_vision
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2.3 放置权重

**权重不入 git**（`*.pt` 已在 `.gitignore` 里）。自己拷一份：

```bash
mkdir -p ~/catkin_ws/src/tl_vision/weights
cp /path/to/best.pt ~/catkin_ws/src/tl_vision/weights/
```

然后二选一告诉节点去哪找：

```bash
# 方式 A：改配置（推荐，长期固定）
#   编辑 config/vision_config.yaml -> model.weights: "weights/best.pt"
#   相对路径相对于 config 的上一级目录解析，整包搬迁不用改

# 方式 B：环境变量（临时调试方便）
export TL_VISION_WEIGHTS=~/catkin_ws/src/tl_vision/weights/best.pt
```

### 2.4 验证

```bash
# 单元测试，不需要权重、不需要 ROS
catkin_make run_tests_tl_vision     # 或
python3 -m unittest discover -s $(rospack find tl_vision)/test

# 冒烟测试：随便找张图跑一下
python3 -m tl_vision.offline -i /path/to/any.jpg -o /tmp/out
```

---

## 3. 实时节点用法

最简：

```bash
roslaunch tl_vision vision.launch
```

换相机 / 换权重 / 调阈值：

```bash
roslaunch tl_vision vision.launch \
    image_topic:=/usb_cam/image_raw \
    weights:=$HOME/tl_vision/weights/best.pt \
    conf:=0.35
```

或直接 `rosrun` + 私有参数（调参不用改文件）：

```bash
rosrun tl_vision tl_vision_node _conf:=0.35 _device:=0 _log_level:=DEBUG
```

### 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera/image_raw` | `sensor_msgs/Image` | 可用 launch 参数改 |
| 发布 | `/tl_vision/detections` | `tl_vision/DetectionArray` | **主输出** |
| 发布 | `/tl_vision/image_annotated` | `sensor_msgs/Image` | 画框图，可在 config 关 |
| 发布 | `/tl_vision/plate_crop` | `tl_vision/PlateCrop` | 车牌裁剪（位置+图像），每个车牌一帧，OCR 节点订阅这个 |
| 发布 | `/tl_vision/plate_crop_image` | `sensor_msgs/Image` | 裸裁剪图，仅给 `rqt_image_view` 看图，默认不发布 |

### 消息定义

```
# tl_vision/Detection
int32   class_id
string  class_name
float32 confidence
float32 x1, y1, x2, y2        # 像素坐标，左上角为原点

# tl_vision/DetectionArray
std_msgs/Header header        # stamp 取自输入图像
Detection[]  detections
float32 inference_ms          # 纯推理耗时
uint32  frame_width
uint32  frame_height
```

**`detections` 的顺序是有约定的**：行人 → 红绿灯 → 车牌，组内按置信度降序。
下游（语音播报、闸机控制）取 `detections[0]` 即最高优先级目标。

### 下游例子

```python
from tl_vision.msg import DetectionArray
import rospy

def on_detections(msg):
    for d in msg.detections:
        if d.class_name == 'license_plate':
            rospy.loginfo("车牌: conf=%.2f", d.confidence)
        elif d.class_name in ('red_on', 'yellow_on', 'green_on'):
            rospy.loginfo("灯色: %s", d.class_name)

rospy.Subscriber('/tl_vision/detections', DetectionArray, on_detections)
```

---

## 4. 车牌 OCR

### 4.1 为什么拆成独立节点

检测和识别**必须**是两个进程，不只是为了帧率：

| | 原因 |
|---|---|
| 1 | **依赖装不到一起**。检测要 `torch`/`ultralytics`，识别要 `paddleocr`/`paddlepaddle`，两套对 `numpy`、`opencv`、Python 版本的要求互相冲突。实测 paddleocr 2.7 认 py3.8+numpy 1.24+opencv 4.6，而 ultralytics 8.x 已在 py3.10+numpy 2.2+opencv 5.0 上验证——装不进同一个环境 |
| 2 | **速度差两个数量级**。检测 13~25 ms/帧，识别 370 ms/张（2.x CPU）~ 5.6 s/张（3.x CPU）。绑一起相机会掉到 0.2~2.7 FPS |
| 3 | **识别是可选的**。车上没装 paddleocr 时检测照跑，不该被拖死 |
| 4 | **可分开部署**。识别很慢，可以扔到另一台机器上跑 |

接缝只有一个话题：检测发 `PlateCrop`（位置 + 像素绑在同一条消息里），
识别回 `PlateText`（原样带回 `header`，所以 `(stamp, index)` 唯一确定说的是哪块车牌）。

> 不用裸 `sensor_msgs/Image` 是有意的：图像消息里塞不下位置，下游就只能靠
> 「同帧内第 N 张图 = 第 N 个检测框」去反推，队列一丢消息对应关系就错位了。

### 4.2 跑起来

```bash
# 检测 + 识别，同一台机器两个进程
roslaunch tl_vision vision.launch ocr:=true

# 只起识别（检测已在别处跑着，或识别放在另一台机器）
roslaunch tl_vision ocr.launch

# 没装 paddleocr 也能验证话题链路：stub 引擎走完整流程，只是不真识别
roslaunch tl_vision ocr.launch ocr_engine:=stub
```

`rosrun` + 私有参数：

```bash
rosrun tl_vision tl_vision_ocr_node _engine:=stub _strict:=true _min_conf:=0.85
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `_engine` | `paddleocr` | `paddleocr` 真识别 / `stub` 空结果，仅联调 |
| `_gpu` | `false` | 车载机多半没 GPU；开了能快一个数量级 |
| `_strict` | `false` | 校验不通过时**同时清空 `text` 和 `display`**。下游若是「非空即信任」必须开 |
| `_min_conf` | `0.0` | OCR 置信度低于此值也标为不合规（文本仍保留便于排查）|
| `_drop_invalid` | `false` | 只发布合规结果，不合规的只写日志 |
| `_mkldnn` | `false` | 见 §10 的 oneDNN 说明 |

### 4.3 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/tl_vision/plate_crop` | `tl_vision/PlateCrop` | 检测节点发的车牌裁剪 |
| 发布 | `/tl_vision/plate_text` | `tl_vision/PlateText` | **主输出**，每块车牌一条 |
| 发布 | `/tl_vision/plate_text_image` | `sensor_msgs/Image` | 识别结果标回裁剪图，绿框=合规红框=不合规 |

```
# tl_vision/PlateCrop  (检测 -> 识别)
std_msgs/Header header
int32   index               # 该帧内第几个车牌，从 0 开始
float32 x1, y1, x2, y2      # 车牌在原图中的位置
float32 confidence          # 检测置信度
sensor_msgs/Image image     # 裁剪图, bgr8

# tl_vision/PlateText  (识别 -> 下游)
std_msgs/Header header      # 原样回传 PlateCrop 的 header
int32   index
float32 x1, y1, x2, y2
string   raw                # 引擎原始输出，未处理，排查用
string   text               # 规范化后 "苏AB8Q62"；strict 且非法时为空
string   display            # 展示用 "苏A·B8Q62"
float32  confidence         # OCR 置信度（不是检测置信度）
float64  elapsed_ms
bool     valid              # 是否通过 GA 36-2018 号牌规则校验
string   issue              # 未通过的原因；valid 为 true 时为空
```

> `PlateText` 里的 `x1..y2` 是**当初检测到车牌时的位置**。识别要几百毫秒，
> 那时车已经开走了，所以这个坐标用来回溯「当时在哪」，不是「现在在哪」。

### 4.4 下游例子

```python
import rospy
from tl_vision.msg import PlateText

def on_plate(msg):
    if not msg.valid:
        rospy.logwarn("车牌不合规: %s", msg.issue)
        return
    rospy.loginfo("车牌 %s (%.2f, %.0f ms)", msg.display, msg.confidence, msg.elapsed_ms)
    # 闸机放行之类的判断放这里

rospy.Subscriber('/tl_vision/plate_text', PlateText, on_plate)
```

**背压**：识别远慢于相机帧率，`ocr.queue_size` 固定为 1——处理不过来时直接丢旧帧。
绝不能加大，车牌文本几百毫秒后就过期了，堆积只会让结果更没用。

### 4.5 离线识别

```bash
# 识别一批已裁好的车牌图
python3 -m tl_vision.ocr -i crop_plate/ -o ocr_result.txt

# 出 json，便于后续统计
python3 -m tl_vision.ocr -i crop_plate/ --json result.json

# 不装 paddleocr 时验证脚本本身能跑
python3 -m tl_vision.ocr -i crop_plate/ --engine stub
```

或者在离线推理里一条龙做完（检测 → 裁剪 → 识别）：

```bash
python3 -m tl_vision.offline -i images/val -o out/ --save-crops --ocr \
    --ocr-output out/ocr.txt --json out/summary.json
```

---

## 5. 离线批处理

不需要 ROS，适合换模型、对比阈值、批量出评测报告：

```bash
# 整个目录，画框图 + 车牌裁剪 + json 汇总
python3 -m tl_vision.offline \
    -i ~/datasets/tl/images/val \
    -o ~/out/val_run1 \
    -w weights/best.pt \
    --save-crops --json ~/out/val_run1/summary.json

# 只关心红绿灯和车牌，顺便放宽阈值
python3 -m tl_vision.offline -i imgs/ -o out/ --classes 2,3,4,5,6,7,8 --conf 0.3

# 冒烟/CI 用：任何一张图没检出就返回非零
python3 -m tl_vision.offline -i imgs/ -o out/ --fail-on-empty
```

退出码：`0` 正常 / `1` 权重或初始化失败 / `2` 参数或配置错误 / `3` 触发 `--fail-on-empty`。

---

## 6. 训练

```bash
python3 -m tl_vision.train --data config/data.yaml --epochs 100
```

`config/data.yaml` 里的 `path:` 要改成训练机上的数据集根目录。

相对旧版 `train.py` 的改动，都是为了这个数据集的具体问题：

- 路径全走参数/env，不再硬编码 `D:\traffic_light_dataset`
- 加了早停（`--patience`，默认 20）——本数据集 mAP50 在 **第 53 轮就到顶**，
  后面 47 轮纯属浪费算力
- `save_period=-1`，旧版每 10 轮存一个 ~90MB 的 checkpoint，10 个就是 900MB
- `exist_ok=False`，避免第二次训练直接覆盖第一次的结果
- 默认开 AMP。旧版为绕开一个 ultralytics 自动下载 `yolo26n.pt` 的问题而全局关掉了
  AMP，那个是治标；这里只在需要时显式关掉权重自动下载
- `--export-onnx` 可直接导出 ONNX，方便后面转 TensorRT

---

## 7. 数据集体检

```bash
python3 -m tl_vision.dataset --root ~/datasets/tl check
python3 -m tl_vision.dataset --root ~/datasets/tl stats
python3 -m tl_vision.dataset --root ~/datasets/tl rebalance
```

- `check` — 图片/标签一一对应、空标签、字段数、归一化坐标越界
- `stats` — 每类数量与占比
- `rebalance` — 长尾诊断与建议采样倍数（**只报告，不改数据**）

---

## 8. 当前模型的已知问题

用仓库里的这份 best.pt 跑验证集，18 张里有 **4 张完全检不出目标**：

| 图片 | 真值 | 模型输出 |
|---|---|---|
| `red.on.png` | `red_on` | 无（降到 conf 0.05 才出 `license_plate@0.06`，纯噪声） |
| `red_off.png` | `red_off` | 无 |
| `yellow_off.png` | `yellow_off` | 无（conf 0.05 时出 `resident@0.41`） |
| `yellow_on.png` | `yellow_on` | 无 |

**原因**：这 4 张是 1024×1024 的纯色灯色特写，而 285 张训练图**全部**是 480×640 的
真实场景图。训练集里根本没有这种分布，模型没见过，所以不是调阈值能解决的。

### 处理建议（按性价比排序）

1. **补训练数据**：把这类整幅灯色特写图补进训练集，或对训练图做尺度增广。
2. **确认这类图是否是真实需求**：如果小车不会遇到这种画面，就从验证集里剔除，
   否则指标长期被这 4 张拖住，且没有任何指导意义。
3. **别用文件名特判去补**——旧版 `test_vision.py` 就是这么干的：命中 `红灯亮`/`红灯暗`
   就跳过模型直接返回整图框、`conf` 写死 0.9。那不是预测结果。
   本包保留了同样能力，但默认关闭、必须显式开启、结果会被标 `forced=true`，
   json 汇总里单独统计，`--fail-on-empty` 之类的评测也不会被它骗过。

详细分析见 [`docs/dataset_report.md`](docs/dataset_report.md)。

---

## 9. 项目结构

```
tl_vision/
├── config/
│   ├── vision_config.yaml    节点配置（唯一需要改的运行时配置）
│   └── data.yaml             训练数据配置
├── msg/
│   ├── Detection.msg
│   ├── DetectionArray.msg
│   ├── PlateCrop.msg         检测 -> 识别
│   └── PlateText.msg         识别 -> 下游
├── launch/
│   ├── vision.launch         检测（ocr:=true 可带上识别）
│   └── ocr.launch            只起识别
├── src/tl_vision/
│   ├── detector.py           推理核心        ← 无 ROS 依赖
│   ├── visualize.py          绘制 / 车牌裁剪  ← 无 ROS 依赖
│   ├── config.py             配置解析与校验    ← 无 ROS 依赖
│   ├── plate_text.py         车牌文本规范化+校验 ← 无 ROS 依赖
│   ├── ocr.py                PaddleOCR 2.x/3.x 适配 ← 无 ROS 依赖
│   ├── offline.py            离线批处理      ← 无 ROS 依赖
│   ├── train.py              训练            ← 无 ROS 依赖
│   ├── dataset.py            数据集体检      ← 无 ROS 依赖
│   ├── ocr_cli.py            离线识别命令行    ← 无 ROS 依赖
│   ├── node.py               ROS 实时检测节点   ← 需 torch
│   └── ocr_node.py           ROS 车牌识别节点   ← 需 paddleocr，不需要 torch
├── scripts/                  rosrun 入口薄壳
├── test/                     单元测试（不需权重、不需 ROS、不需 paddleocr）
└── docs/
```

**无 ROS 依赖**的模块可以直接 `import` 使用，也可以单独跑单测。

还有一条**更硬**的边界要守住：`ocr.py` / `plate_text.py` / `ocr_node.py`
**不能 import 任何会拉起 torch 的东西**。因为 paddleocr 与 torch 常常装不进
同一个环境（§4.1），一旦 `ocr_node.py` 里引入了会拉起 torch 的路径，队友在
paddleocr 环境里就根本起不来这个节点。`detector.py` 内部对 ultralytics 是
**延迟导入**，`config.py` 也只用它不碰 torch 的那部分——改这两个文件时请保持住。

要加新功能（比如接 ROS2、换 OCR 引擎）时同样请保持边界：
ROS 相关代码只出现在 `node.py` / `ocr_node.py` 里。

---

## 10. 常见问题

**节点起不来，报"未指定模型权重"**
按 §2.3 放权重，或 `export TL_VISION_WEIGHTS=...`。

**`catkin_make` 报找不到 `tl_vision.msg`**
消息还没生成。确认 `CMakeLists.txt` 里有 `add_message_files` + `generate_messages`，
且 `package.xml` 里有 `message_generation` / `message_runtime`。

**图上标注是 `red_on` 不是"红灯亮"**
`cv2.putText` 的 Hershey 字体不支持中文。中文语义看 `class_name` 字段或终端日志。

**推理很慢**
CPU 上约 155ms/张。确认 `device=0` 真的用上了 GPU：
`python3 -m tl_vision.offline -i 单张.jpg -w best.pt` 看日志里的 `device=`。
节点启动有预热，第一帧不会特别慢。

**`torch.load` 报 `UnpicklingError`**
torch>=2.6 收紧了 `weights_only` 默认值。训练脚本已显式传
`torch_safe_load=False`；若自己写推理代码遇到，需要同样处理。

**OCR 节点报「未安装 paddleocr」**
识别是可选依赖，装不了也不影响检测节点。想验证话题链路可以先
`roslaunch tl_vision ocr.launch ocr_engine:=stub`。

**OCR 全部失败，报 `(Unimplemented) ConvertPirAttribute2RuntimeAttribute`**
`paddlepaddle 3.3.x` 在 PIR 新执行器下开 oneDNN 的已知问题。本包在 3.x 上
**默认已关掉** oneDNN，若你手动传了 `_mkldnn:=true` 就会撞上。2.x 不受影响。
（实测 3.7.0 关掉后每张约 5.6 s，2.7.0 约 0.37 s——CPU 上嫌慢的话，
优先考虑装 2.x，而不是打开 oneDNN。）

**paddleocr 和 torch 装不进同一个环境**
这正是把 OCR 拆成独立节点的原因：两套依赖的 numpy/opencv/Python 版本冲突。
开一个单独的 conda 环境给 paddleocr，两个节点各跑各的环境即可，见 §4.1。

**识别出 `XYZ12345` 这种明显不对的车牌**
那是 OCR 认错了，`valid` 会是 `false` 并在 `issue` 里写明原因。
开 `_strict:=true` 会让 `text` 和 `display` 一起清空，下游「非空即信任」
的逻辑就不会误用脏数据。

---

## 11. 约定

- 改动类别顺序 → 必须同步 `data.yaml`、`detector.py`、`msg/*.msg` 三处，并跑单测
- 调阈值先试离线工具，不要直接改 `config` 提交
- 不要把 `images/`、`labels/`、`*.pt` 提交进仓库（`.gitignore` 已挡，提交前确认一次）
