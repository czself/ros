# 消息与接口约定

## Detection.msg

```
int32   class_id      # 0-8，见下表
string  class_name    # 与 class_id 对应的名称
float32 confidence    # 0-1
float32 x1, y1, x2, y2   # 像素坐标，左上角为原点
```

坐标是**原图像素坐标**，未经缩放。若下游要换算归一化坐标：

```python
cx = (d.x1 + d.x2) / 2 / frame_width
cy = (d.y1 + d.y2) / 2 / frame_height
nw = (d.x2 - d.x1) / frame_width
nh = (d.y2 - d.y1) / frame_height
```

正好是 YOLO 标注格式，反向也成立。

## DetectionArray.msg

```
std_msgs/Header header    # stamp 取自输入图像；frame_id 见 config 的 frame_id
Detection[]  detections
float32 inference_ms      # 仅推理耗时，不含预处理与后处理
uint32  frame_width       # 输入图像宽
uint32  frame_height      # 输入图像高
```

`frame_width` / `frame_height` 放在消息里，是为了让下游做坐标换算时不必
另外订阅一次图像，也避免图像与检测结果错配到不同帧。

## detections 顺序约定

**行人 → 红绿灯 → 车牌，组内按置信度降序。**

由 `detector._sorted_by_group` 保证，并有单元测试锁定。

含义：下游可以安全地取 `detections[0]` 作为"最高优先级目标"，
不必自己排序。这是刻意的设计——语音播报、闸机放行这类逻辑
最常见的 bug 就是忘了排序。

```python
msg.detections[0]           # 置信度最高的行人（若有）
[d for d in msg.detections if d.class_name == 'green_on']   # 精确取灯色
```

## 快捷筛选

`detector.InferenceResult` 提供了一些便捷方法（离线场景直接用）：

```python
result.plates             # 所有车牌
result.traffic_lights     # 所有红绿灯
result.people             # 所有行人
result.dominant_light()   # 置信度最高的灯, 返回 Detection 或 None
```

`Detection` 上还有 `is_plate` / `is_person` / `is_traffic_light` 属性，
以及 `light_state`，把 6 个灯色类归并成 `"red"` / `"yellow"` / `"green"`：

```python
det.light_state == 'green'      # green_on 和 green_off 都是 'green'
```

## PlateCrop.msg

检测节点 → OCR 节点。

```
std_msgs/Header header    # stamp 取自输入图像；frame_id 见 config 的 frame_id
int32   index             # 该帧内第几个车牌，从 0 开始
float32 x1, y1, x2, y2    # 车牌在原图中的位置（未含裁剪时外扩的 margin）
float32 confidence        # 检测置信度 —— 不是 OCR 置信度
sensor_msgs/Image image   # 裁剪图，bgr8
```

**为什么不直接发 `sensor_msgs/Image`**：图像消息里塞不下位置，下游就只能靠
「同一帧内按发布顺序第 N 张 = 第 N 个检测框」去反推对应关系。相机一旦多帧交错、
或中间有消息被队列丢掉，关系就错位了。把位置和像素绑进同一条消息就不需要任何约定。

`index` 按 `result.plates` 的顺序编号，与 `DetectionArray.detections` 里
`class_name == "license_plate"` 的出现顺序一致。

若下游只想拿裸图看（例如 `rqt_image_view`），开
`publishing.plate_crop_image: true`，会额外发一份到
`/tl_vision/plate_crop_image`（`sensor_msgs/Image`）。

## PlateText.msg

OCR 节点 → 下游。每块车牌一条。

```
std_msgs/Header header    # 原样回传对应 PlateCrop 的 header
int32   index             # 与 PlateCrop.index 一致
float32 x1, y1, x2, y2
string   raw              # OCR 引擎原始输出，未做任何处理
string   text             # 规范化后 "苏AB8Q62"；strict 且非法时为 ""
string   display          # 展示用 "苏A·B8Q62"；strict 且非法时为 ""
float32  confidence       # OCR 置信度 0-1
float64  elapsed_ms       # 本次识别耗时
bool     valid            # 是否通过 GA 36-2018 号牌规则校验
string   issue            # 未通过的原因；valid 为 true 时为 ""
```

**如何定位来源**：`(header.stamp, index)` 唯一确定一条结果对应的是哪块车牌。
一帧里有多个车牌时会收到多条消息，按 `header.stamp` 归并即可。

`x1..y2` 是**检测到车牌那一刻**在原图中的位置。OCR 要几百毫秒，那时车已经
开走了，所以它是「当初在哪」，不是「现在在哪」。

### strict 同时清空 text 和 display

`ocr.strict: true` 时，校验不通过的结果 `text` 和 `display` **都会**置空。
只清 `text` 是不够的——`display` 是给人看的字段，下游多半读的是它，
而 `PlateText.__str__` 也优先返回 `display`，脏车牌照样会漏出去。
`raw` 始终原样保留，排查时还能看到引擎究竟认出了什么。

下游若是「非空即信任」的逻辑（闸机放行之类），**应当开 strict**。

### 编号规则（GA 36-2018）

`plate_text.validate()` 放行 7 位普通车牌与 8 位新能源车牌，另有 2~4 位
特殊号牌只给 warning 级提示。规则细节与已知取舍见 `src/tl_vision/plate_text.py`
的模块 docstring。

## 坐标系与时间戳

- 图像坐标：像素，左上原点，x 向右、y 向下（OpenCV 惯例，与 cv_bridge 一致）
- 若输入是 ROS 标准 `sensor_msgs/Image` 且 `encoding` 不是 `bgr8`，
  `cv_bridge` 会自动转换
- `header.stamp` 沿用输入图像的时间戳（`publishing.stamp_from_input: true`），
  这样检测结果与图像严格同帧；关掉则用 `rospy.Time.now()`
- 图像队列深度默认 1：处理不过来时丢旧帧保实时性。做人车交互不要设大
- OCR 节点队列同样固定为 1：车牌文本几百毫秒后即过期，堆积毫无意义

## 扩类别时

1. `config/data.yaml` 的 `names`（注意顺序）
2. `src/tl_vision/detector.py` 的 `DEFAULT_CLASS_NAMES`（顺序必须一致）
3. `msg/Detection.msg` — 只有需要新字段时才改，纯加类别不用动
4. 跑 `python3 -m unittest discover -s test` 验证一致性

`PERSON_CLASSES` / `TRAFFIC_LIGHT_CLASSES` / `LIGHT_STATE_GROUPS` /
`visualize.DEFAULT_COLORS` 里有按名字的分类，新增类别记得归类，
否则 `is_traffic_light` 之类的判断和上色会失效。
