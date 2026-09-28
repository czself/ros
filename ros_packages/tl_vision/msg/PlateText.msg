# 单个车牌的 OCR 识别结果
#
# 由 tl_vision_ocr_node 发布。header 与它对应的 PlateCrop 一致,
# 因此 (stamp, index) 唯一确定"这条结果说的是哪一块车牌"。
#
# 一帧里有多个车牌时会收到多条消息, 用 header.stamp 归并即可。

std_msgs/Header header
int32   index               # 与 PlateCrop.index 一致

float32 x1                  # 车牌在被检测那一帧原图中的位置。
float32 y1                  # OCR 有几百毫秒延迟, 车已经开走了, 所以这是
float32 x2                  # "当初在哪" 而不是 "现在在哪"。
float32 y2

string   raw                # OCR 引擎的原始输出, 未做任何处理, 便于排查
string   text               # 规范化后的车牌, 如 "苏AB8Q62";
                            # strict=true 且校验不通过时为空串
string   display            # 展示用, 带间隔号, 如 "苏A·B8Q62"
float32  confidence         # OCR 置信度 (0~1), 不是检测置信度
float64  elapsed_ms         # 本次识别耗时, 含预处理

bool     valid              # 是否通过 GA 36-2018 号牌规则校验
string   issue              # 校验未通过的原因; valid 为 true 时为空串
