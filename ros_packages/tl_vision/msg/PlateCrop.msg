# 单个车牌裁剪图, 连同它在原图里的位置
#
# 为什么不让下游直接订阅 sensor_msgs/Image:
#   图像消息里塞不下位置, 下游就只能靠"同一帧内按发布顺序第 N 张 = 第 N 个
#   检测框"这种约定去反推。相机一旦多帧交错, 或者中间有消息被队列丢掉,
#   对应关系就错位了。这里把位置和像素绑成一条消息, 不需要任何约定。
#
# 由 tl_vision_node 发布, tl_vision_ocr_node 订阅。

std_msgs/Header header      # stamp 沿用输入图像的时间戳, frame_id 见配置

int32   index               # 该帧内第几个车牌, 从 0 开始
float32 x1                  # 车牌在原图中的位置 (像素, float32 避免取整丢精度)
float32 y1
float32 x2
float32 y2
float32 confidence          # 检测置信度 —— 注意这不是 OCR 置信度

sensor_msgs/Image image     # 向外扩过 plate_crop_margin 像素的裁剪图, bgr8
