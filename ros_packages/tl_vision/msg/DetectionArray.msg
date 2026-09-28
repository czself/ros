# 一帧图像的全部检测结果
#
# 约定: detections 按 class_name 分组时的顺序为
#   行人 -> 红绿灯 -> 车牌, 组内按置信度降序。
# 便于下游 (如语音播报 / 闸机逻辑) 直接取 [0]。

std_msgs/Header header      # stamp = 接收该帧的时间, frame_id 见配置 frame_id
Detection[]  detections

float32 inference_ms        # 纯推理耗时 (不含预处理/后处理), 单位毫秒
uint32  frame_width         # 输入图像宽 (像素)
uint32  frame_height        # 输入图像高 (像素)
