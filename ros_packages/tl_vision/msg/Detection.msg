# 单个目标的检测结果 (像素坐标, 左上原点)

int32   class_id      # 类别 id, 对应 config/data.yaml 的 names 顺序
string  class_name    # 类别名, 如 "red_on" / "license_plate"
float32 confidence    # 置信度 0~1

float32 x1            # 左上角 x
float32 y1            # 左上角 y
float32 x2            # 右下角 x
float32 y2            # 右下角 y
